/*
 * WS2812B LED strip driver for the Raspberry Pi 4 (BCM2711).
 *
 * The strip expects a self-clocked bit stream with ~150ns of timing slack,
 * which the CPU cannot produce reliably under a preemptible kernel. Instead
 * the PWM block is put into serialiser mode and DMA feeds it, so the timing
 * comes from the PWM clock and no CPU jitter can reach the wire.
 *
 * The PWM registers are driven directly because pwm-bcm2835 exposes only
 * PWM mode; the clock and the DMA channel use the normal kernel APIs so we
 * do not fight cprman or bcm2835-dma over shared hardware.
 */

#include <linux/bits.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "led_strip_regs.h"

#define DRIVER_NAME	"led_strip"
#define DEVICE_NAME	"led_strip"
#define CLASS_NAME	"led_strip"

static unsigned int led_count;
module_param(led_count, uint, 0444);
MODULE_PARM_DESC(led_count, "LEDs on the strip (0 = use led-count from device tree)");

static unsigned int reset_us = WS2812_RESET_US_DEFAULT;
module_param(reset_us, uint, 0444);
MODULE_PARM_DESC(reset_us, "latch time held low after a frame, in microseconds");

struct led_strip {
	struct device		*dev;

	void __iomem		*pwm;
	struct clk		*clk;
	unsigned long		pwm_hz;

	struct dma_chan		*dma;
	struct completion	dma_done;
	phys_addr_t		fifo_bus;	/* bus address of PWM_FIF1 */

	u32			*buf;		/* encoded sub-bits, DMA coherent */
	dma_addr_t		buf_dma;
	size_t			buf_words;

	u8			*rgb;		/* frame as written by userspace */
	size_t			rgb_len;
	unsigned int		n_leds;

	struct mutex		lock;

	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
};

/* ------------------------------------------------------------------ */
/* RGB -> PWM sub-bit encoding                                         */
/* ------------------------------------------------------------------ */

/*
 * Expand every colour bit into WS2812_SUBBITS PWM sub-bits and pack them
 * MSB-first into 32-bit words, which is the order the serialiser shifts a
 * FIFO word out. Words past the colour data stay zero and become the latch.
 */
static void led_strip_encode(struct led_strip *ls)
{
	unsigned int led, byte, bit, sub;
	u32 acc = 0;
	size_t w = 0;
	int nbits = 0;

	memset(ls->buf, 0, ls->buf_words * sizeof(u32));

	for (led = 0; led < ls->n_leds; led++) {
		const u8 *px = &ls->rgb[led * WS2812_BYTES_PER_LED];
		/* userspace hands us R,G,B; the strip clocks in G,R,B */
		const u8 grb[WS2812_BYTES_PER_LED] = { px[1], px[0], px[2] };

		for (byte = 0; byte < WS2812_BYTES_PER_LED; byte++) {
			for (bit = 8; bit-- > 0; ) {
				u32 sym = (grb[byte] & BIT(bit)) ?
					  WS2812_SYMBOL_1 : WS2812_SYMBOL_0;

				for (sub = WS2812_SUBBITS; sub-- > 0; ) {
					acc = (acc << 1) | ((sym >> sub) & 1);
					if (++nbits == PWM_WORD_BITS) {
						ls->buf[w++] = acc;
						acc = 0;
						nbits = 0;
					}
				}
			}
		}
	}

	if (nbits)
		ls->buf[w] = acc << (PWM_WORD_BITS - nbits);
}

static int led_strip_alloc_buffers(struct led_strip *ls)
{
	size_t data_subbits, latch_subbits, total_subbits;

	ls->rgb_len = ls->n_leds * WS2812_BYTES_PER_LED;
	ls->rgb = devm_kzalloc(ls->dev, ls->rgb_len, GFP_KERNEL);
	if (!ls->rgb)
		return -ENOMEM;

	data_subbits  = (size_t)ls->n_leds * WS2812_SUBBITS_PER_LED;
	latch_subbits = DIV_ROUND_UP((u64)reset_us * ls->pwm_hz, USEC_PER_SEC);
	total_subbits = data_subbits + latch_subbits;

	ls->buf_words = DIV_ROUND_UP(total_subbits, PWM_WORD_BITS);
	ls->buf = dmam_alloc_coherent(ls->dev, ls->buf_words * sizeof(u32),
				      &ls->buf_dma, GFP_KERNEL);
	if (!ls->buf)
		return -ENOMEM;

	dev_info(ls->dev, "%u LEDs, %zu DMA words (%zu data + %zu latch sub-bits)\n",
		 ls->n_leds, ls->buf_words, data_subbits, latch_subbits);

	return 0;
}

/* ------------------------------------------------------------------ */
/* PWM serialiser + DMA                                                */
/* ------------------------------------------------------------------ */

static void led_strip_dma_callback(void *param)
{
	struct led_strip *ls = param;

	complete(&ls->dma_done);
}

static void led_strip_pwm_stop(struct led_strip *ls)
{
	writel(0, ls->pwm + PWM_CTL);
	writel(0, ls->pwm + PWM_DMAC);
}

/* Push the encoded frame out of the PWM FIFO. Caller holds ls->lock. */
static int led_strip_send(struct led_strip *ls)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	unsigned long timeout;
	u32 sta;
	int ret;

	led_strip_encode(ls);

	/* Flush any stale FIFO content and the sticky error bits (write-1-to-clear) */
	writel(PWM_CTL_CLRF1, ls->pwm + PWM_CTL);
	writel(PWM_STA_ERRS, ls->pwm + PWM_STA);
	writel(PWM_WORD_BITS, ls->pwm + PWM_RNG1);
	writel(PWM_DMAC_ENAB | PWM_DMAC_PANIC(PWM_DMAC_PANIC_LVL) |
	       PWM_DMAC_DREQ(PWM_DMAC_DREQ_LVL), ls->pwm + PWM_DMAC);

	desc = dmaengine_prep_slave_single(ls->dma, ls->buf_dma,
					   ls->buf_words * sizeof(u32),
					   DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(ls->dev, "failed to prepare DMA descriptor\n");
		ret = -EIO;
		goto out_stop;
	}

	desc->callback = led_strip_dma_callback;
	desc->callback_param = ls;

	reinit_completion(&ls->dma_done);

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(ls->dev, "DMA submit failed\n");
		goto out_stop;
	}

	/*
	 * The FIFO drains only once the serialiser runs, but the PWM block
	 * already asserts DREQ, so issuing first lets DMA prefill the FIFO
	 * and the very first word goes out without an underrun.
	 */
	dma_async_issue_pending(ls->dma);
	writel(PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1, ls->pwm + PWM_CTL);

	/* Generous bound: frame time plus slack, never less than 100ms */
	timeout = msecs_to_jiffies(100 +
		(ls->buf_words * PWM_WORD_BITS * 1000) / ls->pwm_hz);

	if (!wait_for_completion_timeout(&ls->dma_done, timeout)) {
		dev_err(ls->dev, "DMA timed out\n");
		dmaengine_terminate_sync(ls->dma);
		ret = -ETIMEDOUT;
		goto out_stop;
	}

	/*
	 * The callback fires when the last word reaches the FIFO, not when it
	 * has been shifted onto the wire. Cutting PWEN now would truncate the
	 * frame, so wait for the FIFO to empty and for that word to clock out.
	 */
	ret = readl_poll_timeout(ls->pwm + PWM_STA, sta, sta & PWM_STA_EMPT1,
				 10, 10000);
	if (ret)
		dev_warn(ls->dev, "FIFO did not drain (STA=0x%08x)\n",
			 readl(ls->pwm + PWM_STA));

	udelay(DIV_ROUND_UP(PWM_WORD_BITS * USEC_PER_SEC, ls->pwm_hz) + 1);

	sta = readl(ls->pwm + PWM_STA);
	if (sta & PWM_STA_ERRS) {
		dev_warn(ls->dev, "PWM reported errors, STA=0x%08x\n", sta);
		writel(PWM_STA_ERRS, ls->pwm + PWM_STA);
	}

	ret = 0;

out_stop:
	led_strip_pwm_stop(ls);
	return ret;
}

/* ------------------------------------------------------------------ */
/* character device                                                    */
/* ------------------------------------------------------------------ */

static int led_strip_open(struct inode *inode, struct file *file)
{
	struct led_strip *ls = container_of(inode->i_cdev, struct led_strip, cdev);

	file->private_data = ls;
	return 0;
}

static ssize_t led_strip_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct led_strip *ls = file->private_data;
	int ret;

	if (*ppos >= ls->rgb_len)
		return -ENOSPC;

	count = min_t(size_t, count, ls->rgb_len - *ppos);
	if (!count)
		return 0;

	if (mutex_lock_interruptible(&ls->lock))
		return -ERESTARTSYS;

	if (copy_from_user(ls->rgb + *ppos, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	ret = led_strip_send(ls);
	if (ret)
		goto out;

	*ppos += count;
	ret = count;

out:
	mutex_unlock(&ls->lock);
	return ret;
}

static int led_strip_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations led_strip_fops = {
	.owner		= THIS_MODULE,
	.open		= led_strip_open,
	.write		= led_strip_write,
	.release	= led_strip_release,
	.llseek		= default_llseek,
};

static int led_strip_register_cdev(struct led_strip *ls)
{
	int ret;

	ret = alloc_chrdev_region(&ls->devt, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&ls->cdev, &led_strip_fops);
	ls->cdev.owner = THIS_MODULE;

	ret = cdev_add(&ls->cdev, ls->devt, 1);
	if (ret)
		goto err_region;

	ls->class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ls->class)) {
		ret = PTR_ERR(ls->class);
		goto err_cdev;
	}

	if (IS_ERR(device_create(ls->class, ls->dev, ls->devt, ls, DEVICE_NAME))) {
		ret = -ENODEV;
		goto err_class;
	}

	return 0;

err_class:
	class_destroy(ls->class);
err_cdev:
	cdev_del(&ls->cdev);
err_region:
	unregister_chrdev_region(ls->devt, 1);
	return ret;
}

static void led_strip_unregister_cdev(struct led_strip *ls)
{
	device_destroy(ls->class, ls->devt);
	class_destroy(ls->class);
	cdev_del(&ls->cdev);
	unregister_chrdev_region(ls->devt, 1);
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

/*
 * The DMA controller addresses peripherals through the 0x7e000000 bus
 * window, not the ARM physical window that platform_get_resource reports,
 * so take the untranslated value straight out of the device tree.
 */
static int led_strip_fifo_bus_addr(struct led_strip *ls)
{
	struct device_node *np = ls->dev->of_node;
	const __be32 *reg;
	u64 size;

	reg = of_get_address(np, 0, &size, NULL);
	if (!reg)
		return -EINVAL;

	ls->fifo_bus = be32_to_cpup(reg) + PWM_FIF1;
	return 0;
}

static int led_strip_setup_dma(struct led_strip *ls)
{
	struct dma_slave_config cfg = {
		.direction	 = DMA_MEM_TO_DEV,
		.dst_addr	 = ls->fifo_bus,
		.dst_addr_width	 = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst	 = 1,
		.src_addr_width	 = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst	 = 1,
	};
	int ret;

	ls->dma = dma_request_chan(ls->dev, "pwm");
	if (IS_ERR(ls->dma))
		return dev_err_probe(ls->dev, PTR_ERR(ls->dma),
				     "failed to get DMA channel\n");

	ret = dmaengine_slave_config(ls->dma, &cfg);
	if (ret) {
		dev_err(ls->dev, "failed to configure DMA channel\n");
		dma_release_channel(ls->dma);
		return ret;
	}

	return 0;
}

static int led_strip_setup_clock(struct led_strip *ls)
{
	unsigned long delta;
	int ret;

	ls->clk = devm_clk_get(ls->dev, NULL);
	if (IS_ERR(ls->clk))
		return dev_err_probe(ls->dev, PTR_ERR(ls->clk),
				     "failed to get PWM clock\n");

	ret = clk_set_rate(ls->clk, WS2812_PWM_HZ);
	if (ret)
		return dev_err_probe(ls->dev, ret, "failed to set PWM clock\n");

	ret = clk_prepare_enable(ls->clk);
	if (ret)
		return dev_err_probe(ls->dev, ret, "failed to enable PWM clock\n");

	ls->pwm_hz = clk_get_rate(ls->clk);
	if (!ls->pwm_hz) {
		clk_disable_unprepare(ls->clk);
		return -EINVAL;
	}

	/* Beyond ~5% the encoded pulse widths leave the WS2812B timing window */
	delta = abs((long)ls->pwm_hz - WS2812_PWM_HZ);
	if (delta * 20 > WS2812_PWM_HZ)
		dev_warn(ls->dev, "PWM clock is %lu Hz, wanted %u Hz\n",
			 ls->pwm_hz, WS2812_PWM_HZ);
	else
		dev_info(ls->dev, "PWM clock %lu Hz\n", ls->pwm_hz);

	return 0;
}

static int led_strip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct led_strip *ls;
	u32 dt_count = 0;
	int ret;

	ls = devm_kzalloc(dev, sizeof(*ls), GFP_KERNEL);
	if (!ls)
		return -ENOMEM;

	ls->dev = dev;
	mutex_init(&ls->lock);
	init_completion(&ls->dma_done);
	platform_set_drvdata(pdev, ls);

	of_property_read_u32(dev->of_node, "led-count", &dt_count);
	ls->n_leds = led_count ? led_count : dt_count;
	if (!ls->n_leds) {
		dev_err(dev, "LED count not set (led-count in DT or led_count param)\n");
		return -EINVAL;
	}

	ls->pwm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ls->pwm))
		return PTR_ERR(ls->pwm);

	ret = led_strip_fifo_bus_addr(ls);
	if (ret)
		return dev_err_probe(dev, ret, "no usable reg in device tree\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = led_strip_setup_clock(ls);
	if (ret)
		return ret;

	ret = led_strip_setup_dma(ls);
	if (ret)
		goto err_clk;

	ret = led_strip_alloc_buffers(ls);
	if (ret)
		goto err_dma;

	ret = led_strip_register_cdev(ls);
	if (ret)
		goto err_dma;

	/* Strips power up with random colours; blank them once. */
	mutex_lock(&ls->lock);
	led_strip_send(ls);
	mutex_unlock(&ls->lock);

	dev_info(dev, "ready on /dev/%s\n", DEVICE_NAME);
	return 0;

err_dma:
	dma_release_channel(ls->dma);
err_clk:
	clk_disable_unprepare(ls->clk);
	return ret;
}

static int led_strip_remove(struct platform_device *pdev)
{
	struct led_strip *ls = platform_get_drvdata(pdev);

	led_strip_unregister_cdev(ls);

	mutex_lock(&ls->lock);
	memset(ls->rgb, 0, ls->rgb_len);
	led_strip_send(ls);
	mutex_unlock(&ls->lock);

	led_strip_pwm_stop(ls);
	dma_release_channel(ls->dma);
	clk_disable_unprepare(ls->clk);

	return 0;
}

static const struct of_device_id led_strip_of_match[] = {
	{ .compatible = "roadpainter,led-strip" },
	{ }
};
MODULE_DEVICE_TABLE(of, led_strip_of_match);

static struct platform_driver led_strip_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table	= led_strip_of_match,
	},
	.probe	= led_strip_probe,
	.remove	= led_strip_remove,
};

module_platform_driver(led_strip_driver);

MODULE_DESCRIPTION("WS2812B LED strip driver using BCM2711 PWM serialiser + DMA");
MODULE_LICENSE("GPL");
