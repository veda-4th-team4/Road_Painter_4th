/*
 * Raw I2S audio output driver for a MAX98357A amp on the Raspberry Pi 4
 * (BCM2711), bypassing ALSA/ASoC entirely.
 *
 * This exists as a learning exercise mirroring led_strip_driver.c: same
 * shape (platform driver bound via device tree, PCM peripheral programmed
 * directly, DMA feeds the FIFO, a character device is how userspace talks
 * to it), applied to the PCM/I2S block instead of the PWM block.
 *
 * Unlike WS2812B, I2S is also supported by the standard ASoC/ALSA stack.
 * This dedicated character driver is the Road Painter integration contract;
 * the prior hifiberry-dac setup remains the documented rollback path.
 *
 * Key structural difference from the LED driver: audio is a stream. Each
 * userspace write queues one DMA period; DRAIN waits for the queue and FIFO
 * to empty, while DROP immediately cancels it. This keeps timing in DMA
 * without allowing a cyclic transfer to replay stale periods after EOF.
 *
 * STATUS: functionally complete but UNTESTED ON HARDWARE. Module load/unload,
 * the character device, clock/DMA acquisition, PCM register programming, and
 * queued DMA are wired up and build cleanly against this Pi's kernel headers.
 * No overlay switch or speaker-output test has been performed yet.
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/cdev.h>
#include <linux/clk.h>
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
#include <linux/wait.h>

#include "../include/AudioStripDevice.h"
#include "audio_strip_regs.h"

#define DRIVER_NAME	"audio_strip"
#define DEVICE_NAME	"audio_strip"
#define CLASS_NAME	"audio_strip"

static const unsigned int sample_rate = AUDIO_SAMPLE_RATE_DEFAULT;

/*
 * Ring buffer sizing: userspace must write() in exact period_bytes chunks
 * (simpler than reassembling arbitrary-sized writes into periods - see
 * audio_strip_write()). periods * period_bytes of total buffering is
 * roughly periods * period_bytes / (sample_rate * 4) seconds at 16-bit
 * stereo; the defaults below are ~186ms across 8 periods, which is a lot
 * of slack for a userspace writer but costs little memory.
 */
static const unsigned int period_bytes = AUDIO_STRIP_PERIOD_BYTES;

static unsigned int periods = 8;
module_param(periods, uint, 0444);
MODULE_PARM_DESC(periods, "number of periods in the ring buffer (1-64)");

struct audio_strip;

struct audio_period_ctx {
	struct audio_strip	*as;
	u64			generation;
};

struct audio_strip {
	struct device		*dev;

	void __iomem		*pcm;
	struct clk		*clk;		/* PCM/I2S bit clock (BCLK) */
	unsigned long		bclk_hz;

	struct dma_chan		*dma;
	phys_addr_t		fifo_bus;	/* bus address of PCM_FIFO_A */

	/* DMA-coherent period slots. A slot is reused only after its completion
	 * callback advances completed, so queued audio is never overwritten. */
	void			*ring;
	dma_addr_t		ring_dma;
	size_t			ring_bytes;
	size_t			period_bytes;
	unsigned int		n_periods;

	atomic64_t		submitted;
	atomic64_t		completed;
	atomic64_t		generation;	/* invalidates stale callbacks on DROP */
	struct audio_period_ctx *period_ctx;
	wait_queue_head_t	wait;

	bool			playing;
	struct mutex		lock;
	atomic_t		opened;

	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
};

/* ------------------------------------------------------------------ */
/* PCM peripheral + DMA                                                */
/* ------------------------------------------------------------------ */

/*
 * Program the PCM peripheral for 16-bit stereo I2S and start the bit
 * clock + frame sync running. Called once, when the first write() opens
 * playback (see audio_strip_write()) - unlike led_strip_send(), which
 * reprograms and re-triggers the PWM block per frame, this configures
 * once and then leaves the clocks free-running for as long as playback
 * continues, since audio has no "next frame" boundary to restart on.
 *
 * CLKI/FSI and packed-channel placement follow the Raspberry Pi
 * bcm2835-i2s implementation for standard I2S. They still require a scope
 * and speaker check on the target hardware.
 */
static int audio_strip_pcm_configure(struct audio_strip *as)
{
	u32 mode, txc;

	/* Enable the block, leave standby, and clear the TX FIFO. The hardware
	 * requires several PCM clocks between these state changes. */
	writel(PCM_CS_EN, as->pcm + PCM_CS_A);
	writel(PCM_CS_EN | PCM_CS_STBY, as->pcm + PCM_CS_A);
	udelay(10);
	writel(PCM_CS_EN | PCM_CS_STBY | PCM_CS_TXCLR, as->pcm + PCM_CS_A);
	udelay(2);

	/*
	 * Frame = 64 BCLK cycles (two 32-bit slots). CLKM/FSM = 0 means the
	 * Pi drives both BCLK and LRCLK (MAX98357A has no clock output mode
	 * of its own, so the Pi must be master). FSLEN = half the frame
	 * gives LRCLK a 50% duty square wave, which is what I2S expects
	 * (low = left slot, high = right slot).
	 */
	mode = PCM_MODE_FLEN(AUDIO_FRAME_BCLKS - 1) |
	       PCM_MODE_FSLEN(AUDIO_FRAME_BCLKS / 2) |
	       PCM_MODE_FTXP |
	       PCM_MODE_CLKI |
	       PCM_MODE_FSI;
	writel(mode, as->pcm + PCM_MODE_A);

	/*
	 * Two 16-bit channels packed into the frame's two 32-bit slots.
	 * CHnWID is "width - 8" per the datasheet (16-bit -> 8). CH1 (left)
	 * starts AUDIO_I2S_DELAY_BCLKS after the FS edge - standard I2S
	 * delays the MSB by one BCLK relative to the FS transition, unlike
	 * left-justified formats which would use CH1POS = 0. CH2 (right)
	 * follows exactly one slot later.
	 */
	txc = PCM_TXC_CH1EN |
	      PCM_TXC_CH1WID(AUDIO_SAMPLE_BITS - 8) |
	      PCM_TXC_CH1POS(AUDIO_I2S_DELAY_BCLKS) |
	      PCM_TXC_CH2EN |
	      PCM_TXC_CH2WID(AUDIO_SAMPLE_BITS - 8) |
	      PCM_TXC_CH2POS(AUDIO_I2S_DELAY_BCLKS + AUDIO_SLOT_BITS);
	writel(txc, as->pcm + PCM_TXC_A);

	/* DMA is asked for more data once the FIFO empties below this level */
	writel(PCM_DREQ_TX(PCM_DREQ_TX_LVL) | PCM_DREQ_TX_PANIC(PCM_DREQ_PANIC_LVL),
	       as->pcm + PCM_DREQ_A);

	/*
	 * Start the peripheral: bit clock and frame sync begin running
	 * immediately, and DMAEN lets the already-submitted first descriptor
	 * feed real audio into the FIFO. TXERR is write-1-to-clear; include it
	 * here so clearing stale status does not momentarily clear EN/STBY.
	 */
	writel(PCM_CS_EN | PCM_CS_STBY | PCM_CS_TXON | PCM_CS_DMAEN |
	       PCM_CS_TXTHR(1) | PCM_CS_TXERR, as->pcm + PCM_CS_A);

	dev_info(as->dev, "PCM configured: %u Hz, %d-bit, %d ch, frame=%d BCLKs\n",
		 sample_rate, AUDIO_SAMPLE_BITS, AUDIO_CHANNELS, AUDIO_FRAME_BCLKS);

	return 0;
}

static void audio_strip_pcm_stop(struct audio_strip *as)
{
	if (as->pcm)
		writel(PCM_CS_EN | PCM_CS_STBY, as->pcm + PCM_CS_A);
}

static int audio_strip_setup_dma(struct audio_strip *as)
{
	struct dma_slave_config cfg = {
		.direction	 = DMA_MEM_TO_DEV,
		.dst_addr	 = as->fifo_bus,
		.dst_addr_width	 = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_maxburst	 = 2,
		.src_addr_width	 = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst	 = 2,
	};
	int ret;

	as->dma = dma_request_chan(as->dev, "tx");
	if (IS_ERR(as->dma))
		return dev_err_probe(as->dev, PTR_ERR(as->dma),
				     "failed to get DMA channel\n");

	ret = dmaengine_slave_config(as->dma, &cfg);
	if (ret) {
		dev_err(as->dev, "failed to configure DMA channel\n");
		dma_release_channel(as->dma);
		return ret;
	}

	return 0;
}

/* Mirrors led_strip_alloc_buffers(): one DMA-coherent allocation, sized
 * and logged once at probe time rather than per playback. */
static int audio_strip_alloc_ring(struct audio_strip *as)
{
	unsigned int i;

	if (sample_rate != AUDIO_SAMPLE_RATE_DEFAULT ||
	    period_bytes != AUDIO_STRIP_PERIOD_BYTES || !periods || periods > 64) {
		dev_err(as->dev,
			"unsupported stream contract: rate=%u period=%u periods=%u\n",
			sample_rate, period_bytes, periods);
		return -EINVAL;
	}
	if (period_bytes % (AUDIO_CHANNELS * (AUDIO_SAMPLE_BITS / 8)))
		return -EINVAL;

	as->period_bytes = period_bytes;
	as->n_periods = periods;
	as->ring_bytes = (size_t)as->period_bytes * as->n_periods;

	as->ring = dmam_alloc_coherent(as->dev, as->ring_bytes, &as->ring_dma, GFP_KERNEL);
	if (!as->ring)
		return -ENOMEM;

	as->period_ctx = devm_kcalloc(as->dev, as->n_periods,
					 sizeof(*as->period_ctx), GFP_KERNEL);
	if (!as->period_ctx)
		return -ENOMEM;
	for (i = 0; i < as->n_periods; i++)
		as->period_ctx[i].as = as;

	dev_info(as->dev, "ring buffer: %u periods x %zu bytes = %zu bytes (~%u ms @ %u Hz)\n",
		 as->n_periods, as->period_bytes, as->ring_bytes,
		 (unsigned int)((u64)as->ring_bytes * 1000 /
				(sample_rate * AUDIO_CHANNELS * (AUDIO_SAMPLE_BITS / 8))),
		 sample_rate);

	return 0;
}

/*
 * Fires once per period consumed. All this does is advance completed and
 * wake anyone in write() waiting for room - the actual register/FIFO work
 * is the DMA engine's job once the transfer is submitted below.
 */
static void audio_strip_dma_callback(void *param)
{
	struct audio_period_ctx *ctx = param;
	struct audio_strip *as = ctx->as;

	if (ctx->generation != atomic64_read(&as->generation))
		return;

	atomic64_inc(&as->completed);
	wake_up_interruptible(&as->wait);
}

static int audio_strip_queue_period(struct audio_strip *as, unsigned int slot,
				     u64 generation)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	desc = dmaengine_prep_slave_single(as->dma,
					   as->ring_dma + (size_t)slot * as->period_bytes,
					   as->period_bytes, DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(as->dev, "failed to prepare DMA descriptor\n");
		return -EIO;
	}

	as->period_ctx[slot].generation = generation;
	desc->callback = audio_strip_dma_callback;
	desc->callback_param = &as->period_ctx[slot];

	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		dev_err(as->dev, "DMA submit failed\n");
		return -EIO;
	}

	/* submitted must be visible before DMA can invoke the callback. */
	atomic64_inc(&as->submitted);
	dma_async_issue_pending(as->dma);
	return 0;
}

/* Caller holds as->lock. */
static void audio_strip_drop_locked(struct audio_strip *as)
{
	atomic64_inc(&as->generation);
	dmaengine_terminate_sync(as->dma);
	audio_strip_pcm_stop(as);
	as->playing = false;
	atomic64_set(&as->submitted, 0);
	atomic64_set(&as->completed, 0);
	memset(as->ring, 0, as->ring_bytes);
	wake_up_interruptible(&as->wait);
}

/* Caller holds as->lock and has already queued the first period. */
static int audio_strip_start(struct audio_strip *as)
{
	int ret;

	ret = audio_strip_pcm_configure(as);
	if (ret) {
		audio_strip_drop_locked(as);
		return ret;
	}

	as->playing = true;
	return 0;
}

static int audio_strip_setup_clock(struct audio_strip *as)
{
	unsigned long want_hz = (unsigned long)sample_rate * AUDIO_FRAME_BCLKS;
	int ret;

	as->clk = devm_clk_get(as->dev, NULL);
	if (IS_ERR(as->clk))
		return dev_err_probe(as->dev, PTR_ERR(as->clk),
				     "failed to get PCM bit clock\n");

	ret = clk_set_rate(as->clk, want_hz);
	if (ret)
		return dev_err_probe(as->dev, ret, "failed to set PCM clock\n");

	ret = clk_prepare_enable(as->clk);
	if (ret)
		return dev_err_probe(as->dev, ret, "failed to enable PCM clock\n");

	as->bclk_hz = clk_get_rate(as->clk);
	if (!as->bclk_hz) {
		clk_disable_unprepare(as->clk);
		return -EINVAL;
	}

	dev_info(as->dev, "PCM bit clock %lu Hz (wanted %lu Hz for %u Hz sample rate)\n",
		 as->bclk_hz, want_hz, sample_rate);

	return 0;
}

/* ------------------------------------------------------------------ */
/* character device                                                    */
/* ------------------------------------------------------------------ */

static int audio_strip_open(struct inode *inode, struct file *file)
{
	struct audio_strip *as = container_of(inode->i_cdev, struct audio_strip, cdev);

	if (atomic_cmpxchg(&as->opened, 0, 1) != 0)
		return -EBUSY;

	file->private_data = as;
	return 0;
}

/*
 * Feeds raw 16-bit LE stereo PCM into the ring buffer, one whole period
 * per call - userspace (e.g. AudioStripManager) is responsible for
 * stripping the WAV header before writing and for chunking its writes to
 * exactly period_bytes. Blocks (interruptibly) whenever the ring is full,
 * i.e. write() has gotten a full n_periods ahead of what DMA has actually
	 * completed, until the DMA callback frees a period.
 */
static ssize_t audio_strip_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct audio_strip *as = file->private_data;
	void *dst;
	u64 generation;
	unsigned int slot;
	int ret;

	if (count != as->period_bytes)
		return -EINVAL;

	if (mutex_lock_interruptible(&as->lock))
		return -ERESTARTSYS;
	generation = atomic64_read(&as->generation);

	/*
	 * wait_event_interruptible() must not be called with a mutex held
	 * (it sleeps), so drop the lock while waiting and re-check the
	 * condition once reacquired - standard mutex+waitqueue idiom, and
	 * necessary in general even with a single writer, since the
	 * condition could change again between wake-up and reacquiring the
	 * lock.
	 */
	while (atomic64_read(&as->submitted) - atomic64_read(&as->completed) >=
	       as->n_periods) {
		mutex_unlock(&as->lock);
		ret = wait_event_interruptible(as->wait,
				atomic64_read(&as->generation) != generation ||
				atomic64_read(&as->submitted) -
				atomic64_read(&as->completed) < as->n_periods);
		if (ret)
			return ret;
		if (mutex_lock_interruptible(&as->lock))
			return -ERESTARTSYS;
		if (atomic64_read(&as->generation) != generation) {
			mutex_unlock(&as->lock);
			return -ECANCELED;
		}
	}

	slot = atomic64_read(&as->submitted) % as->n_periods;
	dst = (u8 *)as->ring +
	      (size_t)slot * as->period_bytes;
	if (copy_from_user(dst, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	ret = audio_strip_queue_period(as, slot, generation);
	if (ret)
		goto out;

	if (!as->playing) {
		ret = audio_strip_start(as);
		if (ret)
			goto out;
	}

	ret = count;
out:
	mutex_unlock(&as->lock);
	return ret;
}

static long audio_strip_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct audio_strip *as = file->private_data;
	u64 generation;
	u32 status = 0;
	int ret = 0;

	(void)arg;
	if (_IOC_TYPE(cmd) != AUDIO_STRIP_IOC_MAGIC)
		return -ENOTTY;

	if (cmd == AUDIO_STRIP_IOC_DROP) {
		mutex_lock(&as->lock);
		audio_strip_drop_locked(as);
		mutex_unlock(&as->lock);
		return 0;
	}
	if (cmd != AUDIO_STRIP_IOC_DRAIN)
		return -ENOTTY;

	generation = atomic64_read(&as->generation);
	ret = wait_event_interruptible(as->wait,
			atomic64_read(&as->generation) != generation ||
			atomic64_read(&as->completed) >=
			atomic64_read(&as->submitted));
	if (ret)
		return ret;
	if (atomic64_read(&as->generation) != generation)
		return -ECANCELED;

	mutex_lock(&as->lock);
	if (as->playing) {
		/* Completion means the last DMA word reached the PCM FIFO. Wait until
		 * it has also shifted onto the I2S wire before stopping the clocks. */
		ret = readl_poll_timeout(as->pcm + PCM_CS_A, status,
					 status & PCM_CS_TXE, 10, 100000);
		if (ret)
			dev_warn(as->dev, "FIFO did not drain (CS=0x%08x)\n", status);
	}
	audio_strip_drop_locked(as);
	mutex_unlock(&as->lock);
	return ret;
}

static int audio_strip_release(struct inode *inode, struct file *file)
{
	struct audio_strip *as = file->private_data;

	mutex_lock(&as->lock);
	audio_strip_drop_locked(as);
	writel(0, as->pcm + PCM_CS_A);
	mutex_unlock(&as->lock);
	atomic_set(&as->opened, 0);

	return 0;
}

static const struct file_operations audio_strip_fops = {
	.owner		= THIS_MODULE,
	.open		= audio_strip_open,
	.write		= audio_strip_write,
	.unlocked_ioctl	= audio_strip_ioctl,
	.release	= audio_strip_release,
	.llseek		= noop_llseek,
};

static int audio_strip_register_cdev(struct audio_strip *as)
{
	int ret;

	ret = alloc_chrdev_region(&as->devt, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&as->cdev, &audio_strip_fops);
	as->cdev.owner = THIS_MODULE;

	ret = cdev_add(&as->cdev, as->devt, 1);
	if (ret)
		goto err_region;

	as->class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(as->class)) {
		ret = PTR_ERR(as->class);
		goto err_cdev;
	}

	if (IS_ERR(device_create(as->class, as->dev, as->devt, as, DEVICE_NAME))) {
		ret = -ENODEV;
		goto err_class;
	}

	return 0;

err_class:
	class_destroy(as->class);
err_cdev:
	cdev_del(&as->cdev);
err_region:
	unregister_chrdev_region(as->devt, 1);
	return ret;
}

static void audio_strip_unregister_cdev(struct audio_strip *as)
{
	device_destroy(as->class, as->devt);
	class_destroy(as->class);
	cdev_del(&as->cdev);
	unregister_chrdev_region(as->devt, 1);
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

/*
 * Same reasoning as led_strip_fifo_bus_addr(): the DMA controller wants
 * the 0x7e000000 bus address, not the ARM physical address, so this comes
 * straight from the device tree "reg" property instead of the ioremap'd
 * resource.
 */
static int audio_strip_fifo_bus_addr(struct audio_strip *as)
{
	struct device_node *np = as->dev->of_node;
	const __be32 *reg;
	u64 size;

	reg = of_get_address(np, 0, &size, NULL);
	if (!reg)
		return -EINVAL;

	as->fifo_bus = be32_to_cpup(reg) + PCM_FIFO_A;
	return 0;
}

static int audio_strip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct audio_strip *as;
	int ret;

	as = devm_kzalloc(dev, sizeof(*as), GFP_KERNEL);
	if (!as)
		return -ENOMEM;

	as->dev = dev;
	mutex_init(&as->lock);
	init_waitqueue_head(&as->wait);
	atomic64_set(&as->submitted, 0);
	atomic64_set(&as->completed, 0);
	atomic64_set(&as->generation, 1);
	atomic_set(&as->opened, 0);
	platform_set_drvdata(pdev, as);

	as->pcm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(as->pcm))
		return PTR_ERR(as->pcm);

	ret = audio_strip_fifo_bus_addr(as);
	if (ret)
		return dev_err_probe(dev, ret, "no usable reg in device tree\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = audio_strip_setup_clock(as);
	if (ret)
		return ret;

	ret = audio_strip_setup_dma(as);
	if (ret)
		goto err_clk;

	ret = audio_strip_alloc_ring(as);
	if (ret)
		goto err_dma;

	ret = audio_strip_register_cdev(as);
	if (ret)
		goto err_dma;

	dev_info(dev, "ready on /dev/%s\n", DEVICE_NAME);
	return 0;

err_dma:
	dma_release_channel(as->dma);
err_clk:
	clk_disable_unprepare(as->clk);
	return ret;
}

static int audio_strip_remove(struct platform_device *pdev)
{
	struct audio_strip *as = platform_get_drvdata(pdev);

	audio_strip_unregister_cdev(as);

	mutex_lock(&as->lock);
	audio_strip_drop_locked(as);
	writel(0, as->pcm + PCM_CS_A);
	mutex_unlock(&as->lock);

	dma_release_channel(as->dma);
	clk_disable_unprepare(as->clk);

	return 0;
}

static const struct of_device_id audio_strip_of_match[] = {
	{ .compatible = "roadpainter,audio-strip" },
	{ }
};
MODULE_DEVICE_TABLE(of, audio_strip_of_match);

static struct platform_driver audio_strip_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table	= audio_strip_of_match,
	},
	.probe	= audio_strip_probe,
	.remove	= audio_strip_remove,
};

module_platform_driver(audio_strip_driver);

MODULE_DESCRIPTION("Raw I2S audio driver for MAX98357A using BCM2711 PCM peripheral + queued DMA");
MODULE_LICENSE("GPL");
