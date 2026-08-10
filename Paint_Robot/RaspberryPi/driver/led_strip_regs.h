/*
 * BCM2711 PWM register map and WS2812B encoding constants.
 *
 * The mainline pwm-bcm2835 driver only exposes "PWM mode" (repeat one
 * duty cycle). WS2812B needs "serialiser mode", which shifts an arbitrary
 * bit stream out of the FIFO. No kernel API reaches it, so the PWM block
 * is programmed directly here.
 */

#ifndef _LED_STRIP_REGS_H
#define _LED_STRIP_REGS_H

/* ---- PWM register offsets (from the PWM0 block base) ---- */
#define PWM_CTL			0x00
#define PWM_STA			0x04
#define PWM_DMAC		0x08
#define PWM_RNG1		0x10
#define PWM_DAT1		0x14
#define PWM_FIF1		0x18
#define PWM_RNG2		0x20
#define PWM_DAT2		0x24

/* ---- PWM_CTL bits (channel 1) ---- */
#define PWM_CTL_PWEN1		BIT(0)	/* channel enable */
#define PWM_CTL_MODE1		BIT(1)	/* 0 = PWM, 1 = serialiser */
#define PWM_CTL_RPTL1		BIT(2)	/* repeat last word when FIFO empties */
#define PWM_CTL_SBIT1		BIT(3)	/* output level when idle */
#define PWM_CTL_POLA1		BIT(4)	/* invert output */
#define PWM_CTL_USEF1		BIT(5)	/* take data from FIFO, not DAT1 */
#define PWM_CTL_CLRF1		BIT(6)	/* write 1 to flush FIFO */
#define PWM_CTL_MSEN1		BIT(7)	/* mark/space mode (PWM mode only) */

/* ---- PWM_STA bits ---- */
#define PWM_STA_FULL1		BIT(0)
#define PWM_STA_EMPT1		BIT(1)
#define PWM_STA_WERR1		BIT(2)	/* FIFO write while full */
#define PWM_STA_RERR1		BIT(3)	/* FIFO read while empty */
#define PWM_STA_GAPO1		BIT(4)	/* channel 1 ran out of data */
#define PWM_STA_BERR		BIT(8)	/* bus error */
#define PWM_STA_ERRS		(PWM_STA_WERR1 | PWM_STA_RERR1 | \
				 PWM_STA_GAPO1 | PWM_STA_BERR)

/* ---- PWM_DMAC bits ---- */
#define PWM_DMAC_ENAB		BIT(31)
#define PWM_DMAC_PANIC(x)	(((x) & 0xff) << 8)
#define PWM_DMAC_DREQ(x)	(((x) & 0xff) << 0)

/*
 * FIFO watermarks the PWM block reports to the DMA controller. PANIC raises
 * the DMA priority when the FIFO drops this low; DREQ is the ordinary
 * "send me more" threshold. Values follow the rpi_ws281x reference.
 */
#define PWM_DMAC_PANIC_LVL	7
#define PWM_DMAC_DREQ_LVL	3

/* DMA request line the PWM block drives (BCM2835 TRM, table "DREQ peripherals") */
#define PWM_DREQ		5

/* Serialiser shifts one 32-bit FIFO word out per PWM_RNG1 clock ticks */
#define PWM_WORD_BITS		32

/* ---- WS2812B protocol ---- */
/*
 * Each colour bit is 1.25us and is encoded as three PWM sub-bits, so the
 * PWM clock runs at 3 / 1.25us = 2.4MHz (416.7ns per sub-bit):
 *
 *   bit 1 -> 0b110 : 833ns high, 417ns low   (spec T1H 800+-150, T1L 450+-150)
 *   bit 0 -> 0b100 : 417ns high, 833ns low   (spec T0H 400+-150, T0L 850+-150)
 */
#define WS2812_SUBBITS		3
#define WS2812_SYMBOL_1		0b110
#define WS2812_SYMBOL_0		0b100

#define WS2812_PWM_HZ		2400000
#define WS2812_BITS_PER_LED	24		/* G8 R8 B8 */
#define WS2812_BYTES_PER_LED	3

/* Sub-bits emitted per LED */
#define WS2812_SUBBITS_PER_LED	(WS2812_BITS_PER_LED * WS2812_SUBBITS)

/*
 * After the last bit the line must stay low so the strip latches the frame.
 * WS2812B needs >50us; WS2813 and several clones want ~280us, so default to
 * 300us and let it be tuned at load time.
 */
#define WS2812_RESET_US_DEFAULT	300

#endif /* _LED_STRIP_REGS_H */
