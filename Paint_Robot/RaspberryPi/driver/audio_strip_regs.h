/*
 * BCM2711 PCM/I2S peripheral register map and I2S framing constants,
 * for driving a MAX98357A I2S Class-D amplifier directly (no ALSA/ASoC).
 *
 * The project uses a dedicated character driver so the audio path follows the
 * same kernel/userspace ownership split as led_strip_driver.c. Standard ALSA
 * and hifiberry-dac remain the rollback path documented in AUDIO_STRIP_KR.md.
 *
 * Register field layout below follows the BCM2835 ARM Peripherals
 * datasheet, chapter 8 ("PCM / I2S Audio"), and Raspberry Pi's
 * sound/soc/bcm/bcm2835-i2s.c implementation. Actual signal timing still
 * needs to be checked during hardware bring-up.
 */

#ifndef _AUDIO_STRIP_REGS_H
#define _AUDIO_STRIP_REGS_H

/* ---- PCM register offsets (from the PCM peripheral base) ---- */
#define PCM_CS_A		0x00	/* Control and Status */
#define PCM_FIFO_A		0x04	/* FIFO Data */
#define PCM_MODE_A		0x08	/* Mode */
#define PCM_RXC_A		0x0C	/* Receive Configuration (unused, TX only) */
#define PCM_TXC_A		0x10	/* Transmit Configuration */
#define PCM_DREQ_A		0x14	/* DMA Request Level */
#define PCM_INTEN_A		0x18	/* Interrupt Enables */
#define PCM_INTSTC_A		0x1C	/* Interrupt Status & Clear */
#define PCM_GRAY		0x20	/* Gray Mode Control (unused) */

/* ---- PCM_CS_A bits ---- */
#define PCM_CS_EN		BIT(0)	/* peripheral enable */
#define PCM_CS_RXON		BIT(1)	/* receive enable (unused, TX only) */
#define PCM_CS_TXON		BIT(2)	/* transmit enable */
#define PCM_CS_TXCLR		BIT(3)	/* clear TX FIFO */
#define PCM_CS_RXCLR		BIT(4)	/* clear RX FIFO */
#define PCM_CS_TXTHR(x)		(((x) & 0x3) << 5)	/* TX FIFO threshold select */
#define PCM_CS_RXTHR(x)		(((x) & 0x3) << 7)	/* RX FIFO threshold select */
#define PCM_CS_DMAEN		BIT(9)	/* enable DMA DREQ generation */
#define PCM_CS_TXSYNC		BIT(13)	/* TX FIFO sync */
#define PCM_CS_TXERR		BIT(15)	/* TX FIFO underrun occurred */
#define PCM_CS_TXW		BIT(17)	/* TX FIFO needs writing (below threshold) */
#define PCM_CS_TXD		BIT(19)	/* TX FIFO has room for at least one sample */
#define PCM_CS_TXE		BIT(21)	/* TX FIFO empty */
#define PCM_CS_SYNC		BIT(24)	/* clock-domain synchronization flag */
#define PCM_CS_STBY		BIT(25)	/* disable standby while set */

/* ---- PCM_MODE_A bits ---- */
#define PCM_MODE_FSLEN(x)	(((x) & 0x3FF) << 0)	/* frame sync (LRCLK) pulse width, PCM clocks */
#define PCM_MODE_FLEN(x)	(((x) & 0x3FF) << 10)	/* frame length - 1, PCM clocks (= BCLK-per-LRCLK - 1) */
#define PCM_MODE_FSI		BIT(20)	/* frame sync invert */
#define PCM_MODE_FSM		BIT(21)	/* 0 = FS is output (master), 1 = input (slave) */
#define PCM_MODE_CLKI		BIT(22)	/* bit clock invert */
#define PCM_MODE_CLKM		BIT(23)	/* 0 = BCLK is output (master), 1 = input (slave) */
#define PCM_MODE_FTXP		BIT(24)	/* pack two <=16-bit TX channels per FIFO word */
#define PCM_MODE_FRXP		BIT(25)	/* pack two <=16-bit RX channels per FIFO word */
#define PCM_MODE_PDME		BIT(29)	/* PDM enable (unused) */

/* ---- PCM_TXC_A bits ---- */
#define PCM_TXC_CH2WID(x)	(((x) & 0xF) << 0)
#define PCM_TXC_CH2POS(x)	(((x) & 0x3FF) << 4)
#define PCM_TXC_CH2EN		BIT(14)
#define PCM_TXC_CH2WEX		BIT(15)
#define PCM_TXC_CH1WID(x)	(((x) & 0xF) << 16)
#define PCM_TXC_CH1POS(x)	(((x) & 0x3FF) << 20)
#define PCM_TXC_CH1EN		BIT(30)
#define PCM_TXC_CH1WEX		BIT(31)

/* ---- PCM_DREQ_A bits ---- */
#define PCM_DREQ_RX(x)		(((x) & 0x7F) << 0)
#define PCM_DREQ_TX(x)		(((x) & 0x7F) << 8)
#define PCM_DREQ_RX_PANIC(x)	(((x) & 0x7F) << 16)
#define PCM_DREQ_TX_PANIC(x)	(((x) & 0x7F) << 24)

/* FIFO is 64 words (256 bytes) deep, shared between TX and RX halves in HW terms */
#define PCM_FIFO_DEPTH_WORDS	64
#define PCM_DREQ_TX_LVL		0x30	/* Raspberry Pi bcm2835-i2s TX threshold */
#define PCM_DREQ_PANIC_LVL	0x10

/* DMA request line the PCM/I2S TX half drives (BCM2835 TRM, table "DREQ peripherals") */
#define PCM_DREQ_LINE		2

/*
 * ---- I2S framing for MAX98357A (16-bit stereo, standard I2S, master mode) ----
 *
 * MAX98357A has no control bus: it infers word length purely from the BCLK
 * to LRCLK ratio (32*Fs = 16-bit, 48*Fs = 24-bit, 64*Fs = 32-bit slots).
 * We drive it in the simplest mode: 16-bit samples in 32-bit slots (BCLK =
 * 64*Fs), standard I2S justification (MSB one BCLK after the LRCLK edge,
 * which is PCM_TXC_CH1POS/CH2POS = 1 below, not 0).
 */
#define AUDIO_SAMPLE_BITS	16
#define AUDIO_CHANNELS		2
#define AUDIO_SLOT_BITS		32		/* BCLK cycles per channel slot */
#define AUDIO_FRAME_BCLKS	(AUDIO_SLOT_BITS * AUDIO_CHANNELS)	/* BCLK cycles per LRCLK period = 64 */
#define AUDIO_I2S_DELAY_BCLKS	1		/* standard I2S: MSB starts 1 BCLK after the FS edge */

#define AUDIO_SAMPLE_RATE_DEFAULT	44100	/* matches play.sh's wav_files/ (see README) */

#endif /* _AUDIO_STRIP_REGS_H */
