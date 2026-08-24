#ifndef __AUDIO_STRIP_DEVICE_H__
#define __AUDIO_STRIP_DEVICE_H__

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

/* Shared userspace/kernel contract for /dev/audio_strip. */
#define AUDIO_STRIP_PERIOD_BYTES 4096U
#define AUDIO_STRIP_IOC_MAGIC    'A'

/* Finish queued PCM, stop the I2S clocks, and reset for the next sound. */
#define AUDIO_STRIP_IOC_DRAIN _IO(AUDIO_STRIP_IOC_MAGIC, 0)

/* Immediately discard queued PCM. Safety announcements use this to preempt. */
#define AUDIO_STRIP_IOC_DROP  _IO(AUDIO_STRIP_IOC_MAGIC, 1)

#endif /* __AUDIO_STRIP_DEVICE_H__ */
