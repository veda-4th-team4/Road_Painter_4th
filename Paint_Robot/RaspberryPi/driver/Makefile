obj-m += mpu6050_driver.o
obj-m += led_strip_driver.o
obj-m += audio_strip_driver.o

.PHONY: all audio clean

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) modules
	dtc -@ -I dts -O dtb -o mpu6050_overlay.dtbo mpu6050_overlay.dts
	dtc -@ -I dts -O dtb -o led_strip_overlay.dtbo led_strip_overlay.dts
	dtc -@ -I dts -O dtb -o audio_strip_overlay.dtbo audio_strip_overlay.dts

audio:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) audio_strip_driver.ko
	dtc -@ -I dts -O dtb -o audio_strip_overlay.dtbo audio_strip_overlay.dts

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) clean
	rm -f mpu6050_overlay.dtbo
	rm -f led_strip_overlay.dtbo
	rm -f audio_strip_overlay.dtbo
