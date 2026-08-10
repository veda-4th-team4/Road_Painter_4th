obj-m += mpu6050_driver.o
obj-m += led_strip_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	dtc -@ -I dts -O dtb -o mpu6050_overlay.dtbo mpu6050_overlay.dts
	dtc -@ -I dts -O dtb -o led_strip_overlay.dtbo led_strip_overlay.dts

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f mpu6050_overlay.dtbo
	rm -f led_strip_overlay.dtbo
