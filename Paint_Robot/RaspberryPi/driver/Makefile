obj-m += mpu6050_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	dtc -@ -I dts -O dtb -o mpu6050_overlay.dtbo mpu6050_overlay.dts

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f mpu6050_overlay.dtbo
