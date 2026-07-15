#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define REG_PWR_MGMT_1       0x6B
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_GYRO_XOUT_H      0x43
#define REG_GYRO_YOUT_H      0x45
#define REG_GYRO_ZOUT_H      0x47

// Global tracking variables (Fixed-point representation in millidegrees, i.e., 1/1000 deg)
static s32 current_yaw_mdeg = 0;
static s32 prev_gyro_z_mdeg_s = 0;
static s32 gyro_x_offset = 0;
static s32 gyro_y_offset = 0;
static s32 gyro_z_offset = 0;

static dev_t dev_num;
static struct cdev my_cdev;
static struct class *my_class;
static struct device *my_device;
static struct i2c_client *mpu_client = NULL;
static struct task_struct *imu_thread = NULL;

static DEFINE_MUTEX(yaw_mutex);

// Read 16-bit data from IMU registers via SMBus
static s16 read_raw_data(struct i2c_client *client, int addr, bool *error) {
    int high = i2c_smbus_read_byte_data(client, addr);
    int low = i2c_smbus_read_byte_data(client, addr + 1);
    if (high < 0 || low < 0) {
        *error = true;
        return 0;
    }
    *error = false;
    return (s16)((high << 8) | low);
}

// Sensor configuration on startup
static int mpu6050_init(struct i2c_client *client) {
    int ret;
    // Wake up MPU-6050
    ret = i2c_smbus_write_byte_data(client, REG_PWR_MGMT_1, 0x00);
    if (ret < 0) return ret;
    msleep(50);
    // GYRO_CONFIG: FS_SEL = 0 (±250 deg/s, 131.0 LSB/deg/s)
    ret = i2c_smbus_write_byte_data(client, REG_GYRO_CONFIG, 0x00);
    if (ret < 0) return ret;
    // CONFIG: DLPF ~42Hz
    ret = i2c_smbus_write_byte_data(client, REG_CONFIG, 0x03);
    return ret;
}

// 500-sample calibration logic
static void mpu6050_calibrate(struct i2c_client *client) {
    int i;
    long long sum_x = 0, sum_y = 0, sum_z = 0;
    int valid = 0;
    bool err = false;

    pr_info("mpu6050_driver: Calibrating sensors. Keep device still...\n");
    for (i = 0; i < 500; i++) {
        s16 rx = read_raw_data(client, REG_GYRO_XOUT_H, &err);
        s16 ry = read_raw_data(client, REG_GYRO_YOUT_H, &err);
        s16 rz = read_raw_data(client, REG_GYRO_ZOUT_H, &err);
        
        if (!err) {
            sum_x += rx;
            sum_y += ry;
            sum_z += rz;
            valid++;
        }
        usleep_range(4000, 5000);
    }
    if (valid > 0) {
        gyro_x_offset = (s32)div_s64(sum_x, valid);
        gyro_y_offset = (s32)div_s64(sum_y, valid);
        gyro_z_offset = (s32)div_s64(sum_z, valid);
    }
    pr_info("mpu6050_driver: Calibration complete. Samples: %d, Offset Z: %d\n", valid, gyro_z_offset);
}

// Kernel thread running the 50Hz integration and filtering loop
static int imu_kthread_fn(void *data) {
    struct i2c_client *client = (struct i2c_client *)data;
    ktime_t last_time = ktime_get();
    bool err = false;
    
    // Safety & Noise parameters in millidegree units
    const s32 GYRO_DEADBAND = 150;        // 0.15 deg/s * 1000
    const s32 MAX_PHYSICAL_RATE = 120000; // 120.0 deg/s * 1000
    const s32 MAX_SLEW_RATE = 80000;      // 80.0 deg/s * 1000
    const s32 IMU_ANOMALY_LIMIT = 200000; // 200.0 deg/s * 1000

    pr_info("mpu6050_driver: Background tracking kernel thread active.\n");
    
    while (!kthread_should_stop()) {
        ktime_t now = ktime_get();
        s64 elapsed_ns = ktime_to_ns(ktime_sub(now, last_time));
        last_time = now;

        short raw_gx = read_raw_data(client, REG_GYRO_XOUT_H, &err);
        short raw_gy = read_raw_data(client, REG_GYRO_YOUT_H, &err);
        short raw_gz = read_raw_data(client, REG_GYRO_ZOUT_H, &err);
        
        s32 gyro_x_mdeg_s = 0;
        s32 gyro_y_mdeg_s = 0;
        s32 gyro_z_mdeg_s = 0;

        if (err) {
            gyro_z_mdeg_s = prev_gyro_z_mdeg_s;
        } else {
            // Remove offset, scale to millidegrees (multiply by 1000 and divide by 131 Sensitivity)
            gyro_x_mdeg_s = ((s32)raw_gx - gyro_x_offset) * 1000 / 131;
            gyro_y_mdeg_s = ((s32)raw_gy - gyro_y_offset) * 1000 / 131;
            gyro_z_mdeg_s = ((s32)raw_gz - gyro_z_offset) * 1000 / 131;
            
            // 1) Roll/Pitch Anomaly cross-check
            if (abs(gyro_x_mdeg_s) > IMU_ANOMALY_LIMIT || abs(gyro_y_mdeg_s) > IMU_ANOMALY_LIMIT) {
                gyro_z_mdeg_s = prev_gyro_z_mdeg_s;
            } else {
                // 2) Slew rate filter (max rate change per 20ms)
                s32 diff = gyro_z_mdeg_s - prev_gyro_z_mdeg_s;
                if (diff > MAX_SLEW_RATE) gyro_z_mdeg_s = prev_gyro_z_mdeg_s + MAX_SLEW_RATE;
                else if (diff < -MAX_SLEW_RATE) gyro_z_mdeg_s = prev_gyro_z_mdeg_s - MAX_SLEW_RATE;

                // 3) Clamping turn rates
                if (gyro_z_mdeg_s > MAX_PHYSICAL_RATE) gyro_z_mdeg_s = MAX_PHYSICAL_RATE;
                else if (gyro_z_mdeg_s < -MAX_PHYSICAL_RATE) gyro_z_mdeg_s = -MAX_PHYSICAL_RATE;

                // 4) Deadband noise suppression
                if (abs(gyro_z_mdeg_s) < GYRO_DEADBAND) {
                    gyro_z_mdeg_s = 0;
                }
            }
        }
        prev_gyro_z_mdeg_s = gyro_z_mdeg_s;

        // Integrate millidegree heading (Yaw)
        // delta_yaw = speed (millideg/sec) * elapsed_ns / 1,000,000,000 ns
        {
            s64 delta_yaw_ns = (s64)gyro_z_mdeg_s * elapsed_ns;
            s32 delta_yaw_mdeg = (s32)div_s64(delta_yaw_ns, 1000000000LL);
            
            mutex_lock(&yaw_mutex);
            current_yaw_mdeg += delta_yaw_mdeg;
            mutex_unlock(&yaw_mutex);
        }

        // Sleep interval ~20ms (50Hz)
        usleep_range(19000, 21000);
    }
    return 0;
}

// User-space read handler: Copies s32 Yaw (4 bytes) to user buffer
static ssize_t imu_read(struct file *file, char __user *buf, size_t count, loff_t *offset) {
    s32 temp_yaw;
    if (count < sizeof(s32)) return -EINVAL;

    mutex_lock(&yaw_mutex);
    temp_yaw = current_yaw_mdeg;
    mutex_unlock(&yaw_mutex);

    if (copy_to_user(buf, &temp_yaw, sizeof(s32))) {
        return -EFAULT;
    }
    return sizeof(s32);
}

// User-space write handler: Recovers reference s32 (4 bytes) to reset Yaw
static ssize_t imu_write(struct file *file, const char __user *buf, size_t count, loff_t *offset) {
    s32 new_yaw;
    if (count < sizeof(s32)) return -EINVAL;

    if (copy_from_user(&new_yaw, buf, sizeof(s32))) {
        return -EFAULT;
    }

    mutex_lock(&yaw_mutex);
    current_yaw_mdeg = new_yaw;
    mutex_unlock(&yaw_mutex);

    pr_info("mpu6050_driver: External Yaw reset to %d mdeg\n", new_yaw);
    return sizeof(s32);
}

static int imu_open(struct inode *inode, struct file *file) {
    return 0;
}

static int imu_release(struct inode *inode, struct file *file) {
    return 0;
}

static struct file_operations imu_fops = {
    .owner = THIS_MODULE,
    .open = imu_open,
    .release = imu_release,
    .read = imu_read,
    .write = imu_write,
};

// --- Standard I2C Probe Lifecycle Interface ---
static int mpu6050_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    int ret;
    pr_info("mpu6050_driver: Device match probe triggered. Setting up hardware...\n");
    
    mpu_client = client;

    // 1. Allocate device numbers
    ret = alloc_chrdev_region(&dev_num, 0, 1, "imu_device");
    if (ret < 0) {
        pr_err("mpu6050_driver: Failed to allocate major number.\n");
        return ret;
    }

    // 2. Initialize and add character device
    cdev_init(&my_cdev, &imu_fops);
    my_cdev.owner = THIS_MODULE;
    ret = cdev_add(&my_cdev, dev_num, 1);
    if (ret < 0) {
        unregister_chrdev_region(dev_num, 1);
        pr_err("mpu6050_driver: Failed to add cdev.\n");
        return ret;
    }

    // 3. Create class and device nodes (/dev/imu)
    my_class = class_create(THIS_MODULE, "imu_class");
    if (IS_ERR(my_class)) {
        cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("mpu6050_driver: Failed to create class.\n");
        return PTR_ERR(my_class);
    }

    my_device = device_create(my_class, NULL, dev_num, NULL, "imu");
    if (IS_ERR(my_device)) {
        class_destroy(my_class);
        cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("mpu6050_driver: Failed to create /dev/imu node.\n");
        return PTR_ERR(my_device);
    }

    // 4. Config MPU-6050 register maps
    ret = mpu6050_init(mpu_client);
    if (ret < 0) {
        device_destroy(my_class, dev_num);
        class_destroy(my_class);
        cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("mpu6050_driver: Failed to initialize register configuration.\n");
        return ret;
    }

    // 5. Calibrate sensor Gyro offsets
    mpu6050_calibrate(mpu_client);

    // 6. Fire-up tracking kthread
    imu_thread = kthread_run(imu_kthread_fn, mpu_client, "imu_kthread");
    if (IS_ERR(imu_thread)) {
        device_destroy(my_class, dev_num);
        class_destroy(my_class);
        cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        pr_err("mpu6050_driver: Failed to start tracking kthread.\n");
        return PTR_ERR(imu_thread);
    }

    pr_info("mpu6050_driver: MPU-6050 bound and initialized successfully!\n");
    return 0;
}

// --- Standard I2C Remove Lifecycle Interface ---
static void mpu6050_remove(struct i2c_client *client) {
    pr_info("mpu6050_driver: Device removal triggered. Cleaning up...\n");

    if (imu_thread) {
        kthread_stop(imu_thread);
    }

    device_destroy(my_class, dev_num);
    class_destroy(my_class);
    cdev_del(&my_cdev);
    unregister_chrdev_region(dev_num, 1);

    mpu_client = NULL;
    pr_info("mpu6050_driver: Cleanup complete.\n");
}

// Device Tree Match Tables
static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "roadpainter,mpu6050", },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

// Traditional I2C Match Table
static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

// Custom I2C Driver Struct
static struct i2c_driver mpu6050_i2c_driver = {
    .driver = {
        .name = "mpu6050_custom_driver",
        .of_match_table = mpu6050_of_match,
    },
    .probe = mpu6050_probe,
    .remove = mpu6050_remove,
    .id_table = mpu6050_id,
};

// Module entrypoints register and deregister I2C driver structure
static int __init imu_driver_init(void) {
    pr_info("mpu6050_driver: Registering MPU-6050 I2C custom driver...\n");
    return i2c_add_driver(&mpu6050_i2c_driver);
}

static void __exit imu_driver_exit(void) {
    pr_info("mpu6050_driver: Deregistering MPU-6050 I2C custom driver...\n");
    i2c_del_driver(&mpu6050_i2c_driver);
}

module_init(imu_driver_init);
module_exit(imu_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Antigravity");
MODULE_DESCRIPTION("MPU-6050 DTS Character Device Driver for Yaw Estimation");
MODULE_VERSION("2.0");
