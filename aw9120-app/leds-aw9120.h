#ifndef _AW9120_H_
#define _AW9120_H_

#define MAX_I2C_BUFFER_SIZE 65536

#define AW9120_ID 0xb223

struct aw9120 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct work_struct brightness_work;

	int reset_gpio;

	unsigned int chipid;

	int imax;
};

#endif
