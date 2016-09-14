#ifndef MPU9150_SIMPLE_H
#define MPU9150_SIMPLE_H

struct s_mpu9150 {
	float ypr[3];
	float gyro[3];
};

extern struct s_mpu9150 mpu9150;

int mpu9150_open(unsigned int rate);
int mpu9150_update();

#endif
