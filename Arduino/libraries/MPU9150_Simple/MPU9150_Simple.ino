#include <Wire.h>
#include "I2Cdev.h"
#include "mpu9150_simple.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

int ret;
void setup() {
  //  Fastwire::setup(1000,0);
  Wire.begin();
  Serial.begin(115200);
  ret = mpu9150_open(200);
  Serial.print("MPU init: "); Serial.println(ret);
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
  ret = mpu9150_update();

  switch (ret) {
    case 0: c++; break;
    case 1: np++; return;
    case 2: err_o++; return;
    case 3: err_c++; return;
    default:
      Serial.print("READ ERROR!  ");
      Serial.println(ret);
      return;
  }

  if (!(c % 25)) {
    Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
    Serial.print(" Y: "); Serial.print(mpu9150.ypr[0]);
    Serial.print(" P: "); Serial.print(mpu9150.ypr[1]);
    Serial.print(" R: "); Serial.print(mpu9150.ypr[2]);
    Serial.print("\tgy: "); Serial.print(mpu9150.gyro[0]);
    Serial.print(" gp: "); Serial.print(mpu9150.gyro[1]);
    Serial.print(" gr: "); Serial.println(mpu9150.gyro[2]);
  }
}
