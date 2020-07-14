#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
}

void loop() {
  mpu.update();
  float angle[3] = {mpu.getAngleX(),mpu.getAngleY(),mpu.getAngleZ()};
  float gyro[3] = {mpu.getGyroX(),mpu.getGyroY(),mpu.getGyroZ()};
  // process this data for your needs...
  Serial.print(angle[0]);
  Serial.print("\t");
  Serial.println(angle[1]);
}
