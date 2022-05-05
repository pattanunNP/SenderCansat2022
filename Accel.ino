
void readAccel(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {


  /// Update Value With Pointer;

  //GY86 Accelerometer
  // read raw accel/gyro measurements from device

  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro.getAcceleration(&ax, &ay, &az);
  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);





}
