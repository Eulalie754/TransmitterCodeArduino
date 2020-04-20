void sendSensorData() {
  
  // Send 3x eulers over bluetooth as 1x byte array
  float data[3];
  data[0] = angle_yaw;
  data[1] = angle_pitch;
  data[2] = angle_roll;
  imuCharacteristic.setValue((byte *) &data, 12);

}
