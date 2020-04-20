///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_signal(void) {
    
   IMU.readAcceleration(acc_x, acc_y, acc_z);
  
   //acc_y -= manual_acc_pitch_cal_value;                         //Subtact the manual accelerometer pitch calibration value.
   //acc_x -= manual_acc_roll_cal_value;                          //Subtact the manual accelerometer roll calibration value.
   
   IMU.readGyroscope(gyro_roll, gyro_pitch, gyro_yaw);
   
   gyro_pitch *= -1;                                            //Invert the direction of the axis.
   gyro_yaw *= -1;                                              //Invert the direction of the axis.
                            
   gyro_roll -= manual_gyro_roll_cal_value;                     //Subtact the manual gyro roll calibration value.
   gyro_pitch -= manual_gyro_pitch_cal_value;                   //Subtact the manual gyro pitch calibration value.
   gyro_yaw -= manual_gyro_yaw_cal_value;                       //Subtact the manual gyro yaw calibration value.   
  

  // Read the magnenometer data available with the LSM9DS1 library of the Nano BLE 33 (here the code is for LSM6DS3 of the Nano 33 BLE IOT)
  /* 
   IMU.readMagneticField(magn_x, magn_y, magn_z);
 */
}
