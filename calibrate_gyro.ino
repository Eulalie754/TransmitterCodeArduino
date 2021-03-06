///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the average gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
  else {
    cal_int = 0;                                                                      //If manual calibration is not used.
    manual_gyro_pitch_cal_value = 0;                                                  //Set the manual pitch calibration variable to 0.
    manual_gyro_roll_cal_value = 0;                                                   //Set the manual roll calibration variable to 0.
    manual_gyro_yaw_cal_value = 0;                                                    //Set the manual yaw calibration variable to 0.
  }

  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      IMU_signal();                                                                 //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      delay(9.6153846153846);                                                         //Small delay to simulate a 104Hz loop during calibration.
    }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
    manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
    manual_gyro_roll_cal_value = gyro_roll_cal;                                       //Set the manual roll calibration variable to the detected value.
    manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         //Set the manual yaw calibration variable to the detected value.
  }
}
