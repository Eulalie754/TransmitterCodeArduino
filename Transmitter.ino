
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;    // Set to false or true;
float manual_gyro_pitch_cal_value = 0;
float manual_gyro_roll_cal_value = 0;
float manual_gyro_yaw_cal_value = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t cal_int;
float acc_x, acc_y, acc_z;
float gyro_pitch, gyro_roll, gyro_yaw;
float magn_x, magn_y, magn_z;

float acc_total_vector;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float battery_voltage;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // initialize the BLE hardware
  BLE.begin();
  
  Serial.println("BLE Central - Receiver");

  // start scanning for peripherals
  BLE.scanForUuid("917649A0-D98E-11E5-9EEC-0002A5D5C51B");
  
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.print(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.print(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");

  Serial.print("Calibration please wait");
  calibrate_gyro();
}

void loop() {

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "ArduinoIMU") {
      return;
    }
    // stop scanning
    BLE.stopScan();

    readReceiver(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("917649A0-D98E-11E5-9EEC-0002A5D5C51B");

  }

  IMU_signal(); //Read the gyro and accelerometer data.

  //Gyro angle calculations (integration)
  //0,009615= 1 / (104Hz)
  angle_pitch += (float)gyro_pitch*0.009615;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll*0.009615;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_yaw += (float)gyro_yaw*0.009615;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.

  /*
  Serial.print("gyro angle");
  Serial.print('\t');
  Serial.print(angle_pitch);
  Serial.print('\t');
  Serial.print(angle_roll);
  Serial.print('\t');
  Serial.print(angle_yaw);
  Serial.print('\t');
  */
  
  //0,000167835 = 0,009615 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll*sin((float)gyro_yaw*0.000167835);                  //If the IMU has yawed transfer the roll angle to the pitch angle.
  angle_roll += angle_pitch*sin((float)gyro_yaw*0.000167835);                  //If the IMU has yawed transfer the pitch angle to the roll angle.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));    //Calculate the total accelerometer vector.
 
  if (abs(acc_y)<acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc=asin((float)acc_y/acc_total_vector)*57.296;              //Calculate the pitch angle in degrees.
  }
  if (abs(acc_x)<acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc=asin((float)acc_x/acc_total_vector)*57.296;               //Calculate the roll angle in degrees.
  }

  /*
  Serial.print("acc angle");
  Serial.print('\t');
  Serial.print(angle_pitch_acc);
  Serial.print('\t');
  Serial.print(angle_roll_acc);
  Serial.print('\t');
   */
   
  angle_pitch = angle_pitch*0.9+angle_pitch_acc*0.1;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll*0.9+angle_roll_acc*0.1;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  /*
  Serial.print("correct angle");
  Serial.print('\t');
  Serial.print(angle_pitch);
  Serial.print('\t');
  Serial.print(angle_roll);
  Serial.println('\t');
  */

}

void readReceiver(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering service ...");
  if (peripheral.discoverService("917649A0-D98E-11E5-9EEC-0002A5D5C51B")) {
    Serial.println("Service discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();

    while(1);
    return;
  }

  // retrieve the Transmitter characteristic
  BLECharacteristic imuCharacteristic = peripheral.characteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B");

  // subscribe to the simple key characteristic
  Serial.println("Subscribing to simple key characteristic ...");
  if (!imuCharacteristic) {
    Serial.println("Peripheral does not have imucharacteristic!");
    peripheral.disconnect();
    return;
  } else if (!imuCharacteristic.canSubscribe()) {
    Serial.println("imucharacteristic is not subscribable!");
    peripheral.disconnect();
    return;
  } else if (!imuCharacteristic.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  } else {
    Serial.println("Subscribed");
    Serial.println("Press the right and left buttons on your SensorTag.");
  }
  
  // while the peripheral is connected
  while (peripheral.connected()) {
    float data[3];
    // read the sensor data
    imuCharacteristic.readValue((byte *) &data, 12);
    Serial.print(data[0]);
    Serial.print('\t');
    Serial.print(data[1]);
    Serial.print('\t');
    Serial.println (data[2]);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Add the code for generating the output signal to the ESC using the PID calculation 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  Serial.println("Peripheral disconnected");
}
