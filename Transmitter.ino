
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;    // Set to false or true;
float manual_gyro_pitch_cal_value = 0;
float manual_gyro_roll_cal_value = 0;
float manual_gyro_yaw_cal_value = 0;


// BLE Service
BLEService imuService("917649A0-D98E-11E5-9EEC-0002A5D5C51B"); // Custom UUID

// BLE Characteristic
BLECharacteristic imuCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLENotify, 12);

long previousMillis = 0;  // last timechecked, in ms
unsigned long micros_per_reading, micros_previous;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t cal_int;
float acc_x, acc_y, acc_z;
float gyro_pitch, gyro_roll, gyro_yaw;
float magn_x, magn_y, magn_z;

float acc_total_vector;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;

void setup() {
  
  Serial.begin(9600); // initialize serial communication
  while (!Serial);
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  
  // begin initialization
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }  

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  
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

  // Setup bluetooth
  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuCharacteristic);
  BLE.addService(imuService);

  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  micros_per_reading = 1000000 / 104;
  micros_previous = micros();
}

void loop() {

  //BLE Communication 

  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a BLE central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected:
    while (central.connected()) {
      unsigned long micros_now;
      micros_now = micros();

      if (micros_now - micros_previous >= micros_per_reading) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() /*&& IMU.magneticFieldAvailable()*/) { // XX
          IMU_signal(); //Read the gyro and accelerometer data.

           //Gyro angle calculations (integration)
          //0,009615= 1 / (104Hz)
          angle_pitch += (float)gyro_pitch*0.009615;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
          angle_roll += (float)gyro_roll*0.009615;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
          angle_yaw += (float)gyro_yaw*0.009615;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
          
          Serial.print("gyro angle");
          Serial.print('\t');
          Serial.print(angle_pitch);
          Serial.print('\t');
          Serial.print(angle_roll);
          Serial.print('\t');
          Serial.print(angle_yaw);
          Serial.print('\t');
          
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
          
          Serial.print("acc angle");
          Serial.print('\t');
          Serial.print(angle_pitch_acc);
          Serial.print('\t');
          Serial.print(angle_roll_acc);
          Serial.print('\t');
           
          angle_pitch = angle_pitch*0.9+angle_pitch_acc*0.1;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
          angle_roll = angle_roll*0.9+angle_roll_acc*0.1;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.
        
          Serial.print("correct angle");
          Serial.print('\t');
          Serial.print(angle_pitch);
          Serial.print('\t');
          Serial.print(angle_roll);
          Serial.println('\t');
                  
          sendSensorData();
          micros_previous = micros_previous + micros_per_reading;
        }
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }

}
