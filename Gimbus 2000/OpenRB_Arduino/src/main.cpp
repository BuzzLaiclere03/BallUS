// Created by Ballus on 2024-04-13.
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <Wire.h>
#include <cmath>
#include <MPU6050_light.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 
int mode = 2; //1 = HMI controle position exacte 2 = IMU  3= YOLO 4 = jeu 5 = home

const uint8_t DXL_ID1 = 20;
const uint8_t DXL_ID2 = 15;
const uint8_t DXL_ID3 = 40; 

const uint8_t ANGLE_0_DXL_ID1 = 167; //a verif
const uint8_t ANGLE_0_DXL_ID2 = 188; //a verif
const uint8_t ANGLE_0_DXL_ID3 = 180; //a verif

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
//double t, tx, tf, pitch, roll, yaw =0;

double previousTime = 0.0;

float qA = 0.0 ; //degree
float qB = 0.0 ;
float qC = 0.0 ;

float theta_x = 0; //degree
float theta_y = 0;
float theta_z = 0 ;

float modMoteur1 = 0;
float modMoteur2 = 0;
float modMoteur3 = 0;

// Modify these values according to the IMU's 0, 0, 0 position
float initialYaw = 0;
float initialPitch = 0;
float initialRoll = 0;

// Initialize variables to track the last motor position and decide if an update is needed based on threshold
float lastAngle1 = 0;
float lastAngle2 = 0;
float lastAngle3 = 0;

int seuil = 0; //degree

// Initialization for IMU
MPU6050 mpu(Wire);
unsigned long timer = 0;
unsigned long timer2 = 0;

// Initialize angle memory
float last_Yaw=0;
float last_pitch=0;
float last_Roll=0;

int motorYaw = 0;
int motorPitch = 0;
int motorRoll = 0;

int oldMotorYaw = 0;
int oldMotorPitch = 0;

float motorYawHMI = 0;
float motorPitchHMI = 0;
float motorRollHMI =0;

float motorYawYolo = 0;
float motorPitchYolo = 0;

const float alpha = 0.4f;  // Adjustable according to the desired level of responsiveness

float smoothedMotorYawYolo = 0.0f;   // Smoothed value for the Yolo yaw direction
float smoothedMotorPitchYolo = 0.0f; // Smoothed value for the Yolo pitch direction

//-------------------------------------------------------------------- Initialization for the calculation of yaw, pitch, and roll -------------------------------------------------------------------------------------

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
 
float pitch, roll, yaw;
float pitchGyro, rollGyro, yawGyro;
long lastGyroTime = 0;
float previousGyroZ = 0;
float previousGyroX = 0;
float previousGyroY = 0;
 
float angle_pitch = 0.0f;
float angle_roll = 0.0f;
float angle_yaw = 0.0f;
float gyroX_rate = 0.0f;
float gyroY_rate = 0.0f;
float gyroZ_rate = 0.0f;
float angleX, angleY, angleZ;
float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;

// ------------------------------------------------------------------- quaternion --------------------------------------------------------------

// Definition of a structure for quaternions
struct Quaternion1 {
  float w, x, y, z;

  // Multiplication of quaternions
  Quaternion1 operator*(const Quaternion1& q) const {
      return {
          w * q.w - x * q.x - y * q.y - z * q.z,
          w * q.x + x * q.w + y * q.z - z * q.y,
          w * q.y - x * q.z + y * q.w + z * q.x,
          w * q.z + x * q.y - y * q.x + z * q.w
      };
  }

  // Inverse of a unit quaternion (q⁻¹ = conj(q) since |q| = 1)
  Quaternion1 inverse() const {
      return {w, -x, -y, -z};
  }

  // Conversion to Euler angles
  void toEuler(float &yaw, float &pitch, float &roll) const {
      yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / PI;
      pitch = asin(2.0 * (w * y - z * x)) * 180.0 / PI;
      roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / PI;
  }

  // Normalize the quaternion
  void normalize() {
      float norm = sqrt(w * w + x * x + y * y + z * z);
      if (norm > 1e-6) { // Avoid division by zero
          w /= norm;
          x /= norm;
          y /= norm;
          z /= norm;
      }
  }
};
//--------------------------------------------------------------- function eulerToQuaternion 

// Function to convert Euler angles to quaternion
Quaternion1 eulerToQuaternion(float yaw, float pitch, float roll) {
  float cy = cos(yaw * PI / 180 /2);
  float sy = sin(yaw * PI / 180 /2);
  float cp = cos(pitch * PI / 180 /2);
  float sp = sin(pitch * PI / 180 /2);
  float cr = cos(roll * PI / 180 /2);
  float sr = sin(roll * PI / 180 /2);

  return {
      cy * cp * cr + sy * sp * sr, // w
      cy * cp * sr - sy * sp * cr, // x
      sy * cp * sr + cy * sp * cr, // y
      sy * cp * cr - cy * sp * sr  // z
  };
}
// --------------------------------------------------------------------- // Initialize quaternion
// Gimbal reference angles
Quaternion1 gimbalReference = {1, 0, 0, 0};
   
void updatemotorposition(float yaw, float pitch, float roll, float qA, float qC, float qB ) {
    // Convert to quaternion
    Quaternion1 q_g = eulerToQuaternion(yaw, pitch, roll);

    // Use the last motor position as a reference
    Quaternion1 gimbalReference = eulerToQuaternion(qA, qC, qB);
    
    // Calculate compensation: q_corr = q_g^-1 * q_ref
    Quaternion1 q_corr =  gimbalReference * q_g.inverse();

    // Normalize
    q_corr.normalize();

    // Extract correction angles
    float yawComp, pitchComp, rollComp;
    q_corr.toEuler(yawComp, pitchComp, rollComp);

    // Convert to motor position (0 to 360°)
    motorYaw = int(fmod((yawComp + qA ), 360));
    motorPitch = -int(fmod((pitchComp + qC ), 360));
    motorRoll = int(fmod((rollComp + qB ), 360));
}

float DegToRad(float angle) {
  angle = ((angle) * M_PI / 180.0);
  return angle; // Converts degrees to radians
}

float RadToDeg(float angle) {
  angle = ((angle) * 180 / M_PI);
  return angle; // Converts radians to degrees
}

float sinDeg(float angle) {
  return sin(DegToRad(angle)); // Converts to radians and applies sin()
}

float cosDeg(float angle) {
  return cos(DegToRad(angle)); // Converts to radians and applies cos()
}

void readAngle() {
  //Read angle on each motor
  qA = dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE);
  qB = dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE);
  qC = dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE);
}

void sendNewPositionToMotors()
{
  float Motor1GoToPosition = ANGLE_0_DXL_ID1  + motorYaw;
  float Motor2GoToPosition = ANGLE_0_DXL_ID2  + motorRoll;
  float Motor3GoToPosition = ANGLE_0_DXL_ID3 + motorPitch;

  // Verification that the angles remain between 0 and 360 because the motors only cover these two limits
  if (Motor1GoToPosition > 360) {
    Motor1GoToPosition = 360;
  }

  if (Motor1GoToPosition < 0){
    Motor1GoToPosition = 0;
  }

  if (Motor2GoToPosition > 180+70) {
    Motor2GoToPosition = 180+70;
  }

  if (Motor2GoToPosition < 0) {
    Motor2GoToPosition = 0;
  }

  if (Motor3GoToPosition > 360) {
    Motor3GoToPosition = 360;
  }

  if (Motor3GoToPosition < 0) {
    Motor3GoToPosition = 0;
  }

  if (fabs((Motor1GoToPosition) -  lastAngle1) >= seuil )  {
    dxl.setGoalPosition(DXL_ID1, Motor1GoToPosition, UNIT_DEGREE);
    lastAngle1 = Motor1GoToPosition;
  }
  
  if (fabs((Motor2GoToPosition) -  lastAngle2) >= seuil  )  {
    dxl.setGoalPosition(DXL_ID2, Motor2GoToPosition, UNIT_DEGREE);
    lastAngle2 = Motor2GoToPosition;
  }

  if (fabs((Motor3GoToPosition) -  lastAngle3) >= seuil )  {
    dxl.setGoalPosition(DXL_ID3, Motor3GoToPosition, UNIT_DEGREE);
    lastAngle3 = Motor3GoToPosition;
  }
}

void sendAngleToHMI(){
  readAngle();
  Serial.print(qA-ANGLE_0_DXL_ID1);
  Serial.print(",");
  Serial.print(qB-ANGLE_0_DXL_ID2);
  Serial.print(",");
  Serial.println(qC-ANGLE_0_DXL_ID3);
}

void receiveModeAndAngleFromHMI(){
  if (Serial.available()) {
    String ligne = Serial.readStringUntil('\n');  // Read the full line until newline character
    ligne.trim();  // Remove any leading or trailing spaces
    
    // We split the line by commas
    int modeIndex = ligne.indexOf(',');  // Find the index of the first comma
    mode = ligne.substring(0, modeIndex).toInt();  // Extract the mode (before the first comma) and convert it to an integer
    
    ligne.remove(0, modeIndex + 1);  // Remove the mode and the comma from the string

    // Now we read the next values separated by commas
    motorYawHMI = ligne.substring(0, ligne.indexOf(',')).toFloat();  // Extract yaw (next part) and convert it to a float
    ligne.remove(0, ligne.indexOf(',') + 1);  // Remove the yaw value and the comma

    motorRollHMI = ligne.substring(0, ligne.indexOf(',')).toFloat();  // Extract roll (next part) and convert it to a float
    ligne.remove(0, ligne.indexOf(',') + 1);  // Remove the roll value and the comma

    motorPitchHMI = ligne.substring(0, ligne.indexOf(',')).toFloat();  // Extract pitch (next part) and convert it to a float
    ligne.remove(0, ligne.indexOf(',') + 1);  // Remove the pitch value and the comma
    
    motorYawYolo = ligne.substring(0, ligne.indexOf(',')).toFloat();  // Extract pitch (next part) and convert it to a float
    ligne.remove(0, ligne.indexOf(',') + 1);  // Remove the pitch value and the comma

    motorPitchYolo = ligne.substring(0, ligne.indexOf(',')).toFloat();  // Extract pitch (next part) and convert it to a float
    ligne.remove(0, ligne.indexOf(',') + 1);  // Remove the pitch value and the comma

    // Extract the unused value (it will be at the end after all the commas) and convert it to float
    float INUTIL = ligne.toFloat(); 
    }
}

void setup() {
  Wire.begin();
  Serial.begin(9600); //9600

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) {
    // stop everything if could not connect to MPU6050
    delay(100);
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // Gyro and accelerometer calibration
  Serial.println("Done!\n");

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(2000);   
  DEBUG_SERIAL.println("Starting position control ...");
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL); 

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  if (dxl.getLastLibErrCode()) {
    DEBUG_SERIAL.println("Could not init serial port!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  if (!dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION)) {
    DEBUG_SERIAL.println("Could not set protocol version!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  // Get DYNAMIXEL information
  bool ping = dxl.ping(DXL_ID1);
  bool ping2 = dxl.ping(DXL_ID2);
  bool ping3 = dxl.ping(DXL_ID3);
  if (!ping || !ping2  || !ping3) {
    DEBUG_SERIAL.println("Could not ping motor!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
    return;
  }

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);
  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 90);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 90);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 90);

  DEBUG_SERIAL.println("Super Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());

  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + 0, UNIT_DEGREE);

  delay(2000);
}

void loop() {

  receiveModeAndAngleFromHMI();
  sendAngleToHMI();

  switch (mode) {
    case 1: // HMI control
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 0);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 0);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 0);
        motorYaw = motorYawHMI;
        motorPitch = motorPitchHMI;
        motorRoll = motorRollHMI;
      
        sendNewPositionToMotors();
        break;

    case 2: // IMU
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 90);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 90);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 90);
        mpu.update();  // Update sensor data
        angle_yaw = mpu.getAngleZ();
        angle_roll = mpu.getAngleX();
        angle_pitch = mpu.getAngleY();

        updatemotorposition(angle_yaw, angle_pitch, angle_roll, 0, 0, 0);
        sendNewPositionToMotors();
        break;

    case 3: { // Object detection
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 40);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 40);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 40);


        // IIR Filter
        smoothedMotorYawYolo = alpha/2 * motorYawYolo + (1 - alpha)/2 * smoothedMotorYawYolo;
        smoothedMotorPitchYolo = alpha/2 * motorPitchYolo + (1 - alpha)/2 * smoothedMotorPitchYolo;

        readAngle();

        int desiredAngleMotor1 = qA + smoothedMotorYawYolo;
        int desiredAngleMotor2 = qB + smoothedMotorPitchYolo;

        dxl.setGoalPosition(DXL_ID1, desiredAngleMotor1, UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, desiredAngleMotor2, UNIT_DEGREE);

        // Update of the previous state
        oldMotorYaw = motorYaw;
        oldMotorPitch = motorPitch;
        break;
    }

    case 4:
        printf("ADD A NEW MODE\n");
        break;

    case 5: // Back to home (0,0,0)
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 0);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 0);
        dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 0);
        motorYaw = 0;
        motorPitch = 0;
        motorRoll = 0;
        sendNewPositionToMotors();
        break;

    default:
        printf("PLEASE SELECT A MODE\n");
        break;
  }
}
  

  
  

