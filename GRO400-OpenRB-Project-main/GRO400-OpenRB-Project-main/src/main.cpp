// GRO400 - Exemple d'utilisation du OpenRB avec un moteur Dynamixel sous Platform.IO.
// Basé sur l'exemple de Position Control.
// Opère un moteur (à définir par la variable DXL_ID - 1 par défaut) en position en le faisant passer
// d'une position en pulsations (1000) à une autre en degrés (5.7) et vice-versa à chaque
// seconde.
// Écrit la position en cours en pulsations à la console série (accessible par DEBUG_SERIAL).
// N'oubliez-pas de configurer votre port série pour cette console à 115200 bauds.

#include <Dynamixel2Arduino.h>
#include <math.h>
#include <Wire.h>
#include <cmath>
#include "I2Cdev.h"
#include "MPU6050.h"

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
 

const uint8_t DXL_ID1 = 20;
const uint8_t DXL_ID2 = 15;
const uint8_t DXL_ID3 = 40; 

const uint8_t ANGLE_0_DXL_ID1 = 180; //a verif
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

//float angle =0, ax=0, ay=0, az=0;

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

//Modifier ces valeurs selon le position 0, 0, 0 de l'IMU
float initialYaw = 0;
float initialPitch = 0;
float initialRoll = 0;

//Initialisation des variables pour tenir en memoire la derniere position du moteur pour savoir si on doit envoyer une nouvelle position ou le seuil n'est pas dépassé
float lastAngle1 = 0;
float lastAngle2 = 0;
float lastAngle3 = 0;

int seuil = 1; //degree

//-------------------------------------------------------------------- Initiation pour le calcule de yaw pitch roll -------------------------------------------------------------------------------------

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
 
MPU6050 accelgyro;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
 
float pitch, roll, yaw;
float pitchGyro, rollGyro, yawGyro;
long lastGyroTime = 0;
float previousGyroZ = 0;
float previousGyroX = 0;
float previousGyroY = 0;
 
// Kalman filter variables
float Q_angle = 0.001f;   // Process noise covariance for the angle
float Q_gyro = 0.003f;    // Process noise covariance for the gyro rate
float R_angle = 0.03f;    // Measurement noise covariance for the angle
 
float angle_pitch = 0.0f;
float angle_roll = 0.0f;
float angle_yaw = 0.0f;
float gyroX_rate = 0.0f;
float gyroY_rate = 0.0f;
float gyroZ_rate = 0.0f;
float angleX, angleY, angleZ;
float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;
 
// Kalman filter states (flattened covariance matrix)
float P_pitch[4] = {1, 0, 0, 1};  // Flattened Covariance matrix for pitch
float P_roll[4] = {1, 0, 0, 1};   // Flattened Covariance matrix for roll
float P_yaw[4] = {1, 0, 0, 1};    // Flattened Covariance matrix for yaw

// ------------------------------------------------------------------- quaternion --------------------------------------------------------------
// quaternion 1

 // Définition d'une structure pour les quaternions
struct Quaternion1 {
    float w, x, y, z;

    // Multiplication de quaternions
    Quaternion1 operator*(const Quaternion1& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }
    // Inverse d'un quaternion unitaire (q⁻¹ = conj(q) car |q| = 1)
    Quaternion1 inverse() const {
        return {w, -x, -y, -z};
    }

    // Conversion en angles d'Euler
    void toEuler(float &yaw, float &pitch, float &roll) const {
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / PI;
        pitch = asin(2.0 * (w * y - z * x)) * 180.0 / PI;
        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / PI;
    }

    void normalize() {
        float norm = sqrt(w * w + x * x + y * y + z * z);
        if (norm > 1e-6) { // Éviter la division par zéro
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    }
};
//--------------------------------------------------------------- fonction  eulerToQuaternion 
// Fonction pour convertir des angles d'Euler en quaternion
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
// --------------------------------------------------------------------- initiation pour quaternion 
// Angles de référence du gimbal
Quaternion1 gimbalReference = {1, 0, 0, 0};

int motorYaw = 0;
int motorPitch = 0;
int motorRoll = 0;

void updatemotorposition(float yaw, float pitch, float roll, float qA, float qC, float qB ) {
    // Convertir en quaternion
    Quaternion1 q_g = eulerToQuaternion(yaw, pitch, roll);

    // Utiliser la dernière position moteur comme référence
    Quaternion1 gimbalReference = eulerToQuaternion(qA, qC, qB);
    
    // Calculer la compensation : q_corr = q_g^-1 * q_ref
    Quaternion1 q_corr = q_g.inverse() * gimbalReference;

    // normalise
    q_corr.normalize();

    // Extraire les angles de correction
    float yawComp, pitchComp, rollComp;
    q_corr.toEuler(yawComp, pitchComp, rollComp);

    // Mise à jour des positions moteur
    //lastYaw = yawComp;
    //lastPitch = pitchComp;
    //lastRoll = rollComp;

    Serial.println("yawComp: ");
    Serial.println(yawComp);
    Serial.println("pitchComp: ");
    Serial.println(pitchComp);
    Serial.println("rollComp: ");
    Serial.println(rollComp);


    // Convertir en position moteur (0 à 360°)
    motorYaw = int(fmod((yawComp + qA ), 360));
    motorPitch = int(fmod((pitchComp + qC ), 360));
    motorRoll = int(fmod((rollComp + qB ), 360));
    //motorYaw = qA + motorYaw;
    //motorPitch = qC + motorPitch;
    //motorRoll = qC + motorRoll;

    //delay(10);  // Petit délai

}



// quaternion 2
/*
struct Quaternion {
  double w, x, y, z;

  // Normalisation
  void normalize() {
      double norm = std::sqrt(w * w + x * x + y * y + z * z);
      w /= norm;
      x /= norm;
      y /= norm;
      z /= norm;
  }

  // Produit de quaternions
  Quaternion operator*(const Quaternion& q) const {
      return {
          w * q.w - x * q.x - y * q.y - z * q.z,
          w * q.x + x * q.w + y * q.z - z * q.y,
          w * q.y - x * q.z + y * q.w + z * q.x,
          w * q.z + x * q.y - y * q.x + z * q.w
      };
  }


  // Multiplication d'un quaternion par un scalaire
  Quaternion operator*(double scalar) const {
    return {w * scalar, x * scalar, y * scalar, z * scalar};
  }

  // Addition
  Quaternion operator+(const Quaternion& q) const {
    return {w + q.w, x + q.x, y + q.y, z + q.z};
  }
  // Conversion en angles d'Euler
  void toEuler(float &yaw, float &pitch, float &roll) const {
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / PI;
    pitch = asin(2.0 * (w * y - z * x)) * 180.0 / PI;
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / PI;
  }
};

int motorYaw = 0;
int motorPitch = 0;
int motorRoll = 0;

// Mise à jour du quaternion avec la vitesse angulaire
void updateQuaternion(Quaternion& q, double wx, double wy, double wz, double dt) {
  // Conversion en quaternion de la vitesse angulaire
  Quaternion omega = {0, -wx, -wy, -wz};

  // Calcul de dq/dt = 0.5 * q * omega
  Quaternion dq = (q * omega) * (0.5 * dt);

  // Mise à jour du quaternion
  q = q + dq;

  // Normalisation pour éviter les erreurs numériques
  q.normalize();

  float yawComp, pitchComp, rollComp;
  q.toEuler(yawComp, pitchComp, rollComp);


  // Convertir en position moteur (0 à 360°)
  motorYaw = int(fmod((yawComp), 360));
  motorPitch = int(fmod((pitchComp), 360));
  motorRoll = int(fmod((rollComp), 360));
  

}
*/

// ---------------------------------------------------------------  Kalman filter functions ------------------------------------------------------
void updateKalmanFilter(float *angle, float *bias, float *P, float newAngle, float newRate, float dt) {
  // Predict step
  *angle += dt * (newRate - *bias);
  P[0] += dt * (dt * P[3] - P[1] - P[2] + Q_angle);  // P[0][0]
  P[1] -= dt * P[3];  // P[0][1] = P[1][0]
  P[2] -= dt * P[3];  // P[1][0] = P[0][1]
  P[3] += Q_gyro * dt; // P[1][1] = P[3][3]
 
  // Measurement update step (using accelerometer or magnetometer)
  float S = P[0] + R_angle;
  float K[2];  // Kalman gain
  K[0] = P[0] / S;
  K[1] = P[2] / S;
 
  float y = newAngle - *angle;
  *angle += K[0] * y;
  *bias += K[1] * y;
 
  // Update covariance
  float P00_temp = P[0];
  float P01_temp = P[1];
 
  P[0] -= K[0] * P00_temp;
  P[1] -= K[0] * P01_temp;
  P[2] -= K[1] * P00_temp;
  P[3] -= K[1] * P01_temp;
}

// angle original
/*

void getAngle(int Ax, int Ay, int Az, int Gy) {
 double x = Ax;
 double y = Ay;
 double z = Az;

 double yaw_rad =0;

 pitch = -(atan2(z, sqrt(y * y)));

 roll = atan2(x , sqrt((y * y)));

 //yaw = atan(x / sqrt((z * z)+(y * y)));
 
 double currentTime = millis() / 1000.0; // Temps en secondes //unsigned long     micros() au lieu de millis
 double deltaTime = currentTime - previousTime;    //unsigned long
 previousTime = currentTime;
 //Serial.println("deltaTime: ");
//Serial.println(deltaTime);
//Intégrer la vitesse angulaire pour obtenir l'angle yaw
if (Gy > 100 || Gy < -100)                                                       // float gyroY = readGyroY() / GYRO_SENSITIVITY;
{
  yaw_rad += (-Gy/600) * deltaTime;
  Serial.println("Hello ");
}
double a ;
  a = (-Gy/600) * deltaTime;
  Serial.println("a ");
  Serial.println(a);
 //pitch = pitch * (180.0 / PI) + initialPitch;
 //roll = roll * (180.0 / PI) + initialRoll;
 //yaw = yaw_rad * (180.0 / PI) + initialYaw;
 

  // Convertir en position moteur (0 à 360°)
  pitch = int(fmod((pitch * (180.0 / PI) + initialPitch ), 360));
  roll = int(fmod((roll * (180.0 / PI) + initialRoll ), 360));
  yaw = int(fmod((yaw_rad * (180.0 / PI) + initialYaw ), 360));

 Serial.println("yaw: ");
 Serial.println(yaw);
 Serial.println("roll: ");
 Serial.println(roll);
 Serial.println("pitch: ");
 Serial.println(pitch);

}


float DegToRad(float angle) {
  angle = ((angle) * M_PI / 180.0);
  return angle; // Convertit les degrés en radians
}

float RadToDeg(float angle) {
  angle = ((angle) * 180 / M_PI);
  return angle; // Convertit les degrés en radians
}

float sinDeg(float angle) {
    return sin(DegToRad(angle)); // Convertit en radians et applique sin()
}
float cosDeg(float angle) {
    return cos(DegToRad(angle)); // Convertit en radians et applique cos()
}

void readAngle() {
  //Lecture angle des moteurs
  qA = dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE);
  qB = dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE);
  qC = dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE);

  // Convertir en position moteur (0 à 360°)
  //qA = int(fmod((qA), 360));
 //qB = int(fmod((qB), 360));
 //qC = int(fmod((qA), 360));


  //Serial.println(qA);
  //Serial.println(qB);
  //Serial.println(qC);
}


void readIMU() 
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcXcal = -950;
  AcYcal = -300;
  AcZcal = 0;
  tcal = -1600;
  GyXcal = 480;
  GyYcal = 170;
  GyZcal = 210;
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  tx = Tmp + tcal;
  t = tx / 340 + 36.53;
  tf = (t * 9 / 5) + 32;
  //Serial.println("AcX: ");
  //Serial.println(AcX);
  //Serial.println("AcY: ");
  //Serial.println(AcY);
  //Serial.println("AcZ: ");
  //Serial.println(AcZ);

  //Serial.println("GyX: ");
  //Serial.println(GyX);
  //Serial.println("GyY: ");
  //Serial.println(GyY);
  //Serial.println("GyZ: ");
  //Serial.println(GyZ);
}
*/

// -----------------------------------------------------------------  sendNewPositionToMotors ----------------------------------------------------------

void sendNewPositionToMotors()
{
  float Motor1GoToPosition = ANGLE_0_DXL_ID1  + motorYaw;
  float Motor2GoToPosition = ANGLE_0_DXL_ID2  + motorRoll;
  float Motor3GoToPosition = ANGLE_0_DXL_ID3 + motorPitch;

  //Verification que les angles restent entre 0 et 360 car les moteurs couvrent seulement entre ces 2 bornes
  if (Motor1GoToPosition > 360)
  {
    Motor1GoToPosition = 360;
  }

  if (Motor1GoToPosition < 0)
  {
    Motor1GoToPosition = 0;
  }

  if (Motor2GoToPosition > 360)
  {
    Motor2GoToPosition = 360;
  }

  if (Motor2GoToPosition < 0)
  {
    Motor2GoToPosition = 0;
  }

  if (Motor3GoToPosition > 360)
  {
    Motor3GoToPosition = 360;
  }

  if (Motor3GoToPosition < 0)
  {
    Motor3GoToPosition = 0;
  }

  if (fabs((Motor1GoToPosition) -  lastAngle1) >= seuil )  
  {
    dxl.setGoalPosition(DXL_ID1, Motor1GoToPosition, UNIT_DEGREE);
    lastAngle1 = Motor1GoToPosition;
  }
  
  if (fabs((Motor2GoToPosition) -  lastAngle2) >= seuil  )  
  {
    dxl.setGoalPosition(DXL_ID2, Motor2GoToPosition, UNIT_DEGREE);
    lastAngle2 = Motor2GoToPosition;
  }

  if (fabs((Motor3GoToPosition) -  lastAngle3) >= seuil )  
  {
    dxl.setGoalPosition(DXL_ID3, Motor3GoToPosition, UNIT_DEGREE);
    lastAngle3 = Motor3GoToPosition;
  }
   
  //delay(10);

}


// ---------------------------------------------------------------------- setup --------------------------------------------------------------------
// setup 1
/*
void setup() {


 Wire.begin();
 Wire.beginTransmission(MPU);
 Wire.write(0x6B);
 Wire.write(0);
 Wire.endTransmission(true);

  // put your setup code here, to run once:
  delay(2000);    // Délai additionnel pour avoir le temps de lire les messages sur la console.
  DEBUG_SERIAL.println("Starting position control ...");
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL); // On attend que la communication série pour les messages soit prête.

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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 30);

  DEBUG_SERIAL.println("Super Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  

  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + 0, UNIT_DEGREE);
  delay(2000);
  

}
*/

// setup 2
void setup() {
  Wire.begin();
  Serial.begin(9600);
 
  accelgyro.initialize();

 Wire.beginTransmission(MPU);
 Wire.write(0x6B);
 Wire.write(0);
 Wire.endTransmission(true);

  // put your setup code here, to run once:
  delay(2000);    // Délai additionnel pour avoir le temps de lire les messages sur la console.
  DEBUG_SERIAL.println("Starting position control ...");
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL); // On attend que la communication série pour les messages soit prête.

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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 30);

  DEBUG_SERIAL.println("Super Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  

  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + 0, UNIT_DEGREE);
  delay(2000);
  
}

// --------------------------------------------------------------------- get Angles ------------------------------------------------------------------
void getAngles() {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
 
  // Calculate Pitch and Roll based on accelerometer data
  pitch = atan2(-az, sqrt(ay*ay + ax*ax)) * (180.0 / PI);  // Pitch
  roll = atan2(ax, sqrt(ay*ay + az*az)) * (180.0 / PI);   // Roll
 
  // Apply the gyroscope data (integration over time)
  long currentGyroTime = millis();
  float dt = (currentGyroTime - lastGyroTime) / 1000.0;
  lastGyroTime = currentGyroTime;
 
  // Gyro values in degrees per second
  gyroX_rate = -gx / 131.0;
  gyroY_rate = -gz / 131.0;
  gyroZ_rate = gy / 131.0;
 
  // Update angles using gyroscope data
  pitchGyro += gyroX_rate * dt;
  rollGyro += gyroY_rate * dt;
  yawGyro += gyroZ_rate * dt;
 
  // Yaw angle from magnetometer (assuming calibrated magnetometer)
  float magX = -mx * cos(roll)  - mz * sin(pitch) * sin(roll) - my * cos(pitch) * sin(roll);
  float magY = -mz * cos(pitch) + my * sin(pitch);
  yaw = atan2(magY, magX) * (180.0 / PI);
 
  // Use complementary filter for yaw (fusing magnetometer and gyro)
  float alpha = 0.98;  // Complementary filter constant
  yaw = alpha * (yawGyro) + (1 - alpha) * yaw;  // Filtered yaw
 
  // Kalman filter for pitch and roll
  updateKalmanFilter(&angle_pitch, &biasX, P_pitch, pitch, gyroX_rate, dt);
  updateKalmanFilter(&angle_roll, &biasY, P_roll, roll, gyroY_rate, dt);
  updateKalmanFilter(&angle_yaw, &biasZ, P_yaw, yaw, gyroZ_rate, dt);
 
  // Print out the angles
  Serial.print("Pitch: ");
  Serial.print(angle_pitch);
  Serial.print("\tRoll: ");
  Serial.print(angle_roll);
  Serial.print("\tYaw: ");
  Serial.println(angle_yaw);
}


// ------------------------------------------------------------------------ loop -----------------------------------------------------------------------
// loop 1
/*
void loop() {

  readIMU();

  getAngle(AcX, AcY, AcZ, GyY);

  readAngle();
  
  updatemotorposition(yaw, pitch, roll, 0,  0, 0);
  
  Serial.println("motorYaw: ");
  Serial.println(motorYaw);
  
  Serial.println("motorPitch: ");
  Serial.println(motorPitch);
  Serial.println("motorRoll: ");
  Serial.println(motorRoll);
  

  sendNewPositionToMotors();
  
}
*/

// loop 2
/*
Quaternion q = {1, 0, 0, 0};
void loop() {

  double currentTime = millis() / 1000.0; // Temps en secondes //unsigned long     micros() au lieu de millis
  double dt = currentTime - previousTime;    //unsigned long
  previousTime = currentTime;

  readIMU();

  GyX = GyX/600;
  GyY = GyY/600;
  GyZ = GyZ/600;

  //readAngle();
  
  //Quaternion q = {1, qA, qC, qB};
  
  // Mise à jour du quaternion avec la vitesse angulaire
  updateQuaternion(q, GyX, GyY, GyZ, dt);
  
  Serial.println("motorYaw: ");
  Serial.println(motorYaw);


  sendNewPositionToMotors();
  
}
*/

void testEulerToQuaternion() {
  float yaw = 0, pitch = 10, roll = 10;
  Quaternion1 q = eulerToQuaternion(yaw, pitch, roll);

  Serial.println("Quaternion from Euler (10,10,0): ");
  Serial.print("w: "); Serial.println(q.w,6);
  Serial.print("x: "); Serial.println(q.x,6);
  Serial.print("y: "); Serial.println(q.y,6);
  Serial.print("z: "); Serial.println(q.z,6);
}

void testQuaternionInverse() {
  Quaternion1 q = eulerToQuaternion(0, 10, 10);
  Quaternion1 q_inv = q.inverse();

  Serial.println("Inverse Quaternion: ");
  Serial.print("w: "); Serial.println(q_inv.w,6);
  Serial.print("x: "); Serial.println(q_inv.x,6);
  Serial.print("y: "); Serial.println(q_inv.y,6);
  Serial.print("z: "); Serial.println(q_inv.z,6);
}
void testToEuler() {
  Quaternion1 q = eulerToQuaternion(0, 10, 10);
  float yaw, pitch, roll;
  q.toEuler(yaw, pitch, roll);

  Serial.println("Euler Angles from Quaternion: ");
  Serial.print("Yaw: "); Serial.println(yaw);
  Serial.print("Pitch: "); Serial.println(pitch);
  Serial.print("Roll: "); Serial.println(roll);
}

void testcalcule() {
  Quaternion1 q = eulerToQuaternion(0, 10, 10);
  Quaternion1 q_i = eulerToQuaternion(0, -10, -10);
  Quaternion1 gimbalReference = eulerToQuaternion(0, 0, 0);
  Quaternion1 q_corr = q.inverse() * gimbalReference;

  Serial.println("Inverse Quaternion 1: ");
  Serial.print("w: "); Serial.println(q_i.w,6);
  Serial.print("x: "); Serial.println(q_i.x,6);
  Serial.print("y: "); Serial.println(q_i.y,6);
  Serial.print("z: "); Serial.println(q_i.z,6);

  Serial.println("Inverse Quaternion 2: ");
  Serial.print("w: "); Serial.println(q_corr.w,6);
  Serial.print("x: "); Serial.println(q_corr.x,6);
  Serial.print("y: "); Serial.println(q_corr.y,6);
  Serial.print("z: "); Serial.println(q_corr.z,6);


}

void loop() {
  //getAngles();
  //delay(500);
  
  angle_yaw=0;
  angle_pitch=10;
  angle_roll=10;

  updatemotorposition(angle_yaw, angle_pitch, angle_roll, 0,  0, 0);
  testEulerToQuaternion();
  testQuaternionInverse();
  testToEuler();

 Serial.println("motorYaw: ");
  Serial.println(motorYaw);
  Serial.println("motorPitch: ");
  Serial.println(motorPitch);
  Serial.println("motorRoll: ");
  Serial.println(motorRoll);

  
  //sendNewPositionToMotors();

}



// code teste 1
/*
#include <Wire.h>
#include <MPU9250_WE.h>
#include <math.h>

#define MPU9250_ADDR 0x68
MPU9250_WE imu = MPU9250_WE(MPU9250_ADDR);

// Définition de la structure de quaternion
struct Quaternion {
    double w, x, y, z;

    // Normalisation du quaternion
    void normalize() {
        double norm = sqrt(w * w + x * x + y * y + z * z);
        w /= norm; x /= norm; y /= norm; z /= norm;
    }

    // Produit de quaternions
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Multiplication par un scalaire
    Quaternion operator*(double scalar) const {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }

    // Addition de quaternions
    Quaternion operator+(const Quaternion& q) const {
        return {w + q.w, x + q.x, y + q.y, z + q.z};
    }

    // Conversion en angles d'Euler
    void toEuler(float &yaw, float &pitch, float &roll) const {
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
        pitch = asin(2.0 * (w * y - z * x)) * 180.0 / M_PI;
        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / M_PI;
    }
};

// Variables globales
Quaternion q = {1, 0, 0, 0};
int motorYaw = 0, motorPitch = 0, motorRoll = 0;

// Initialisation de l'IMU
void setup() {
    Serial.begin(115200);
    Wire.begin();

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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 30);

  DEBUG_SERIAL.println("Super Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  

  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + 0, UNIT_DEGREE);
  delay(2000);

    if (!imu.init()) {
        Serial.println("Erreur : MPU9250 non détecté !");
        while (1);
    }

    imu.enableAccDLPF(true);
    imu.enableGyrDLPF();
    imu.initMagnetometer();

    Serial.println("IMU prête !");
    previousTime = millis() / 1000.0; // Temps initial
}

// Lecture des données de l'IMU
void readIMU2(double &wx, double &wy, double &wz, double &ax, double &ay, double &az, double &mx, double &my, double &mz) {
    xyzFloat gyr = imu.getGyrValues();
    wx = gyr.x; wy = gyr.y; wz = gyr.z;

    xyzFloat acc = imu.getAccRawValues();
    ax = acc.x; ay = acc.y; az = acc.z;

    xyzFloat mag = imu.getMagValues();
    mx = mag.x; my = mag.y; mz = mag.z;

    Serial.println("wx: ");
    Serial.println(wx);
}

// Mise à jour des quaternions avec correction par le magnétomètre
void updateQuaternion(Quaternion &q, double wx, double wy, double wz, double ax, double ay, double az, double mx, double my, double mz, double dt) {
    // Création du quaternion de vitesse angulaire
    Quaternion omega = {0, wx, wy, wz};

    // Calcul de dq/dt = 0.5 * q * omega
    Quaternion dq = (q * omega) * (0.5 * dt);

    // Mise à jour du quaternion
    q = q + dq;
    q.normalize();

    // Correction avec l'accéléromètre et le magnétomètre (Fusion de capteurs)
    float normAcc = sqrt(ax * ax + ay * ay + az * az);
    ax /= normAcc; ay /= normAcc; az /= normAcc;

    float normMag = sqrt(mx * mx + my * my + mz * mz);
    mx /= normMag; my /= normMag; mz /= normMag;

    // Calcul de la direction de la gravité avec le quaternion
    double vx = 2 * (q.x * q.z - q.w * q.y);
    double vy = 2 * (q.w * q.x + q.y * q.z);
    double vz = 1 - 2 * (q.x * q.x + q.y * q.y);

    // Correction de l'erreur de dérive
    double ex = (ay * vz - az * vy) + (my * vy - mz * vx);
    double ey = (az * vx - ax * vz) + (mz * vx - mx * vz);
    double ez = (ax * vy - ay * vx) + (mx * vy - my * vx);

    // Ajustement du gyroscope avec la correction d'erreur
    wx += 0.1 * ex;
    wy += 0.1 * ey;
    wz += 0.1 * ez;

    // Mise à jour du quaternion corrigé
    omega = {0, wx, wy, wz};
    dq = (q * omega) * (0.5 * dt);
    q = q + dq;
    q.normalize();

    // Conversion en angles
    float yawComp, pitchComp, rollComp;
    q.toEuler(yawComp, pitchComp, rollComp);

    // Convertir en position moteur (0 à 360°)
    motorYaw = int(fmod(yawComp, 360));
    motorPitch = int(fmod(pitchComp, 360));
    motorRoll = int(fmod(rollComp, 360));
}

// Mise à jour de la position des moteurs
void sendNewPositionToMotors()
{
  float Motor1GoToPosition = ANGLE_0_DXL_ID1  + motorYaw;
  float Motor2GoToPosition = ANGLE_0_DXL_ID2  + motorRoll;
  float Motor3GoToPosition = ANGLE_0_DXL_ID3 + motorPitch;

  //Verification que les angles restent entre 0 et 360 car les moteurs couvrent seulement entre ces 2 bornes
  if (Motor1GoToPosition > 360)
  {
    Motor1GoToPosition = 360;
  }

  if (Motor1GoToPosition < 0)
  {
    Motor1GoToPosition = 0;
  }

  if (Motor2GoToPosition > 360)
  {
    Motor2GoToPosition = 360;
  }

  if (Motor2GoToPosition < 0)
  {
    Motor2GoToPosition = 0;
  }

  if (Motor3GoToPosition > 360)
  {
    Motor3GoToPosition = 360;
  }

  if (Motor3GoToPosition < 0)
  {
    Motor3GoToPosition = 0;
  }

  if (fabs((Motor1GoToPosition) -  lastAngle1) >= seuil )  
  {
    dxl.setGoalPosition(DXL_ID1, Motor1GoToPosition, UNIT_DEGREE);
    lastAngle1 = Motor1GoToPosition;
  }
  
  if (fabs((Motor2GoToPosition) -  lastAngle2) >= seuil  )  
  {
    dxl.setGoalPosition(DXL_ID2, Motor2GoToPosition, UNIT_DEGREE);
    lastAngle2 = Motor2GoToPosition;
  }

  if (fabs((Motor3GoToPosition) -  lastAngle3) >= seuil )  
  {
    dxl.setGoalPosition(DXL_ID3, Motor3GoToPosition, UNIT_DEGREE);
    lastAngle3 = Motor3GoToPosition;
  }
   
  //delay(10);

}

// Boucle principale
void loop() {
    double currentTime = millis() / 1000.0;
    double dt = currentTime - previousTime;
    previousTime = currentTime;

    double wx, wy, wz, ax, ay, az, mx, my, mz;
    readIMU2(wx, wy, wz, ax, ay, az, mx, my, mz);

    updateQuaternion(q, wx, wy, wz, ax, ay, az, mx, my, mz, dt);
    sendNewPositionToMotors();
}

*/

// code gyro 1
/*
#include <Wire.h>

// Adresse I2C du MPU9250
#define MPU_ADDR 0x68  // Adresse du MPU9250, à vérifier si c'est 0x68 ou 0x69 en fonction du câblage
#define AK8963_ADDR 0x0C // Adresse du magnétomètre AK8963

// Registres du MPU9250 et AK8963
#define WHO_AM_I_MPU9250 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MAGNETOMETER_XOUT_L 0x03

// Variables pour les données des capteurs
int16_t ax2, ay2, az2;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// Calibration (si nécessaire)
float accX, accY, accZ;

// Fonction d'initialisation du MPU9250
void initMPU9250() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);  // Accéder au registre de gestion de l'alimentation
  Wire.write(0x00);  // Sortir du mode veille
  Wire.endTransmission(true);

  // Vérifier la connexion au MPU9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I_MPU9250);  // Lire l'identifiant
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  uint8_t whoAmI = Wire.read();
  if (whoAmI == 0x71) {
    Serial.println("MPU9250 est connecté.");
  } else {
    Serial.println("MPU9250 non connecté!");
  }
}

// Fonction d'initialisation du magnétomètre AK8963
void initMagnetometer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); // Registre de configuration du magnétomètre
  Wire.write(0x02); // Mode à 16 bits, à 100 Hz
  Wire.endTransmission(true);
}

void setup() {
  // Initialisation de la communication série et I2C
  Serial.begin(115200);
  Wire.begin();
  
  // Initialisation du MPU9250
  initMPU9250();

  // Initialisation du Magnétomètre
  initMagnetometer();
}

// Fonction de lecture des données IMU (accéléromètre, gyroscope et magnétomètre)
void readIMU3() {
  // Lire les données de l'accéléromètre (6 octets)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  ax2 = (Wire.read() << 8) | Wire.read();
  ay2 = (Wire.read() << 8) | Wire.read();
  az2 = (Wire.read() << 8) | Wire.read();

  // Lire les données du gyroscope (6 octets)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  // Lire les données du magnétomètre (6 octets)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MAGNETOMETER_XOUT_L);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 6, true);
  mx = (Wire.read() | (Wire.read() << 8));
  my = (Wire.read() | (Wire.read() << 8));
  mz = (Wire.read() | (Wire.read() << 8));
}

void loop() {
  // Lire les données du capteur IMU
  readIMU3();

  // Afficher les valeurs lues
  Serial.print("Ax: "); Serial.print(ax2); Serial.print(" ");
  Serial.print("Ay: "); Serial.print(ay2); Serial.print(" ");
  Serial.print("Az: "); Serial.print(az2); Serial.print(" ");
  Serial.print("Gx: "); Serial.print(gx); Serial.print(" ");
  Serial.print("Gy: "); Serial.print(gy); Serial.print(" ");
  Serial.print("Gz: "); Serial.print(gz); Serial.print(" ");
  Serial.print("Mx: "); Serial.print(mx); Serial.print(" ");
  Serial.print("My: "); Serial.print(my); Serial.print(" ");
  Serial.print("Mz: "); Serial.println(mz);

  delay(500);
}

*/


 

 

 

 




/*
#include <Wire.h>

struct Quaternion {
    float w, x, y, z;

    // Produit quaternion x vecteur (approximation)
    Quaternion operator*(const float v[3]) const {
        return {
            -x * v[0] - y * v[1] - z * v[2],
             w * v[0] + y * v[2] - z * v[1],
             w * v[1] + z * v[0] - x * v[2],
             w * v[2] + x * v[1] - y * v[0]
        };
    }

    // Addition de quaternions
    Quaternion operator+(const Quaternion& q) const {
        return {w + q.w, x + q.x, y + q.y, z + q.z};
    }

    // Normalisation du quaternion
    void normalize() {
        float norm = sqrt(w * w + x * x + y * y + z * z);
        w /= norm; x /= norm; y /= norm; z /= norm;
    }
};

// Quaternion d'orientation initiale (identité)
Quaternion q = {1.0, 0.0, 0.0, 0.0};

unsigned long previousTime = 0;
const float GYRO_SENSITIVITY = 131.0;  // ±250°/s pour MPU6050

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // Initialisation du gyroscope ici (ex: MPU6050)
}

void loop() {
    unsigned long currentTime = micros();
    float dt = (currentTime - previousTime) / 1e6; // Temps en secondes
    previousTime = currentTime;

    // Lire la vitesse angulaire depuis le gyroscope
    float gyro[3] = {readGyroX() / GYRO_SENSITIVITY, 
                     readGyroY() / GYRO_SENSITIVITY, 
                     readGyroZ() / GYRO_SENSITIVITY};

    // Calcul de dq = (1/2) * q * ω
    Quaternion dq = q * gyro;
    dq.w *= 0.5f * dt; dq.x *= 0.5f * dt; 
    dq.y *= 0.5f * dt; dq.z *= 0.5f * dt;

    // Mise à jour du quaternion : q_new = q_old + dq
    q = q + dq;
    q.normalize();  // Normalisation pour éviter la dérive

    // Convertir quaternion en angles d'Euler
    float roll  = atan2(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * 180.0f / PI;
    float pitch = asin(2.0f * (q.w * q.y - q.z * q.x)) * 180.0f / PI;
    float yaw   = atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / PI;

    // Affichage des angles
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);

    delay(10); // Petit délai pour stabiliser l'affichage
}
*/


/*
struct Quaternion {
  double w, x, y, z;

  // Normalisation du quaternion
  void normalize() {
      double norm = std::sqrt(w * w + x * x + y * y + z * z);
      w /= norm;
      x /= norm;
      y /= norm;
      z /= norm;
  }

  // Produit quaternionique
  Quaternion operator*(const Quaternion& q) const {
      return {
          w * q.w - x * q.x - y * q.y - z * q.z,
          w * q.x + x * q.w + y * q.z - z * q.y,
          w * q.y - x * q.z + y * q.w + z * q.x,
          w * q.z + x * q.y - y * q.x + z * q.w
      };
  }

  // Mise à jour avec les données du gyroscope
  void update(double wx, double wy, double wz, double dt) {
      Quaternion omega_q = {0, wx, wy, wz};  // Quaternion vitesse angulaire
      Quaternion q_dot = (*this) * omega_q * 0.5;  // dq/dt = 1/2 * q * omega_q

      // Intégration discrète : q_new = q_old + q_dot * dt
      w += q_dot.w * dt;
      x += q_dot.x * dt;
      y += q_dot.y * dt;
      z += q_dot.z * dt;

      normalize();  // Normalisation pour éviter les dérives
  }
};

int main() {
  Quaternion q = {1, 0, 0, 0};  // Orientation initiale
  double wx = 0.1, wy = 0.2, wz = 0.3;  // Vitesses angulaires en rad/s
  double dt = 0.01;  // Intervalle de temps (10 ms)

  q.update(wx, wy, wz, dt);  // Mise à jour du quaternion

  std::cout << "Nouvelle orientation : (" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")\n";
  return 0;
}
  */

