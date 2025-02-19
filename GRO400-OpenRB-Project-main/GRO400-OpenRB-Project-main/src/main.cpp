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

const uint8_t ANGLE_0_DXL_ID1 = 177; //a verif
const uint8_t ANGLE_0_DXL_ID2 = 188; //a verif
const uint8_t ANGLE_0_DXL_ID3 = 180; //a verif

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
double t, tx, tf, pitch, roll, yaw =0;

float angle =0, ax=0, ay=0, az=0;

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

int seuil = 8; //degree

 // Définition d'une structure pour les quaternions
struct Quaternion {
    float w, x, y, z;

    // Multiplication de quaternions
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Inverse d'un quaternion unitaire (q⁻¹ = conj(q) car |q| = 1)
    Quaternion inverse() const {
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
        if (norm > 0.0f) { // Éviter la division par zéro
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    }
};
//--------------------------------------------------------------- fonction  eulerToQuaternion 
// Fonction pour convertir des angles d'Euler en quaternion
Quaternion eulerToQuaternion(float yaw, float pitch, float roll) {
    float cy = cos(yaw * PI / 360);
    float sy = sin(yaw * PI / 360);
    float cp = cos(pitch * PI / 360);
    float sp = sin(pitch * PI / 360);
    float cr = cos(roll * PI / 360);
    float sr = sin(roll * PI / 360);

    return {
        cy * cp * cr + sy * sp * sr, // w
        cy * cp * sr - sy * sp * cr, // x
        sy * cp * sr + cy * sp * cr, // y
        sy * cp * cr - cy * sp * sr  // z
    };
}
// --------------------------------------------------------------------- initiation pour quaternion 
// Angles de référence du gimbal
Quaternion gimbalReference = {1, 0, 0, 0};

int motorYaw = 0;
int motorPitch = 0;
int motorRoll = 0;

void updatemotorposition(float yaw, float pitch, float roll, float qA, float qC, float qB ) {
    // Convertir en quaternion
    Quaternion q_g = eulerToQuaternion(yaw, pitch, roll);

    // Utiliser la dernière position moteur comme référence
    Quaternion gimbalReference = eulerToQuaternion(qA, qC, qB);
    
    // Calculer la compensation : q_corr = q_g^-1 * q_ref
    Quaternion q_corr = q_g.inverse() * gimbalReference;

    // normalise
    q_corr.normalize();

    // Extraire les angles de correction
    float yawComp, pitchComp, rollComp;
    q_corr.toEuler(yawComp, pitchComp, rollComp);

    // Mise à jour des positions moteur
    //lastYaw = yawComp;
    //lastPitch = pitchComp;
    //lastRoll = rollComp;

    /*Serial.println("yawComp: ");
    Serial.println(yawComp);
    Serial.println("pitchComp: ");
    Serial.println(pitchComp);
    Serial.println("rollComp: ");
    Serial.println(rollComp);*/

    // Convertir en position moteur (0 à 360°)
    motorYaw = int(fmod((yawComp + qA ), 360));
    motorPitch = int(fmod((pitchComp + qC ), 360));
    motorRoll = int(fmod((rollComp + qB ), 360));
    //motorYaw = qA + motorYaw;
    //motorPitch = qC + motorPitch;
    //motorRoll = qC + motorRoll;

    //delay(10);  // Petit délai

}

void getAngle(int Ax, int Ay, int Az, int Gy) {
 double x = Ax;
 double y = Ay;
 double z = Az;
 
 pitch = atan(z / sqrt((y * y))) ;

 roll = atan(x / sqrt((y * y)));

 //yaw = atan(x / sqrt((z * z)+(y * y)));

 double currentTime = millis() / 1000.0; // Temps en secondes
 double deltaTime = currentTime - previousTime;
 previousTime = currentTime;
 Serial.println("deltaTime: ");
Serial.println(deltaTime);
//Intégrer la vitesse angulaire pour obtenir l'angle yaw
  float yaw_rad =+ (-Gy/600) * deltaTime;

 pitch = pitch * (180.0 / PI) + initialPitch;
 roll = roll * (180.0 / PI) + initialRoll;
 yaw = yaw_rad * (180.0 / PI) + initialYaw;
 

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
  /*qA = int(fmod((qA), 360));
 qB = int(fmod((qB), 360));
 qC = int(fmod((qA), 360));*/


  /*Serial.println(qA);
  Serial.println(qB);
  Serial.println(qC);*/
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
   /*Serial.println("AcX: ");
  Serial.println(AcX);
  Serial.println("AcY: ");
  Serial.println(AcY);
  Serial.println("AcZ: ");
  Serial.println(AcZ);*/

  Serial.println("GyX: ");
  Serial.println(GyX);
  Serial.println("GyY: ");
  Serial.println(GyY);
  Serial.println("GyZ: ");
  Serial.println(GyZ);
}

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
   
  delay(50);

}

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

void loop() {

  readIMU();

  getAngle(AcX, AcY, AcZ, GyY);

  readAngle();
 
  updatemotorposition(yaw, pitch, roll, 0, 0, 0);
  
  Serial.println("motorYaw: ");
  Serial.println(motorYaw);
  Serial.println("motorPitch: ");
  Serial.println(motorPitch);
  Serial.println("motorRoll: ");
  Serial.println(motorRoll);

  sendNewPositionToMotors();
  
}
