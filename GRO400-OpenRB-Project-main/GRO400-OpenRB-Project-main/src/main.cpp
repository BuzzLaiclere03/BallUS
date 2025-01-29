// GRO400 - Exemple d'utilisation du OpenRB avec un moteur Dynamixel sous Platform.IO.
// Basé sur l'exemple de Position Control.
// Opère un moteur (à définir par la variable DXL_ID - 1 par défaut) en position en le faisant passer
// d'une position en pulsations (1000) à une autre en degrés (5.7) et vice-versa à chaque
// seconde.
// Écrit la position en cours en pulsations à la console série (accessible par DEBUG_SERIAL).
// N'oubliez-pas de configurer votre port série pour cette console à 115200 bauds.

#include <Dynamixel2Arduino.h>
#include <math.h>

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
 
// TODO: À changer selon l'ID de votre moteur :
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

float qA = 0.0;
float qB = 0.0;
float qC = 0.0;

float theta_x = 0 * (M_PI / 180.0);
float theta_y = 0 * (M_PI / 180.0);
float theta_z = 0 * (M_PI / 180.0);  //degree

float modMoteur1 = 0;
float modMoteur2 = 0;
float modMoteur3 = 0;


float MatriceMoteurs[3][3] = {{(cos(qC)*cos(qA) - sin(qC)*sin(qB)*sin(qA)), (cos(qC)*sin(qA) + sin(qC)*sin(qB)*cos(qA)), (-sin(qC)*cos(qB))}, {(-cos(qB)*sin(qA)), (cos(qB)*cos(qA)), (sin(qB))}, {(sin(qC)*cos(qA)+cos(qC)*sin(qB)*sin(qA)), (sin(qC)*sin(qA)-cos(qC)*sin(qB)*cos(qA)), (cos(qC)*cos(qB))}};
float MatriceAccelero[3][3] = {{(cos(theta_y)*cos(theta_z)- sin(theta_y)*sin(theta_x)*cos(theta_z)), (-cos(theta_x)*sin(theta_z)), (sin(theta_y)*cos(theta_z)+cos(theta_y)*sin(theta_x)*sin(theta_z))}, {(cos(theta_y)*sin(theta_z)+sin(theta_y)*sin(theta_x)*cos(theta_z)), (cos(theta_x)*cos(theta_z)), (sin(theta_y)*sin(theta_z)-cos(theta_y)*sin(theta_x)*cos(theta_z))}, {(-sin(theta_y)*cos(theta_x)), (sin(theta_x)), (cos(theta_y)*cos(theta_x))}};
float MatriceGimbal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

void multiplierMatrices(float A[3][3], float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void calculeModificationAngles(){
  //Lecture angle des moteurs
  qA = dxl.getPresentPosition(DXL_ID1) ;
  qB = dxl.getPresentPosition(DXL_ID1) ;
  qC = dxl.getPresentPosition(DXL_ID1) ;

  //Lecture des angles de l'accéléromètre


  multiplierMatrices(MatriceMoteurs, MatriceAccelero, MatriceGimbal);

  modMoteur1 = (MatriceGimbal[0][0] + MatriceGimbal[0][1] + MatriceGimbal[0][2])*(180/PI)- ANGLE_0_DXL_ID1;
  modMoteur2 = (MatriceGimbal[1][0] + MatriceGimbal[1][1] + MatriceGimbal[1][2])*(180/PI)- ANGLE_0_DXL_ID2;
  modMoteur3 = (MatriceGimbal[2][0] + MatriceGimbal[2][1] + MatriceGimbal[2][2])*(180/PI) - ANGLE_0_DXL_ID3;

}

void printModificationAngles() {
  Serial.println("Modification moteur 1 : ");
  Serial.println(modMoteur1);
  Serial.println("Modification moteur 2 : ");
  Serial.println(modMoteur2);
  Serial.println("Modification moteur 3 : ");
  Serial.println(modMoteur3);

}

void afficherMatrice(float matrice33[3][3]){
    for (int i = 0; i < 3; i++) {  
        for (int j = 0; j < 3; j++) {  
            Serial.print(matrice33[i][j]);
            Serial.print(" ");
        }
        Serial.println();  
    }
}


void setup() {
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
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 100);

  DEBUG_SERIAL.println("Super Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());


  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + 0, UNIT_DEGREE);
  delay(2000);


  //Lecture angle des moteurs
  qA = dxl.getPresentPosition(DXL_ID1);
  qB = dxl.getPresentPosition(DXL_ID1);
  qC = dxl.getPresentPosition(DXL_ID1);

  multiplierMatrices(MatriceMoteurs, MatriceAccelero, MatriceGimbal);

  afficherMatrice(MatriceGimbal);

}

void loop() {

  calculeModificationAngles();

  //printModificationAngles();
 
  /*dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1 + modMoteur1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2 + modMoteur2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3 + modMoteur3, UNIT_DEGREE);
  delay(2000);*/

  
  /*
  Serial.println(a);
  dxl.setGoalPosition(DXL_ID1, ANGLE_0_DXL_ID1-a, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, ANGLE_0_DXL_ID2-a, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, ANGLE_0_DXL_ID3-a, UNIT_DEGREE);
  delay(2000);*/
}
