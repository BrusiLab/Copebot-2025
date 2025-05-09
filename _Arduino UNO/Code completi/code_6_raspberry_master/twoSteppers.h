#ifndef TWO_STEPPER_H
#define TWO_STEPPER_H

//#define NO_GIROSCOPIO
#define GIROSCOPIO
bool giroscopio_attivo = true;  // Variabile di stato che abilita le funzioni con il giroscopio
                                // Se il giroscopio fallisce il codice procede lo stesso senza il giroscopio

// Librerie
#include "Arduino.h"       // include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h>  // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Costanti
#define micropassi 4       // micropassi per ogni passo
#define numeroPassi 200    // Passi interi per giro
#define diametroRuote 63   // Diametro ruote in mm
#define distanzaRuote 175  // Distanza tra le ruote in mm
#define durata 208000      // Durata gara in millis
// Fattore contatore numero passi tra due rilevazioni dell'angolo
const int K_volte_misura_angolo = 50;  // Più è grande, più la possibilità che non si fermi all'angolo stabilito è maggiore
                                       // Più è piccolo, più la possibiltà che rilevi misure "false" è maggiore
const float K_angolo = 0.9865f;        // Fattore di correzione per calcolo angolo
const int K_accelerazione = 4;         // Fattore incremento accelerazione
const int delayAcc = 0;


// ===================================================================
// ==============          PINS LED - STEPPER        =================
// ===================================================================

// Pin LED errore
#define LED_ERRORE 12
// Pins motore destro
#define STEPPERDX_DIR_PIN 6
#define STEPPERDX_STEP_PIN 5
// Pins motore sinistro
#define STEPPERSX_DIR_PIN 4
#define STEPPERSX_STEP_PIN 3

// Steppers
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN);  // motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN);  // motore sinistro


// ===================================================================
// ================          VARIABILI TEST        ===================
// ===================================================================

const int velocita_motori_vai = 1500;
const int velocita_motori_gira = 600;

// ===================================================================
// =============          VARIABILI GIROSCOPIO        ================
// ===================================================================

#ifdef GIROSCOPIO
// Interrupt pin
#define INTERRUPT_PIN 2

// Variabili giroscopio
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Girscopio
MPU6050 mpu;

// Variabili PID
float setpoint = 0;
float input, output;
float Kp = 20, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;

// Variabile velocità motori
int velocita = 0;
// Contatore misura angolo
int contatore = 0;
// angolo misurato dal giroscopio
float angoloMisura = 0.0f;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Funzione giroscopio
void dmpDataReady();

// Funzione interrupt per giroscopio
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void rilevaAngolo() {
  if (!dmpReady) {
    //Serial.println("impallato");
    return;
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    // Serial.print("euler\t");
    // Serial.println(euler[0] * 180 / M_PI);
    angoloMisura = float(euler[0] * 180 / M_PI);
  }
}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}
#endif

unsigned long tempo_start = 0;

// Funzione di interruzione
void interrompi() {
  if (millis() - tempo_start >= durata) {
    // Serial.println("finito");
    while (true) {
    }
  }
}


// ===================================================================
// =================          CLASSE ROBOT        ====================
// ===================================================================

class Robot {
public:
  Robot();

  void set();
  void vai(int length, int rpm, String verso, String accelerazione);
  void gira(int angoloGradi, int rpm, String verso);
  void giraRuote(int angoloGradi, int rpm);
  void test(bool giroVal);
};


Robot::Robot() {}


// ==================================================================
// ===================          SETTING        ======================
// ==================================================================

#ifdef GIROSCOPIO
void Robot::set() {
  tempo_start = millis();
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection

  delay(100);  // delay di attesa per inizializzare la comunicazione con in gyro

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    giroscopio_attivo = true;
    return;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    giroscopio_attivo = false;
    return;
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::set() {
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);
  giroscopio_attivo = false;
  //Serial.print("ciao");
  return;
}
#endif


// ==================================================================
// ===================           VAI           ======================
// ==================================================================

unsigned long passiDaCompiere;  // Passi da compiere
int statoVerso = 0;             // Variabile direzionale
int savePos;                    // Variabile salvataggio posizione

float ampiezzaAngolo = 0.0f;
float memoriaAngoli = 0.0f;

void Robot::vai(int length, int rpm, String verso, String accelerazione) {

  if (verso == "avanti") {
    statoVerso = 1;  // mantieni direzione invariata
  } else if (verso == "indietro") {
    statoVerso = -1;  // inverti direzione
  }

  if (accelerazione == "on") {
    stepperDX.setCurrentPosition(0);
    for (int i = 200; i < rpm; i += K_accelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      delay(delayAcc);
      interrompi();
    }

    savePos = stepperDX.currentPosition();
    passiDaCompiere = (abs(length) / (PI * diametroRuote)) * numeroPassi - (abs(savePos) * 2 / micropassi);

    stepperDX.setCurrentPosition(0);                                             // imposta come 0 la posizione di partenza
    while (abs(stepperDX.currentPosition()) < (passiDaCompiere * micropassi)) {  //raggiungi posizione da lunghezza calcolata
      stepperDX.setSpeed(rpm * statoVerso);                                      // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 200; i -= K_accelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      delay(delayAcc);
      interrompi();
    }
    savePos += stepperDX.currentPosition();

  } else if (accelerazione == "off") {

    passiDaCompiere = (abs(length) / (PI * diametroRuote)) * numeroPassi;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
      // velocità motori
      stepperDX.setSpeed(rpm * statoVerso);
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
  }
}

// ==================================================================
// =============         GIRA UNA SOLA RUOTA         ================
// ==================================================================

#ifdef GIROSCOPIO  // MdB + So
void Robot::gira(int angoloGradi, int rpm, String perno) {
  if (giroscopio_attivo == true) {
    memoriaAngoli += float(angoloGradi) * K_angolo;  //diminuendo il fattore moltiplicativo l'angolo diminuisce
                                                     //il valore corretto sarebbe circa 0.9865f
    int n = abs(int(memoriaAngoli) / 180) + 2;
    float versoAngolo = 0.0f;

    if (memoriaAngoli >= 0.0f) {
      versoAngolo = 1.0f;
    }
    if (memoriaAngoli < 0.0f) {
      versoAngolo = -1.0f;
    }
    // Normalizza l'angolo tra -180 e +180
    if (n % 2 == 0) {
      ampiezzaAngolo = (float(abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    } else if (n % 2 == 1) {
      ampiezzaAngolo = (float(-180 + abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    }

    // angoloMisura = angolo misurato dal giroscipio
    while (angoloMisura < (ampiezzaAngolo - 0.4) || angoloMisura > (ampiezzaAngolo + 0.4)) {
      interrompi();
      if (contatore > 2) {
        rilevaAngolo();
        contatore = 0;
      }
      contatore++;

      if (perno == "destra") {

        if (angoloGradi > 0) {
          stepperSX.moveTo(1);
          stepperSX.setSpeed(-rpm*2);
          stepperSX.runSpeed();
        } else {
          stepperSX.moveTo(1);
          stepperSX.setSpeed(rpm*2);
          stepperSX.runSpeed();
        }
      } else if (perno == "sinistra") {

        if (angoloGradi > 0) {
          stepperDX.moveTo(1);
          stepperDX.setSpeed(-rpm*2);
          stepperDX.runSpeed();
        } else {
          stepperDX.moveTo(1);
          stepperDX.setSpeed(rpm*2);
          stepperDX.runSpeed();
        }
      }
    }
    return;
  } else if (giroscopio_attivo == false) {

    // converti misura in deg in radianti (arduino utilizza i radianti)
    float rad = abs(angoloGradi) * 17.45329 / 1000;
    // calcola la distanza che la ruota deve percorrere
    int length = rad * distanzaRuote;
    // calcola i passi che lo stepper deve compiere
    unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;

    int statoVerso = 0;  // variabile direzionale

    if (angoloGradi < 0) {
      statoVerso = 1;
    }
    if (angoloGradi > 0) {
      statoVerso = -1;  // inverti la direzione
    }

    if (perno == "destra") {
      stepperSX.setCurrentPosition(0);
      while (abs(stepperSX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
        interrompi();
        stepperSX.setSpeed(rpm * statoVerso * 2);
        stepperSX.runSpeed();
      }
    }

    if (perno == "sinistra") {
      stepperDX.setCurrentPosition(0);
      while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
        interrompi();
        stepperDX.setSpeed(rpm * statoVerso * 2);
        stepperDX.runSpeed();
      }
    }
    return;
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::gira(int angoloGradi, int rpm, String perno) {
  // converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = float(abs(angoloGradi)) * 17.45329f / 1000.0f;
  // calcola la distanza che la ruota deve percorrere
  int length = rad * distanzaRuote;
  // calcola i passi che lo stepper deve compiere
  unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;

  int statoVerso = 0;  // variabile direzionale

  if (angoloGradi < 0) {
    statoVerso = 1;
  }
  if (angoloGradi > 0) {
    statoVerso = -1;  // inverti la direzione
  }

  if (perno == "destra") {
    stepperSX.setCurrentPosition(0);
    while (abs(stepperSX.currentPosition()) <= abs(passiDaCompiere * micropassi)) {
      interrompi();
      stepperSX.setSpeed(rpm * statoVerso * 2);
      stepperSX.runSpeed();
      interrompi();
    }
  }

  if (perno == "sinistra") {
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) <= abs(passiDaCompiere * micropassi)) {
      interrompi();
      stepperDX.setSpeed(rpm * statoVerso * 2);
      stepperDX.runSpeed();
      interrompi();
    }
  }
  return;
}
#endif


// ==================================================================
// ==============        GIRA ENTRAMBE LE RUOTE        ==============
// ==================================================================

#ifdef GIROSCOPIO
void Robot::giraRuote(int angoloGradi, int rpm) {
  if (giroscopio_attivo == true) {
    memoriaAngoli += float(angoloGradi) * K_angolo;  //diminuendo il fattore moltiplicativo l'angolo diminuisce
                                                     //il valore corretto sarebbe circa K_angolo
    int n = abs(int(memoriaAngoli) / 180) + 2;
    float versoAngolo = 0.0f;

    if (memoriaAngoli >= 0.0f) {
      versoAngolo = 1.0f;
    }
    if (memoriaAngoli < 0.0f) {
      versoAngolo = -1.0f;
    }
    // Normalizza l'angolo tra -180 e +180
    if (n % 2 == 0) {
      ampiezzaAngolo = (float(abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    } else if (n % 2 == 1) {
      ampiezzaAngolo = (float(-180 + abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    }

    if (angoloGradi > 0) {
      rpm = -rpm;
    }

    // angoloMisura = angolo misurato dal giroscipio
    while (angoloMisura < (ampiezzaAngolo - 0.4) || angoloMisura > (ampiezzaAngolo + 0.4)) {
      rilevaAngolo();
      stepperDX.moveTo(1);
      stepperSX.moveTo(1);
      stepperDX.setSpeed(rpm);
      stepperSX.setSpeed(rpm);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
    return;

  } else if (giroscopio_attivo == false) {
    // conversione degrees in radianti
    float rad = abs(angoloGradi) * 17.45329 / 1000;
    // calcolo distanza necessaria da compiere per singola ruota
    int length = rad * distanzaRuote * 0.5;
    unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
    int direzione = 0;
    if (angoloGradi < 0) {
      direzione = -1;
    } else if (angoloGradi > 0) {
      direzione = 1;
    }
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) <= abs(passiDaCompiere * micropassi)) {
      stepperDX.setSpeed(rpm * direzione);
      stepperSX.setSpeed(rpm * direzione);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::giraRuote(int angoloGradi, int rpm) {
  // conversione degrees in radianti
  float rad = abs(angoloGradi) * 17.45329 / 1000;
  // calcolo distanza necessaria da compiere per singola ruota
  int length = rad * distanzaRuote * 0.5;
  unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
  int direzione = 0;
  if (angoloGradi < 0) {
    direzione = 1;
  } else if (angoloGradi > 0) {
    direzione = -1;
  }
  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) <= abs(passiDaCompiere * micropassi)) {
    stepperDX.setSpeed(rpm * direzione);
    stepperSX.setSpeed(rpm * direzione);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    interrompi();
  }
}
#endif

// ==================================================================
// =======================        TEST        =======================
// ==================================================================

void Robot::test(bool giroVal) {
  if (giroVal == true) {
    giroscopio_attivo = true;
  } else if (giroVal == false) {
    giroscopio_attivo = false;
  }
  vai(200, velocita_motori_vai, "avanti", "on");
  delay(500);
  giraRuote(90, velocita_motori_gira);
  delay(500);
  vai(200, velocita_motori_vai, "avanti", "on");
  delay(500);
  giraRuote(90, velocita_motori_gira);
  delay(500);
  vai(200, velocita_motori_vai, "avanti", "on");
  delay(500);
  giraRuote(90, velocita_motori_gira);
  delay(500);
  vai(200, velocita_motori_vai, "avanti", "on");
  delay(500);
  giraRuote(90, velocita_motori_gira);
  delay(500);
/*
  vai(200, velocita_motori_vai, "avanti", "on");
  delay(500);
  vai(200, velocita_motori_vai, "indietro", "on");
  delay(1000);
  vai(200, velocita_motori_vai, "avanti", "off");
  delay(500);
  vai(200, velocita_motori_vai, "indietro", "off");
  delay(2000);

  giraRuote(90, velocita_motori_gira);
  delay(500);
  giraRuote(-180, velocita_motori_gira);
  delay(500);
  giraRuote(90, velocita_motori_gira);
  delay(2000);

  gira(90, velocita_motori_gira, "destra");  // ruota destra fa da perno
  delay(500);
  gira(-180, velocita_motori_gira, "destra");
  delay(500);
  gira(90, velocita_motori_gira, "destra");
  delay(2000);

  gira(90, velocita_motori_gira, "sinistra");
  delay(500);
  gira(-180, velocita_motori_gira, "sinistra");
  delay(500);
  gira(90, velocita_motori_gira, "sinistra");
  delay(2000);
  */
}

#endif