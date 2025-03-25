/*
  PID = Proporzionale Integrale Derivativo
  Funzione per il calcolo dell'errore
*/

#ifndef TWO_STEPPER_H
#define TWO_STEPPER_H

// #define FUNZIONI_SENZA_GYRO

// Librerie
#include "Arduino.h"       //include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h>  // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Costanti
#define MICROSTEPS 4        // MICROSTEPS per ogni passo
#define stepsPerLap 200     // Passi interi per giro
#define diameterWheels 68   // Diametro ruote in mm
#define distanceWheels 172  // Distanza tra le ruote in mm
#define tempo_fine 208000   // Durata gara
const int constAcc = 1;

// Pins motore destro
#define STEPPERDX_DIR_PIN 6
#define STEPPERDX_STEP_PIN 5
// Pins motore sinistro
#define STEPPERSX_DIR_PIN 4
#define STEPPERSX_STEP_PIN 3
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Funzione giroscopio
void dmpDataReady();

// Steppers
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN);  // motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN);  // motore sinistro

// Girscopio
MPU6050 mpu;

// Variabili PID
float setpoint = 0;
float input, output;
float Kp = 20, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;
// Variabile velocità motori
int velocita;
// Variabile contatore
int counter = 0;

// Funzione calcolo angoli giroscopio
void getGyroAngle();
// Funzione PID
float PIDControl(float setpoint, float input);

// Funzione di interruzione
void interrompi() {
  if (millis() >= tempo_fine) {
    Serial.println("finito");
    while (true) {
    }
  }
}

// Funzione interrupt per giroscopio
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// classe Command
class Command {
public:
  Command();

  void set();
  void go(int lenght, int rpm, String wayTravel, String accel);
  void turn(int degree, int rpm, String wayTurn, String wayTravel);
  void Command::turnBothWheels(int degree, int rpm, String wayTurn);
};

Command::Command() {}

// angolo misurato dal giroscopio
float angoloMisura = 0;

void getGyroAngle() {
  if (!dmpReady)
    return;
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

// ==================================================
//                      SETTING
// ==================================================

void Command::set() {
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  delay(100);  // delay di attesa per inizializzare la comunicazione con in gyro

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
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
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

unsigned long stride;  // Passi da compiere
int steps = 0;         // Variabile direzionale
int savePos;           // Variabile salvataggio posizione

// ==================================================
//                        GO
// ==================================================

void Command::go(int lenght, int rpm, String wayTravel, String accel) {
  // Angolo da utilizzare come riferimento per andare dritti
  getGyroAngle();
  float angoloSet = angoloMisura;
  if (wayTravel == "ahead") {
    steps = 1;  // mantieni direzione invariata
  } else if (wayTravel == "back") {
    steps = -1;  // inverti direzione
  }

  // ============== ACCELERAZIONE ==============
  if (accel == "on") {
    // Imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    // Velocità motori a regime
    // Velocità negativa o positiva in base a steps
    velocita = rpm * steps;

    // ---------- Accelerazione ----------
    // La variabile "i" gestisce incrementa la velocità ogni microstep
    for (int i = 100; i < rpm; i += constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps);
      stepperSX.setSpeed(-i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }

    // Salva la posizione attuale
    savePos = abs(stepperDX.currentPosition());
    // Calcola i passi che lo stepper deve compiere
    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap - savePos * 2 / MICROSTEPS;
    // Imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    stepperSX.setCurrentPosition(0);
    // Imposta la correzione dell'errore a 0
    output = 0;
    int passiValore = (stride * MICROSTEPS * 2);
    // ---------- Movimento a velcoità costante ----------
    while ((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())) < passiValore) {
      // Calcola l'errore ogni 100 microsteps
      if (counter > 100) {
        getGyroAngle();
        input = angoloMisura;
        output = 0;
        setpoint = angoloSet;
        output = PIDControl(setpoint, input);
        counter = 0;
        //Serial.println((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())));
      }
      stepperDX.setSpeed(velocita - output);
      stepperSX.setSpeed(-1 * (velocita + output));
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      // Incrementa il contatore
      counter++;
    }

    // ---------- Decelerazione ----------
    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 100; i -= constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps);
      stepperSX.setSpeed(-i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    // savePos += stepperDX.currentPosition();

    // ============== NO ACCELERAZIONE ==============
  } else if (accel == "off") {
    // velocità motori
    velocita = rpm * steps;
    // numero passi
    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    stepperSX.setCurrentPosition(0);
    // Imposta output PID a 0
    output = 0;
    counter = 0;

    // raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition()) != (stride * MICROSTEPS * 2)) {
      // Calcola errore ogni 100 microsteps
      if (counter == 100) {
        getGyroAngle();
        input = angoloMisura;
        setpoint = angoloSet;
        output = PIDControl(setpoint, input);
        counter = 0;
      }
      stepperDX.setSpeed(velocita - output);
      stepperSX.setSpeed(-velocita - output);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }
  }
}

// ==================================================
//                      TURN
// ==================================================

// funzione per sterzare con una sola ruota (l'altra fa da perno)
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
  // converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.45329 / 1000;
  // calcola la distanza che la ruota deve percorrere
  int lenght = rad * distanceWheels;
  // calcola i passi che lo stepper deve compiere
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;

  int steps = 0;  // variabile direzionale

  if (wayTravel == "ahead") {
    steps = -1;
  }
  if (wayTravel == "back") {
    steps = 1;  // inverti la direzione
  }

  if (wayTurn == "right") {
    stepperSX.setCurrentPosition(0);
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {
      interrompi();
      stepperSX.setSpeed(rpm * steps);
      stepperSX.runSpeed();
      interrompi();
    }
  }
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) {
      interrompi();
      stepperDX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      interrompi();
    }
  }
  return;
}

// ==================================================
//                   TURN BOTH WHEELS
// ==================================================
float memoriaAngoli = 0.0f;
float valueAngolo = 0.0f;

void Command::turnBothWheels(int degree, int rpm, String wayTurn) {
  float floatDegree = float(degree) * 0.9865f; //diminuendo il fattore moltiplicativo l'angolo diminuisce
                                             //il valore corretto sarebbe circa 0.986f
  memoriaAngoli += floatDegree;

  int n = abs(memoriaAngoli / 180) + 2;

  // Normalizza l'angolo tra -180 e +180
  if (n % 2 == 0) {
    valueAngolo = float(abs(int(memoriaAngoli) % 180)) + 0.3f;
  } else if (n % 2 == 1) {
    valueAngolo = float(-180 + abs(int(memoriaAngoli) % 180)) + 0.3f;
  }

  if (wayTurn == "left") {
    rpm = -rpm;
  }

  Serial.println(valueAngolo);
  // angoloMisura = angolo misurato dal giroscipio
  while (angoloMisura < (valueAngolo - 0.4) || angoloMisura > (valueAngolo + 0.4)) {
    getGyroAngle();
    //Serial.print(valueAngolo);
    //Serial.print("\t");
    //Serial.println(angoloMisura);
    stepperDX.moveTo(1);
    stepperSX.moveTo(1);
    // Senso orario
    stepperDX.setSpeed(-rpm);
    stepperSX.setSpeed(-rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
  }
  return;
}


#ifdef FUNZIONI_SENZA_GYRO
void Command::turnBothWheels(int degree, int rpm) {
  // conversione degrees in radianti
  float rad = abs(degree) * 17.45329 / 1000;
  // calcolo distanza necessaria da compiere per singola ruota
  int lenght = rad * distanceWheels * 0.5;
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;
  int direzione = 0;
  if (degree < 0) {
    direzione = -1;
  } else if (degree > 0) {
    direzione = 1;
  }
  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) {
    stepperDX.setSpeed(rpm * direzione);
    stepperSX.setSpeed(rpm * direzione);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    stepperDX.stop();
    stepperSX.stop();
    interrompi();
  }
}
#endif

#endif TWO_STEPPER_H