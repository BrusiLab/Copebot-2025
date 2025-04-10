/*
  PID = Proporzionale Integrale Derivativo
  Funzione per il calcolo dell'errore
*/

#ifndef ROBOT_H
#define ROBOT_H

//#define NO_GIROSCOPIO
#define GIROSCOPIO

// Librerie
#include "Arduino.h"       //include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h>  // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Costanti
#define micropassi 4       // micropassi per ogni passo
#define numeroPassi 200    // Passi interi per giro
#define diametroRuote 68   // Diametro ruote in mm
#define distanzaRuote 172  // Distanza tra le ruote in mm
#define durata 208000      // Durata gara in millis
#define PIN_BLINK 7
const int incrementoAccelerazione = 1;

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
// Variabile contatore
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

#endif


// ===================================================================
// =================   FUNZIONE DI INTERRUZIONE   ====================
// ===================================================================

void interrompi() {
  if (millis() >= durata) {
    Serial.println("finito");
    while (true) {
    }
  }
}

void blink(int pin);

// ===================================================================
// =================          CLASSE ROBOT        ====================
// ===================================================================

class Robot {
public:
  Robot();

  void set();
  void vai(int lenght, int rpm, String verso, String accelerazione);
  void gira(int AngoloGradi, int rpm, String direzione, String verso);
#ifdef GIROSCOPIO
  void giraRuote(int AngoloGradi, int rpm, String direzione);
#endif
#ifdef NO_GIROSCOPIO
  void giraRuote(int AngoloGradi, int rpm);
#endif

  String ricevi();
  void invia(String line);
};

Robot::Robot() {}


// ==================================================================
// ===================          SETTING        ======================
// ==================================================================

#ifdef GIROSCOPIO

void Robot::set() {
  Serial.begin(9600);
  while (!Serial)
    ;

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

#endif
#ifdef NO_GIROSCOPIO

void Robot::set() {
  Serial.begin(9600);
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);
}

#endif


// ==================================================================
// ===================           VAI           ======================
// ==================================================================

unsigned long passiDaCompiere;  // Passi da compiere
int statoVerso = 0;             // Variabile direzionale
int savePos;                    // Variabile salvataggio posizione

#ifdef GIROSCOPIO

void Robot::vai(int lenght, int rpm, String verso, String accelerazione) {
  // Angolo da utilizzare come riferimento per andare dritti
  rilevaAngolo();
  float angoloSet = angoloMisura;

  if (verso == "avanti") {
    statoVerso = 1;  // mantieni direzione invariata
  } else if (verso == "indietro") {
    statoVerso = -1;  // inverti direzione
  }

  // ============== ACCELERAZIONE ==============
  if (accelerazione == "on") {
    // Imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    // Velocità motori a regime
    // Velocità negativa o positiva in base a statoVerso
    velocita = rpm * statoVerso;

    // ---------- Accelerazione ----------
    // La variabile "i" gestisce incrementa la velocità ogni microstep
    for (int i = 100; i < rpm; i += incrementoAccelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }

    // Salva la posizione attuale
    savePos = abs(stepperDX.currentPosition());
    // Calcola i passi che lo stepper deve compiere
    passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi - savePos * 2 / micropassi;
    // Imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    stepperSX.setCurrentPosition(0);
    // Imposta la correzione dell'errore a 0
    output = 0;
    long passiValore = (passiDaCompiere * micropassi * 2);
    // ---------- Movimento a velcoità costante ----------
    while ((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())) < passiValore) {
      // Calcola l'errore ogni 100 micropassi
      if (contatore > 100) {
        rilevaAngolo();
        input = angoloMisura;
        output = 0;
        setpoint = angoloSet;
        output = PIDControl(setpoint, input);
        contatore = 0;
        //Serial.println((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())));
      }
      stepperDX.setSpeed(velocita - output);
      stepperSX.setSpeed(-1 * (velocita + output));
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      // Incrementa il contatore
      contatore++;
      interrompi();
    }

    // ---------- Decelerazione ----------
    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 100; i -= incrementoAccelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
    // savePos += stepperDX.currentPosition();

    // ============== NO ACCELERAZIONE ==============
  } else if (accelerazione == "off") {
    // velocità motori
    velocita = rpm * statoVerso;
    // numero passi
    passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    stepperSX.setCurrentPosition(0);
    // Imposta output PID a 0
    output = 0;
    contatore = 0;

    long passiValore = (passiDaCompiere * micropassi * 2);
    // raggiungi posizione da lunghezza calcolata
    while (labs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition()) < passiValore) {
        // Calcola errore ogni 100 micropassi
        if (contatore == 100) {
          rilevaAngolo();
          input = angoloMisura;
          setpoint = angoloSet;
          output = PIDControl(setpoint, input);
          contatore = 0;
        }
        stepperDX.setSpeed(velocita - output);
        stepperSX.setSpeed(-velocita - output);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
        interrompi();
        contatore++;
      }
  }
}

#endif
#ifdef NO_GIROSCOPIO

void Robot::vai(int lenght, int rpm, String verso, String accelerazione) {
  if (verso == "avanti") {
    statoVerso = 1;  // mantieni direzione invariata
  } else if (verso == "indietro") {
    statoVerso = -1;  // inverti direzione
  }

  if (accelerazione == "on") {
    //Serial.println("1");
    //Serial.println(abs(lenght) / (PI * diametroRuote)* numeroPassi);
    stepperDX.setCurrentPosition(0);
    for (int i = 300; i < rpm; i += incrementoAccelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
    savePos = stepperDX.currentPosition();

    passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi - savePos * 2 / micropassi;
    //Serial.println(passiDaCompiere);
    stepperDX.setCurrentPosition(0);                                              // imposta come 0 la posizione di partenza
    while (abs(stepperDX.currentPosition()) != (passiDaCompiere * micropassi)) {  //raggiungi posizione da lunghezza calcolata
      stepperDX.setSpeed(rpm * statoVerso);                                       // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 300; i -= incrementoAccelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
    savePos += stepperDX.currentPosition();

  } else if (accelerazione == "off") {

    // numero passi
    passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) != (passiDaCompiere * micropassi)) {
      // velocità motori
      stepperDX.setSpeed(rpm * statoVerso);
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
  }
}

#endif


// ==================================================
//                      TURN
// ==================================================

#ifdef GIROSCOPIO

#endif

#ifdef NO_GIROSCOPIO

// funzione per sterzare con una sola ruota (l'altra fa da perno)
void Robot::gira(int AngoloGradi, int rpm, String direzione, String verso) {
  // converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(AngoloGradi) * 17.45329 / 1000;
  // calcola la distanza che la ruota deve percorrere
  int lenght = rad * distanzaRuote;
  // calcola i passi che lo stepper deve compiere
  unsigned long passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi;

  int statoVerso = 0;  // variabile direzionale

  if (verso == "avanti") {
    statoVerso = -1;
  }
  if (verso == "indietro") {
    statoVerso = 1;  // inverti la direzione
  }

  if (direzione == "destra") {
    stepperSX.setCurrentPosition(0);
    while (abs(stepperSX.currentPosition()) != (passiDaCompiere * micropassi)) {
      stepperSX.setSpeed(rpm * statoVerso);
      stepperSX.runSpeed();
      interrompi();
    }
  }
  if (direzione == "sinistra") {
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) != (passiDaCompiere * micropassi)) {
      stepperDX.setSpeed(rpm * statoVerso);
      stepperDX.runSpeed();
      interrompi();
    }
  }
  return;
}

#endif


// ==================================================
//                   TURN BOTH WHEELS
// ==================================================

#ifdef GIROSCOPIO

float memoriaAngoli = 0.0f;

void Robot::giraRuote(int AngoloGradi, int rpm, String direzione) {
  float ampiezzaAngolo = 0.0f;
  memoriaAngoli += float(AngoloGradi) * 0.9865f;  //diminuendo il fattore moltiplicativo l'angolo diminuisce
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

  if (direzione == "sinistra") {
    rpm = -rpm;
  }

  Serial.println(ampiezzaAngolo);
  // angoloMisura = angolo misurato dal giroscipio
  while (angoloMisura < (ampiezzaAngolo - 0.4) || angoloMisura > (ampiezzaAngolo + 0.4)) {
    rilevaAngolo();
    stepperDX.moveTo(1);
    stepperSX.moveTo(1);
    // Senso orario
    stepperDX.setSpeed(-rpm);
    stepperSX.setSpeed(-rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    interrompi();
  }
  return;
}

#endif
#ifdef NO_GIROSCOPIO

void Robot::giraRuote(int AngoloGradi, int rpm) {
  // conversione degrees in radianti
  float rad = abs(AngoloGradi) * 17.45329 / 1000;
  // calcolo distanza necessaria da compiere per singola ruota
  int lenght = rad * distanzaRuote * 0.5;
  unsigned long passiDaCompiere = abs(lenght) / (PI * diametroRuote) * numeroPassi;
  int direzione = 0;
  if (AngoloGradi < 0) {
    direzione = -1;
  } else if (AngoloGradi > 0) {
    direzione = 1;
  }
  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) != (passiDaCompiere * micropassi)) {
    stepperDX.setSpeed(rpm * direzione);
    stepperSX.setSpeed(rpm * direzione);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    interrompi();
  }
}


// ===================================================================
// =================          COMUNICAZIONE        ===================
// ===================================================================

int dt1;
long tp1;
long in = 0;

String Robot::ricevi() {
  while (Serial.available() == 0) {
    blink(PIN_BLINK);
    interrompi();
  }                                            // Attendi che ci siano dati disponibili
  String data = Serial.readStringUntil('\n');  // Leggi fino a '\n'
  Serial.println("ok");                        // Invia conferma a Raspberry Pi
  return data;
}

void Robot::invia(String line) {
  Serial.println(line);  // Invia il messaggio a Raspberry Pi

  while (true) {
    while (Serial.available() > 0) {
      String response = Serial.readStringUntil('\n');
      blink(PIN_BLINK);
      if (response == "ok") {
        return;  // Esci quando Raspberry Pi conferma la ricezione
      }
      interrompi();
    }
  }
}

#define t1 50  // Tempo di lampeggio

void blink(int pin) {
  dt1 = millis() - tp1;
  if (dt1 > t1) {
    digitalWrite(pin, (in % 2 == 0) ? HIGH : LOW);
    in++;
    tp1 = millis();
  }
}

#endif
#endif