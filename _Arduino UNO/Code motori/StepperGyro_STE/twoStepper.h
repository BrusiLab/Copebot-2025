/*
   LIBRERIA GESTIONE MOTORI RUOTE

   libreria creata da noi per utilizzare attraverso funzioni più agevoli e
   intuitive i motori e gli spostamenti del robot

   PROBLEMA

   FUNZIONE TURN BOTH WHEELS
   IL SECONDO WHILE NON FUNZIONA E NON SO IL PERCHE'
   FUNZIONA SOLO CON I NUMERI NEGATIVI, CON QUELLI POSITIVI NO
*/

#ifndef twoStepper_h
#define twoStepper_h

//include librerie
#include "Arduino.h" //include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h> // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Definizione valori costanti
#define MICROSTEPS 4 // MICROSTEPSs per ogni passo
#define stepsPerLap 200 // passi interi per lap
#define diameterWheels 65  // diametro ruote in cm
#define distanceWheels 25   // distanza tra le ruote in cm
const int constAcc = 1;

// The DX Stepper pins
#define STEPPERDX_DIR_PIN 4
#define STEPPERDX_STEP_PIN 5
// The SX stepper pins
#define STEPPERSX_DIR_PIN 6
#define STEPPERSX_STEP_PIN 3

#define tempo_fine 208000

float setpoint = 0;
float input, output;
float Kp = 5, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;
int velocita;
int counter = 0;
float angle = 0;
unsigned long t0 = 0;

float getGyroAngle();
float PIDControl(float setpoint, float input);

// Define some steppers and the pins the will use
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN); //motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN); //motore sinistro


// Funzione per far interrompere il robot allo scadere del tempo
void interrompi() {
  if (millis() >= tempo_fine) {
    Serial.println("finito");
    while (true) {}
  }
}

// classe
class Command {
  public:
    Command();

    void set();
    void go(int lenght, int rpm, String wayTravel, String accel);
    void turn(int degree, int rpm, String wayTurn, String wayTravel);
    void turnBothWheels(int d, int rpm);
};

Command::Command() {}

float getGyroAngle() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calcolo del delta time (in secondi)
  float dt = (millis() - t0) / 1000.0; // Converti ms in secondi
  t0 = millis();

  // Aggiornamento dell'angolo Z con integrazione
  angle += g.gyro.z * dt * 57.295779513082320876798154814105; // Converti rad/s in gradi

  return angle;
}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  return Kp * error;
}


// ==================================================
//                      SETTING
// ==================================================
//tutti i valori si devono intendere come passi/secondo --> il numero di passi del motore è 3200 (200*16)
void Command::set() {
  
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));

  //inizializza mpu6050
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1) yield();
  }

  t0 = millis();

}


unsigned long stride;
int steps = 0; // variabile direzionale
int savePos;

// ==================================================
//                        GO
// ==================================================

void Command::go(int lenght, int rpm, String wayTravel, String accel) {
  
  if (wayTravel == "ahead") {
    steps = 1; // mantieni direzione invariata
  } else if (wayTravel == "back") {
    steps = -1; // inverti direzione
  }

  if (accel == "on") {
    //Serial.println("1");
    //Serial.println(abs(lenght) / (PI * diameterWheels)* stepsPerLap);
    stepperDX.setCurrentPosition(0);
    velocita = rpm * steps;
    for (int i = 300; i < rpm; i += constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(velocita);
      stepperSX.setSpeed(velocita);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos = stepperDX.currentPosition();

    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap - savePos * 2 / MICROSTEPS;
    //Serial.println(stride);
    stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) { //raggiungi posizione da lunghezza calcolata
      stepperDX.setSpeed(rpm * steps); // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 300; i -= constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps); // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos += stepperDX.currentPosition();

  } else if (accel == "off") {

    // velocità motori
    velocita = rpm * steps;

    // numero passi
    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    bool controll = true;

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) {
      if (controll == true) {
        stepperDX.setSpeed(velocita);
        stepperSX.setSpeed(velocita);
      }
      if (counter == 100) {
        input = getGyroAngle();
        setpoint = 0;
        output = PIDControl(setpoint, input);
        counter = 0;
        controll = false;
      }
      stepperDX.setSpeed(velocita + output);
      stepperSX.setSpeed(velocita - output);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }
  }
}



// ==================================================
//                      TURN
// ==================================================

//funzione per sterzare con una sola ruota (l'altra fa da perno)
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
  //converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.45329 / 1000;
  // calcola la distanza che la ruota deve percorrere
  int lenght = rad * distanceWheels;
  //calcola i passi che lo stepper deve compiere
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;

  int steps = 0; //variabile direzionale

  if (wayTravel == "ahead") {
    steps = 1;
  }
  if (wayTravel == "back") {
    steps = -1; //inverti la direzione
  }

  if (wayTurn == "right") {
    stepperSX.setCurrentPosition(0);
    
    //while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {

    float inizio = getGyroAngle();

    while(getGyroAngle() <= inizio + degree){
      interrompi();
      stepperSX.setSpeed(rpm * steps);
      stepperSX.runSpeed();
      interrompi();
    }
  }
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);
    
    //while (stepperDX.currentPosition() != (stride * MICROSTEPS)) {

    float inizio = getGyroAngle();

    while(getGyroAngle() >= inizio - degree){
      interrompi();
      stepperDX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      interrompi();
    }
  }
}

bool pregnante(int angolo) {
  //Serial.println(getGyroAngle());
  if (getGyroAngle() < angolo) {
    return true;
  } else if (getGyroAngle() > angolo) {
    return false;
  } else if (getGyroAngle() == angolo) {
    return;
  }
}

// ==================================================
//                   TURN BOTH WHEELS
// ==================================================

//funzione per sterzare con entrambe le ruote (il centro dell'asse posteriore resta fermo)
void Command::turnBothWheels(int d, int rpm) {
  bool sam = true;
  while (pregnante(d) == true) {
    Serial.println("  basso");
    stepperDX.setSpeed(-rpm);
    stepperSX.setSpeed(rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    sam = false;
  }
  if (sam == false) {
    return;
  }
  while (pregnante(d) == false) {
    Serial.println("  alto");
    stepperDX.setSpeed(rpm);
    stepperSX.setSpeed(-rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    sam = false;
  }
  if (sam == false) {
    return;
  }
}

#endif twoStepper_h
