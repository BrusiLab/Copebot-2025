/*
   LIBRERIA GESTIONE MOTORI RUOTE

   libreria creata da noi per utilizzare attraverso funzioni più agevoli e
   intuitive i motori e gli spostamenti del robot
*/

#ifndef twoStepper_h
#define twoStepper_h

#define Gyro_curva

float posizione_angolare = 0;

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
float gyroZ_offset = 0; // Offset iniziale

float getGyroAngle();
float PIDControl(float setpoint, float input);
void aggiorna_posizione_angolare(int degree, String wayTurn, String wayTravel);
void calibrateGyro();

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
    void turnBothWheels(int degree, int rpm, String wayTurn, String wayTravel = "ahead");
};

Command::Command() {}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");
  float sum = 0;
  int n = 100; // Numero di campioni per la calibrazione
  for (int i = 0; i < n; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(3); // Piccola pausa per evitare sovraccarico I2C
  }
  gyroZ_offset = sum / n;
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZ_offset, 6);
}

float getGyroAngle() {
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calcolo del delta time (in secondi)
  float dt = (micros() - t0) / 100000.0; // Converti ms in secondi
  t0 = micros();

  // Aggiornamento dell'angolo Z con integrazione
  angle += (g.gyro.z - gyroZ_offset) * dt * 57.295779513082320876798154814105; // Converti rad/s in gradi

  return angle;
}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void aggiorna_posizione_angolare(int degree, String wayTurn, String wayTravel){

  if((wayTravel == "ahead" && wayTurn == "right") | (wayTravel == "back" && wayTurn == "left")) {
    posizione_angolare += degree;
  }
  if ((wayTravel == "back" && wayTurn == "right") | (wayTravel == "ahead" && wayTurn == "left")) {
    posizione_angolare -= degree;
  }
  
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

  calibrateGyro(); // Calibrazione iniziale

  t0 = micros();

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
      stepperDX.setSpeed(i);
      stepperSX.setSpeed(i);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos = abs(stepperDX.currentPosition());

    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap - savePos * 2 / MICROSTEPS;
    //Serial.println(stride);
    
    stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza
    bool controll = true;

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) {
      if (controll == true) {
        stepperDX.setSpeed(velocita);
        stepperSX.setSpeed(velocita);
      }
      if (counter == 100) {
        output = PIDControl(posizione_angolare, getGyroAngle()); //imposto setpoint e input
        counter = 0;
        controll = false;
      }
      stepperDX.setSpeed(velocita + output); //correggo le velocità in base a errore
      stepperSX.setSpeed(velocita - output);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }

    counter = 0;

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
        output = PIDControl(posizione_angolare, getGyroAngle()); //imposto setpoint e input
        counter = 0;
        controll = false;
      }
      stepperDX.setSpeed(velocita + output); //correggo le velocità in base a errore
      stepperSX.setSpeed(velocita - output);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }

    counter = 0;
  }
}

// ==================================================
//                      TURN
// ==================================================

//funzione per sterzare con una sola ruota (l'altra fa da perno)
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
  
  aggiorna_posizione_angolare(degree, wayTurn, wayTravel);
  
  //converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.453292519943 / 1000;
  
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

#ifdef Gyro_curva

    while(true){
      if(posizione_angolare - 0.5 <= getGyroAngle() <= posizione_angolare + 0.5){
        break;
      }

#else
      
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {

#endif

      interrompi();
      stepperSX.setSpeed(rpm * steps);
      stepperSX.runSpeed();
      interrompi();
    }
  }

  
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);

    #ifdef Gyro_curva

    while(true){
      if(posizione_angolare - 0.5 <= getGyroAngle() <= posizione_angolare + 0.5){
        break;
      }

#else
      
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {

#endif
    
      interrompi();
      stepperDX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      interrompi();
    }
  }
}

// ==================================================
//                   TURN BOTH WHEELS
// ==================================================

//funzione per sterzare con entrambe le ruote (il centro dell'asse posteriore resta fermo)
void Command::turnBothWheels(int degree, int rpm, String wayTurn, String wayTravel = "ahead") {
  
  aggiorna_posizione_angolare(degree, wayTurn, wayTravel);

  //converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.453292519943 / 1000;
  
  // calcola la distanza che la ruota deve percorrere
  int lenght = rad * distanceWheels/2;
  
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

#ifdef Gyro_curva

    while(true){
      if(posizione_angolare - 0.5 <= getGyroAngle() <= posizione_angolare + 0.5){
        break;
      }

#else
      
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {

#endif

      interrompi();
      stepperSX.setSpeed(rpm * steps);
      stepperDX.setSpeed(-rpm * steps);
      stepperSX.runSpeed();
      stepperDX.runSpeed();
      interrompi();
    }
  }

  
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);

    #ifdef Gyro_curva

    while(true){
      if(posizione_angolare - 0.5 <= getGyroAngle() <= posizione_angolare + 0.5){
        break;
      }

#else
      
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {

#endif
    
      interrompi();
      stepperDX.setSpeed(rpm * steps);
      stepperSX.setSpeed(-rpm * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
  }
}

#endif twoStepper_h
