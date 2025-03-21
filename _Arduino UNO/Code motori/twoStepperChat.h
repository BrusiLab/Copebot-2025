/*
   LIBRERIA GESTIONE MOTORI RUOTE

   Libreria creata da noi per utilizzare attraverso funzioni più agevoli e
   intuitive i motori e gli spostamenti del robot
*/

#ifndef TWO_STEPPER_H
#define TWO_STEPPER_H

//#define GYRO_CURVA

#include "Arduino.h"
#include <AccelStepper.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Definizione valori costanti
#define MICROSTEPS 4
#define STEPS_PER_LAP 200
#define DIAMETER_WHEELS 6.6  // diametro ruote in cm
#define DISTANCE_WHEELS 16.5 // distanza tra le ruote in cm

const int CONST_ACC = 1;

// Pin per i motori stepper
#define STEPPER_DX_DIR_PIN 4
#define STEPPER_DX_STEP_PIN 5
#define STEPPER_SX_DIR_PIN 6
#define STEPPER_SX_STEP_PIN 3

// Definizione costanti PID
float posizione_angolare = 0;
float setpoint = 0;
float input, output;
float Kp = 5, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;
int velocita;
int counter = 0;
float angle = 0;
unsigned long t0, t1 = 0;
float gyroZ_offset = 0;

// Dichiarazione della classe Adafruit MPU6050
Adafruit_MPU6050 mpu;

// Dichiarazione dei motori stepper
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPER_DX_STEP_PIN, STEPPER_DX_DIR_PIN);
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPER_SX_STEP_PIN, STEPPER_SX_DIR_PIN);

// Dichiarazione delle funzioni
float getGyroAngle();
float PIDControl(float setpoint, float input);
void aggiorna_posizione_angolare(int degree, String wayTurn, String wayTravel);
void calibrateGyro();
void interrompi();

// Definizione della classe Command
class Command {
public:
    Command();
    void set();
    void go(int length, int rpm, String wayTravel, String accel);
    void turn(int degree, int rpm, String wayTurn, String wayTravel = "ahead");
    void turnBothWheels(int degree, int rpm, String wayTurn, String wayTravel = "ahead");
};

Command::Command() {}

// ==================================================
//                      SETTING
// ==================================================
void Command::set() {
    stepperDX.setMaxSpeed(4000.0);
    stepperSX.setMaxSpeed(4000.0);

    Serial.println(F("Initializing I2C devices..."));

    if (!mpu.begin()) {
        Serial.println("Sensor init failed");
        while (1) yield();
    }

    calibrateGyro();
    t0 = micros();
}

// ==================================================
//                        GO
// ==================================================
void Command::go(int length, int rpm, String wayTravel, String accel) {
    int steps = (wayTravel == "ahead") ? 1 : -1;
    velocita = rpm * steps;
    stepperDX.setCurrentPosition(0);
    stepperSX.setCurrentPosition(0);
    unsigned long stride = abs(length) / (PI * DIAMETER_WHEELS) * STEPS_PER_LAP;

    if (accel == "on") {
        for (int i = 100; i < rpm; i += CONST_ACC) {
            stepperDX.setSpeed(i * steps);
            stepperSX.setSpeed(i * steps);
            stepperDX.runSpeed();
            stepperSX.runSpeed();
        }
    }

    while (abs(stepperDX.currentPosition()) < (stride * MICROSTEPS)) {
        if (counter == 100) {
            output = PIDControl(posizione_angolare, getGyroAngle());
            counter = 0;
        }
        stepperDX.setSpeed(velocita + output);
        stepperSX.setSpeed(velocita - output);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
        counter++;
    }

    if (accel == "on") {
        for (int i = rpm; i > 100; i -= CONST_ACC) {
            stepperDX.setSpeed(i * steps);
            stepperSX.setSpeed(i * steps);
            stepperDX.runSpeed();
            stepperSX.runSpeed();
        }
    }
}

// ==================================================
//                      TURN
// ==================================================
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
    aggiorna_posizione_angolare(degree, wayTurn, wayTravel);
    float rad = abs(degree) * 17.453292519943 / 1000;
    int length = rad * DISTANCE_WHEELS;
    unsigned long stride = abs(length) / (PI * DIAMETER_WHEELS) * STEPS_PER_LAP;
    int steps = (wayTravel == "ahead") ? 1 : -1;
    velocita = rpm * steps;

    if (wayTurn == "right") {
        stepperSX.setCurrentPosition(0);
        while (abs(stepperSX.currentPosition()) < (stride * MICROSTEPS)) {
            interrompi();
            stepperSX.setSpeed(velocita);
            stepperDX.setSpeed(2);
            stepperSX.runSpeed();
            stepperDX.runSpeed();
        }
    } else if (wayTurn == "left") {
        stepperDX.setCurrentPosition(0);
        while (abs(stepperDX.currentPosition()) < (stride * MICROSTEPS)) {
            interrompi();
            stepperDX.setSpeed(velocita);
            stepperSX.setSpeed(2);
            stepperDX.runSpeed();
            stepperSX.runSpeed();
        }
    }
}

// ==================================================
//                   TURN BOTH WHEELS
// ==================================================
void Command::turnBothWheels(int degree, int rpm, String wayTurn, String wayTravel) {
    aggiorna_posizione_angolare(degree, wayTurn, wayTravel);
    float rad = abs(degree) * 17.453292519943 / 1000;
    int length = rad * DISTANCE_WHEELS / 2;
    unsigned long stride = abs(length) / (PI * DIAMETER_WHEELS) * STEPS_PER_LAP;
    int steps = (wayTravel == "ahead") ? 1 : -1;
    
    if (wayTurn == "right") {
        stepperSX.setCurrentPosition(0);
        while (abs(stepperSX.currentPosition()) < (stride * MICROSTEPS)) {
            interrompi();
            stepperSX.setSpeed(rpm * steps);
            stepperDX.setSpeed(-rpm * steps);
            stepperSX.runSpeed();
            stepperDX.runSpeed();
        }
    } else if (wayTurn == "left") {
        stepperDX.setCurrentPosition(0);
        while (abs(stepperDX.currentPosition()) < (stride * MICROSTEPS)) {
            interrompi();
            stepperDX.setSpeed(rpm * steps);
            stepperSX.setSpeed(-rpm * steps);
            stepperDX.runSpeed();
            stepperSX.runSpeed();
        }
    }
}

#endif // TWO_STEPPER_H
