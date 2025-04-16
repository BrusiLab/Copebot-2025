//include librerie
#include "twoSteppers.h"  // Stepper ruote
#include "lidar.h"
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Wire.h>

// Oggetti
Robot robot = Robot();                                      // Dichiaro oggetto robot (gestione stepper)
Lidar lidar = Lidar();                                      // Dichiaro oggetto Lidar
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();  // Dichiaro oggetto driver

// Prototipi funzioni comunicazione
String ricevi();
void invia(String line);
void blink(int pin);

// Prototipi funzioni servo
void servo_set();
void apri_L();
void chiudi_L(int posizione = 1);
void apri_leva();
void chiudi_leva();

// Prototipi funzioni percorso
void inizio();
void avanza();
void raccogli();
void scarica();
void ritorna();

// PINS
#define ferma_servo 7  // Interrompi corrente
#define kfcaperto 9    // Fine corsa per L aperta
#define kfcchiuso 10   // Fine corsa per L chiusa
#define LED 8          // LED pin


void setup() {
  Serial.begin(9600);
  // Inizializzazione comunicazione I2C gyro e servo
  robot.set();
  servo_set();

  // Dichiarazione pins
  pinMode(kfcchiuso, INPUT);
  pinMode(kfcaperto, INPUT);
  pinMode(ferma_servo, OUTPUT);
  pinMode(LED, OUTPUT);

  while (!Serial) {}
}


void loop() {
  delay(35000);
  while (true) {
    avanza();
  }
  //digitalWrite(LED, HIGH);
}