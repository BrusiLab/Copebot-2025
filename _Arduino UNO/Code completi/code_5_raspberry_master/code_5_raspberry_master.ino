//======================================================================
//                         ATTENZIONE ATTENZIONE
//    Per avviare il robot collegare raspberry e arduino assieme tramite
//    seriale.
//    Alimentare arduino e raspberry contemporaneamente.
//    Il robot inizia la gara quando raspberry invia il comando "j" ad
//    arduino.
//======================================================================

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
void posiziona(int numero_blocchi);
float converti(int angolo);
void interrompiTutto();
void killer();

// PINS
#define ferma_servo 7  // Interrompi corrente
#define kfcchiuso 9    // Fine corsa per L chiusa
#define kfcaperto 10   // Fine corsa per L aperta
#define LED 8          // LED pin

int blocchi_raccolti = 0;

void setup() {
  Serial.begin(9600);

  // Inizializzazione comunicazione servo
  servo_set();

  // Dichiarazione pins
  pinMode(kfcaperto, INPUT);
  pinMode(kfcchiuso, INPUT);
  pinMode(ferma_servo, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);
  delay(1000);
}


void loop() {
  String comando = ricevi();
  // String comando = "j";

  if (comando == "j") {
    digitalWrite(LED, HIGH);
    while (true) {
      if (digitalRead(kfcaperta) == HIGH) {
        // inizializzazione giroscopio e motori
        robot.set();

        avanza(1000);

        while (true) {}
      }
    }
  }
}