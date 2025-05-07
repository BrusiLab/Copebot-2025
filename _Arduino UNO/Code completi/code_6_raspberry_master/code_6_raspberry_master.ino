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
void chiudi_L();
void posiziona_L();
void apri_leva();
void chiudi_leva();
void chiudi_ruota();

// Prototipi funzioni percorso
void inizio();
void avanza(int distanzaAvanza);
void raccogli();
void scarica();
void posiziona(int numero_blocchi);
float converti(int angolo);
void interrompiTutto();
void interrompi();
void killer();

// PINS
#define ferma_servo 7  // Interrompi corrente
#define kfcchiuso 10   // Fine corsa per L chiusa
#define kfcaperto 9    // Fine corsa per L aperta
#define LED 8          // LED pin

int blocchi_raccolti = 0;
unsigned long tempo_inizio = 0;

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

  //digitalWrite(ferma_servo, LOW);
  //apri_L();
  //chiudi_L();
  //posiziona_L();
  //digitalWrite(ferma_servo, HIGH);  // Servo spento

  digitalWrite(LED, LOW);

  //robot.set();
}

void loop() {
  //UMCM

  String comando = ricevi();

  if (comando == "j") {

    while (true) {
      blink(LED);
      if (digitalRead(kfcaperto) == HIGH) {

        tempo_inizio = millis();
        digitalWrite(LED, LOW);

        robot.set();  // inizializza stepper e giroscopio

        inizio();
        //robot.vai(1000, 2000, "avanti", "on");
        avanza(1000);
        gira1();
        //robot.vai(900, 2000, "avanti", "on");
        avanza(900);
        gira2();

        while (true) {}  // interrompi programma
      }
    }
  }
}