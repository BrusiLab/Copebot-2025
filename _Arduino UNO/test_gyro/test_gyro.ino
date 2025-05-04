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
void killer();

// PINS
#define ferma_servo 7  // Interrompi corrente
#define kfcchiuso 10   // Fine corsa per L chiusa
#define kfcaperto 9    // Fine corsa per L aperta
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

  digitalWrite(ferma_servo, LOW);  
  posiziona_L();
  digitalWrite(ferma_servo, HIGH);  // Servo spento

  digitalWrite(LED, LOW);

  robot.set();
  delay(100);
  if(giroscopio_attivo == true){
    digitalWrite(LED, HIGH);
  }

  delay(1000);
}

#define velocitaBassa 300  //non meno di 300
#define velocitaAlta 2000  //usata dovunque si può usare (e non solo), MASSIMA 2500
#define velocitaMedia 1200
#define velocitaGiro 900  //meno di 1000


void loop() {
  
  robot.gira(90, 500, "destra");
  delay(3000);
  robot.gira(-90, 500, "destra");
  delay(3000);
  robot.gira(-90, 500, "sinistra");
  delay(3000);
  robot.gira(90, 500, "sinistra");
  delay(3000);

  robot.giraRuote(90, 500);
  delay(3000);
  robot.giraRuote(-90, 500);
  delay(3000);

  /*delay(500);
  robot.vai(50, velocitaBassa, "avanti", "off");
  robot.vai(250, velocitaAlta, "avanti", "on");
  robot.gira(90, velocitaGiro, "destra");
  robot.vai(900, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(360, velocitaAlta, "avanti", "on");
  robot.vai(1000, 2000, "avanti", "on");
  robot.vai(300, velocitaMedia, "avanti", "off");
  robot.giraRuote(90, velocitaGiro);
  robot.vai(100, velocitaMedia, "avanti", "off");
  robot.gira(-90, velocitaGiro, "sinistra");
  //scarica();  //SCARICA
  delay(2000);
  robot.vai(330, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(580, velocitaAlta, "avanti", "on");
  robot.gira(-90, velocitaGiro, "sinistra");
  robot.vai(200, velocitaAlta, "avanti", "on");
  robot.vai(900, 2000, "avanti", "on");
  robot.gira(90, velocitaGiro, "destra");
  robot.vai(50, velocitaAlta, "avanti", "off");
  robot.giraRuote(-90, velocitaGiro);
  //robot.vai(50, velocitaMedia, "avanti", "off");
  //scarica();
  delay(2000);
  robot.vai(650, velocitaAlta, "avanti", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(250, velocitaAlta, "avanti", "on");
  //scarica();
  delay(2000);
  robot.vai(300, velocitaAlta, "indietro", "on");
  robot.giraRuote(-90, velocitaGiro);
  robot.vai(250, velocitaAlta, "indietro", "on");
  */

}