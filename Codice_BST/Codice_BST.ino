#include "lidar.h"
#include "twoStepper.h" //stepper ruote
#include "Wire.h" //comunicazione I2C per sensori

#define t1 50          // Tempo di lampeggio
#define rileva_colore "colore"

Lidar lidar = Lidar(); //Sensore di distanza
Command robot = Command();  //creazione oggetto master (gestione motori)

String ricevi();
void invia(String line);
void blink(int pin);

void prendi_blocco();
void sposta_blocco();
void scarica_blocco();

int dt1;
long tp1;
long i = 0;
String thekolors;
int rpm = 800;

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  robot.set();
  
}

void loop() {

  if(lidar.misura()){
    invia(rileva_colore);
    thekolors = ricevi();
  }

  if(thekolors == "rosso"){
    prendi_blocco();
  }

}

//INOLTRE IL ROBOT DEVE RILEVARE LA DISTANZA MENTRE SI MUOVE
