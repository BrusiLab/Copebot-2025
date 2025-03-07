#include "lidar.h"

#define t1 50          // Tempo di lampeggio
#define rileva_colore "colore"

Lidar lidar = Lidar(); //Sensore di distanza

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

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
}

void loop() {

  if(lidar.misura() >= 0 && lidar.misura() < 5){
    invia(rileva_colore);
    thekolors = ricevi();
  }

  if(thekolors == "rosso"){
    prendi_blocco();
  }

}
