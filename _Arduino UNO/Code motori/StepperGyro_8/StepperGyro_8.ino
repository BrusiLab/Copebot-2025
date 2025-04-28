//include librerie
#include "twoSteppers.h"  // Stepper ruote
//#include "comunicazione.h"
#include "Wire.h"  // Comunicazione I2C per sensori

Robot robot = Robot();  // Creazione oggetto master (gestione motori)

int velocita_vai = 2000;
int velocita_gira = 2000;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  robot.set();
}

void loop() {
  // true --> giroscopio attivo
  // valse --> giroscopio disattivato
  //robot.test(true);

  delay(500);

  robot.vai(500, 2000, "avanti", "on");
  delay(5000);
  robot.vai(500, 2000, "indietro", "on");
  delay(500);

  robot.vai(200, 2000, "avanti", "off");
  delay(5000);
  robot.vai(200, 2000, "indietro", "off");
  delay(500);
  
  robot.giraRuote(90, 500);
  delay(1000);
  robot.giraRuote(-90, 500);
  delay(1000);

  robot.gira(90, 500, "destra");
  delay(1000);
  robot.gira(-90, 500, "destra");
  delay(1000);
  robot.gira(90, 500, "sinistra");
  delay(1000);
  robot.gira(-90, 500, "sinistra");
  while (true) {}
}

/*
ACCELERAZIONE SOLO SOPRA 255 MM --> senza vado piano
HARDWARE INVERTITO, SX VA AVANTI CON -RPM
LA DIREZIONE DEL GIRA INDICA IL PERNO (ORARIO ANGOLI +, ANTIORARIO ANGOLI -)
*/
