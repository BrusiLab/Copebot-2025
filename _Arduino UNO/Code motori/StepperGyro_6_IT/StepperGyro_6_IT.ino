//include librerie
#include "twoSteppers.h"  //stepper ruote
#include "Wire.h"        //comunicazione I2C per sensori

Robot robot = Robot();  //creazione oggetto robot

int rpm = 1000;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  robot.set();
}

void loop() {
  //le distanze sono da inserire in millimetri
}
