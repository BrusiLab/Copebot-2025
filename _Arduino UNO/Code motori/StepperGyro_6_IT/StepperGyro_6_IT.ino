//include librerie
#include "twoSteppers.h"  //stepper ruote
#include "Wire.h"        //comunicazione I2C per sensori

Robot robot = Robot();  //creazione oggetto master (gestione motori)

int rpm = 1000;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  robot.set();
}

void loop() {
  //le distanze sono da inserire in millimetri
  /*
  robot.go(500, rpm, "ahead", "off");
  robot.turn(90, rpm, "right", "ahead");
  delay(1000);
  robot.turn(90, rpm, "left", "ahead");
  delay(1000);
  robot.turn(90, rpm, "right", "back");
  delay(1000);
  robot.turn(90, rpm, "left", "back");
  delay(1000);
  robot.go(500, rpm, "ahead", "on");
  delay(1000);
  robot.go(500, rpm, "back", "on");
  delay(1000);
  robot.turnBothWheels(90, rpm);
  delay(1000);
  robot.turnBothWheels(0, rpm);
  delay(1000);
  */
  delay(1000);
  robot.vai(200, 500, "avanti", "on");
  delay(1000);
  robot.gira(90, rpm, "destra");
  //robot.go(200, 500, "ahead", "on");
  //delay(1000);
}
