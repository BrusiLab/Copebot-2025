//include librerie
#include "twoStepper.h" //stepper ruote
#include "Wire.h" //comunicazione I2C per sensori

Command robot = Command();  //creazione oggetto master (gestione motori)

int rpm = 800;

void setup() {
  Serial.begin(115200);
  robot.set();
}

void loop() {
  robot.go(10, rpm, "ahead", "on");
  robot.go(10, rpm, "back", "off");
  robot.turn(90, rpm, "right", "ahead");
  robot.turn(90, rpm, "left", "ahead");
  robot.turn(90, rpm, "left", "back");
  robot.turn(90, rpm, "right", "back");
  robot.turnBothWheels(90, rpm, "right");
  robot.turnBothWheels(90, rpm, "left");
  delay(50000);
}
