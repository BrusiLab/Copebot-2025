#include "lidar.h"

Lidar lidar = Lidar();

void setup() {

  Serial.begin(115200);
    
}

void loop() {
  
  Serial.println(lidar.misura());
  
}
