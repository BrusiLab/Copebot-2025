// Funzione per far interrompere il robot allo scadere del tempo
void interrompi() {
  if (millis() >= tempo_fine) {
    Serial.println("finito");
    while (true) {}
  }
}

//=====================================================
//                    GIROSCOPIO
//=====================================================

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");
  float sum = 0;
  int n = 100; // Numero di campioni per la calibrazione
  for (int i = 0; i < n; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(3); // Piccola pausa per evitare sovraccarico I2C
  }
  gyroZ_offset = sum / n;
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZ_offset, 6);
}

float getGyroAngle() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  t1 = micros();

  // Calcolo del delta time (in secondi)
  float dt = (micros() - t0) / 1000000.0; // Converti ms in secondi

  t0 = t1;

  // Aggiornamento dell'angolo Z con integrazione
  angle += (g.gyro.z - gyroZ_offset) * dt * 57.2957795; // Converti rad/s in gradi

  Serial.println(angle);

  delay(1);

  return angle;

}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

void aggiorna_posizione_angolare(int degree, String wayTurn, String wayTravel) {

  if ((wayTravel == "ahead" && wayTurn == "right") | (wayTravel == "back" && wayTurn == "left")) {
    posizione_angolare -= degree;
  }
  if ((wayTravel == "back" && wayTurn == "right") | (wayTravel == "ahead" && wayTurn == "left")) {
    posizione_angolare += degree;
  }

  Serial.println(posizione_angolare);

}
