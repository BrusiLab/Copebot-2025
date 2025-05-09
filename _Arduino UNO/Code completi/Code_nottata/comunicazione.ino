#define t1 200
int dt1, tp1 = 0;
unsigned long i = 0;


String ricevi() {
  while (Serial.available() == 0)
    ;                                          // Attendi che ci siano dati disponibili
  String data = Serial.readStringUntil('\n');  // Leggi fino a '\n'
  Serial.println("ok");                        // Invia conferma a Raspberry Pi
  return data;
}

//UMCM

//****************************

unsigned long tempo_invia = 0;

void invia(String line) {
  /*
  tempo_invia = millis();
  while (millis() - tempo_invia > 20) {
    Serial.println(line);  // Invia il messaggio a Raspberry Pi
  }*/
  Serial.println(line);

  tempo_invia = millis();

  bool valbool = true;

  while (true) {
    //Serial.println(line);

    if (millis() - tempo_invia > 3000 && valbool == true) {
      Serial.println(line);
      valbool = false;
    }

    while (Serial.available() > 0) {
      String response = Serial.readStringUntil('\n');
      if (response == "ok") {
        return;  // Esci quando Raspberry Pi conferma la ricezione
      }
    }
  }
}

void blink(int pin) {
  dt1 = millis() - tp1;
  if (dt1 > t1) {
    digitalWrite(pin, (i % 2 == 0) ? HIGH : LOW);
    i++;
    tp1 = millis();
  }
}