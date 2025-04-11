#ifndef comunicazione_h 
#define comunicazione_h

class Comunicazione {

  public:
  Comunicazione();

  String ricevi();
  void invia(String line);

};

Comunicazione::Comunicazione() {}

String Comunicazione::ricevi() {
  while (Serial.available() == 0);  // Attendi che ci siano dati disponibili
  String data = Serial.readStringUntil('\n');  // Leggi fino a '\n'
  Serial.println("ok");  // Invia conferma a Raspberry Pi
  return data;
}

void Comunicazione::invia(String line) {

  Serial.println(line);  // Invia il messaggio a Raspberry Pi
  
  while (true) {
    while (Serial.available() > 0) {
      String response = Serial.readStringUntil('\n');
      if (response == "ok") {
        return;  // Esci quando Raspberry Pi conferma la ricezione
      }
    }
  }
}

#endif 
