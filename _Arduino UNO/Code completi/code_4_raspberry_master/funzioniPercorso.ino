#define velocitaAvanza 400
#define distanzaAvanza 500  //mm
unsigned int avanzaCM = 0;


void inizio() {
}


void avanza() {
  robot.vai(10, velocitaAvanza, "avanti", "off");
  int misuraCM = lidar.misura();

  if (misuraCM < 4) {
    robot.vai(40, velocitaAvanza, "avanti", "off");

    while (true) {

      invia("rileva");
      String colore = ricevi();

      if (colore == "rosso" || colore == "giallo") {
        digitalWrite(LED, HIGH);
        invia("nothing");
        raccogli();
        digitalWrite(LED, LOW);
        delay(50);
      }

      break;
    }

    robot.vai(50, velocitaAvanza, "avanti", "off");
    misuraCM = lidar.misura();
  }
}


void raccogli() {
  delay(100);
  apri_leva();
  delay(500);
  apri_L();
  robot.vai(100, velocitaAvanza, "avanti", "off");
  delay(500);
  chiudi_leva();
  delay(500);
  chiudi_L();
  delay(500);
  apri_leva();
  blocchi_raccolti ++;
  posiziona(blocchi_raccolti);
}

