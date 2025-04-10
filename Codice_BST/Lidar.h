#ifndef lidar_h     //max 11 cm
#define lidar_h

#define pinlidar 5  //pin a cui è collegato il sensore

int16_t tempo;      //tempo di volo della luce
int distanza;       //distanza rilevata

class Lidar {

  public:
  Lidar();

  bool misura();

};

Lidar::Lidar() {}

bool Lidar::misura(){

  tempo = pulseIn(pinlidar, HIGH);    //rileva tempo di volo
  
  if(tempo > 1800 || tempo == 0){     //se intervallo non è ragionevole (1400)
    distanza = -1;
  } else {                            //se intervallo ragionevole
    distanza = (tempo - 1000) * 1/4;  //calcola la distanza (3/4)
    distanza = distanza/10;           //distanza in cm
  }

  if(distanza >= 0 && distanza < 5){
    return true;
  } else {
    return false;
  }

  return distanza;
  
}

#endif 
