from picamera2 import Picamera2
from time import sleep
import numpy as np 
import cv2 

path0 = '/home/pi/Desktop/Raspberry/Catania_2025/image0.jpg'  # Percorso in cui salvare la foto
path1 = '/home/pi/Desktop/Raspberry/Catania_2025/image1.jpg'  # Percorso in cui salvare la foto

color = "Undefined"
a = 1
mean_color = [0, 0, 0]

camera = Picamera2()
camera_config = camera.create_still_configuration({"size": (1920, 1080)})  # Risoluzionw 1920x1080
camera.configure(camera_config)

def elabora_immagine(img0, img1):
   
   global color  # Aggiunto per aggiornare la variabile globale color
   
   img0 = cv2.convertScaleAbs(img0, 1, 1) 
   img1 = cv2.convertScaleAbs(img1, 1, 1) 
   
   hsv_frame0 = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)
   hsv_frame1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
   
   height, width, _ = img0.shape

   # Coordinate del centro
   cx = int(width / 2)
   cy = int(height / 2)

   # Definire una regione 100x100 attorno al centro
   roi_size = 50  # Metà dimensione della ROI
   roi0 = hsv_frame0[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi1 = hsv_frame1[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

   # Calcolare il colore medio nella ROI
   mean_color0 = cv2.mean(roi0)[:3]  # Ignorare il canale alpha
   mean_color1 = cv2.mean(roi1)[:3]
   
   for i in range (0,3,+1):
      mean_color[i] = (mean_color0[i] + mean_color1[i]) / 2
   
   hue_value = int(mean_color[0])
   sat_value = int(mean_color[1])
   value_value = int(mean_color[2])

   # Determinare il colore predominante
   if value_value < 50:
      color = "BLACK"
   elif sat_value < 30:
      color = "WHITE"
   else:
      if hue_value < 5:
         color = "RED"
      elif hue_value < 20:
         color = "ORANGE"
      elif hue_value < 33:
         color = "YELLOW"
      elif hue_value < 78:
         color = "GREEN"
      elif hue_value < 131:
         color = "BLUE"
      elif hue_value < 167:
         color = "VIOLET"
      else:
         color = "RED"

while a == 1:
   
   camera.start()              # Avvia la fotocamera
   sleep(1)                    # Attendi che la fotocamera si avvii
   camera.capture_file(path0)  # Salva l'immagine nella cartella
   camera.capture_file(path1)  # Salva l'immagine nella cartella
   camera.stop()               # Arresta la fotocamera
   
   img0 = cv2.imread(path0)    # Leggi l'immagine salvata
   sleep(0.2)                  # delay tra le letture
   img1 = cv2.imread(path1)

   elabora_immagine(img0, img1)      # Ricava colore e fai la media delle immagini
   print(color)

   a += 1
   
   sleep(1)                   # Aggiunto per evitare un ciclo infinito troppo veloce

