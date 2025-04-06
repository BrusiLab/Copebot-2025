from picamera2 import Picamera2
from time import sleep
import numpy as np 
import cv2 

path0 = '/home/pi/Desktop/Raspberry/Catania_2025/image0.jpg'  # Percorso in cui salvare la foto
path1 = '/home/pi/Desktop/Raspberry/Catania_2025/image1.jpg'  # Percorso in cui salvare la foto
path2 = '/home/pi/Desktop/Raspberry/Catania_2025/image2.jpg'
path3 = '/home/pi/Desktop/Raspberry/Catania_2025/image3.jpg'
path4 = '/home/pi/Desktop/Raspberry/Catania_2025/image4.jpg'

color = "Undefined"
a = 0
mean_color = [0, 0, 0]

camera = Picamera2()
camera_config = camera.create_still_configuration({"size": (1920, 1080)})  # Risoluzionw 1920x1080
camera.configure(camera_config)

def elabora_immagine(img0, img1, img2, img3, img4):
   
   global color  # Aggiunto per aggiornare la variabile globale color
   
   img0 = cv2.convertScaleAbs(img0, 1, 1) 
   img1 = cv2.convertScaleAbs(img1, 1, 1) 
   img2 = cv2.convertScaleAbs(img2, 1, 1) 
   img3 = cv2.convertScaleAbs(img3, 1, 1) 
   img4 = cv2.convertScaleAbs(img4, 1, 1) 
   
   hsv_frame0 = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)
   hsv_frame1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
   hsv_frame2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
   hsv_frame3 = cv2.cvtColor(img3, cv2.COLOR_BGR2HSV)
   hsv_frame4 = cv2.cvtColor(img4, cv2.COLOR_BGR2HSV)
   
   height, width, _ = img0.shape

   # Coordinate del centro
   cx = int(width / 2)
   cy = int(height / 2)

   # Definire una regione 100x100 attorno al centro
   roi_size = 50  # Metà dimensione della ROI
   roi0 = hsv_frame0[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi1 = hsv_frame1[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi2 = hsv_frame2[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi3 = hsv_frame3[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi4 = hsv_frame4[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

   # Calcolare il colore medio nella ROI
   mean_color0 = cv2.mean(roi0)[:3]  # Ignorare il canale alpha
   mean_color1 = cv2.mean(roi1)[:3]
   mean_color2 = cv2.mean(roi2)[:3]
   mean_color3 = cv2.mean(roi3)[:3]
   mean_color4 = cv2.mean(roi4)[:3]
   
   for i in range (0,3,+1):
      mean_color[i] = (mean_color0[i] + mean_color1[i] + mean_color2[i] + mean_color3[i] + mean_color4[i]) / 5
   
   hue_value = int(mean_color[0])
   sat_value = int(mean_color[1])
   value_value = int(mean_color[2])

   if hue_value < 10 or hue_value > 135:
      color = "RED"
   elif hue_value < 40:
      color = "YELLOW"
   elif hue_value < 93:
      color = "GREEN"
   elif hue_value < 135:
      color = "BLUE"
         
   print(hue_value, sat_value, value_value)

while True:

   if input() == "A":
   
      camera.start()              # Avvia la fotocamera
      sleep(0.5)                    # Attendi che la fotocamera si avvii
      
      camera.capture_file(path0)  # Salva l'immagine nella cartella
      sleep(0.1)
      camera.capture_file(path1)  # Salva l'immagine nella cartella
      sleep(0.1)
      camera.capture_file(path2)
      sleep(0.1)
      camera.capture_file(path3)
      sleep(0.1)
      camera.capture_file(path4)
      
      camera.stop()               # Arresta la fotocamera
      
      img0 = cv2.imread(path0)    # Leggi l'immagine salvata
      img1 = cv2.imread(path1)
      img2 = cv2.imread(path2)
      img3 = cv2.imread(path3)
      img4 = cv2.imread(path4)

      elabora_immagine(img0, img1, img2, img3, img4)      # Ricava colore e fai la media delle immagini
      print(color)

      a += 1

