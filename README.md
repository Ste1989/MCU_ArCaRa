# Odroid_ArCaRa
------------ 
Collogamenti Hardware:

.....
-------------
Avvio del software
0) collefarsi via ethernet all'odroid con ssh odroid@192.168.1.1 (password: odroid)
1) avviare roscore
2) avviare il nodo della seriale (controllare che sia su USB0) roslaunch serial_manager SerialManager
3) avviare il nodo che parla con l'autopilota (controllare che sia su USB1) roslaunch pixhawk_manager PixhawkManager
4) rosrun image_view image_saver per salvare le immagini dalla camera

TODO: 
- You can only arm or disarm in Stabilize, ACRO, AltHold and Loiter mode.
- interferenza magnetica sulla bussola mettendo altri componenti, come tenerne conto?
- modalità emergency stop ? (brake)
