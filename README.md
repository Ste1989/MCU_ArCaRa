# Odroid_ArCaRa
------------ 
Collogamenti Hardware:


-------------
Avvio del software
0) collefarsi via ethernet all'odroid con ssh odroid@192.168.1.1 (password: odroid)
1) avviare roscore
2) avviare il nodo della seriale (controllare che sia su USB0) roslaunch serial_manager SerialManager
3) avviare il nod o che parla con l'autopilota (controllare che sia su USB1) roslaunch pixhawk_manager PixhawkManager
