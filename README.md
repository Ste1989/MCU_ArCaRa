# Odroid_ArCaRa
------------------------------------
# Cosa installare sull'Odroid

- Ubuntu 14.04 LTS
- ROS indigo
- dronekit
- lib real sense
- opencv
- ssh e configurare la connessione ethernet
- ros hector loclization https://github.com/tu-darmstadt-ros-pkg/hector_localization.git
------------ 
# Collogamenti Hardware:

.....
-------------
# Avvio del software
-  collefarsi via ethernet all'odroid con ssh odroid@192.168.1.1 (password: odroid)
- avviare roscore
- avviare il nodo della seriale (controllare che sia su USB0) roslaunch serial_manager SerialManager
- avviare il nodo che parla con l'autopilota (controllare che sia su USB1) roslaunch pixhawk_manager PixhawkManager
- rosrun image_view image_saver per salvare le immagini dalla camera

# TODO: 
- You can only arm or disarm in Stabilize, ACRO, AltHold and Loiter mode.
- interferenza magnetica sulla bussola mettendo altri componenti, come tenerne conto?
- modalità emergency stop ? (brake)
