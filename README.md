# MCU_ArCaRa
------------------------------------
# Cosa installare 

- Ubuntu 14.04 e le librerie real sense, seguendo la guida di installazione
- dronekit, dronekit-sitl:
  sudo apt-get install python-pip python-dev
  sudo pip install dronekit
  sudo pip install dronekit-sitl -UIgurare la connessione ethernet
- ssh e configurare la connessione ethernet : 
    sudo apt-get install openssh-client openssh-server
    sudo service ssh restart
- installato eigen3.3: [non so se necessario]
    hg clone https://bitbucket.org/eigen/eigen/
    cd Eigen
    mkdir build
    cd build
    cmake ..
    sudo make install
-installato libreria mraa scaricandola da git https://github.com/intel-iot-devkit/mraa.git
mkdir build
cd build
cmake ..
make
sudo make install
-installato MAVROS da http://dev.px4.io/ros-mavros-installation.html    :
	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws
	$ catkin_init_workspace
	$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
	$ wstool init ~/catkin_ws/src
	$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
	$ rosinstall_generator --upstream-development mavros | tee /tmp/mavros.rosinstall
	$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
	$ wstool merge -t src /tmp/mavros.rosinstall
	$ wstool update -t src
	$ sudo apt-get update
	$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y
	$ catkin build
- nel .bashrc:
	source /home/sistema/catkin_ws/devel/setup.bash
	source /home/sistema/MCU_ArCaRa/NapoDrone_ws/devel/setup.bash
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/sistema/MCU_ArCaRa/NapoDrone_ws
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/sistema/catkin_ws/src
- la prima volta commentare nei CMakeLists.s.txt di SerialManager, AutopilotMAnager, ObjectDetction e Object3Detection gli eseguibili

#GNUPLOT
sudo apt-get install gnuplot
sudo apt-get install gnuplot-x11
# Cosa installare sull'Odroid

- Ubuntu 14.04 LTS
- ROS indigo
- dronekit
- lib real sense (non istallata)
- opencv  2.4.9
- ssh e configurare la connessione ethernet
- ros hector loclization https://github.com/tu-darmstadt-ros-pkg/hector_localization.git (non installata)
- PCL: sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
  sudo apt-get update
  sudo apt-get install libpcl-all
  http://wiki.ros.org/hydro/Migration#PCL
------------ 
# Collogamenti Hardware:

.....
-------------
# Avvio del software
-GNUPLOT: pl 'file.txt' using 1:2 with lines
-  collefarsi via ethernet all'odroid con ssh odroid@192.168.1.1 (password: odroid)
- avviare roscore
- avviare il nodo della seriale (controllare che sia su USB0) roslaunch serial_manager SerialManager
- avviare il nodo che parla con l'autopilota (controllare che sia su USB1) roslaunch pixhawk_manager PixhawkManager
- rosrun image_view image_saver per salvare le immagini dalla camera

# TODO: 
- You can only arm or disarm in Stabilize, ACRO, AltHold and Loiter mode.
- interferenza magnetica sulla bussola mettendo altri componenti, come tenerne conto?
- modalità emergency stop ? (brake)

# Calibrazione
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
roslaunch camera_calibrarion calibration.lauch se apro la camera con gscam (già nel launch file)

roslaunch camera_calibration calibration2.launch se ho aperto la camera con usb_cam (non nel launch file)


 USB CAM :

rosparam set /usb_cam/video_device "/dev/video1" 
rosrun usb_cam usb_cam
!USARE QUESTO!
ROS_NAMESPACE=v4l/camera rosrun image_proc image_proc
