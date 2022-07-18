# Formación en cubo de Drones

## David Leonardo Ramírez Parada

Todo lo mencionado en este documento fue probado en Ubuntu 20.04.4 LTS.

## Preparación de ROS

Instalación de ROS - Noetic

~~~ bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
~~~

Se debe confirmar de los siguientes paquetes están instalados correctamente:

~~~ bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
~~~

## Preparación Python3

Si no se tiene instalado *Python* en cualquier versión junto con *pip*

~~~ bash
sudo curl https://bootstrap.pypa.io/get-pip.py | sudo python3
sudo curl https://bootstrap.pypa.io/get-pip.py | python3
echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
~~~

Después

~~~ bash
sudo python3 -m pip install -U rosdep catkin_pkg future
python3 -m pip install -U rosdep catkin_pkg future
~~~

## Preparación Catkin

Una vez se confirmó la instalación de ROS se podrá usar todo lo siguiente.

Si no existe el directorio *~/catkin_ws*

~~~ bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace  # Inicialización del workspace
~~~

De lo contrario

~~~ bash
cd ~/catkin_ws/src
wstool init
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
git clone https://github.com/ethz-asl/rotors_simulator
git clone https://github.com/ethz-asl/mav_comm
wstool merge rotors_hil.rosinstall
wstool update
~~~

Una vez hecho lo anterior, se debe copiar los archivos de la carpeta *Otros* de la siguiente forma

~~~ bash
* multiple_hummingbirds_example.launch -> ~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch/
* basic_wall.world -> ~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/
~~~

Además, se deberá copiar toda la carpeta *formation_tracking* y *consensus_lib* a la ubicación *~/catkin_ws/src/*

## Build Worspace

Construir el workspace

~~~ bash
sudo rosdep init
rosdep update
cd ~/catkin_ws/
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

## Ejecución en ROS Gazebo

Una vez se haya completado el build con catkin se puede correr la simulación de la siguiente forma:

En una terminal:

~~~ bash
roslaunch rotors_gazebo multiple_hummingbirds_example.launch
~~~

NOTA: Como los drones están inicializados en una forma específica, se debería dar click derecho en "Modelos > hummingbird1 > follow" para que se posicione la cámara en la posición correcta.

<center>
    <img src="https://i.ibb.co/QYp2xSy/image.pngg" width="70%">
</center>

En otra terminal:

~~~ bash
rosrun formation_tracking formation_tracking
~~~
