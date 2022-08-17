<<<<<<< HEAD
# Formación en cubo de Drones

## David Leonardo Ramírez Parada

## Descripción

Formación de cubo con ocho drones a través de un control por medio de Bearings.

La idea del control es conseguir la formación mientras que dos drones llamados líderes siguen una trayectoria predefinida para cruzar por una ventana de un muro de tal forma que todos los drones continúen en formación como se muestra en la siguiente imagen

<center>
    <img src="https://github.com/deiividramirez/formation_tracking/blob/master/formation_tracking/src/out/formation.gif?raw=true" width=70%>
</center>

## Instalación

Todo lo mencionado en este documento fue probado en Ubuntu 20.04.4 LTS.

### Preparación de ROS

Instalación de ROS - Noetic (el siguiente comando solo funciona en Ubuntu 20.04.4 LTS)

~~~ bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
~~~

Se debe confirmar de los siguientes paquetes están instalados correctamente:

~~~ bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
~~~

### Preparación Python3

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

### Preparación Catkin

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

~~~ bash
cd ~/catkin_ws/src/
git clone https://github.com/deiividramirez/formation_tracking
~~~

### Build Worspace

Construir el workspace

~~~ bash
# Inicialización de rosdep y recolección de paquetes
sudo rosdep init
rosdep update
cd ~/catkin_ws/
# Actualización de paquetes rotos o dependencias
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
# Construcción de catkin_workspace
catkin build
# Añadir variables de entorno
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

### Ejecución en ROS Gazebo

Una vez se haya completado el build con catkin se puede correr la simulación de la siguiente forma:

En una terminal:

~~~ bash
roslaunch rotors_gazebo multiple_hummingbirds_example.launch
~~~

**NOTA:** Como los drones están inicializados en una forma específica, se debería dar click derecho en "Modelos > hummingbird1 > follow" para que se posicione la cámara en la posición correcta.

<center>
    <img src="https://i.ibb.co/QYp2xSy/image.pngg" width="70%">
</center>

En otra terminal:

~~~ bash
rosrun formation_tracking formation_tracking
~~~
=======
# Formación en cubo de Drones

## David Leonardo Ramírez Parada

## Descripción

Formación de cubo con ocho drones a través de un control por medio de Bearings.

La idea del control es conseguir la formación mientras que dos drones llamados líderes siguen una trayectoria predefinida para cruzar por una ventana de un muro de tal forma que todos los drones continúen en formación como se muestra en la siguiente imagen

<center>
    <img src="https://github.com/deiividramirez/formation_tracking/blob/master/formation_tracking/src/out/formation.gif?raw=true" width=70%>
</center>

## Instalación

Todo lo mencionado en este documento fue probado en Ubuntu 20.04.4 LTS.

### Preparación de ROS

Instalación de ROS - Noetic (el siguiente comando solo funciona en Ubuntu 20.04.4 LTS)

~~~ bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
~~~

Se debe confirmar de los siguientes paquetes están instalados correctamente:

~~~ bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
~~~

### Preparación Python3

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

### Preparación Catkin

Una vez se confirmó la instalación de ROS se podrá usar todo lo siguiente.

Si no existe el directorio *~/catkin_ws*

~~~ bash
mkdir -p ~/catkin_ws/src
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

~~~ bash
cd ~/catkin_ws/src/
git clone https://github.com/deiividramirez/formation_tracking
~~~

### Build Worspace

Construir el workspace

~~~ bash
# Inicialización de rosdep y recolección de paquetes
sudo rosdep init
rosdep update
cd ~/catkin_ws/
# Actualización de paquetes rotos o dependencias
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
# Construcción de catkin_workspace
catkin build
# Añadir variables de entorno
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

### Ejecución en ROS Gazebo

Una vez se haya completado el build con catkin se puede correr la simulación de la siguiente forma:

En una terminal:

~~~ bash
roslaunch rotors_gazebo multiple_hummingbirds_example.launch
~~~

**NOTA:** Como los drones están inicializados en una forma específica, se debería dar click derecho en "Modelos > hummingbird1 > follow" para que se posicione la cámara en la posición correcta.

<center>
    <img src="https://i.ibb.co/QYp2xSy/image.pngg" width="70%">
</center>

En otra terminal:

~~~ bash
rosrun formation_tracking formation_tracking
~~~
>>>>>>> f314eb9dee612564e46ca70db07d595ab0bb56e7
