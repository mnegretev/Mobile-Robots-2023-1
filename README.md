# Curso Robots Móviles 2023-1 FI, UNAM

Material para el curso de Robots Móviles de la Facultad de Ingeniería, UNAM, Semestre 2023-1

## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2023-1
* $ cd Mobile-Robots-2023-1
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2023-1/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-1/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-1/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-1/blob/master/Media/gui.png" alt="GUI" width="319"/>

## Máquina virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con Ubuntu y ROs ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1DYhmegVFEz7VA69uncpYsL8Ck0HbaIEz?usp=sharing) <br>
En esa misma carpeta hay un video con instrucciones para usar la máquina virtual. <br>
Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.
Usuario: cire2022 <br>
Contraseña: cire2022


## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
