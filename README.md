# drone_application
This package contains the implementation corresponding to the detection by artificial vision of people and monitoring of these through ARDrone 2.0, all in real time through a laptop or computer with Wi-Fi connection.

# 1. Quickstart / Minimal Setup
First, install the following packages, depending on your Ubuntu / ROS version.

# Package ardrone_autonomy 
[ROS](http://ros.org) Driver for [Parrot AR-Drone](http://ardrone2.parrot.com/) 1.0 & 2.0 Quadrocopters

* Documentation: http://ardrone-autonomy.readthedocs.org/
* ROS wiki page: http://wiki.ros.org/ardrone_autonomy
* Code API: http://docs.ros.org/indigo/api/ardrone_autonomy/html
* Patched _ARDroneLib_ repository: https://github.com/AutonomyLab/ardronelib
* Author: [Mani Monajjemi](http://mani.im) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca)) + [other contributers](http://ardrone-autonomy.readthedocs.org/en/latest/contributers.html)

# Package tum_simulator 

This package contains the implementation of a gazebo simulator for the Ardrone 2.0 and has been written by Hongrong Huang and Juergen Sturm of the Computer Vision Group at the Technical University of Munich.

This package is based on the ROS package tu-darmstadt-ros-pkg by Johannes Meyer and Stefan Kohlbrecher and the Ardrone simulator which is provided by Matthias Nieuwenhuisen.

The simulator can simulate both the AR.Drone 1.0 and 2.0, the default parameters however are optimized for the AR.Drone 2.0 by now.

# Package tum_ardrone

This package contains the implementation corresponding to the following publications:

- [Scale-Aware Navigation of a Low-Cost Quadrocopter with a Monocular Camera](https://vision.in.tum.de/_media/spezial/bib/engel14ras.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Camera-Based Navigation of a Low-Cost Quadrocopter](https://vision.in.tum.de/_media/spezial/bib/engel12iros.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing](https://vision.in.tum.de/_media/spezial/bib/engel12vicomor.pdf) (J. Engel, J. Sturm, D. Cremers) 

You can find a [video](https://www.youtube.com/watch?feature=player_embedded&v=eznMokFQmpc) on *youtube*.
This Package builds on the well known monocular SLAM framework PTAM presented by Klein & Murray in their paper at ISMAR07. Please study the original PTAM website and the corresponding paper for more information on this part of the software. Also, be aware of the license that comes with it. 

The code works for both the AR.Drone 1.0 and 2.0, the default-parameters however are optimized for the AR.Drone 2.0 by now.


# 2. Installation
We tested drone_application on one system configurations, using Ubuntu 16.04 (Xenial Xerus) and ROS Kinetic. Note that building without ROS is not supported, however ROS is only used for input and output, facilitating easy portability to other platforms.

In your ROS package path, clone the repository:

    git clone https://github.com/cifucg/ardrone_application_tfg

Compile the package by typing:
    catkin_make
        
# 3 Usage

## 3.1 `simulator`
     roslaunch drone_package my_world.launch 
     python src/drone_package/src/computerVision.py -m method -c confidence -e environment
## 3.2 `real`    
     roslaunch ardrone_autonomy ardrone.launch 
     python src/drone_package/src/computerVision.py -m method -c confidence -e environment

## 3.3 Parameters
* `method`: [string] The name of the algorithm that we are going to use to obtain the detection of people through computer vision. The implemented methods : HOG, CAFFE, YOLO.
* `confidence`: [double] It is the amount with which the detection obtained by the chosen algorithm is correct for us. If the detection is below this value, it is discarded, while if it is higher it is accepted. Example: 0.1, 0.2, 0.9.
* `environment`: [string] This variable configures the system according to the environment in which we are going to execute it. It can be simulated or real.
