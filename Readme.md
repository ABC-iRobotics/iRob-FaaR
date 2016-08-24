# Falcon as a Robot


--------------------

## Description

This is a special application of the Novint Falcon. The Falcon is used as a robot with [compliance control](http://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter9.pdf). We used an [Optoforce OMD-10-SE-10N](http://optoforce.com/wp-content/uploads/2014/05/OMD-10-SE-10N-DATASHEET-1.41.pdf) sensor to measure the forces. In this application the device run on a real-time patched linux kernel. We used xenomai 2.6.4 to make a RTOS. The GUI application for the device is written in Qt, in c++ language. To communicate with the Falcon we used the [libnifalcon](http://qdot.github.io/libnifalcon/) library.

## Fatures

- Without any command the robot stays at home position.
- Path logging mode: you can log a path by moving your hand the end-effector. And you can save or load the logging path. After saving/opening you can replay it.
- Trajectory planning: you can generate any path with trapezoid velocity profile by adding positions in the workspace. Yaou can also set the maximum velocity and acceleration.
- Compliance control mode: the compliance mode can be used in any operation mode.
- You can change the parameters of the PD controller and of the compliance model.
- Plotting: you can plot in real-time the encoder angles and the position of the robot and you can plot the positions, the velocities and the acceleratings of the generated trajectory.

## Installation Guide

### Dependencies

- Ubuntu linux 14.04
- Xenomai 2.6.x version
- Qt Creator with Qt version: Qt 4.8.6 (qt4)

### Installation steps:

##### Xenomai patched linux:

There are two ways to install xenomai patched linux.


First method:

1. Install Ubuntu linux 14.04 from [here](http://www.ubuntu.com/download/alternative-downloads).
2. Install Xenomai 2.6.x by using [this](https://xenomai.org//installing-xenomai-2-x/) guide from the Xenomai homepage.

Second method:

1. Go to RTXI installtion [page](http://rtxi.org/install/)
2. Select the Custom method
3. Follow the insturctions to step 6.

I prefer the second method because it is faster and more conforable than the first method.

##### Qt Creator

1. Install the GCC compiler:
	```
	sudo apt-get install g++
	```
2. Install Qt Creator 4 or 5:

	install qt4:
     ```
    sudo apt-get install qt4-default
    ```

	to install Qt Creator 4 use this command:

    ```
    sudo apt-get install qtcreator
    ```
    
    to install Qt Creator 5 go to [Qt download page](https://www.qt.io/download-open-source/#section-2) then download and install it.
    

3. Set up the kit in the Qt Creator:

	- open Qt Creator
	- go to "Tools" -> "Options"
	- choose "Build & Run"
	- select the "Kit" tab
	- click "Add"
	- type a name (for example: Qt 4)
	- choose the GCC compiler (32 or 64 bit version)
	- at the "Qt version" choose "Qt 4.8.6 (qt4)"
		- if you cannot find it then follow these steps:
			1. go to "Qt Versions" tab
			2. click "Add"
			3. navigate to /usr/lib/x86_64-linux-gnu/qt4/bin
			4. choose the qmake file and click "Open"
			5. go back to the "Kits" tab and now you can select the Qt4 version
	- click to "Make Default" then "Apply" and "OK"

##### Install the libraries:

1. Extract the "install-FaaR.tar. gz":
	
    ```
    sudo tar -xf install-FaaR.tar.gz
    ```
2. In the terminal navigate to the extracted folder then type these commands:
	
    ```
    sudo chmod +x install-dependencies.sh
    ```
    ```
    sudo ./install-dependencies.sh
    ```
    