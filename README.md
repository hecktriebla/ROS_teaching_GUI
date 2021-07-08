# ROS Teaching GUI C++ Qt5

This ROS-Teaching-GUI was created to have a user-friendly solution for teaching robots during a robotics project in the masters course of UAS Technikum Vienna.
Written in C++, ROS-Distro: Noetic, GUI-Framework: Qt5

![gui](https://user-images.githubusercontent.com/71969898/124924016-4d347180-dffb-11eb-8eec-91b680006081.png)

The robots in the project have Makeblock Smart-Servos which are connected to a Arduino Mega. The Arduino-program publishes the current angle-values from the servos to a rostopic.<br/>
The Teaching-GUI subscribes to this topic, stores the values in a vector. The user can decide if the current pose has an active or ineffective endeffector. <br/>
If all poses are saved with the designated buttons the vector is saved in a txt-File, ready to be read. The teached routine can be executed with the "Load and Execute" button. <br/>

I want to share this project because I think that there are not many GUI examples for ROS (written in C++) out there. Adapt the code so it fits your needs.<br/>

## Install
Install ROS: http://wiki.ros.org/noetic/Installation/Ubuntu <br/>
Clone the repository into the source folder of your catkin workspace <br/>
Clone catkin_simple as well: https://github.com/catkin/catkin_simple <br/>
Build with:
```
catkin_make
```

## Usage
Start with "Start"-Button <br/>
Bring the robot in the designated position <br/>
Save poses with "Save Pose with Endeffector OFF" or "Save Pose with Endefector ON" <br/>
Repeat until the routine is completed <br/>
Save the poses to a txt-File, change the name if you want <br/>
The txt-File is saved in the packages /path/ folder and can be loaded and executed with the "Load and Execute" Button <br/>

## Credits
Template made with: http://wiki.ros.org/qt_create/Tutorials/Qt%20App%20Templates <br/>
Also based on: https://github.com/WelinLee/ROS_QT_GUI <br/>
