# <div align="center">Path Finder</div>
![image](images/UCSDLogo_JSOE_BlueGold.png)
### <div align="center"> MAE 148 Final Project </div>
#### <div align="center"> Team 10 Winter 2025 </div>

![car_image](images/carpic.jpg)

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#slam-simultaneous-localization-and-mapping">SLAM (Simultaneous Localization and Mapping)</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#troubleshooting-bring-ups">Troubleshooting Bring ups</a></li>
        <ul>
            <li><a href="#lidar-setup">Lidar Setup</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>

<hr>

## Team Members
- Max Guerrero - Embedded Software UCSD Ext (B.Sc. Comp. Physics 2021)
- Jerry Ko (B.S Electrical Engineering)
- Congge Xu (B.S. Computer Engineering)
- Jamie Miller (B.Sc Aerospace Engineering)

<hr>

## Abstract
Pathfinder was envisioned as an autonomous vehicle designed to provide assistance to search and rescue personnel in remote environments.
- Navigate GPS waypoints
- Identify humans in need
- Generate a map for first responders to use in rescue missions.


<hr>

## What We Promised
### Must Have
* 

### Nice to Have
* Use ["MapViz"](https://swri-robotics.github.io/mapviz/) to visualize the realtime location of the robot, and click on the map to publish coordinate for the robot to follow.
  - Could use MapViz to publish and read coordinate but the map is not showing
<hr>

## Accomplishments: What we have done
* The custom model is fine-tuned with the custom dataset and trained on human detection tasks. Multiple model versions utilizing pre-trained model weights (YOLOv11, Roboflow 3.0) have their performances compared and the best one is applied using RoboflowOak API.
  - Best model performance: mAP = 81.6%, Precision = 86.1%, Recall = 72.3%. 
  - RoboflowOak API interacts with the camera and provides human detection results outside of ROS, which the script then processes and uses to publish movement commands within ROS. 
<hr>

## Challenges
* Faced multiple malfunctioning VESCs:
  - First did not properly steer
  - Second did not properly throttle
  - Third would not turn on
  - Fourth was borrowed from UCSD Blue on the day of the final
* Spent too much time trying to get Docker and ROS2 running on PC's and VMs
* 

<hr>

## Final Project Videos
[![Watch the video](https://img.youtube.com/vi/18XZ_vilVA8/maxresdefault.jpg)](https://youtu.be/18XZ_vilVA8)





<hr>

## Software

### Overall Architecture

### MapViz
  - Ros2 tool to load tile maps and display GPS coordinate inputed to the corresponding Ros2 topic.
  - However, after trying with the built-in tile map, Google Maps, Stadia maps with working API keys, and [offline Google Map Tile map](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite), we could not make the tile map to show on MapViz.
  - But we did succeed to let MapViz read mock coordinate data by manually publishing data, and published coordinate by clicking on the MapViz interface.
  ![mapviz_screenshot](images/MapViz%20Screenshot.png)
<hr>

## Hardware 

* __3D Printing:__ Camera Stand, Jetson Nano Case, GPS Plate, Lidar Mount
* __Laser Cut:__ Base plate to mount electronics and other components.

__Parts List__

* Traxxas Chassis with steering servo and sensored brushless DC motor
* Jetson Nano
* WiFi adapter
* 64 GB Micro SD Card
* Adapter/reader for Micro SD Card
* Logitech F710 controller
* OAK-D Lite Camera
* LD19 Lidar (LD06 Lidar)
* VESC
* Point One GNSS with antenna
* Anti-spark switch with power switch
* DC-DC Converter
* 4-cell LiPo battery
* Battery voltage checker/alarm
* DC Barrel Connector
* XT60, XT30, MR60 connectors

*Additional Parts used for testing/debugging*

* Car stand
* USB-C to USB-A cable
* Micro USB to USB cable
* 5V, 4A power supply for Jetson Nano

### __Mechanical Designs__
__Camera Mount__
![camera mount1](images/cam%20mount1.png)
Credit: https://www.thingiverse.com/thing:5336496

__Circuit Diagram__

Our team made use of a select range of electronic components, primarily focusing on the OAK-D Lite camera, Jetson NANO, a GNSS board / GPS.
Our circuit assembly process was guided by a circuit diagram provided by our class TAs.

<img src="images/Circuit Diagram.png">

<hr>

## Gantt Chart
<div align="center">
    <img src="images/Gantt.png">
</div>
<hr>

## Course Deliverables
Here are our autonomous laps as part of our class deliverables:

* [3 Autonomous Laps](https://drive.google.com/file/d/1rYdKuJp95L6o8hU5gIAqILZzjU2uGJZW/view?usp=sharing) 
* [GPS Laps](https://youtu.be/jbZKhUSmnkc?si=7ICiua8Jcp4CKKOG)
* [Oak-D Camera Model](https://cdn.discordapp.com/attachments/1344142206776119386/1346536237007372288/IMG_3482.mov?ex=67e04618&is=67def498&hm=6a8eb74b5d8431f3bcc59a255ffcd0d7b85c87cf8b08f66f8d74673bdc5d60cd&)


Presentations:  
* [Final Presentation](https://docs.google.com/presentation/d/1ZoWhRLhPfD_xljGeW3dEyOw8T_ZkKze7IONw5t4ZVvc/edit?usp=sharing)
<hr>

## Troubleshooting Bring ups
### __Lidar Setup__
* Method 1:
``` 
  source_ros2
  ros2 launch ucsd_robocar_sensor2_pkg lidar_ld06.launch.py
```
* Method 2:
```
  cd ~
  mkdir -p ldlidar_ros2_ws/src
  cd ldlidar_ros2_ws/src
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
  cd ~/ldlidar_ros2_ws
  sudo chmod 777 /dev/ttyUSB0
```

 Enter launch file corresponding to ld19.launch.py and ensure that the port name is setcorrectly to USB0.

 ```
  cd ~/ldlidar_ros2_ws
  colcon build
  source install/setup.bash
 ```
 To make the whole thing permanent for new windows you can do:
```
echo source ~/ldlidar_ros2_ws/install/setup.bash >> ~/.bashrc
source ~/.bashrc
```
__To start the node:__
```
cd ~/ldlidar_ros2_ws
ros2 launch ldlidar_stl_ros2 ld19.launch.py
```
__To Visualize:__

Two methods
1. After launching the node, do `<rviz2>`, within the window open up the file in the
readme section of ldlidar_ros2_ws called rviz2 (should be inside a folder called
rviz2)
2. When launching the node, do `<ros2 launch ldlidar_stl_ros2viewer_ld19.launch.py>` instead
Troubleshooting:
*  If on a PC and trying to get the lidar to connect ensure
    - Your micro-usb cable can actually transmit data. In ye olden days you
would be able to tell by looking at the symbol on the cord, where an arrow
on the middle prong of the USB symbol would indicate that it can transmit
data. Unfortunately, there’s a lot of cheap crap these days that has made
this method unreliable. So, ensure that you can transmit data by plugging
in any other device using a micro usb cable. Even then, some cables can
only transmit certain data, so you might be best off just buying a cable to
be absolutely sure it works
    - If you are still having trouble connecting, ensure that you have the latest CP210x drivers for the Serial to COM bridge that is used on the lidar.
Found here:
https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads
    - Ensure that the Lidar is being read by your computer before trying to use it in a VM. This comes with the LDLidar development kit found here:
https://files.waveshare.com/upload/a/a5/Ld_desktop_V2.3.12.zip
* Note, this kit will direct you to a download for the CP210x drivers,
but these are out of date and you should use the ones found on the
manufacturer’s website above


<hr>

## Acknowledgements
TODO

**Programs Reference:**
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)
* And whatever we used


README.md Format, reference to [winter-2024-final-project-team-7](https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-7)

<hr>

## Contacts

* 
