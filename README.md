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
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>

<hr>

## Team Members

<hr>

## Abstract


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
* Best model performance: mAP = 81.6%, Precision = 86.1%, Recall = 72.3%. 
* RoboflowOak API interacts with the camera and provides human detection results outside of ROS, which the script then processes and uses to publish movement commands within ROS. 
<hr>

## Challenges
* 
<hr>

## Final Project Videos
**Click** any of the clips below to **reroute to the video**. 

#### Demo Videos (replace the img src and link)




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


__Circuit Diagram__

Our team made use of a select range of electronic components, primarily focusing on the OAK-D Lite camera, Jetson NANO, a GNSS board / GPS, and an additional Seeed Studio XIAO nRF52840 Sense (for IMU usage).
Our circuit assembly process was guided by a circuit diagram provided by our class TAs.

<img src="images/Circuit Diagram.png">

<hr>

## Gantt Chart
<div align="center">
    <img src="images\gantt_chart.webp" height="500">
</div>
<hr>

## Course Deliverables
Here are our autonomous laps as part of our class deliverables:

* 

Presentations:  
* 
<hr>

## Project Reproduction
If you are interested in reproducing our project, here are a few steps to get you started with our repo:


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
