# deep learning with robot car
### Purpose
> The purpose of this lab is to enable you to know how to implement deep learning in robot
car and detect objects by the camera sensor. We use the yolo as the network to do the object
detection and visualize the object in the fixed frame with marker.

### Lab Objectives
> By following the directions in this lab, you are expected to achieve the following:

- Implement the deep learning to detect object in the computer vision.
- Put the deep learning part as a node in the Ros environment and know how to publish
the topic and design your own data type which help the deep learning node to connect
with the other node.
- Observe the visualization markers by rviz to how this works in the robot car.

### Requirements
1. Cuda6.5 support
2. Opencv 2.4.13(must)
3. Ros environment
4. Yolo
5. Cudnn(optional)

### Programming
> You have set the Arduino Mega2560 as the ros node, so when you start the yolo detection
in the TK1, the TK1 will publish the topic for marker to read. And during the process, the TK1
will detect the different class and label them in the camera stream, also it will publish the
boundary boxes’ coordinates in topic for programmer to design the other process.
> <p> You can read some information by **rostopic show /topic's name**, and this demo will offer the information of the object coordinate and the detected objected.

### Steps
> download the code and put it in src

- git clone https://git.ram-lab.com/wangyuan/yolo_ros.git
- git clone https://git.ram-lab.com/wangyuan/msg_boundingbox.git

> To build the code, use the following command when you are in `~/catkin_ws/`:

- catkin_make

> durint the process, there are some problems which you will face:
 
 - include <*******/boudingbox_msg.h>doesn't have, 

> you can try source echo "source ~/catkin_ws/devel/setup.bash (if you are familiar with ros, you can put this in the bashrc: ~/catkin_ws/devel/setup.bash" and then source ~/.bashrc)

- you will miss some ****-config

> you can use sudo apt-cache search ros-jade-*****, and when you find the ****, you can
sudo apt-get install this and catkin_make again.

- make sure that you have install the cuda and opencv successfully, otherwise the error will
happen during making process

- ## download the weight file
>  'cd yolo_ros" and "mkdir weight && cd weight'
 1. "wget http://pjreddie.com/media/files/yolo.weights" -------------------trained by coco
datasets
 2. "wget http://pjreddie.com/media/files/yolo-voc.weights"--------------trained by voc
datasets
 3. "wget http://pjreddie.com/media/files/tiny-yolo-voc.weights"---------trained by voc
datasets...
 4. roslaunch yolo_v2_ros usb_web_cam.launch       </p>this will publish the topic about the coordinate of the boundary box and its width and height.</p>
5. rosrun rviz rviz  </p>you can add the topic to see the results by the visualization.</p>

- Also, you can download the marker to see the objects' marker in the frame
</p>you need go to catkin_ws/src
and then 
</p> git clone https://git.ram-lab.com/wangyuan/marker_for_yolo.git
</p> roslaunch yolo_v2_ros usb_web_cam.launch</p>

1. rosrun marker marker
2. rosrun rviz rviz
</p>you can set the global frame to usb_cam during test and write the
transform function when you apply this in the robot car.</p>


