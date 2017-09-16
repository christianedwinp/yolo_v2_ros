# Lab2 deep learning with robot car
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
> <p> You can read some information by **rostopic show /topic's name**, and this demo will offer the information of the object coordinate and the detected objects.

### Steps
><p> open the terminal and ready to run the following command
><p> download the code and put it in **catkin_ws/src**
- `cd ~/catkin_ws/src`
- `git clone https://github.com/ywangeq/yolo_v2_ros.git`
- `git clone https://github.com/ywangeq/msg_boundingbox.git`

> To build the code, use the following command when you are in `catkin_ws`:

- ` catkin_make`

> durint the process, there are some problems which you will face:

 - `include <...../boudingbox_msg.h>`doesn't have,

> you can try source echo "source ~/catkin_ws/devel/setup.bash (if you are familiar with ros, you can put this in the bashrc: ~/catkin_ws/devel/setup.bash" and then source ~/.bashrc)

- you will miss some `****-config`

> you can use `sudo apt-cache search ros-jade-*** `, and when you find the package, you can run
`sudo apt-get install ***` in the terminal to install this package and `catkin_make` again.

- make sure that you have install the cuda and opencv successfully, otherwise the error will
happen during making process

- ## download the weight file
> <p> `cd yolo_ros" and "mkdir weight && cd weight`
><p> Now you have create a file called weight which is used to store the weights file and get into it
<p> 1. `wget http://pjreddie.com/media/files/yolo.weights` -------------------trained by coco datasets
 <p>2. `wget http://pjreddie.com/media/files/yolo-voc.weights`--------------trained by voc datasets
 <p>3. `wget http://pjreddie.com/media/files/tiny-yolo-voc.weights`---------trained by voc datasets
<p> after you download the weights file, you can use them by change the route in the launch file in the `yolo_v2_ros/usb_web_cam.launch`
<p> when you decide to change the weights in **weighfile**, please change the related param **model_cfg** and **datafile**  followed the datasets names!
<p>to start run the yolo detection with ros, run the following in the terminal:       
 <p>4. `roslaunch yolo_v2_ros usb_web_cam.launch`      
  </p>this will publish the topic about the coordinate of the boundary box and its width and height.
<p>5. `rosrun rviz rviz `
    </p>you can add the topics of **/yolo/AAox/usb_cam** to see the results by the visualization.</p>

- Also, you can download the marker to see the objects' marker in the frame
</p>you need go to catkin_ws/src by run `cd ~/catkin_ws/src`
and then
</p>run ` git clone https://github.com/ywangeq/marker_for_yolo.git`
<p> then you need go to the workspace by `cd ~/catkin_ws` and run the build command `catkin_make`
<p> to start the system, run the next command
</p> `roslaunch yolo_v2_ros usb_web_cam.launch`</p>
<p> And start the marker to mark the objects in the visualization,you can run
1. `rosrun marker marker`
<p> run `rosrun rviz rviz` to visualize the topics
<p> to know what topics you have published, please run:
- `rostopic list`
and in the rviz, you can choose any of topics to see the results
</p>you can set the global frame to **usb_cam** during test and write the
transform function when you apply this in the robot car.</p>
