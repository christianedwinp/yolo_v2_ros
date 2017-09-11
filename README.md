# yolo_ros

this is the one version of darknet that can compile the yolo with ros(jade or indigo) and publish the topic message with user's own data format

you can download my msg git clone https://git.ram-lab.com/wangyuan/msg_boundingbox.git

you can change the CPU/GPU, Opencv in the Makefile.

And the weight is too large which i can not upload. You can just download it in the darknet website.

after catkin_make
you can use roslaunch to load it
this code read the image streams from the /usb_cam and out the bboxs to topic /yolo/AAox/.. (please use the rostopic list to see)