# medicbot

Since we are using ROS1 right now, we need one master to run initially, in this case it is controller

For Motion control purpose:
Step 0: source ~/.rosrc // This will make the multiple machines run in same network
Step 1: Start a master, with command 'roscore' in one terminal
Step 2: Start a rosserial_arduino node in main board eg. odroid to connect with arduino( if using ):
        rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0 ( or ACM0 )
After this the ros code running in arduino will communicate with odroid i.e joy_publisher and joy_subscriber node, which can be inspected with commands 'rostopic list' or 'rostopic hz <topic_name>' or 'rostopic echo <topic_name>'

For video and audio purpose:
This nodes can be also found in ~/catkin_ws/src/catkin_pkg/launch/catkin.launch file, the ros nodes which needs to start are:
1. publishing video stream from controller
   rosrun cv_camera cv_camera_node ns:="cap1"
2. publishing audio stream from controller
   roslaunch audio_capture capture.launch ns:="aud1" sample_rate:=44100 device:="hw:1,0"
the ros nodes which needs to start capturing video and audio are:
1. Subscribing and playing the video stream captured from robot in controller display
   rosrun rqt_image_view rqt_image_view // than choose the compressed imatge topic
2. Subscribing and playing the audio stream captured from robot in controller microphone
   roslaunch audio_play play.launch ns:="aud2" do_timestamp:=false

Similary to capture and play the video and audio captured from controller to the robot just change the namespacce i.e ns

Note: This all nodes can be automatically run during boot by putting this nodes in ~/catkin_ws/src/catkin_pkg/launch/catkin.launch file. Even if the puslising nodes(camera,audio) are not run yet, it is not a problem.

