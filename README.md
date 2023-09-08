# Preliminary Real-robot OPI-GA 

Start-up instructions (one robot)
1. Turn on robot while connected to power or make sure portable battery is charged 
2. Make sure your robot is connected to the same wifi network as your computer
3. Find the IP address for your robot
4. Use the following command to remote-in:
   a. ssh ubuntu@"ip-address"
5. You should see a terminal that corresponds to the robot you are remoting into


Overview of sensors
1. To get robot started use the following commands (use separate terminals for each):
   a. roscore
   b. roslaunch turtlebot3_bringup turtlebot3_robot.launch
 NOTE: If you are unable to connect using these commands, you may need to update the ROS_MASTER_IP to be either your own computer if    you are using your computer as the ROS master or just the IP address of the turtlebot itself. To find the IP address, refer to the     "inet" portion when you call "ifconfig" in the terminal.
2. The ultrasonic sensor is an additional accessory. To see the script to create a topic for that sensor, refer to a file called ultrasonic_sensor_topic.py

TODO:
1. Figure out collection behavior using given sensors. 
