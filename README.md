# Week 3 Submission - Vahitha SJ

<<<<<<< HEAD
This repository contains my ROS 2 package named kratos_vahitha for the Week 3 assignment.  
The package includes solutions to four questions covering publishers, subscribers, custom messages, and services in ROS 2.

Student Information  
Name: Vahitha SJ  
Course/Assignment: Week 3 ROS 2 Assignment  
ROS 2 Distribution Used: Humble  

Package Information  
Package Name: kratos_vahitha  
Language: Python  
Dependencies: rclpy, std_msgs, geometry_msgs, builtin_interfaces  
The package is located in the src/kratos_vahitha directory.  
All scripts are stored in the scripts folder.  
Custom message and service definitions are in msg and srv folders.  

Question 1: Publisher and Subscriber  
Files: q1_pub.py and q1_sub.py  
Description: A basic publisher node publishes string messages at regular intervals. A subscriber node listens to the topic and prints the received messages.  
Concepts Covered: Topics, publishing, subscribing, and message passing.  

Question 2: Two Node Communication  
Files: q2_s1.py and q2_s2.py  
Description: Implemented two nodes where one node sends data (numbers or text) and the other node receives and processes it.  
Concepts Covered: Multi-node communication and synchronization between publisher and subscriber.  

Question 3: Custom Message - Rover Data  
File: q3_rover.py  
Custom Message: RoverData.msg  
Description: A custom message named RoverData was created containing rover-related data such as speed, direction, and position. A node publishes and subscribes to this message.  
Concepts Covered: Custom messages, defining .msg files, using and testing them.  

Question 4: Service and Client  
Files: q4_server.py and q4_client.py  
Custom Service: CalculateArea.srv  
Description: A service definition was created to calculate the area of a rectangle. The server receives length and width from the client, calculates the area, and sends back the result. The client sends values and prints the response.  
Concepts Covered: Service and client communication, defining .srv files, request and response handling in ROS 2.  

How to Build and Run  

1. Clone the Repository  
git clone https://github.com/vahithasj/week3_submission_vahitha.git  
cd week3_submission_vahitha  

2. Build the Package  
cd ~/ros2_ws  
colcon build  
source install/setup.bash  

3. Run the Scripts  

Q1 Publisher and Subscriber  
Terminal 1: ros2 run kratos_vahitha q1_pub.py  
Terminal 2: ros2 run kratos_vahitha q1_sub.py  

Q2 Two Node Communication  
Terminal 1: ros2 run kratos_vahitha q2_s1.py  
Terminal 2: ros2 run kratos_vahitha q2_s2.py  

Q3 Custom Message Rover Data  
ros2 run kratos_vahitha q3_rover.py  

Q4 Service and Client  
Terminal 1 (server): ros2 run kratos_vahitha q4_server.py  
Terminal 2 (client): ros2 run kratos_vahitha q4_client.py  

Notes  
Always source the workspace before running:  
source install/setup.bash  

Executable Python files are inside kratos_vahitha/scripts  
Custom messages are inside kratos_vahitha/msg  
Custom services are inside kratos_vahitha/srv  

Conclusion  
This package demonstrates the following  
1. Basic ROS 2 publishers and subscribers  
2. Communication between multiple nodes  
3. Creating and using custom messages  
4. Implementing a service and client using custom .srv definitions  
=======
This repository contains my ROS 2 package named kratos_vahitha for the Week 3 assignment. 
The package includes solutions to four questions covering publishers, subscribers, custom messages, and services in ROS 2.

 
Name: Vahitha SJ 
Course/Assignment: Week 3 ROS 2 Assignment 
ROS 2 Distribution Used: Humble 

Package Information 
Package Name: kratos_vahitha 
Language: Python 
Dependencies: rclpy, std_msgs, geometry_msgs, builtin_interfaces 
The package is located in the src/kratos_vahitha directory. 
All scripts are stored in the scripts folder. 
Custom message and service definitions are in msg and srv folders. 

Question 1: Publisher and Subscriber 
Files: q1_pub.py and q1_sub.py 
Description: A basic publisher node publishes string messages at regular intervals. A subscriber node listens to the topic and prints the received messages. 
Concepts Covered: Topics, publishing, subscribing, and message passing. 

Question 2: Two Node Communication 
Files: q2_s1.py and q2_s2.py 
Description: Implemented two nodes where one node sends data (numbers or text) and the other node receives and processes it. 
Concepts Covered: Multi-node communication and synchronization between publisher and subscriber. 

Question 3: Custom Message - Rover Data
File: q3_rover.py 
Custom Message: RoverData.msg 
Description: A custom message named RoverData was created containing rover-related data such as speed, direction, and position. A node publishes and subscribes to this message. 
Concepts Covered: Custom messages, defining .msg files, using and testing them. 

Question 4: Service and Client 
Files: q4_server.py and q4_client.py 
Custom Service: CalculateArea.srv 
Description: A service definition was created to calculate the area of a rectangle. The server receives length and width from the client, calculates the area, and sends back the result. The client sends values and prints the response. 
Concepts Covered: Service and client communication, defining .srv files, request and response handling in ROS 2. 

How to Build and Run 

1. Clone the Repository 
git clone https://github.com/vahithasj/week3_submission_vahitha.git 
cd week3_submission_vahitha 

2. Build the Package 
cd ~/ros2_ws 
colcon build 
source install/setup.bash 

3. Run the Scripts 

Q1 Publisher and Subscriber 
Terminal 1: ros2 run kratos_vahitha q1_pub.py 
Terminal 2: ros2 run kratos_vahitha q1_sub.py 

Q2 Two Node Communication 
Terminal 1: ros2 run kratos_vahitha q2_s1.py 
Terminal 2: ros2 run kratos_vahitha q2_s2.py 

Q3 Custom Message Rover Data 
ros2 run kratos_vahitha q3_rover.py 

Q4 Service and Client 
Terminal 1 (server): ros2 run kratos_vahitha q4_server.py 
Terminal 2 (client): ros2 run kratos_vahitha q4_client.py 

Notes 
Always source the workspace before running: 
source install/setup.bash 

Executable Python files are inside kratos_vahitha/scripts 
Custom messages are inside kratos_vahitha/msg 
Custom services are inside kratos_vahitha/srv 

Conclusion 
This package demonstrates the following 
1. Basic ROS 2 publishers and subscribers 
2. Communication between multiple nodes 
3. Creating and using custom messages 
4. Implementing a service and client using custom .srv definitions 
>>>>>>> 571171d (Week 4A and 4B solutions completed)

