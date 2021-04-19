# EG2310 Group 9 Code
These repo includes different algorithms for mapping using lidar on Turtlebot. R7auto_nav implements the code that was used in the demo and integrated the firing mechanism. </br>
Scroll down to getting started if you are not interested in the other codes. </br>
R2auto_nav is implements basic unmapped area detected, R3 and R4 are duplicates of R2. R5 attempts to implement breath first search and has worked in gazebo but not on actual map. R6 and R7 implements wall following algorithm. R6 takes the turtlebot as the front when moving forward while R7 takes the payload as the front. </br>
Information about the turtlebot3 burger used can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

------
### Getting Started
1. Connect to Wifi hotspot
2. SSH into turtlebot using `sshrp`
3. Run `rosbu` in terminal connected to Turtlebot
4. Run `rslam` in another terminal
5. `cd ~/colcon_ws` and do `colcon build` in a new terminal
6. Open another terminal and `sshrp`
7. Run `python3 fly_sub3.py` in the same terminal (Make sure to copy the code for fly_sub3.py onto the rpi on Turtlebot)
8. Open another terminal and `sshrp`
9. Run `python3 fly_pub3.py` in the same terminal (Make sure to copy the code for fly_pub3.py onto the rpi on Turtlebot)
10. `ros2 run auto_nav r7auto_nav` to run the wall following algorithm
11. Voilà you are done 
