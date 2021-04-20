# EG2310 Group 9 Code
These repo includes different algorithms for mapping using lidar on Turtlebot. R7auto_nav implements the code that was used in the demo and integrated the firing mechanism.

R2auto_nav is implements basic unmapped area detected, R3 and R4 are duplicates of R2. R5 attempts to implement breath first search and has worked in gazebo but not on actual map. R6 and R7 implements wall following algorithm. R6 takes the turtlebot as the front when moving forward while R7 takes the payload as the front.

Information about the turtlebot3 burger used can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).

Installation can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) (Make sure to click on Foxy tab).

------
### Getting Started
1. Under *~/colcon_ws/build/auto_nav*, do `git clone https://github.com/ideasxiang/r2auto_nav.git`
2. Connect to Wifi hotspot
3. SSH into turtlebot using `sshrp`
4. Run `rosbu` in terminal connected to Turtlebot
5. Run `rslam` in another terminal
6. Open another terminal and `sshrp`
7. Copy the code for fly_sub3.py onto the rpi on Turtlebot and edit the port connected to thermal sensor (See below to find the port used)
8. Run `python3 fly_sub3.py` in the same terminal
9. Open another terminal and `sshrp`
10. Copy the code for fly_pub3.py onto the rpi on Turtlebot and edit the port connected to thermal sensor (See below to find the port used)
11. Run `python3 fly_pub3.py` in the same terminal
12. In a new terminal, `cd ~/colcon_ws` and do `colcon build` (Ensure that installation for Turtlebot3 Foxy is done)
13. `ros2 run auto_nav r7auto_nav` to run the wall following algorithm (Ensure in the folder auto_nav that setup.py has correct code)
14. Voilà you are done

### Checking USB port used
1. `sudo apt-get install minicom`
2. `dmesg | grep tty`
3. Note the tty port used eg. *ttyUSB1*
