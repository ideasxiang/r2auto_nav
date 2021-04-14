# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.center_x = 0
        self.center_y = 0

    def odom_callback(self, msg):
        self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        self.get_logger().info(str(self.occdata))
        np.savetxt(mapfile, self.occdata)
        
        self.center_x = msg.info.width //2
        self.center_y = msg.info.height//2
        
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


#________________________________________________________________________________       
    def Uboarder(self,x,y,occdata):       
        
        for i in range (0,x):
           if (occdata[i][y]==3): return False
        return True
        
        
    def Dboarder(self,x,y,occdata):
        for i in range(x,np.size(occdata, 0)):
           if (occdata[i][y]==3): return False
        return True
    
    
    def Lboarder(self,x,y,occdata):
        for j in range (0,y):
           if (occdata[x][j]==3): return False
        return True
     
     
    def Rboarder(self,x,y,occdata):
        for j in range (y,np.size(occdata,1)):
           if (occdata[x][j]==3): return False
        return True   
    
    
    def rmUDLR(self,x,y,occdata):
    #remove D
       for p in range(len(occdata[x+1][y])):
          if (occdata[x+1][y][p]=='D'):  occdata[x][y].remove('D')
             
    #remove U
       for p in range(len(occdata[x-1][y])):
          if (x>=1 & (occdata[x-1][y][p]=='U')):  occdata[x][y].remove('U')
       
    #remove R
       for p in range(len(occdata[x][y+1])):
          if (occdata[x][y+1][p]=='R'):  occdata[x][y].remove('R')       
          
    #remove L
       for p in range(len(occdata[x][y-1])):
          if (y>=1 & (occdata[x][y-1][p]=='L')):  occdata[x][y].remove('L')   
       return occdata
#_________________________________________________________________________________________________  
    def pick_direction(self):
        occdata = self.occdata
        self.get_logger().info('In pick_direction_test')
        self.get_logger().info(str(self.occdata))
        
        if (self.laser_range.size != 0 & np.size(occdata,0)!=0 ):
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)        
#_________________________________________________________________________________________________
            self.get_logger().info(str(np.size(occdata,0)))
            for i in range(np.size(occdata,0)):
               for j in range(np.size(occdata, 1)):
                   if (occdata[i][j]!=0): self.get_logger().info('yes!!!!!!!!')
                   #if(occdata[i][j]>=10): occdata[i][j]=3
                   #elif(occdata[i][j]<10 & occdata[i][j]>0): occdata[i][j]=2
                   #else: occdata[i][j]=1;
            
            for i in range(np.size(occdata,0)):
               for j in range(np.size(occdata, 1)):
                  self.get_logger().info('1')
                  if (occdata[i][j]==2):
                      if(self.Uboarder(i,j,occdata)==False):
                          occdata[i][j]=list(occdata[i][j])+['U']
                                 
                      if(self.Dboarder(i,j,occdata)==False):
                          occdata[i][j]=list(occdata[i][j])+['D']
                              
                      if(self.Rboarder(i,j,occdata)==False):
                          occdata[i][j]=list(occdata[i][j])+['R']                             
                            
                      if(self.Dboarder(i,j,occdata)==False):
                          occdata[i][j]=list(occdata[i][j])+['L']                  
            for i in range(np.size(occdata, 0)):
              for j in range(np.size(occdata, 1)):                          
                if(self.Uboarder(i,j,occdata)==True & self.Dboarder(i,j,occdata)==True & self.Rboarder(i,j,occdata)==True & self.Lboarder(i,j,occdata)==True):
                    occdata[i][j]=list(occdata[i][j])+['P']

            for i in range(np.size(occdata, 0)):
                for j in range(np.size(occdata, 1)):    
                  if (occdata[i][j][1]!='P'):
                    occdata=self.rmUDLR(i,j,occdata)
                              
            for i in range(np.size(occdata, 0)):
                 for j in range(np.size(occdata, 1)):
                     if (len(occdata[i][j])==1): 
                       occdata[i][j]=occdata[i][j]+['P']

            for i in range(np.size(occdata, 0)):
                 for j in range(np.size(occdata, 1)):
                     if (occdata[i][j][1]!='P'): 
                         unmap_x = i
                         unmap_y = j
                 
            our_x = self.center_x
            our_y = self.center_y

            x_dist = math.dist((unmap_x, 0), (our_x, 0))
            y_dist = math.dist((0, unmap_y), (0, our_y))
            self.dist_x = x_dist
            self.dist_y = y_dist

            if (x_dist != 0):
               angle_to_unmap = math.atan(y_dist / x_dist)
               self.angle_to_unmap = angle_to_unmap
               self.get_logger().info('Angle chosen %f' % angle_to_unmap)  
                 
            if self.angle_to_unmap != 0:
                 lr2i = self.angle_to_unmap      
            
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
            
#____________________________________________________________________________________   

        else:
            lr2i = 0
            self.get_logger().info('No data!')   
            
        #  to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist) 




    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()
            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
