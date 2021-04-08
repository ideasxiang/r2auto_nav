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
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
import math
import cmath
import time
import scipy.stats
import random
from PIL import Image, ImageDraw

# constants
rotatechange = 0.5
speedchange = 0.1
occ_bins = [-1, 0, 60, 100]
stop_distance = 0.5
front_angle = 30
front_angles = range(-front_angle, front_angle + 1, 1)
scanfile = 'lidar.txt'
mapfile = f"newmap{time.strftime('%Y%m%d%H%M%S')}.txt"
laserfile = 'laser.txt'
angle_array = []
points = []
nan_array = []
laser_array = []
map_bg_color = 1


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

    return roll_x, pitch_y, yaw_z  # in radians


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Created publisher')

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.center_x = 0
        self.center_y = 0
        self.unmap_x = 0
        self.unmap_y = 0
        self.dist_x = 0
        self.dist_y = 0
        self.angle_to_unmap = 0
        self.laser_count = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

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

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y,
                                                                orientation_quat.z, orientation_quat.w)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # compute histogram to identify bins with -1, values between 0 and below 50,
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image
        occ_counts, edges, binnum = scipy.stats.binned_statistic(msgdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return

        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        # set current robot location to 0
        odata[grid_y][grid_x] = 0
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # find center of image
        i_centerx = iwidth / 2
        i_centery = iheight / 2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)

        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)

        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw) - 90, expand=True, fillcolor=map_bg_color)

        self.center_x = rotated.width // 2
        self.center_y = rotated.height // 2

        plt.xlabel(
            'Center X: %i, Center Y: %i, Unmapped X: %i, Unmapped Y: %i\n Dist X: %i, Dist Y: %i, Angle to unmapped: %f degrees' %
            (rotated.width // 2, rotated.height // 2, self.unmap_x, self.unmap_y, self.dist_x, self.dist_y,
             self.angle_to_unmap * (180 / math.pi)))
        plt.grid()

        plt.plot(self.unmap_x, self.unmap_y, 'rx')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        plt.savefig(f"{time.strftime('%Y%m%d%H%M%S')}.png")
        plt.cla()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        # self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        self.occdata = np.array(rotated)
        # print to file
        np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

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
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
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
        while (c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
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

    def pick_direction(self):
        self.get_logger().info('In pick_direction')


        # if not np.empty(self.occdata):
        #     occdata = self.occdata
        #     self.get_logger().info('in new code' + str(occdata))
        #     self.get_logger().info('in new 2 code')
        #     for i in range(np.size(occdata, 0)):
        #         for j in range(np.size(occdata, 1)):
        #             if (occdata[i][j] == 2):
        #                 if (self.Uboarder(i, j, occdata) == False):
        #                     occdata[i][j] = list(occdata[i][j]) + ['U']
        #
        #                 if (self.Dboarder(i, j, occdata) == False):
        #                     occdata[i][j] = list(occdata[i][j]) + ['D']
        #
        #                 if (self.Rboarder(i, j, occdata) == False):
        #                     occdata[i][j] = list(occdata[i][j]) + ['R']
        #
        #                 if (self.Dboarder(i, j, occdata) == False):
        #                     occdata[i][j] = list(occdata[i][j]) + ['L']
        #
        #     for i in range(np.size(occdata, 0)):
        #         for j in range(np.size(occdata, 1)):
        #             if (self.Uboarder(i, j, occdata) == True & self.Dboarder(i, j, occdata) == True & self.Rboarder(i, j,
        #                                                                                                             occdata) == True & self.Lboarder(
        #                     i, j, occdata) == True):
        #                 occdata[i][j] = list(occdata[i][j]) + ['P']
        #
        #     for i in range(np.size(occdata, 0)):
        #         for j in range(np.size(occdata, 1)):
        #             if (occdata[i][j][1] != 'P'):
        #                 occdata = self.rmUDLR(i, j, occdata)
        #
        #     for i in range(np.size(occdata, 0)):
        #         for j in range(np.size(occdata, 1)):
        #             if (len(occdata[i][j]) == 1):
        #                 occdata[i][j] = occdata[i][j] + ['P']
        #
        #     for i in range(np.size(occdata, 0)):
        #         for j in range(np.size(occdata, 1)):
        #             if (occdata[i][j][1] != 'P'):
        #                 self.unmap_x = i
        #                 self.unmap_y = j
        #
        # # np.savetxt(laserfile, self.laser_range)
        # # self.get_logger().info(self.laser_range.size)
        #
        # lri = (self.laser_range[front_angles] < float(stop_distance)).nonzero()
        #
        # unmap_x = self.unmap_x
        # unmap_y = self.unmap_y
        #
        # our_x = self.center_x
        # our_y = self.center_y
        #
        # x_dist = math.dist((unmap_x, 0), (our_x, 0))
        # y_dist = math.dist((0, unmap_y), (0, our_y))
        # self.dist_x = x_dist
        # self.dist_y = y_dist
        #
        # if (x_dist != 0):
        #     angle_to_unmap = math.atan(y_dist / x_dist)
        #     self.angle_to_unmap = angle_to_unmap
        #     self.get_logger().info('Angle chosen %f' % angle_to_unmap)

        if self.laser_range.size != 0:
            lr2i = np.nanargmax(self.laser_range)
            if self.angle_to_unmap != 0:
                lr2i = self.angle_to_unmap

        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
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

    # ______________________________________________________________________________
    def Uboarder(self, x, y, occdata):

        for i in range(0, x):
            if (occdata[i][y] == 3): return False
        return True

    def Dboarder(self, x, y, occdata):
        for i in range(x, np.size(occdata, 0)):
            if (occdata[i][y] == 3): return False
        return True

    def Lboarder(self, x, y, occdata):
        for j in range(0, y):
            if (occdata[x][j] == 3): return False
        return True

    def Rboarder(self, x, y, occdata):
        for j in range(y, np.size(occdata, 1)):
            if (occdata[x][j] == 3): return False
        return True

    def rmUDLR(self, x, y, occdata):
        # remove D
        for p in range(len(occdata[x + 1][y])):
            if (occdata[x + 1][y][p] == 'D'):  occdata[x][y].remove('D')

        # remove U
        for p in range(len(occdata[x - 1][y])):
            if (x >= 1 & (occdata[x - 1][y][p] == 'U')):  occdata[x][y].remove('U')

        # remove R
        for p in range(len(occdata[x][y + 1])):
            if (occdata[x][y + 1][p] == 'R'):  occdata[x][y].remove('R')

            # remove L
        for p in range(len(occdata[x][y - 1])):
            if (y >= 1 & (occdata[x][y - 1][p] == 'L')):  occdata[x][y].remove('L')

        return occdata

    # ___________________________________________________________________________________

    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():

                if (self.laser_range.size != 0):
                    # self.laser_count += 1
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles] < float(stop_distance)).nonzero()

                    # unmap_x = self.unmap_x
                    # unmap_y = self.unmap_y
                    #
                    # our_x = self.center_x
                    # our_y = self.center_y
                    #
                    # x_dist = math.dist((unmap_x, 0), (our_x, 0))
                    # y_dist = math.dist((0, unmap_y), (0, our_y))
                    # self.dist_x = x_dist
                    # self.dist_y = y_dist
                    #
                    # if (x_dist != 0):
                    #     angle_to_unmap = math.atan(y_dist / x_dist)
                    #     self.angle_to_unmap = angle_to_unmap
                    #     self.get_logger().info('Angle chosen %f' % angle_to_unmap)

                    # self.get_logger().info('Unmap X: %i, Unmap Y: %i, Ours X: %i, Ours Y: %i' % (unmap_x, unmap_y, our_x, our_y))
                    # self.get_logger().info('Dist X: %i, Dist Y: %i' % (x_dist, y_dist))

                    # if the list is not empty
                    if (len(lri[0]) > 0):
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