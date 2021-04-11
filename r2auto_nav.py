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
from std_msgs.msg import String
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
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.55
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
random_angle = [0, -90, 90, 180]


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
        self.fly_ = self.create_publisher(String, 'fly', 11)
        self.get_logger().info('Created publisher')

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.temp_subscription = self.create_subscription(
            String,
            'temp',
            self.temp,
            12)
        self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        self.temp_subscription
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
        self.dist_to_unmap = 0
        self.prev_dist_to_unmap = []
        self.laser_count = 0
        self.why_stop = 0
        self.shoot = 0
        self.mapped = 0
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

    def temp(self, msg):
        self.get_logger().info("In temp callback")
        self.get_logger().info("%s" % msg.data)
        obj_temp, ambient_temp = str(msg.data).split(',')
        if float(obj_temp) > 31:
            self.shoot = 1
            msg2 = String()
            msg2.data = "fly"
            self.fly_.publish(msg2)

    def bfs(self, graph, start, end):
        # maintain a queue of paths
        queue = []
        # visited = set([start])
        visited = []
        # push the first path into the queue
        queue.append(start)  # [[25,25]]
        # queue= collections.deque(start)
        visited.append(start)
        w = []
        l = 0
        while len(queue) > 0:
            # get the first path from the queue

            path = queue.pop(0)

            if (isinstance(path[0], int)):
                p = path
                l = 1
            else:
                p = path[-1]
                l = 0

            # xx.append(path)
            # print(new_path)
            # get the last node from the path
            # node = path[-1]
            # new_path = []
            # path found

            x = p[0]
            y = p[1]

            # enumerate all adjacent nodes, construct a new path and push it into the queue
            # new_path= list()
            # node x+1 y
            # print(x)
            # print(y)
            if x + 1 < 100 and [x + 1, y] not in visited and graph[x + 1, y] != 0:
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x + 1, y])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x + 1 == end[0] and y == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])
                        i = i + 1
                    new.append([x + 1, y])
                    queue.append(new)
                    if x + 1 == end[0] and y == end[1]:
                        # print("ccc")

                        return new
                # new_path.append([x+1,y])

                visited.append([x + 1, y])
            # node x+1 y+1
            if x + 1 < 100 and y + 1 < 40 and [x + 1, y + 1] not in visited and graph[x + 1, y + 1] != 0:
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x + 1, y + 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x + 1 == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x + 1, y + 1])
                    queue.append(new)
                    if x + 1 == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return new
                # new_path.append([x+1,y])
                visited.append([x + 1, y + 1])
            # node x y+1
            if y + 1 < 40 and [x, y + 1] not in visited and graph[x, y + 1] != 0:

                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x, y + 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x, y + 1])
                    queue.append(new)
                    if x == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return new
                visited.append([x, y + 1])
            # node x-1 y+1
            if x - 1 > -1 and y + 1 < 40 and [x - 1, y + 1] not in visited and graph[x - 1, y + 1] != 0:
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x - 1, y + 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x - 1 == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x - 1, y + 1])
                    queue.append(new)
                    if x - 1 == end[0] and y + 1 == end[1]:
                        # print("ccc")

                        return new
                visited.append([x - 1, y + 1])

            # node x-1 y
            if x - 1 > -1 and [x - 1, y] not in visited and graph[x - 1, y] != 0:
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x - 1, y])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x - 1 == end[0] and y == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x - 1, y])
                    queue.append(new)
                    if x - 1 == end[0] and y == end[1]:
                        # print("ccc")

                        return new
                # new_path.append([x+1,y])
                visited.append([x - 1, y])
            # node x-1 y-1
            if x - 1 > -1 and y - 1 > -1 and [x - 1, y - 1] not in visited and graph[x - 1, y - 1] != 0:
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x - 1, y - 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x - 1 == end[0] and y - 1 == end[1]:
                        print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x - 1, y - 1])
                    queue.append(new)
                    if x - 1 == end[0] and y - 1 == end[1]:
                        # print("ccc")

                        return new
                visited.append([x - 1, y - 1])

            # node x y-1
            if y - 1 > -1 and [x, y - 1] not in visited and graph[x, y - 1] != 0:

                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x, y - 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x == end[0] and y - 1 == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x, y - 1])
                    queue.append(new)
                    if x == end[0] and y - 1 == end[1]:
                        # print("ccc")

                        return new
                # new_path.append([x+1,y])
                visited.append([x, y - 1])
            # node x+1 y-1
            if x + 1 < 100 and y - 1 > -1 and [x + 1, y - 1] not in visited and graph[x + 1, y - 1] != 0:

                # new_path.append([x+1,y-1])
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x + 1, y - 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if x + 1 == end[0] and y - 1 == end[1]:
                        # print("ccc")

                        return q
                else:
                    i = 0
                    new = []
                    while (i <= len(path) - 1):
                        new.append(path[i])

                        i = i + 1

                    new.append([x + 1, y - 1])
                    queue.append(new)
                    if x + 1 == end[0] and y - 1 == end[1]:
                        # print("ccc")

                        return new
                visited.append([x + 1, y - 1])
        # print(len(queue))

        return None

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

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        # self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))


        # prev_unmap_x = self.unmap_x
        # prev_unmap_y = self.unmap_y
        #
        # if prev_unmap_x == self.unmap_x:
        #     self.unmap_x = 0
        # if prev_unmap_y == self.unmap_y:
        #     self.unmap_y = 0

        if self.mapped == 0:
            self.mapped = 1
            self.occdata = np.array(rotated)
            for i in range(1, np.size(self.occdata, 0) - 1):
                for j in range(1, np.size(self.occdata, 1) - 1):
                    # self.get_logger().info(str(self.occdata[i, j]))
                    if self.occdata[i, j] == 1 and self.occdata[i + 1, j] == 2 and self.occdata[i, j + 1] == 3:
                        # self.get_logger().info('%d %d %d' % (self.occdata[i, j], self.occdata[i + 1, j], self.occdata[i, j+1]))
                        self.unmap_x = j
                        self.unmap_y = i

            unmap_x = self.unmap_x
            unmap_y = self.unmap_y
            # self.get_logger().info('%d %d' % (unmap_x, unmap_y))

            our_x = self.center_x
            our_y = self.center_y

            x_dist = math.dist((unmap_x, 0), (our_x, 0))
            y_dist = math.dist((0, unmap_y), (0, our_y))

            x_negative = our_x > unmap_x
            y_negative = our_y > unmap_y

            self.dist_x = x_dist
            self.dist_y = y_dist
            self.dist_to_unmap = math.dist((unmap_x, unmap_y), (our_x, our_y))
            self.prev_dist_to_unmap.append(self.dist_to_unmap)

            if x_dist != 0:
                angle_to_unmap = math.atan(y_dist / x_dist)
                if x_negative and y_negative:
                    self.angle_to_unmap = (angle_to_unmap + math.pi) * 180 / math.pi
                elif x_negative and (not y_negative):
                    self.angle_to_unmap = (angle_to_unmap - math.pi / 2) * 180 / math.pi
                elif not x_negative and y_negative:
                    self.angle_to_unmap = (angle_to_unmap + math.pi / 2) * 180 / math.pi
                else:
                    self.angle_to_unmap = angle_to_unmap * 180 / math.pi

            plt.xlabel(
                'Center X: %i, Center Y: %i, Unmapped X: %i, Unmapped Y: %i\n Dist X: %i, Dist Y: %i, Angle to unmapped: '
                '%f degrees Distance to unmap: %i' %
                (rotated.width // 2, rotated.height // 2, self.unmap_x, self.unmap_y, self.dist_x, self.dist_y,
                 self.angle_to_unmap, self.dist_to_unmap))
            plt.grid()
            plt.plot(self.unmap_x, self.unmap_y, 'rx')
            plt.imshow(rotated, cmap='gray', origin='lower')
            plt.draw_all()
            plt.savefig(f"{time.strftime('%Y%m%d%H%M%S')}.png")
            plt.cla()
            # pause to make sure the plot gets created
            plt.pause(0.00000000001)
            # print to file
            np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
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
        # self.get_logger().info('In pick_direction')

        # np.savetxt(laserfile, self.laser_range)
        if self.laser_range.size != 0:
            rnum = random.randint(0, 360)

            lr2i = rnum
            # if self.why_stop == 1:
            #     lr2i = rnum
            #     self.get_logger().info('stop for unmap angle: %i' % lr2i)
            #     self.why_stop = 0
            #
            # if self.why_stop == 2:
            #     lr2i = 45
            #     self.get_logger().info('stop for obstacle')
            #     self.why_stop = 0

            # use nanargmax as there are nan's in laser_range added to replace 0's
            # laser_array = self.laser_range
            # occdata = self.occdata

            # angle = np.nanargmax(laser_array)
            # lr2i = np.nanargmax(self.laser_range)
            # if (angle in nan_array):
            #     np.delete(laser_array, angle)

            # for i in range(0, 360):
            #     if (self.laser_range[i] not in nan_array):
            #         if (math.isnan(self.laser_range[i])):
            #             nan_array.append(i)
            #             lr2i = i

            # for i in range(0,360):
            #     if (math.isnan(self.laser_range[i])):
            #         lr2i = i

            # points.append([self.x, self.y, angle])

            # self.get_logger().info(str(laser_array))
            # self.get_logger().info(str(occdata))
            # for angle in range(0,360):
            #     for r in np.linspace(0,3,50):
            #         x_coord = int(self.x)
            #         y_coord = int(self.y)
            #         if (x_coord in occdata and y_coord in occdata[0]):
            #             y_add = int(r * math.cos(angle))
            #             if (y_add < len(occdata[0])):
            #                 if (occdata[r][y_add] == 0):
            #                     lr2i = angle

            # for i in range(len(points)):
            #     if not(self.x in range(int(points[i][0])-10, int(points[i][0])+10) and self.y in range(int(points[i][1])-10, int(points[i][1])+10) and angle in range(points[i][2]-10,points[i][2]+10)):
            #         lr2i = angle
            #     else:
            #         np.delete(laser_array, angle)
            #         angle = np.nanargmax(laser_array)

            # lr2i = angle
            # if (angle in angle_array):
            #     np.delete(laser_array, angle)
            #
            # else:
            #     lr2i = angle
            #     angle_array.append(angle)

            # self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
            # self.get_logger().info(str(angle_array))

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

    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():

                if self.shoot == 1:
                    self.stopbot()
                    time.sleep(15)
                    self.pick_direction()

                if self.laser_range.size != 0:
                    # self.laser_count += 1
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles] < float(stop_distance)).nonzero()
                    # if len(self.prev_dist_to_unmap) - 1 > 0:
                    #     if self.dist_to_unmap > self.prev_dist_to_unmap[len(self.prev_dist_to_unmap) - 2]:
                    #         self.get_logger().info(
                    #             'prev dist to unmap: %i' % self.prev_dist_to_unmap[len(self.prev_dist_to_unmap) - 2])
                    #         self.why_stop = 1
                    #         self.stopbot()
                    #         self.pick_direction()

                    # self.get_logger().info('Angle chosen %f' % angle_to_unmap)

                    # self.get_logger().info('Unmap X: %i, Unmap Y: %i, Ours X: %i, Ours Y: %i' % (unmap_x, unmap_y, our_x, our_y))
                    # self.get_logger().info('Dist X: %i, Dist Y: %i' % (x_dist, y_dist))

                    # self.get_logger().info('Distances: %s' % str(lri))
                    # laser_array.append(self.laser_range)
                    # self.get_logger().info(str((self.x, self.y)))
                    # self.get_logger().info(str(laser_array))
                    # self.get_logger().info('Laser count %d' % (self.laser_count))
                    # if (self.laser_count-10 >= 0):
                    #     prev_laser = laser_array[self.laser_count-10]
                    #     self.get_logger().info('Prev at 90 degress %f' % (prev_laser[90]))
                    #     for i in range(0,360):
                    #         if (math.isnan(self.laser_range[i])):
                    #             if not (math.isnan(prev_laser[i])):
                    #                 self.get_logger().info('First Works')
                    #                 # self.stopbot()
                    #                 # self.pick_direction()
                    #
                    #         else:
                    #             if not math.isnan(prev_laser[i]):
                    #                 if (self.laser_range[i] - prev_laser[i]) > 2.0:
                    #                     self.get_logger().info('Second Works')
                    #                     self.stopbot()
                    #                     self.pick_direction()

                    # self.get_logger().info('Distance at 90 degress %f' % (self.laser_range[90]))

                    # for i in range(0, 360):
                    #     if (math.isnan(self.laser_range[i])):
                    #         if (i not in nan_array):
                    #             nan_array.append(i)
                    #             self.stopbot()
                    #             self.pick_direction()

                    # if the list is not empty
                    if (len(lri[0]) > 0):
                        self.why_stop = 2
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
