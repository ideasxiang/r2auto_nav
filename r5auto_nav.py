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
rotatechange = 0.6
speedchange = 0.2
occ_bins = [-1, 0, 60, 100]
stop_distance = 0.6
front_angle = 30
front_angles = range(-front_angle, front_angle + 1, 1)
scanfile = 'lidar.txt'
mapfile = f"newmap{time.strftime('%Y%m%d%H%M%S')}.txt"
laserfile = 'laser.txt'
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
        self.shoot = 0
        self.bfs_run = 0
        self.map_resolution = 0
        self.bot_position = [0, 0]
        self.points_to_move = np.array([])
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

    def temp(self, msg):
        self.get_logger().info("In temp callback")
        self.get_logger().info("%s" % msg.data)
        obj_temp, ambient_temp = str(msg.data).split(',')
        if float(obj_temp) > 31:
            self.shoot = 1

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
        self.map_resolution = map_res
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
        self.bot_position = [grid_y, grid_x]
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

        self.occdata = np.array(img)
        # self.get_logger().info(str(self.occdata))
        # print to file

    def bfs(self, graph, start):

        padding = 1

        def checkThree(data, x1, y1):
            for k in range(-padding, padding+1):
                for j in range(-padding, padding+1):
                    if data[x1 + k, y1 + j] == 3:
                        return False
            return True

        ###
        # Note X and Y is swap
        ###

        self.get_logger().info("Running BFS")
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
            if x + 1 < len(graph[0]) - padding - 1 and [x + 1, y] not in visited and graph[x + 1, y] != 3:
                if checkThree(graph, x + 1, y):
                    if l == 1:
                        q = [path, [x + 1, y]]
                        queue.append(q)
                        if graph[x + 1, y] == 1:
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
                        if graph[x + 1, y] == 1:
                            # print("ccc")

                            return new
                    # new_path.append([x+1,y])

                visited.append([x + 1, y])
            # node x+1 y+1
            if x + 1 < len(graph[0]) and y + 1 < len(graph) and [x + 1, y + 1] not in visited and graph[
                x + 1, y + 1] != 3 and checkThree(graph, x + 1, y + 1):
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x + 1, y + 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if graph[x + 1, y + 1] == 1:
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
                    if graph[x + 1, y + 1] == 1:
                        # print("ccc")

                        return new
                # new_path.append([x+1,y])
                visited.append([x + 1, y + 1])
            # node x y+1
            if y + 1 < len(graph) - padding - 1 and [x, y + 1] not in visited and graph[x, y + 1] != 3:
                if checkThree(graph, x, y + 1):
                    if (l == 1):
                        q = []
                        q.append(path)
                        q.append([x, y + 1])  # queue.append( path + [x+1,y])
                        queue.append(q)
                        if graph[x, y + 1] == 1:
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
                        if graph[x, y + 1] == 1:
                            # print("ccc")

                            return new
                    visited.append([x, y + 1])
            # node x-1 y+1
            if x - 1 > -1 and y + 1 < len(graph) and [x - 1, y + 1] not in visited and graph[
                x - 1, y + 1] != 3 and checkThree(graph, x - 1, y + 1):
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x - 1, y + 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if graph[x - 1, y + 1] == 1:
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
                    if graph[x - 1, y + 1] == 1:
                        # print("ccc")

                        return new
                visited.append([x - 1, y + 1])

            # node x-1 y
            if x - 1 > -padding and [x - 1, y] not in visited and graph[x - 1, y] != 3:
                if checkThree(graph, x - 1, y):
                    if (l == 1):
                        q = []
                        q.append(path)
                        q.append([x - 1, y])  # queue.append( path + [x+1,y])
                        queue.append(q)
                        if graph[x - 1, y] == 1:
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
                        if graph[x - 1, y] == 1:
                            # print("ccc")

                            return new
                    # new_path.append([x+1,y])
                    visited.append([x - 1, y])
            # node x-1 y-1
            if x - 1 > -1 and y - 1 > -1 and [x - 1, y - 1] not in visited and graph[x - 1, y - 1] != 3 and checkThree(graph, x - 1, y - 1):
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x - 1, y - 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if graph[x - 1, y - 1] == 1:
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
                    if graph[x - 1, y - 1] == 1:
                        # print("ccc")

                        return new
                visited.append([x - 1, y - 1])

            # node x y-1
            if y - 1 > -padding and [x, y - 1] not in visited and graph[x, y - 1] != 3:
                if checkThree(graph, x, y - 1):
                    if (l == 1):
                        q = []
                        q.append(path)
                        q.append([x, y - 1])  # queue.append( path + [x+1,y])
                        queue.append(q)
                        if graph[x, y - 1] == 1:
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
                        if graph[x, y - 1] == 1:
                            # print("ccc")

                            return new
                    # new_path.append([x+1,y])
                    visited.append([x, y - 1])
            # node x+1 y-1
            if x + 1 < len(graph[0]) and y - 1 > -1 and [x + 1, y - 1] not in visited and graph[
                x + 1, y - 1] != 3 and checkThree(graph, x + 1, y - 1):

                # new_path.append([x+1,y-1])
                if (l == 1):
                    q = []
                    q.append(path)
                    q.append([x + 1, y - 1])  # queue.append( path + [x+1,y])
                    queue.append(q)
                    if graph[x + 1, y - 1] == 1:
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
                    if graph[x + 1, y - 1] == 1:
                        # print("ccc")

                        return new
                visited.append([x + 1, y - 1])
        # print(len(queue))

        return []

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
        if self.laser_range.size != 0:
            lr2i = random.randint(0, 360)
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
            while rclpy.ok():
                twist = Twist()

                if len(self.occdata) != 0:
                    # note that self.occdata is y then x
                    bot_position = self.bot_position
                    self.get_logger().info("bot position %s" % str(bot_position))
                    points_to_move = self.bfs(self.occdata, bot_position)

                    if len(points_to_move) == 0:
                        self.bfs_run = 1
                    else:
                        self.bfs_run = 0

                    for i in points_to_move:
                        self.occdata[i[0]][i[1]] = 0
                    np.savetxt(mapfile, self.occdata)
                    map_res = self.map_resolution
                    max_go = 0

                    # plt.cla()
                    if self.bfs_run == 0:
                        while len(points_to_move) != 0 and max_go < 10:
                            max_go += 1
                            if self.shoot == 1:
                                self.shoot = 0
                                self.stopbot()
                                msg2 = String()
                                msg2.data = "fly"
                                self.fly_.publish(msg2)
                                time.sleep(20)

                            # if len(points_to_move) > 1:
                            #     points_to_move.pop(0)

                            first_point = points_to_move.pop(0)

                            self.get_logger().info("%s" % str(points_to_move))
                            # self.get_logger().info("last point %d %d" % (first_point[1], first_point[0]))
                            distance_to_point = math.dist(first_point, bot_position)
                            angle_to_move = math.degrees(
                                math.atan2(first_point[0] - bot_position[0], first_point[1] - bot_position[1])) % 360
                            plt.grid()
                            plt.xlabel("Rotation %f" % angle_to_move)
                            plt.imshow(Image.fromarray(self.occdata), cmap='gray', origin='lower')
                            plt.draw_all()
                            # plt.savefig(f"{time.strftime('%Y%m%d%H%M%S')}.png")
                            # pause to make sure the plot gets created
                            plt.pause(0.00000000001)
                            current_yaw = self.yaw
                            self.get_logger().info('Current: %f' % math.degrees(current_yaw))
                            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
                            target_yaw = math.radians(angle_to_move)
                            # target_yaw = math.radians(0)
                            c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
                            self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
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
                            twist.angular.z = 0.0
                            self.publisher_.publish(twist)
                            twist.linear.x = speedchange
                            self.publisher_.publish(twist)
                            time.sleep((distance_to_point * map_res * 1.2) / speedchange)
                            twist.linear.x = 0.0
                            self.publisher_.publish(twist)
                            bot_position = first_point
                        # twist.angular.z = rotatechange
                        # self.publisher_.publish(twist)
                        # time.sleep(2*math.pi/rotatechange)
                        # twist.angular.z = 0.0
                        # self.publisher_.publish(twist)
                    # if self.bfs_run == 1:
                    #     if self.laser_range.size != 0:
                    #         lri = (self.laser_range[front_angles] < float(stop_distance)).nonzero()
                    #
                    #         if (len(lri[0]) > 0):
                    #             # stop moving
                    #             self.stopbot()
                    #             # find direction with the largest distance from the Lidar
                    #             # rotate to that direction
                    #             # start moving
                    #         self.pick_direction()

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
