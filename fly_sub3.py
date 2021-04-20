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
import serial
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

servo_pin = 4

GPIO.setup(servo_pin, GPIO.OUT)

p2= GPIO.PWM(servo_pin, 50)

p2.start(5.5)

wheel_pin = 23
# p=GPIO.PWM(wheel_pin,50)
GPIO.setup(wheel_pin, GPIO.OUT)
choice = 0
p = GPIO.PWM(wheel_pin, 50)
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=2)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'fly',
            self.listener_callback,
            11)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == "fly":
            GPIO.output(wheel_pin, 1)
            time.sleep(15)
            GPIO.output(wheel_pin, 0)
            time.sleep(3)
        if "servo" in msg.data:
            p2.ChangeDutyCycle(float(msg.data.split(',')[-1]))
            time.sleep(1)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'temp', 12)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        readText = ser.readline()
        decode = readText.decode('UTF-8')
        msg = String()
        msg.data = decode
        self.publisher_.publish(msg)
        self.get_logger().info('%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_subscriber)
    #rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        p2.stop()
        p.stop()
        GPIO.output(wheel_pin, 0)
        GPIO.output(servo_pin, 0)
        GPIO.cleanup()
