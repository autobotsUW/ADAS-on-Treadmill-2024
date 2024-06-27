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

from std_msgs.msg import String,Int32MultiArray
import time as t
from datetime import datetime
import os

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Int32MultiArray,'car_position',self.listener_callback,10)
        self.subscription = self.create_subscription(Int32MultiArray,'input_position',self.input_listener_callback,10)
        self.subscription = self.create_subscription(Int32MultiArray,'command',self.command_callback,10)
        self.subscription = self.create_subscription(Int32MultiArray,'obstacles_position',self.obstacles_callback,10)


        self.file_name=os.path.expanduser('~/ADAS-on-Treadmill-2024/Mesure/Mesure {}.csv'.format(datetime.now().strftime("%Y-%m-%d %H:%M")))
        self.input=[300,200]
        self.command=[0,0]
        self.obstacles=[]
        self.t0=t.time()
        fichier=open(self.file_name,'w')
        fichier.write("Time (s);    ;car_position;   ;input;    ;command;   ;obstacles\n")
        fichier.close()
        self.input 
        self.subscription  # prevent unused variable warning

    def obstacles_callback(self,msg):
        self.obstacles=msg.data

    def command_callback(self,msg):
        self.command=msg.data

    def input_listener_callback(self,msg):
        if len(msg.data)>=1:
            # self.get_logger().info('I heard: {} {}'.format(self.input[0],self.input[1]))
            self.input=msg.data

    def listener_callback(self, msg):
        if len(msg.data)>3:
            data="{:.3f}".format(t.time()-self.t0)
            data+=';    '
            for i in msg.data:
                data+=";"+str(i)
            data+=';    '
            for input in self.input:
                data+=";"+str(input)
            data+=';    '
            for command in self.command:
                data+=";"+str(command)
            data+=';    '
            for obstacle in self.obstacles:
                data+=";"+str(obstacle)

            file=open(self.file_name,'a')
            file.write(data+'\n')
            file.close()
            # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
