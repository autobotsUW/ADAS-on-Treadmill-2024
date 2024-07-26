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
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import time
from datetime import datetime
import os
from std_msgs.msg import String,Int32MultiArray

class car_Class():
    """
    Class to have all information for one car
    """

    def __init__(self,id,t,x,y):
        self.id=id
        self.xdata=[x]
        self.ydata=[y]
        self.time=[t]
        self.trust_score=[0]
        DictColor={0:'purple',1:'red',2:'blue'}
        self.plot,=ax.plot([], [], lw=5, color=DictColor[id],label='Car {}'.format(id))
        

    def calculate_trust_score(self,t,x,y):
        n=100
        self.time.append(t)
        self.xdata.append(x)
        self.ydata.append(y)
        if len(self.time) > n:
            var_x = self.variance(self.xdata[-n:])
            var_y = self.variance(self.ydata[-n:])
            self.trust_score.append(var_x + var_y)
        else:
            var_x = self.variance(self.xdata)
            var_y = self.variance(self.ydata)
            self.trust_score.append(var_x + var_y)
        self.plot.set_data(self.time, self.trust_score)
        
        
    def variance(self,data):
        n = len(data)
        if n < 2:
            raise ValueError("La variance nécessite au moins deux éléments.")
        
        mean = sum(data) / n
        deviations = [(x - mean) ** 2 for x in data]
        variance = sum(deviations) / (n - 1)  # Utilisation de (n - 1) pour la variance non biaisée
        
        return variance      

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        self.car_sub  # prevent unused variable warning
        self.DictCar={}
        self.file_name=os.path.expanduser('~/ADAS-on-Treadmill-2024/Mesure/Trust_score_{}.png'.format(datetime.now().strftime("%Y-%m-%d %H:%M")))
        plt.ion()  # Mode interactif
        global fig,ax
        fig, ax = plt.subplots()

        # Ajout des noms des axes
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Trust Score')
        self.t0=0
        self.i=0
        # self.line, = self.ax.plot([], [], lw=2)
        # self.ax.set_ylim(0, 1)  # Ajuster la limite y selon vos besoins

    def car_sub_function(self, msg):
        if self.t0==0:
            self.t0=time.time()
            t=0
        else:
            t=time.time()-self.t0

        for i in range(0,len(msg.data),6):
            id,x,y=msg.data[i:i+3]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                car.calculate_trust_score(t,x,y)
            else:
                car=car_Class(id,t,x,y)
                self.DictCar[id]=car
                # car.calculate_trust_score(t,x,y)
                ax.legend()
        self.plot_trust_score()


        self.i+=1
        # self.get_logger().info(str(self.i))
        if (self.i%30)==0:
            # self.get_logger().info("Save Trust score figure")
            fig.savefig(self.file_name)

    def plot_trust_score(self):
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        


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








