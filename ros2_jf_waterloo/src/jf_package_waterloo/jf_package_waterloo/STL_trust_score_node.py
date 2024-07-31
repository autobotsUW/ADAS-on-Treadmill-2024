import rtamt
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime
import os
from std_msgs.msg import String,Int32MultiArray
import rtamt
import matplotlib.pyplot as plt
import numpy as np
import time 


class STL:
    def __init__(self,specification,n_value):
        self.spec = rtamt.StlDiscreteTimeSpecification()
        if 'y' in specification:
            self.spec.declare_var('y', 'float')
            self.var='y'
        else:
            self.spec.declare_var('x', 'float')
            self.var='x'
        self.spec.spec = specification
        self.n_value = n_value
        self.Lt = [0]
        self.Lsignal = [0]
        self.Lrobustness = [0]
        self.spec.parse()
        # self.fig, self.ax1 = plt.subplots()
        # self.ax1.set_xlabel('Time')
        # self.ax1.set_ylabel('Signal', color='tab:blue')
        # self.line_signal, = self.ax1.plot([], [], label='signal', color='tab:blue')
        # self.ax1.tick_params(axis='y', labelcolor='tab:blue')
        # # self.ax1.set_ylim(-20, 20)
        # # self.ax1.axhline(y=-5, color='gray', linestyle='--')
        # # self.ax1.axhline(y=5, color='gray', linestyle='--')
        # self.ax2 = self.ax1.twinx()
        # self.ax2.set_ylabel('Robustness', color='tab:red')
        # self.line_robustness, = self.ax2.plot([], [], label='Robustness', color='tab:red')
        # self.ax2.tick_params(axis='y', labelcolor='tab:red')
        # # self.ax2.set_ylim(-20, 20)
        # self.ax2.axhline(y=0, color='black', linestyle='--')

    def __str__(self):
        return 'STL'


    def parse(self):
        try:
            self.spec.parse()
        except rtamt.RTAMTException as e:
            print(f"Erreur lors de la compilation de la spécification : {e}")
            exit(1)

    def evaluate(self,):
        if len(self.Lt) <= self.n_value:
            n=0
            # print('n_value is too high')
        else:
            n = self.n_value
        trace = {'time': self.Lt[-n:], self.var: self.Lsignal[-n:]}
        robustness = self.spec.evaluate(trace)
        self.Lrobustness.append(robustness[0][1])
    
    def add_value(self,t,x):
        self.Lt.append(t)
        self.Lsignal.append(x)
        self.evaluate()
        # if len(self.Lt)%10==0:
        #     self.update_plot()
    
    def update_plot(self):
        self.line_signal.set_data(self.Lt, self.Lsignal)
        self.line_robustness.set_data(self.Lt, self.Lrobustness)
        self.ax1.set_xlim(0, max(self.Lt))
        self.ax2.set_xlim(0, max(self.Lt))
        self.ax1.set_ylim(min(self.Lsignal)-1, max(self.Lsignal)+1)
        val_max=max(abs(min(self.Lrobustness)),abs(max(self.Lrobustness)))
        self.ax2.set_ylim(-val_max-1, val_max+1)
        plt.draw()
        plt.pause(0.001)

class car_Class():
    """
    Class to have all information for one car
    """

    def __init__(self,id,t,x,y,lines):
        self.id=id
        self.xdata=[x]
        self.ydata=[y]
        self.time=[t]
        DictColor={0:'purple',1:'red',2:'blue'}

        self.fig, self.ax1 = plt.subplots()
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Signal', color='black')
        self.yline_signal, = self.ax1.plot([], [], label='signal', color='black')

        self.ax2 = self.ax1.twinx()
        self.ax2.set_ylabel('Robustness')
        self.ax2.axhline(y=0, color='black', linestyle='--')

        self.Lstl=[]
        for i in range(len(lines)-1):
            line_robustness, = self.ax2.plot([], [], label='Robustness lane {}'.format(i+1))
            self.Lstl.append(['y',STL('G[0,30] (y >= {} and y <= {})'.format(lines[i],lines[i+1]),30),line_robustness])

        self.ax1.set_title('Car {}'.format(id))
        plt.legend()


    def calculate_trust_score(self,t,x,y):
        self.xdata.append(x)
        self.ydata.append(y)
        self.time.append(t)
        for stl in self.Lstl:
            if stl[0]=='y':
                stl[1].add_value(t,y)
            else:
                stl[1].add_value(t,x)
        self.update_plot()

    def update_plot(self):
        self.yline_signal.set_data(self.time,self.ydata)
        
        for stl in self.Lstl:
            # minimal_subscriber.get_logger().info("{} ".format(stl[1]))
            # minimal_subscriber.get_logger().info("{} {}".format(stl[1].Lt,stl[1].Lrobustness))
            stl[2].set_data(stl[1].Lt,stl[1].Lrobustness)
        self.ax1.set_xlim(0, max(self.time))
        self.ax2.set_xlim(0, max(self.time))
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        plt.draw()
        minimal_subscriber.get_logger().info("Plot {:.2f}".format(self.time[-1]))
        plt.pause(0.001)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        self.treadmill_sub = self.create_subscription(Int32MultiArray, 'treadmill_position', self.treadmill_sub_function, 1)
        self.t0=0
        
        self.car_sub  # prevent unused variable warning
        self.DictCar={}
        self.file_name=os.path.expanduser('~/ADAS-on-Treadmill-2024/Mesure/STL_Trust_score_{}.svg'.format(datetime.now().strftime("%Y-%m-%d %H:%M")))
        self.lines=False

    def treadmill_sub_function(self,msg):
        """
        Read treadmill position
        """
        # self.get_logger().info('{}'.format(msg.data))
        if len(msg.data)>=3:
            self.treadmill=msg.data[:3]
        if len(msg.data)>=3 and self.lines==False:
            self.lines=msg.data[3:]

    def car_sub_function(self, msg):
        if self.t0==0 and self.lines==False:
            return
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
                car=car_Class(id,t,x,y,self.lines)
                self.DictCar[id]=car
                # car.calculate_trust_score(t,x,y)

def main(args=None):
    rclpy.init(args=args)
    global minimal_subscriber
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()








