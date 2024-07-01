

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray,Int32MultiArray

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np



class Display(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.get_logger().info("Display Node started.\n")
        self.serial_sub = self.create_subscription(Int32MultiArray, 'command',self.command_sub_function, 10)
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        self.obstacles_sub = self.create_subscription(Int32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.input_sub = self.create_subscription(Int32MultiArray, 'input_position', self.input_sub_function, 10)
        self.treadmill_sub = self.create_subscription(Int32MultiArray, 'treadmill_position', self.treadmill_sub_function, 1)
        self.serial_sub
        self.car_sub
        self.obstacles_sub
        self.input_sub
        self.Lobstacle = []
        self.Lcar = []
        self.input = []
        self.command = []
        self.Lobject = []
        self.CarObject = []
        self.ObstaclesObject = []
        self.InputObject = []
        self.treadmill=[640,480]
        timer_period = 0.01  # seconds
        self.init_display()
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        """
        Update display
        """
        self.update_display()

    def treadmill_sub_function(self,msg):
        """
        Read treadmill position
        """
        if self.treadmill!=msg.data:
            self.treadmill=msg.data
            self.ax.set_xlim(0, 2*int(msg.data[0]))
            self.ax.set_ylim(0, 2*int(msg.data[1]))
    
    def command_sub_function(self, msg):
        """
        Read command
        """
        self.command = msg.data

    def car_sub_function(self, msg):
        """
        Read car_position topic
        """
        self.Lcar = msg.data
        # remove last car plot
        for object in self.CarObject:
            object.remove()
        self.CarObject = []

        # Plot car
        for i in range(0, len(self.Lcar), 6):
            number,x_center, y_center, angle, width, height = self.Lcar[i:i+6]

            # Plot center of the car
            text=self.ax.text(x_center, y_center, str(int(number)), horizontalalignment='center', verticalalignment='center', fontsize=16, color='green')
            self.CarObject.append(text)

            # Plot the rectnagle of the car
            angle_rad=-angle*np.pi/180
            x_corner, y_corner = x_center - width/2, y_center - height/2
            x_corner_rotated = x_center + (x_corner - x_center) * np.cos(angle_rad) - (y_corner - y_center) * np.sin(angle_rad)
            y_corner_rotated =y_center + (x_corner - x_center) * np.sin(angle_rad) + (y_corner - y_center) * np.cos(angle_rad)
            car = patches.Rectangle([x_corner_rotated, y_corner_rotated], width=width, height=height, edgecolor='green', facecolor='none', linewidth=3)
            car.set_angle(-angle)
            self.ax.add_patch(car)
            self.CarObject.append(car)

        self.fig.canvas.draw()
        plt.pause(0.001)

    def obstacles_sub_function(self, msg):
        """
        Read obstacles_position topic
        """
        self.Lobstacle = msg.data
        # remove last obstacles plot
        for object in self.ObstaclesObject:
            object.remove()
        self.ObstaclesObject = []
        # Plot obstacles
        for i in range(0, len(self.Lobstacle), 3):
            obstacle = patches.Circle((self.Lobstacle[i], self.Lobstacle[i+1]), self.Lobstacle[i+2], edgecolor='red', facecolor='red')
            self.ax.add_patch(obstacle)
            self.ObstaclesObject.append(obstacle)
        self.fig.canvas.draw()
        plt.pause(0.001)

    def input_sub_function(self, msg):
        """
        Read the input position
        """
        self.input = msg.data
        # remove last input plot
        for object in self.InputObject:
            object.remove()
        self.InputObject = []
        # Plot input
        for i in range(0,len(self.input),3):
            input= patches.Circle((self.input[i+1], self.input[i+2]), 2, edgecolor='black', facecolor='black',linewidth=2)
            self.ax.add_patch(input)
            self.InputObject.append(input)
        self.fig.canvas.draw()
        plt.pause(0.001)
    
    def init_display(self,xlim=640,ylim=480):
        """
        Initialise display 
        """
        current_fig = plt.gcf()
        if current_fig is not None:
            plt.close()
        self.get_logger().info('I am initializing the display')
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, xlim)
        self.ax.set_ylim(0, ylim)
        self.ax.set_aspect('equal')
        self.ax.set_title('Display')
        plt.show(block=False)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Display()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
