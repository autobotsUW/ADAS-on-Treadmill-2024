

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
        self.car_sub = self.create_subscription(Float32MultiArray, 'car_position', self.car_sub_function, 10)
        self.obstacles_sub = self.create_subscription(Float32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.input_sub = self.create_subscription(Float32MultiArray, 'input_position', self.input_sub_function, 10)
        self.serial_sub
        self.car_sub
        self.obstacles_sub
        self.input_sub
        self.Lobstacle = []
        self.Lcar = []
        self.input = [320,240]
        self.command = []
        self.Lobject = []
        timer_period = 0.01  # seconds
        self.init_display()
        self.timer = self.create_timer(timer_period, self.timer_callback)
 
    def timer_callback(self):
        self.update_display()
    
    def command_sub_function(self, msg):
        self.command = msg.data

    def car_sub_function(self, msg):
        self.Lcar = msg.data

    def obstacles_sub_function(self, msg):
        self.Lobstacle = msg.data

    def input_sub_function(self, msg):
        self.input = msg.data
    
    def update_display(self):
        # self.get_logger().info('I am updating the display')
        for object in self.Lobject:
            object.remove()
        self.Lobject = []

        # Tracer la voiture
        for i in range(0, len(self.Lcar), 5):
            car_center= patches.Circle((self.Lcar[0], self.Lcar[1]), 2, edgecolor='green', facecolor='green',linewidth=5)
            self.ax.add_patch(car_center)
            self.Lobject.append(car_center)
            
            x_center, y_center, angle, width, height = self.Lcar[i:i+5]
            print(width,height,x_center,y_center,angle)
            angle_rad=-angle*np.pi/180
            x_corner, y_corner = x_center - width/2, y_center - height/2
            x_corner_rotated = x_center + (x_corner - x_center) * np.cos(angle_rad) - (y_corner - y_center) * np.sin(angle_rad)
            y_corner_rotated =y_center + (x_corner - x_center) * np.sin(angle_rad) + (y_corner - y_center) * np.cos(angle_rad)
            car = patches.Rectangle([x_corner_rotated, y_corner_rotated], width=self.Lcar[i+3], height=self.Lcar[i+4], edgecolor='green', facecolor='none', linewidth=3)
            car.set_angle(-angle)
            self.ax.add_patch(car)
            self.Lobject.append(car)

        # Tracer les obstacles
        for i in range(0, len(self.Lobstacle), 3):
            obstacle = patches.Circle((self.Lobstacle[i], self.Lobstacle[i+1]), self.Lobstacle[i+2], edgecolor='red', facecolor='red')
            self.ax.add_patch(obstacle)
            self.Lobject.append(obstacle)
        
        # Tracer les points d'entrée
        input= patches.Circle((self.input[0], self.input[1]), 2, edgecolor='black', facecolor='black',linewidth=2)
        self.ax.add_patch(input)
        self.Lobject.append(input)

        # Redessiner la figure
        self.fig.canvas.draw()
        plt.pause(0.005)
        # self.get_logger().info(str(self.Lobject))
    
    def init_display(self):
        self.get_logger().info('I am initializing the display')
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 640)
        self.ax.set_ylim(0, 480)
        self.ax.set_aspect('equal')
        # self.ax.set_axis_off()
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
