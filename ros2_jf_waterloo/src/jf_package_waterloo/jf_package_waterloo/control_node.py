import rclpy
from rclpy.node import Node
import math as m
import numpy as np
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray
import time as t
from message_filters import TimeSynchronizer, Subscriber


class car_Class():
    def __init__(self,id):
        self.id=id
        self.speed,self.angle,self.Xcar,self.Ycar,self.Xinput,self.Yinput,self.car_angle=0,0,0,0,400,200,0
        self.error_sum_speed = 0
        self.error_sum_angle = 0
        self.previous_time = t.time()
        self.last_error_speed=0
        self.last_error_angle=0

    def calculate_command(self):
        """
        Calculate speed and angle with the input and the position of the car
        """
        current_time = t.time()
        delta_time = (current_time - self.previous_time)
        self.previous_time = current_time

        Kp_speed = 0.5
        Ki_speed = 0.2
        Kd_speed = 0.1
        k_stanley = 1e-1
        # Kp_angle = 1e-2
        # Ki_angle = 5e-4
        # Kd_angle = 0

        # Kp_speed = 0.8
        # Ki_speed = 0.2
        # Kd_speed = 0.1
        # Kp_angle = 1e-2
        # Ki_angle = 5e-4
        # Kd_angle = 0

        error_speed = self.Xinput - self.Xcar
        error_angle = self.Yinput - self.Ycar

        if self.Xcar<600:
            self.error_sum_speed += error_speed * delta_time
            self.error_sum_angle += error_angle * delta_time
        
        if self.Xcar<120:
            self.error_sum_speed=0
        
        
        # minimal_publisher.get_logger().info('Error {} speed: {:.3f}'.format(self.id,error_speed))
        # self.get_logger().info('Error {} angle: {:.3f}'.format(self.id,error_angle))
        self.speed = int(Kp_speed * error_speed + Ki_speed * self.error_sum_speed + Kd_speed * (error_speed-self.last_error_speed)/delta_time)
        # self.angle = int(Kp_angle * error_angle + Ki_angle * self.error_sum_angle + Kd_angle * (error_angle-self.last_error_angle)/delta_time)

        self.last_error_speed=error_speed
        self.last_error_angle=error_angle

         # Stanley controller
        cross_track_error = self.Yinput - self.Ycar
        
        heading_error = self.car_angle
        self.angle=int(heading_error + m.atan2(k_stanley * cross_track_error, self.speed)* 180 / m.pi)
        
        # angle saturation: the car cannot be at too great an angle to the axis of the treadmill
        max_angle=10
        if (self.car_angle>max_angle and self.angle<0) or (self.car_angle<-max_angle and self.angle>0):
            self.angle=0     
            
        # managing the car that goes to the edge of the treadmill
        if self.Ycar>350:
            self.angle=-15
        elif self.Ycar<110:
            self.angle=15
        
        if self.Xcar>600:
            self.speed=0
        elif self.Xcar<=100:
            self.speed=60

        center_servo=100
        delta_servo=20
        if self.speed>150:
            self.speed=150
        elif self.speed<0:
            self.speed=0
        
        self.angle+=center_servo
        if self.angle>center_servo+delta_servo:
            self.angle=center_servo+delta_servo
        elif self.angle<center_servo-delta_servo:
            self.angle=center_servo-delta_servo


class Control(Node):

    def __init__(self):
        super().__init__('Control_node')
        self.get_logger().info("Control Node started.\n")
        self.serial_pu = self.create_publisher(Int32MultiArray, 'command', 10)
        self.camera_sub = self.create_subscription(Int32MultiArray, 'car_position', self.camera_sub_function, 10)
        self.input_sub = self.create_subscription(Int32MultiArray, 'input_position', self.input_sub_function, 10)
        self.error_pub = self.create_publisher(String, 'error', 10)
        self.DictCar={}
        self.command=[]
    
    def camera_callback(self, msg):
        self.get_logger().info(f"Received camera position: {msg.data}")

    def input_callback(self, msg):
        self.get_logger().info(f"Received input position: {msg.data}")

    def send_command(self):
        """
        Send the speed,angle to command topic
        """
        # self.speed=0
        msg = Int32MultiArray()
        msg.data = [int(i) for i in self.command]
        self.serial_pu.publish(msg)
        # self.get_logger().info("Send to car: {}".format(msg.data))

    def camera_sub_function(self, msg):
        """
        Read the car_position topic
        """
        self.command=[]
        for i in range(0,len(msg.data),6):
            id,x,y,angle=msg.data[i:i+4]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                car.Xcar=x
                car.Ycar=y
                car.car_angle=angle
                # self.get_logger().info("Car {} xcar {} ycar {} xinput {} yinput {}".format(car.id,car.Xcar,car.Ycar,car.Xinput,car.Yinput))
            else:
                car=car_Class(id)
                car.Xcar=x
                car.Ycar=y
                car.car_angle=angle
                self.DictCar[id]=car
            car.calculate_command()
            self.command+=[id,car.speed,car.angle]
        self.send_command()
    
    def input_sub_function(self, msg):
        """
        Read the input topic
        """
        # self.get_logger().info(str(msg.data))
        for i in range(0,len(msg.data),3):
            id,x,y=msg.data[i:i+3]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                car.Xinput=x
                car.Yinput=y
                # self.get_logger().info("Car {} xinput {} yinput {}".format(car.id,car.Xinput,car.Yinput))
            else:
                self.get_logger().info("Car no exist")
                msg=String()
                msg.data='input: car no exist'
                self.error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    global minimal_publisher
    minimal_publisher = Control()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
