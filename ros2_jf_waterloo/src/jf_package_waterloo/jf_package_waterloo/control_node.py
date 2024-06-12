import rclpy
from rclpy.node import Node
import math as m
import numpy as np
from std_msgs.msg import String,Int32MultiArray,Float32MultiArray



class Control(Node):

    def __init__(self):
        super().__init__('Control_node')
        self.get_logger().info("Control Node started.\n")
        self.serial_pu = self.create_publisher(Int32MultiArray, 'command', 10)
        self.camera_sub = self.create_subscription(Float32MultiArray, 'car_position', self.camera_sub_function, 10)
        self.input_sub = self.create_subscription(Float32MultiArray, 'input_position', self.input_sub_function, 10)
        self.speed,self.angle,self.Xcar,self.Ycar,self.Xinput,self.Yinput,self.car_angle=0,0,0,0,320,200,0
        self.error_sum_speed = 0
        self.error_sum_angle = 0
        self.previous_time = self.get_clock().now()
        self.last_error_speed=0
        self.last_error_angle=0

    def send_command(self):
        """
        Send the speed,angle to command topic

        """
        center_servo=100
        delta_servo=20
        if self.speed>150:
            self.speed=150
        elif self.speed<0:
            self.speed=0
            # self.angle=0
        
        # self.get_logger().info('Send calculate: speed {} angle {}'.format(self.speed,self.angle)) 
        self.angle+=center_servo
        if self.angle>center_servo+delta_servo:
            self.angle=center_servo+delta_servo
        elif self.angle<center_servo-delta_servo:
            self.angle=center_servo-delta_servo

        # self.speed=0
        msg = Int32MultiArray()
        msg.data = [self.speed, self.angle]
        self.serial_pu.publish(msg)
        # self.get_logger().info("Send to car: {}".format(msg.data))

    def camera_sub_function(self, msg):
        """
        Read the car_position topic
        """
        if len(msg.data)>0:
            self.Xcar=msg.data[0]
            self.Ycar=msg.data[1]
            self.car_angle=msg.data[2]
            # self.get_logger().info('Input position x: {}'.format(self.Xcar))
            # self.get_logger().info('Input position y: {}'.format(self.Ycar))
            # self.get_logger().info('Input position angle: {}'.format(self.car_angle))
            
            self.calculate_command()
            self.send_command()
        else:
            self.error_sum_speed = 0
            self.error_sum_angle = 0
    
    def input_sub_function(self, msg):
        """
        Read the input topic
        """
        self.Xinput=msg.data[0]
        self.Yinput=msg.data[1]
        # self.get_logger().info('Input position x: {}'.format(self.Xinput))
        # self.get_logger().info('Input position y: {}'.format(self.Yinput))

    def calculate_command(self):
        """
        Calculate speed and angle with the input and the position of the car
        """
        current_time = self.get_clock().now()
        delta_time = (current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = current_time

        Kp_speed = 0.8
        Ki_speed = 0.2
        Kd_speed = 0.1
        Kp_angle = 1e-1
        Ki_angle = 5e-3
        Kd_angle = 0

        # Kp_speed = 0.8
        # Ki_speed = 0.2
        # Kd_speed = 0.1
        # Kp_angle = 1e-2
        # Ki_angle = 5e-4
        # Kd_angle = 0

        error_speed = self.Xinput - self.Xcar
        error_angle = self.Yinput - self.Ycar

        if self.Xcar<300:
            self.error_sum_speed += error_speed * delta_time
            self.error_sum_angle += error_angle * delta_time
        
        
        # self.get_logger().info('Error speed: {:.3f}'.format(error_speed))
        # self.get_logger().info('Error angle: {:.3f}'.format(error_angle))
        self.speed = int(Kp_speed * error_speed + Ki_speed * self.error_sum_speed + Kd_speed * (error_speed-self.last_error_speed)/delta_time)
        self.angle = int(Kp_angle * error_angle + Ki_angle * self.error_sum_angle + Kd_angle * (error_angle-self.last_error_angle)/delta_time)

        self.last_error_speed=error_speed
        self.last_error_angle=error_angle


        cross_track_error = self.Yinput - self.Ycar
        k_stanley = 1e-1
        # Stanley controller
        heading_error = self.car_angle
        stanley_control = heading_error + m.atan2(k_stanley * cross_track_error, self.speed)* 180 / m.pi
        self.angle = int(stanley_control)
        # self.get_logger().info('Error angle: {} {:.3f} {:.3f}'.format(self.angle,heading_error,m.atan2(k_stanley * cross_track_error, self.speed)*180/m.pi))
        

        
        
        max_angle=15
        if (self.car_angle>max_angle and self.angle<0) or (self.car_angle<-max_angle and self.angle>0):
            self.angle=0     
        # self.get_logger().info('Command calculate: speed {} angle {}'.format(self.speed,self.angle)) 
            
        if self.Ycar>350:
            self.angle=-15
        elif self.Ycar<110:
            self.angle=15
        
        if self.Xcar>400:
            self.speed=0
        elif self.Xcar<125:
            self.speed=100
        
        # self.get_logger().info('Command calculate: speed {} angle {}'.format(self.speed,self.angle))
        
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Control()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
