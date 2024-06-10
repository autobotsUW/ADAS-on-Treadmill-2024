import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
import time

class Security(Node):

    def __init__(self):
        super().__init__('security_node')
        self.get_logger().info("Security Node started.\n")
        self.publisher_treadmill = self.create_publisher(String, 'treadmill', 10)
        
        self.error_sub = self.create_subscription(String,'error',self.error_sub_function,10)
        
        timer_period = 0.1  # seconds
        self.timer_function = self.create_timer(timer_period, self.detect_error)
        
        self.car_sub = self.create_subscription(Float32MultiArray,'car_position',self.car_sub_function,10)
        self.Xcar,self.Ycar,self.car_angle=0,0,0
        self.time_detect_car=time.time()
        self.car_sub  # prevent unused variable warning
        
        self.command_sub = self.create_subscription(Int32MultiArray,'command',self.command_sub_function,10)
        self.speed,self.angle=0,0
        self.time_detect_command=time.time()
        self.car_sub  # prevent unused variable warning
        self.no_detected=0
        
             
    def error_sub_function(self,msg):
        self.send_stop_treadmill()

    def car_sub_function(self, msg):
        if len(msg.data)!=0:
            self.no_detected=0
            self.Xcar,self.Ycar,self.car_angle=msg.data[:3]
            self.time_detect_car=time.time()
        else:
            self.get_logger().info("Car not detected")
            self.no_detected+=1
            if  self.no_detected>30:
                self.send_stop_treadmill()
            
    def command_sub_function(self,msg):
        if len(msg.data)!=0:
            self.speed,self.angle=msg.data[:2]
            self.time_detect_command=time.time()
        else:
            self.get_logger().info("No command")
            self.send_stop_treadmill()
        
    def send_stop_treadmill(self):
        msg=String()
        msg.data='STOP'
        self.publisher_treadmill.publish(msg)
        
    def send_start_treadmill(self):
        msg=String()
        msg.data='START'
        self.publisher_treadmill.publish(msg)
    
    def detect_error(self):
        t=time.time()
        if (t-self.time_detect_car)>1000  or self.Xcar<50:
            self.get_logger().info("Problem car {:.2f} {}".format(t-self.time_detect_car,self.angle))
            self.send_stop_treadmill()
        
        if (t-self.time_detect_command)>1000:
            self.get_logger().info("Problem command {:.2f}".format(t-self.time_detect_command))
            self.send_stop_treadmill()
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Security()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
