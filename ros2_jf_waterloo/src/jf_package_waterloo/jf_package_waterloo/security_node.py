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
        
        # timer_period = 0.1  # seconds
        # self.timer_function = self.create_timer(timer_period, self.detect_error)
        
        self.car_sub = self.create_subscription(Int32MultiArray,'car_position',self.car_sub_function,10)
        self.car=6*[0]
        self.time_detect_car=time.time()
        self.car_sub  # prevent unused variable warning
        
        self.command_sub = self.create_subscription(Int32MultiArray,'command',self.command_sub_function,10)
        self.command=3*[0]
        self.time_detect_command=time.time()
        self.car_sub  # prevent unused variable warning
        self.no_detected=0
        
             
    def error_sub_function(self,msg):
        """
        Read error node. This topic stop the treadmill
        """
        self.send_stop_treadmill()

    def car_sub_function(self, msg):
        """
        Read the position of the car
        """
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
        """
        Read the command of the car
        """
        if len(msg.data)!=0:
            self.command=msg.data
            self.time_detect_command=time.time()
        else:
            self.get_logger().info("No command")
            self.send_stop_treadmill()
        
    def send_stop_treadmill(self):
        """
        Send stop at the treadmill
        """
        msg=String()
        msg.data='STOP'
        self.publisher_treadmill.publish(msg)
        
    def send_start_treadmill(self):
        """
        Send start at the treadmill
        """
        msg=String()
        msg.data='START'
        self.publisher_treadmill.publish(msg)
    
    def detect_error(self):
        """
        Error detection
        """
        t=time.time()
        # Car problems
        if (t-self.time_detect_car)>1000:
            self.get_logger().info("Problem car {:.2f} time detection".format(t-self.time_detect_car))
            self.send_stop_treadmill()
        for i in range(0,len(self.car)):
            if self.car[i+1]<50 or self.car[i+3]:
                self.get_logger().info("Problem  position or angle car {}x={} angle={}".format(self.car[i],self.car[i+1],self.car[i+3]))
                self.send_stop_treadmill()
        
        # Command problems
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
