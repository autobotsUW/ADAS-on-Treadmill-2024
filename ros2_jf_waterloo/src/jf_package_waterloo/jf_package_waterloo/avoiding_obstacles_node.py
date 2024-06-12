import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from tkinter import * 

class Input(Node):

    def __init__(self):
        super().__init__('input_node')
        self.get_logger().info("Input Node started.\n")
        self.publisher_ = self.create_publisher(Float32MultiArray, 'input_position', 10)
        self.car_sub = self.create_subscription(Float32MultiArray, 'car_position', self.car_sub_function, 10)
        self.obstacles_sub = self.create_subscription(Float32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.Xinput,self.Yinput=200,200
        self.Linput=[self.Xinput,self.Yinput]
        self.LastInput=self.Linput
        self.Lobstacle=[]
        self.Lcar=[]
        self.send_input(self.Xinput,self.Yinput)

    def send_input(self,x,y):
        """
        Send input position to input_position topic
        """
        self.Linput=[self.Xinput,self.Yinput]
        if self.Linput!=self.LastInput or self.Linput==[200,200]:
            msg = Float32MultiArray()
            msg.data = [float(x),float(y)]
            self.publisher_.publish(msg)
            self.get_logger().info('Input {} {}'.format(self.Xinput,self.Yinput))
            self.LastInput=self.Linput     
        
    def car_sub_function(self, msg):
        """
        Read car position
        """
        if len(msg.data)>0:
            self.Lcar = msg.data
            self.car_position=msg.data[:2]
            self.define_input()

    def obstacles_sub_function(self, msg):
        """
        Read obstacles positions
        """
        self.Lobstacle=[]
        for i in range(0, len(msg.data), 3):
            self.Lobstacle.append(msg.data[i:i+4])

    def define_input(self):
        """
        Calculate new input for the car with space invaders methods
        """
        if len(self.Lobstacle)==0:
            self.get_logger().info("No obstacles detected") 
            self.send_input(self.Xinput,200)
            return
        i=5

        current_distance = self.distance_car_obstacle(self.Linput)
        right_distance = self.distance_car_obstacle([self.Linput[0],self.Linput[1]-i])
        left_distance = self.distance_car_obstacle([self.Linput[0],self.Linput[1]+i])
        
        if current_distance>left_distance and current_distance>right_distance:
            # Keep position
            pass
            
        elif right_distance>current_distance and right_distance>left_distance:
            # Go to the right
            distance=self.distance_car_obstacle([self.Linput[0],self.Linput[1]-i])
            while right_distance<distance and i<=100:
                right_distance=distance
                i+=10
                
            self.Yinput=self.Linput[1]-i
        
        elif left_distance>current_distance and left_distance>right_distance:
            # Go to the left
            distance=self.distance_car_obstacle([self.Linput[0],self.Linput[1]+i])
            while left_distance<distance and i<=100:
                left_distance=distance
                i+=10
                
            self.Yinput=self.Linput[1]+i
        
        if self.Yinput>350:
            self.Yinput=350
        elif self.Yinput<100:
            self.Yinput=100        
        self.send_input(self.Xinput,self.Yinput)  
        return
    
    def distance_car_obstacle(self,car):
        """
        Calculate distance between the car and the obstacles
        """
        distance=0
        for obstacle in self.Lobstacle:
            if obstacle[0]>self.Lcar[0]:
                distance+=(car[0]-obstacle[0])**2+(car[1]-obstacle[1])**2
        return distance**0.5

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
