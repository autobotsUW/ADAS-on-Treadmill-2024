import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
from tkinter import * 


class car_Class():
    def __init__(self,id):
        self.id=id
        self.Xcar,self.Ycar,self.Xinput,self.Yinput=0,0,320,200

class Input(Node):

    def __init__(self):
        super().__init__('input_node')
        self.get_logger().info("Input Node started.\n")
        self.publisher_ = self.create_publisher(Int32MultiArray, 'input_position', 10)
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        self.obstacles_sub = self.create_subscription(Int32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.treadmill_sub = self.create_subscription(Int32MultiArray, 'treadmill_position', self.treadmill_sub_function, 1)
        self.Lobstacle=[]
        self.DictCar={}
        self.Linput=[]
        self.Lkeys=[]
        self.treadmill=[640,480]

    def treadmill_sub_function(self,msg):
        """
        Read treadmill position
        """
        if self.treadmill!=msg.data:
            self.treadmill=msg.data


    def send_input(self):
        """
        Send input position to input_position topic
        """
        msg = Int32MultiArray()
        msg.data = [int(i) for i in self.Linput]
        self.publisher_.publish(msg)
        # self.get_logger().info('Input {}'.format(msg.data))
        
    def car_sub_function(self, msg):
        """
        Read car position
        """
        self.Lkeys=[]
        for i in range(0,len(msg.data),6):
            id,x,y,angle=msg.data[i:i+4]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                car.Xcar=x
                car.Ycar=y
                car.car_angle=angle
            else:
                car=car_Class(id)
                car.Xcar=x
                car.Ycar=y
                car.car_angle=angle
                self.DictCar[id]=car
            self.Lkeys.append(id)
        self.Lkeys.sort()
        self.define_input()
        self.send_input()

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
            # self.get_logger().info("No obstacles detected") 
            if len(self.Lkeys)==1:
                self.Linput=[self.Lkeys[0],150,self.treadmill[1]]
                return
            
            self.Linput=[]
            self.Lkeys.reverse()
            # 1/3 of the treadmill
            l1_3=self.treadmill[1]*2/3
            i=0
            plus=True
            y=l1_3
            for id in self.Lkeys:
                self.Linput+=[id,150+i*100,y]
                if plus:
                    plus=False
                    y+=l1_3
                else:
                    plus=True
                    y-=l1_3
                i+=1
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
