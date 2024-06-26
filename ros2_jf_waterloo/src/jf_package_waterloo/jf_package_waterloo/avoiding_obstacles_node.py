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
        self.treadmill=[320,240]

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
        if len(self.Lkeys)>=1:
            # self.get_logger().info("Car obstacles detected") 
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
        Xmin=150
        Xcar=120

        if len(self.Lobstacle)==0:
            # self.get_logger().info("No obstacles detected") 
            if len(self.Lkeys)==1:
                self.Linput=[self.Lkeys[0],Xmin,self.treadmill[1]]
                return
            
            self.Linput=[]
            self.Lkeys.reverse()
            # 1/3 of the treadmill
            l1_3=50
            i=0
            plus=True
            y=150
            for id in self.Lkeys:
                self.Linput+=[id,Xmin+i*Xcar,y]
                if plus:
                    plus=False
                    y+=l1_3
                else:
                    plus=True
                    y-=l1_3
                i+=1
            return
        
        # self.get_logger().info(str(self.Lobstacle))

        # we have obstacles
        car=self.DictCar[self.Lkeys[0]]
        first_car_input=[car.Xinput,car.Yinput]
        # self.get_logger().info(str(first_car_input)) 

        i=5

        current_distance = self.distance_car_obstacle(first_car_input)
        right_distance = self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
        left_distance = self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
        # self.get_logger().info('{:.2f} {:.2f} {:.2f}'.format(left_distance,current_distance,right_distance)) 

        if current_distance>=left_distance and current_distance>=right_distance or current_distance>50:
            # Keep position
            Yinput=first_car_input[1]
            
        elif right_distance>current_distance and right_distance>left_distance:
            # Go to the right
            i+=5
            distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
            while right_distance<distance and right_distance<50 and i<=100:
                # self.get_logger().info('right {} {:.2f} {:.2f}'.format(first_car_input[1]-i,distance,right_distance)) 
                right_distance=distance
                i+=5
                distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
                
            Yinput=first_car_input[1]-i
            self.get_logger().info('{}'.format(Yinput)) 
        
        elif left_distance>current_distance and left_distance>right_distance:
            # Go to the left
            i+=5
            distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
            while left_distance<distance and left_distance<50 and i<=100:
                # self.get_logger().info('left {:.2f} {:.2f}'.format(distance,left_distance)) 
                left_distance=distance
                i+=5
                distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
                
            Yinput=first_car_input[1]+i
        else:
            self.get_logger().info('error') 
            Yinput=first_car_input[1]

        
        if Yinput>350:
            Yinput=350
        elif Yinput<80:
            Yinput=80
        
        self.Linput=[]
        self.Lkeys.reverse()
        i=0
        for id in self.Lkeys:
            self.Linput+=[id,Xmin+i*Xcar,Yinput]
            i+=1 
        return

    def distance_car_obstacle(self,car):
        """
        Calculate distance between the car and the obstacles
        """
        distance=[]
        for obstacle in self.Lobstacle:
            if obstacle[0]>=car[0]:
                distance.append(abs(car[1]-obstacle[1]))
        distance.sort()
        if len(distance)>=1:
            return distance[0]
        return 1000

    # def distance_car_obstacle(self,car):
    #     """
    #     Calculate distance between the car and the obstacles
    #     """
    #     distance=0
    #     for obstacle in self.Lobstacle:
    #         if obstacle[0]>car[0]:
    #             distance+=(car[0]-obstacle[0])**2+(car[1]-obstacle[1])**2
    #     return distance**0.5

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
