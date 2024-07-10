import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
import time as time
import numpy as np

class car_Class():
    """
    Class to have all information for one car
    """

    def __init__(self,id):
        self.id=id
        self.Xcar,self.Ycar,self.Xinput,self.Yinput=0,0,500,200
        self.lane=1

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
        self.tobstacle=0
        self.treadmill=[320,240]
        self.lines=[]
        self.t0=0
        self.numberOfCar=1
        self.numberOfLines=1
        self.car_independant=True
        self.tLines=0
        self.line=1
        self.addlines=1

    def treadmill_sub_function(self,msg):
        """
        Read treadmill position
        """
        # self.get_logger().info('{}'.format(msg.data))
        if len(msg.data)>=3:
            self.treadmill==msg.data[:3]
        if len(msg.data)>=3+self.numberOfLines:
            self.numberOfLines=len(msg.data[3:])
            self.lines=msg.data[3:]
            


    def send_input(self):
        """
        Send input position to input_position topic
        """
        for id in self.Lkeys:
            car=self.DictCar[id]
            id=car.id
            X=car.Xinput
            if car.lane<len(self.lines):
                Y=(self.lines[car.lane-1]+self.lines[car.lane])/2
            else:
                Y=self.treadmill[1]
            self.Linput=[id,X,Y]
        if len(self.Linput)/3==self.numberOfCar:
            msg = Int32MultiArray()
            msg.data = [int(i) for i in self.Linput]
            self.publisher_.publish(msg)
        else:
            self.get_logger().info("Error number of cars {} {}".format(self.numberOfCar,self.Linput)) 
        

        # Add the new input in the car information
        for i in range(0,len(msg.data),3):
            id,x,y=msg.data[i:i+3]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                # car.Xinput=x
                car.Yinput=y
        
    def car_sub_function(self, msg):
        """
        Read car position
        """

        # Nothing detetcted
        if len(msg.data)==0:
            return
        
        # No movement of the treadmill
        if self.t0==0 and msg.data[1]<550:
            self.t0=time.time()
            self.tLines=self.t0
            self.get_logger().info('Go')

        # Mannaging detection of several cars
        numberOfCar=len(msg.data)//6
        if numberOfCar>self.numberOfCar:
            self.numberOfCar=numberOfCar
            self.get_logger().info('{} cars'.format(self.numberOfCar))
        
        # Add the new position in the car class
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
        # Begin at the middle of the treadmill and move back.
        Xmin=150
        Xmax=500
        Xcar=150
        dx=0.5
        # To begin we define an input at the middle of the treadmill
        if self.t0==0:
            Xmin=Xmax
            return()

        
        # Ymiddle=self.treadmill[1]
        # if self.numberOfLines!=0:
        #     if(time.time()-self.tLines)>=10:
        #         self.tLines=time.time()
        #         self.line+=self.addlines
        #         if self.line==self.numberOfLines-1:
        #             self.addlines=-1
        #         elif self.line==1:
        #             self.addlines=1
        #     Ymiddle=(self.lines[self.line-1]+self.lines[self.line])/2
            

        # if len(self.Lkeys)==1 and len(self.Lobstacle)==0:
        #     self.Linput=[self.Lkeys[0],Xmin,Ymiddle]
        #     return  
        
        elif len(self.Lobstacle)==0:
            for id in self.Lkeys:
                car=self.DictCar[id]

                for otherid in self.Lkeys:
                    if id!=otherid:
                        othercar=self.DictCar[otherid]
                        if abs(car.Xcar-othercar.Xcar)<Xcar and car.lane==othercar.lane:
                            if car.Ycar>othercar.Ycar:
                                car.lane+=1
                            else:
                                othercar.lane+=1

                if car.lane==1:
                    car.Xinput-=dx
                elif car.lane==2:
                    car.Xinput+=dx
                elif car.lane==3:
                    car.Xinput+=2*dx
                elif car.lane==3:
                    car.Xinput+=3*dx
                # self.get_logger().info("Car {} lane {} Xinput {}".format(car.id,car.lane,car.Xinput)) 
                if car.Xinput<Xmin and car.lane<2:
                    car.lane+=1
                    self.get_logger().info("Car {} lane {}".format(car.id,car.lane)) 
                elif car.Xinput>Xmax and car.lane>1:
                    car.lane-=1
                    self.get_logger().info("Car {} lane {}".format(car.id,car.lane)) 



                
            
       

    def distance_car_obstacle(self,car):
        """
        Calculate minimal distance between the car and the obstacles
        """
        distance=[]
        for obstacle in self.Lobstacle:
            if obstacle[0]>=car[0]:
                distance.append(abs(car[1]-obstacle[1]))
        distance.sort()
        if len(distance)>=1:
            if distance[0]>self.distanceMin:
                return(self.distanceMin)
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
