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
        self.lane=id+1
        self.carAtRight=[]
        self.carAtLeft=[]
        self.overtake=False
        self.ObstacleInTheLane=False
        self.ObstacleInTheLaneRight=False
        self.ObstacleInTheLaneLeft=False
        self.dx=0.5

    def find_lane(self,y,lines):
        for i in range(1,len(lines)):
            if lines[i-1]<y<lines[i]:
                self.lane=i
                self.Ycar=y
                return



    def test_other_car(self,DictCar,numberOfLines):
        self.carAtRight=[]
        self.carAtLeft=[]
        self.overtake=False
        self.ObstacleInTheLane=False
        self.ObstacleInTheLaneRight=False
        self.ObstacleInTheLaneLeft=False
        Xcar=180
        for otherid in DictCar.keys():
            if id!=otherid:
                othercar=DictCar[otherid]
                if abs(self.Xcar-othercar.Xcar)<Xcar and self.lane==othercar.lane and self.Xcar<othercar.Xcar:
                        self.overtake=True
                        # minimal_publisher.get_logger().info("Car {} and {} lane {} Position x: {} {}".format(car.id,othercar.id,car.lane,car.Xcar,othercar.Xcar)) 
                
                if abs(self.Xcar-othercar.Xcar)<Xcar and self.lane+1==othercar.lane and self.lane+1<numberOfLines-1:
                    self.carAtLeft.append(othercar)
                if abs(self.Xcar-othercar.Xcar)<1.5*Xcar and self.lane-1==othercar.lane and self.lane-1>=0:
                    self.carAtRight.append(othercar)
                # minimal_publisher.get_logger().info("Car {} lane {} overtake {} left {} right {}".format(car.id,car.lane,overtake,carAtLeft,carAtRight)) 

    def test_obstacles(self,Obstacles):   
        for obstacle in Obstacles:
            lane,x=obstacle[:]
            if self.lane==lane and x>self.Xcar:
                self.ObstacleInTheLane=True
            elif self.lane+1==lane and x>self.Xcar:
                self.ObstacleInTheLaneLeft=True
            elif self.lane-1==lane and x>self.Xcar:
                self.ObstacleInTheLaneRight=True
    
    def move(self,numberOfLines):
        Xmin=150
        Xmax=500
        
        if self.lane==1:
            self.Xinput-=self.dx
        elif self.lane==2:
            self.Xinput+=self.dx
        elif self.lane==3:
            self.Xinput+=self.dx
        elif self.lane==3:
            self.Xinput+=3*self.dx
        if self.Xinput<Xmin:
            self.Xinput=Xmin
        elif self.Xinput>Xmax:
            self.Xinput=Xmax
        # minimal_publisher.get_logger().info("Car {} lane {} overtake {} left {} right {} obstacles {} {} {}".format(self.id,self.lane,self.overtake,len(self.carAtLeft),len(self.carAtRight),self.ObstacleInTheLaneLeft,self.ObstacleInTheLane,self.ObstacleInTheLaneRight)) 

        if ((self.ObstacleInTheLaneLeft==False and self.ObstacleInTheLane==True and len(self.carAtLeft)==0) or (self.overtake==True and len(self.carAtLeft)==0)) and self.lane<numberOfLines:
            self.lane+=1
            minimal_publisher.get_logger().info("Car {} lane {}".format(self.id,self.lane)) 
        elif (len(self.carAtRight)==0 and self.ObstacleInTheLaneRight==False) and self.lane>1:
            self.lane-=1
            minimal_publisher.get_logger().info("Car {} lane {}".format(self.id,self.lane)) 
        
        elif self.ObstacleInTheLane==True:   
            if self.go_to_the_left(numberOfLines)==False:
                if self.go_to_the_right()==False:
                    self.Xinput-=2*self.dx
                    if self.Xinput<Xmin:
                        self.Xinput=Xmin

        
        # minimal_publisher.get_logger().info("Car {} Xinput {}".format(self.id,self.Xinput)) 
    
    def go_to_the_left(self,numberOfLines):
        if len(self.carAtLeft)==0 and self.lane<numberOfLines:
            self.lane+=1
            return True
        elif self.lane>=numberOfLines:
            return False
        laneFree=True
        for otherCar in self.carAtLeft:
            if otherCar.go_to_the_left(numberOfLines)==False:
                laneFree=False
        if laneFree:
            self.lane+=1
            return True
        return False
    
    def go_to_the_right(self):
        if len(self.carAtRight)==0 and self.lane>1:
            self.lane-=1
            return True
        elif self.lane>1:
            return False
        laneFree=True
        for otherCar in self.carAtRight:
            if otherCar.go_to_the_right()==False:
                laneFree=False
        if laneFree:
            self.lane-=1
            return True
        return False

class Input(Node):

    def __init__(self):
        super().__init__('input_node')
        self.get_logger().info("Input Node started.\n")
        self.publisher_ = self.create_publisher(Int32MultiArray, 'input_position', 10)
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        self.obstacles_sub = self.create_subscription(Int32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.treadmill_sub = self.create_subscription(Int32MultiArray, 'treadmill_position', self.treadmill_sub_function, 1)
        self.Lobstacle=[]
        self.LaneObstacles=[]
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
            if self.numberOfLines<len(msg.data[3:])-1:
                self.numberOfLines=len(msg.data[3:])-1
                self.get_logger().info('{} Lane'.format(self.numberOfLines))
            self.lines=msg.data[3:]

    def send_input(self):
        """
        Send input position to input_position topic
        """
        self.Linput=[]
        for id in self.Lkeys:
            car=self.DictCar[id]
            id=car.id
            X=car.Xinput
            if car.lane<len(self.lines):
                Y=(self.lines[car.lane-1]+self.lines[car.lane])/2
            else:
                Y=self.treadmill[1]
            self.Linput+=[id,X,Y]

        if len(self.Linput)/3==self.numberOfCar:
            msg = Int32MultiArray()
            msg.data = [int(i) for i in self.Linput]
            self.publisher_.publish(msg)

            # Add the new input in the car information
            for i in range(0,len(msg.data),3):
                id,x,y=msg.data[i:i+3]
                if id in self.DictCar.keys():
                    car=self.DictCar[id]
                    # car.Xinput=x
                    car.Yinput=y
        else:
            self.get_logger().info("Error number of cars {} {}".format(self.numberOfCar,self.Linput)) 
              
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
                car.find_lane(y,self.lines)
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
        # else:
        #     self.get_logger().info("Car {}".format(self.Lkeys)) 

    def obstacles_sub_function(self, msg):
        """
        Read obstacles positions
        """
        self.Lobstacle=[]
        self.LaneObstacles=[]
        for i in range(0, len(msg.data), 3):
            x,y,r=msg.data[i:i+3]
            self.Lobstacle.append([x,y,r])
            for lane in range(1,self.numberOfLines):
                if self.lines[lane-1]<y-r<self.lines[lane] or self.lines[lane-1]<y+r<self.lines[lane]:
                    self.LaneObstacles.append([lane,x])
        
        # self.get_logger().info("Obstacles detected {}".format(self.LaneObstacles)) 


    def define_input(self):
        """
        Calculate new input for the car with space invaders methods
        """
        # Begin at the middle of the treadmill and move back.
        Xmax=500
        # To begin we define an input at the middle of the treadmill
        if self.t0==0:
            for id in self.Lkeys:
                car=self.DictCar[id]
                car.Xinput=Xmax
            return()

        for id in self.Lkeys:
            # self.get_logger().info("Car {}".format(id))
            car=self.DictCar[id]
            
            car.test_other_car(self.DictCar,self.numberOfLines)
            if len(self.LaneObstacles)!=0:
                Lobstacles=self.LaneObstacles
                car.test_obstacles(Lobstacles)
            
        for id in self.Lkeys:
            if id!=2:
                car=self.DictCar[id]
                car.move(self.numberOfLines)

def main(args=None): 
    rclpy.init(args=args)
    global minimal_publisher
    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
