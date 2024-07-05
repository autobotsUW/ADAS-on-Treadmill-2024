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
        self.t0=0
        self.numberOfCar=1
        self.car_independant=True

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

        # Add the new input in the car information
        for i in range(0,len(msg.data),3):
            id,x,y=msg.data[i:i+3]
            if id in self.DictCar.keys():
                car=self.DictCar[id]
                car.Xinput=x
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
            # if we have not the good number of input we not change the input (ex: detection of 1 car in a 2 cars situation)
            if len(self.Linput)/3==self.numberOfCar:
                self.send_input()
            else:
                self.get_logger().info("Error number of cars") 

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
        Xmin=300
        Xmax=400
        # To begin we define an input at the middle of the treadmill
        if self.t0==0:
            Xmin=Xmax
        elif Xmax-10*(time.time()-self.t0)>Xmin:
            Xmin=Xmax-10*(time.time()-self.t0)
        Xcar=140
        Ymiddle=self.treadmill[1]
        deltaYmax=50
        self.distanceMin=150
        Ymin=50
        Ymax=350


        if len(self.Lobstacle)==0:
            # self.get_logger().info("No obstacles detected") 

            # If one car: center in Y position
            if len(self.Lkeys)==1:
                self.Linput=[self.Lkeys[0],Xmin,Ymiddle]
                return  
            
            # If several car: we create two columns. We separate these columns if we not have obstacles
            self.Linput=[]
            self.Lkeys.sort()
            # 1/3 of the treadmill
            deltaY=min(int(time.time()-self.tobstacle)-5,deltaYmax)
            if deltaY<0:
                deltaY=0
            i=len(self.Lkeys)-1
            plus=True
            for id in self.Lkeys:
                self.Linput+=[id,Xmin+i*Xcar,Ymiddle+deltaY]
                deltaY*=-1
                i-=1
            return
        
        # self.get_logger().info(str(self.Lobstacle))

        # we have obstacles
        
        
        # self.get_logger().info(str(first_car_input)) 

        # i=10
        # distance_min=150
        # # We use the space invader method. We mesure the distance between the first car and the obstacles and we maximize the minimal distance
        # current_distance = self.distance_car_obstacle(first_car_input)
        # right_distance = self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
        # left_distance = self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
        
        # if current_distance>=left_distance and current_distance>=right_distance or current_distance>distance_min:
        #     # Keep position
        #     self.get_logger().info('Current {:.2f}'.format(current_distance)) 
        #     Yinput=first_car_input[1]
            
        # elif right_distance>current_distance and right_distance>left_distance:
        #     # Go to the right
        #     i+=5
        #     distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
        #     while right_distance<distance and right_distance<distance_min and i<=100:
        #         right_distance=distance
        #         i+=5
        #         distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]-i])
        #     self.get_logger().info('Right {:.2f} '.format(distance))   
        #     Yinput=first_car_input[1]-i
        
        # elif left_distance>current_distance and left_distance>right_distance:
        #     # Go to the left
        #     i+=5
        #     distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
        #     while left_distance<distance and left_distance<distance_min and i<=100:
        #         left_distance=distance
        #         i+=5
        #         distance=self.distance_car_obstacle([first_car_input[0],first_car_input[1]+i])
        #     self.get_logger().info('left {:.2f} '.format(distance))    
        #     Yinput=first_car_input[1]+i

        # else:
        #     self.get_logger().info('error') 
        #     Yinput=first_car_input[1]


        self.tobstacle=time.time()
        self.Linput=[]

        if self.car_independant:
            # each cars choose these position 
            for id in self.Lkeys:
                car=self.DictCar[id]
                first_car_input=[car.Xinput,car.Yinput]
                Ly=[y for y in range(Ymin,Ymax+1,10)]
                Ldistance=[]
                for y in Ly:
                    Ldistance.append(self.distance_car_obstacle([car.Xinput,y]))
                if max(Ldistance)==1000:
                    Yinput=0.1*Ymiddle+0.9*car.Yinput
                elif max(Ldistance)==self.distanceMin:
                    L=[]
                    for i in range(len(Ly)):
                        if Ldistance[i]==self.distanceMin:
                            L.append(Ly[i])
                    Yinput=L[min(range(len(L)),key=lambda i: abs(L[i]-car.Yinput))]
                    # self.get_logger().info('{} {} {} {}'.format(id,Yinput,L,Ldistance)) 
                else:
                    Yinput=Ly[np.argmax(Ldistance)]
                
                if Yinput>Ymax:
                    Yinput=Ymax
                elif Yinput<Ymin:
                    Yinput=Ymin
                
                self.Linput+=[id,car.Xinput,Yinput]
            
        else:
            # We use the position of the car with smallest id to define where to go
            car=self.DictCar[self.Lkeys[0]]
            first_car_input=[car.Xinput,car.Yinput]
            

            Ly=[y for y in range(50,351,10)]
            Ldistance=[]
            for y in Ly:
                Ldistance.append(self.distance_car_obstacle([first_car_input[0],y]))
            if max(Ldistance)==1000:
                Yinput=Ymiddle
            elif max(Ldistance)==self.distanceMin:
                L=[]
                for i in range(len(Ly)):
                    if Ldistance[i]==self.distanceMin:
                        L.append(Ly[i])
                Yinput=L[min(range(len(L)),key=lambda i: abs(L[i]-first_car_input[1]))]
                # self.get_logger().info('{} {} {}'.format(Yinput,L,Ldistance)) 
            else:
                Yinput=Ly[np.argmax(Ldistance)]
            
            # if Yinput>350:
            #     Yinput=350
            # elif Yinput<50:
            #     Yinput=50
            
            # we put the other cars behind the first 
            self.Lkeys.reverse()
            i=0
            for id in self.Lkeys:
                self.Linput+=[id,Xmin+i*Xcar,Yinput]
                i+=1 
        return

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
