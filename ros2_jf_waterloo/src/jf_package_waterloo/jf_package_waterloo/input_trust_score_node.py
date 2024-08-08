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
        self.Xcar,self.Ycar,self.Xinput,self.Yinput=0,0,id*100+250,id*150+100
        # data blue car
        self.xdata=[]
        self.ydata=[]
        self.time=[]
        self.trust_score=[0]
        self.STL_dangerous=[0]
        self.trust=1

    def calculate_input(self,other_car):
        """
        Calculate the input for the car
        """
        if self.trust<0.7:
            # it's time to move away
            vector=[self.Xcar-other_car.Xcar,self.Ycar-other_car.Ycar]
            norme=np.linalg.norm(vector)
            vector=vector/norme
            self.Xinput=self.Xinput+vector[0]
            self.Yinput=self.Yinput+vector[1]
            minimal_publisher.get_logger().info("Move")
            


    def calculate_trust_score(self,t,x,y):
        n=100
        self.time.append(t)
        self.xdata.append(x)
        self.ydata.append(y)
        if len(self.time) > n:
            var_x = self.variance(self.xdata[-n:])
            var_y = self.variance(self.ydata[-n:])
            self.trust_score.append(var_x + var_y)
        else:
            var_x = self.variance(self.xdata)
            var_y = self.variance(self.ydata)
            self.trust_score.append(var_x + var_y)
        
        if self.trust_score[-1]>=6000 or dangerous_param!=1:
            minimal_publisher.get_logger().info("Danger")
            self.STL_dangerous.append(0)
        elif self.trust_score[-1]<=3000:
            self.STL_dangerous.append(1)
        else:
            self.STL_dangerous.append(self.STL_dangerous[-1])

        if len(self.STL_dangerous)>100:
            self.trust=sum(self.STL_dangerous[-100:])/10
        else:
            self.trust=sum(self.STL_dangerous)/len(self.STL_dangerous)
    
    def variance(self,data):
        n = len(data)
        if n < 2:
            return 0
        
        mean = sum(data) / n
        deviations = [(x - mean) ** 2 for x in data]
        variance = sum(deviations) / (n - 1)  # Utilisation de (n - 1) pour la variance non biaisée
        
        return variance 
   

class Input(Node):

    def __init__(self):
        super().__init__('input_node')
        self.get_logger().info("Input Node started.\n")
        self.publisher_ = self.create_publisher(Int32MultiArray, 'input_position', 10)
        self.car_sub = self.create_subscription(Int32MultiArray, 'car_position', self.car_sub_function, 10)
        # self.obstacles_sub = self.create_subscription(Int32MultiArray, 'obstacles_position', self.obstacles_sub_function, 10)
        self.treadmill_sub = self.create_subscription(Int32MultiArray, 'treadmill_position', self.treadmill_sub_function, 1)
        self.Lobstacle=[]
        self.DictCar={}
        self.Linput=[]
        self.Lkeys=[]
        self.treadmill=[320,240]
        self.t0=0
        self.numberOfCar=1

        self.declare_parameter('dangerous', 0)
        
        global dangerous_param
        # Récupération du paramètre
        dangerous_param = self.get_parameter('dangerous').get_parameter_value()
        


    def treadmill_sub_function(self,msg):
        """
        Read treadmill position
        """
        # self.get_logger().info('{}'.format(msg.data))
        if len(msg.data)>=3:
            self.treadmill==msg.data[:3]

    def send_input(self):
        """
        Send input position to input_position topic
        """
        Xmax=500
        Xmin=100
        Ymax=320
        Ymin=50
        self.Linput=[]
        for id in self.Lkeys:
            if id!=2:
                car=self.DictCar[id]
                if car.Xinput>Xmax:
                    car.Xinput=Xmax
                if car.Xinput<Xmin:
                    car.Xinput=Xmin
                if car.Yinput>Ymax:
                    car.Yinput=Ymax
                if car.Yinput<Ymin:
                    car.Yinput=Ymin
                self.Linput+=[car.id,car.Xinput,car.Yinput]

        # if len(self.Linput)/3==self.numberOfCar or (self.numberOfCar==3 and len(self.Linput)/3==2):
        msg = Int32MultiArray()
        msg.data = [int(i) for i in self.Linput]
        self.publisher_.publish(msg)
        # else:
            # self.get_logger().info("Error number of cars {} {}".format(self.numberOfCar,self.Linput)) 
              
    def car_sub_function(self, msg):
        """
        Read car position
        """

        # Nothing detetcted
        if len(msg.data)==0:
            return
        
        # No movement of the treadmill
        if self.t0==0:
            self.t0=time.time()
            t=0
        else:
            t=time.time()-self.t0

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
        
        if 2 in self.Lkeys:
            blue_car=self.DictCar[2]
            distance_min=150
            id_distance_min=None
            # Find the closest car
            for id in self.Lkeys:
                if id!=2:
                    car=self.DictCar[id]
                    distance=((car.Xcar-blue_car.Xcar)**2+(car.Ycar-blue_car.Ycar)**2)**0.5
                    if distance<distance_min:
                        distance_min=distance
                        id_distance_min=id
            
            # Calculate the trust score of the closest car
            if id_distance_min!=None:
                car=self.DictCar[id_distance_min]
                global dangerous_param
                # Récupération du paramètre
                dangerous_param = self.get_parameter('dangerous').get_parameter_value()
                car.calculate_trust_score(t,blue_car.Xcar,blue_car.Ycar)
                self.get_logger().info('Blue car closest with car {}, trust score: {}'.format(id_distance_min,car.trust))
                car.calculate_input(blue_car)
            
            # vérifier la distance entre la voiture 0 et 1
            if 0 in self.Lkeys and 1 in self.Lkeys:
                car0=self.DictCar[0]
                car1=self.DictCar[1]
                if ((car0.Xcar-car1.Xcar)**2+(car0.Ycar-car1.Ycar)**2)**0.5<100:
                    vector=[car0.Xcar-car1.Xcar,car0.Ycar-car1.Ycar]
                    norme=np.linalg.norm(vector)
                    vector=vector/norme
                    car0.Xinput=car0.Xcar+vector[0]
                    car0.Yinput=car0.Ycar+vector[1]
                    car1.Xinput=car1.Xcar-vector[0]
                    car1.Yinput=car1.Ycar-vector[1]
        else:
            self.get_logger().info('No blue car')

        if len(self.Lkeys)>=1:
            self.send_input()

    def obstacles_sub_function(self, msg):
        """
        Read obstacles positions
        """
        self.Lobstacle=[]       


def main(args=None): 
    rclpy.init(args=args)
    global minimal_publisher
    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
