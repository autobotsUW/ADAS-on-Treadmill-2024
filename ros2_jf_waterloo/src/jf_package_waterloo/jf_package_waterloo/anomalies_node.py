import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Int32MultiArray
import random as rd


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Anomalies_node')
        self.get_logger().info("Anomalies Node started.\n")
        self.subscription = self.create_subscription(String,'anomalies',self.anomalies_selection,10)

        self.subscription = self.create_subscription(Int32MultiArray,'car_position',self.car_callback,10)
        self.subscription = self.create_subscription(Int32MultiArray,'input_position',self.input_listener_callback,10)
        self.subscription = self.create_subscription(Int32MultiArray,'command',self.command_callback,10)

        self.subscription  # prevent unused variable warning
        self.topic=''
        self.type=''
        self.anomalies=''
        self.frequency=0
        self.publisher=0
        self.timer=0
        self.numberOfCars=1

    def anomalies_selection(self, msg):
        self.get_logger().info('Anomalies: "%s"' % msg.data)
        data=msg.data.split('|')
        self.topic=data[0]
        self.type=data[1]
        self.frequency=int(data[2])
        self.anomalies=data[3]
        
        if self.timer!=0:
            self.timer.destroy()
        self.publisher = self.create_publisher(Int32MultiArray, self.topic, 10)
        timer_period = 1 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.type=='nothing':
            return
        msg=Int32MultiArray()
        data=[]

        if self.topic=="obstacles_position":
            if self.type=="random":
                nb=rd.randint(0,10)
                for i in range(nb):
                    data.append(rd.randint(0,740))  # x
                    data.append(rd.randint(0,400))  # y
                    data.append(rd.randint(0,50))  # radius

            elif self.type=="fixed" or self.type=="add":
                input=self.anomalies.split(',')
                while len(input)%3:
                    input.pop()
                data=[int(i) for i in input]

        elif self.topic=="car_position":
            if self.type=="random":
                nb=rd.randint(0,self.numberOfCars)
                for i in range(nb):
                    data.append(rd.randint(0,1))    #id
                    data.append(rd.randint(0,740))  # x
                    data.append(rd.randint(0,400))  # y
                    data.append(rd.randint(-40,40))  # angle
                    data.append(rd.randint(10,200))  # width
                    data.append(rd.randint(10,100))  # heigth

            elif self.type=="add":
                for i in range(0,len(self.car)):
                    car=self.anomalies.split(',')
                    if i<len(car):
                        data.append(self.car[i]+int(car[i]))
                    else:
                        data.append(self.car[i])
                        
            elif self.type=="fixed":
                car=self.anomalies.split(',')
                while len(car)%6:
                    car.pop()
                data=[int(i) for i in car]

        elif self.topic=="input_position":
            if self.type=="random":
                nb=rd.randint(0,self.numberOfCars)
                for i in range(nb):
                    data.append(rd.randint(0,1))    #id
                    data.append(rd.randint(0,740))  # x
                    data.append(rd.randint(0,400))  # y

            elif self.type=="add":
                for i in range(0,len(self.input)):
                    input=self.anomalies.split(',')
                    if i<len(input):
                        data.append(self.input[i]+int(input[i]))
                    else:
                        data.append(self.input[i])
                        
            elif self.type=="fixed":
                input=self.anomalies.split(',')
                while len(input)%6:
                    input.pop()
                data=[int(i) for i in input]
        

        elif self.topic=="command":
            if self.type=="random":
                nb=rd.randint(0,self.numberOfCars)
                for i in range(nb):
                    data.append(rd.randint(0,1))    # id
                    data.append(rd.randint(0,150))  # speed
                    data.append(rd.randint(80,120))  # direction

            elif self.type=="add":
                for i in range(0,len(self.command)):
                    command=self.anomalies.split(',')
                    if i<len(command):
                        data.append(self.command[i]+int(command[i]))
                    else:
                        data.append(self.command[i])
                        
            elif self.type=="fixed":
                command=self.anomalies.split(',')
                while len(command)%6:
                    command.pop()
                data=[int(i) for i in command]

        msg.data=data
        self.publisher.publish(msg)

    def command_callback(self,msg):
        """
        Save command data received
        """
        self.command=msg.data

    def input_listener_callback(self,msg):
        """
        Save input data received
        """
        self.input=msg.data
    
    def car_callback(self,msg):
        """
        Save input data received
        """
        self.car=msg.data
        numberOfCars=len(self.car)//6
        if numberOfCars>self.numberOfCars:
            self.numberOfCars=numberOfCars




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
