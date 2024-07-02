import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Int32MultiArray
import random as rd


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Anomalies_node')
        self.get_logger().info("Anomalies Node started.\n")
        self.subscription = self.create_subscription(String,'anomalies',self.anomalies_selection,10)
        self.subscription  # prevent unused variable warning
        self.topic=''
        self.type=''
        self.anomalies=''
        self.frequency=0
        self.publisher=0
        self.timer=0

    def anomalies_selection(self, msg):
        self.get_logger().info('Anomalies: "%s"' % msg.data)
        data=msg.data.split('|')
        self.topic=data[0]
        self.type=data[1]
        self.frequency=data[2]
        self.anomalies=data[3]
        
        if self.publisher!=0 and self.timer!=0:
            self.publisher.destroy()
            self.timer.destroy()
        self.publisher=self.create_publisher(Int32MultiArray,self.topic,10)
        timer_period = 1/self.frequency # seconds
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
                input=self.anomalies.split(';')
                while len(input)%3:
                    input.pop()
                data=[int(i) for i in input]

        msg.data=data
        self.publisher(msg)



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
