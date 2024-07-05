import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray,String
import bluetooth
from bluetooth import Protocols
import time

class SerialCommunication(Node):

    def __init__(self):
        super().__init__('Serial_communication_with_bluetooth')
        self.get_logger().info("Serial Node started.\n")
        self.subscription = self.create_subscription(Int32MultiArray, 'command', self.callback_sub, 10)
        self.error_pub = self.create_publisher(String, 'error', 10)
        self.angle = 0
        self.speed = 0
        self.t0=time.time()
        
        # Define all bluetooth address with is id car
        self.DictAddr={}
        self.DictAddr[0]='98:D3:71:FE:AB:41'
        self.DictAddr[1]='98:D3:51:FE:EC:72'

        # Open all bluetooth connection
        self.DictSock={}
        for key in self.DictAddr.keys():
            bd_addr=self.DictAddr[key]
            port = 1
            state=False
            i=0
            # we try 10 connection attemps for each cars
            while state==False and i<=10:
                try:
                    sock = bluetooth.BluetoothSocket(Protocols.RFCOMM)
                    sock.connect((bd_addr, port))
                    # self.bluetooth_status = True
                    
                    state=True
                    self.DictSock[key]=sock
                except Exception as e:
                    # If error we stop the treadmill
                    msg=String()
                    msg.data='{:.2f} bluetooth connection'.format(time.time()-self.t0)
                    self.error_pub.publish(msg)
                i+=1
                self.get_logger().info('Bluetooth connection NO established. {} {}/10'.format(key,i))    
            self.get_logger().info("Bluetooth connection established. {} {}".format(key,bd_addr))

    def Send_message(self):
        """
        Send the message by bluetooth for each cars
        """
        for i in range(0,len(self.command),3):
            id,speed,angle=self.command[i:i+3]
            if id in self.DictSock.keys():
                msg = "[{},{}]".format(int(speed), int(angle))
                msb_bytes = msg.encode('UTF-8')
                sock=self.DictSock[id]
                try:
                    sock.send(msb_bytes)
                except Exception as e:
                    # If error we stop the treadmill
                    self.get_logger().error("Error sending message: {}".format(e))
                    msg=String()
                    msg.data='{:.2f} bluetooth connection'.format(time.time()-self.t0)
                    self.error_pub.publish(msg)
            else:
                self.get_logger().error("Error sending message: car no exist")
                msg=String()
                msg.data='{:.2f} Error sending message: car no exist'.format(time.time()-self.t0)
                self.error_pub.publish(msg)

    
    def callback_sub(self, msg):
        """
        Read the command give in command topic 
        """
        self.command=msg.data
        self.Send_message()
        

def main(args=None):
    rclpy.init(args=args)
    envoie_serie_node = SerialCommunication()
    rclpy.spin(envoie_serie_node)
    envoie_serie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

