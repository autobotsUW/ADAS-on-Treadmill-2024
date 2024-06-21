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
        
        # Open bluetooth connection
        self.DictAddr={0:'58:56:00:01:06:67',1:'98:D3:71:FE:AB:41'}
        self.DictSock={}
        for key in self.DictAddr.keys():
            bd_addr=self.DictAddr[key]
            self.get_logger().info(bd_addr)
            port = 1
            try:
                sock = bluetooth.BluetoothSocket(Protocols.RFCOMM)
                self.get_logger().info(sock.connect((bd_addr, port)))
                # self.bluetooth_status = True
                self.get_logger().info("Bluetooth connection established.")
                self.DictSock[key]=sock
            except Exception as e:
                # If error we stop the treadmill
                msg=String()
                msg.data='{:.2f} bluetooth connection'.format(time.time()-self.t0)
                self.error_pub.publish(msg)
            self.get_logger().info('Fin')

    def Send_message(self):
        """
        Send the message by bluetooth
        """
        for i in range(0,len(self.command),3):
            id,speed,angle=self.command[i:i+3]
            if id in self.DictSock.keys():
                msg = "[{},{}]".format(int(speed), int(angle))
                msb_bytes = msg.encode('UTF-8')
                # self.get_logger().error("Sending message...")
                sock=self.DictSock[id]
                try:
                    sock.send(msb_bytes)
                    # self.get_logger().info('Message sent {}: speed {} angle {}'.format(id,self.speed, self.angle))
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

