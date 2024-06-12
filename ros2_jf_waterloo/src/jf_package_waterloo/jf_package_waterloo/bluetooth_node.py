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
        bd_addr = '58:56:00:01:06:67'
        port = 1
        try:
            self.sock = bluetooth.BluetoothSocket(Protocols.RFCOMM)
            self.sock.connect((bd_addr, port))
            # self.bluetooth_status = True
            self.get_logger().info("Bluetooth connection established.")
        except Exception as e:
            # If error we stop the treadmill
            msg=String()
            msg.data='{:.2f} bluetooth connection'.format(time.time-self.t0)
            self.error_pub.publish(msg)

    def Send_message(self):
        """
        Send the message by bluetooth
        """
        if not (0 <= self.speed <= 250):
            self.get_logger().info('Invalid speed value')
            return
        
        if not (50 <= self.angle <= 130):
            self.get_logger().info('Invalid angle value')
            return
        
        msg = "[{},{}]".format(int(self.speed), int(self.angle))
        msb_bytes = msg.encode('UTF-8')
        # self.get_logger().error("Sending message...")
        try:
            self.sock.send(msb_bytes)
            # self.get_logger().info('Message sent: speed {} angle {}'.format(self.speed, self.angle))
        except Exception as e:
            # If error we stop the treadmill
            self.get_logger().error("Error sending message: {}".format(e))
            msg=String()
            msg.data='{:.2f} bluetooth connection'.format(time.time-self.t0)
            self.error_pub.publish(msg)
    
    def callback_sub(self, msg):
        """
        Read the command give in command topic 
        """
        self.speed = msg.data[0]
        self.angle = msg.data[1]
        # self.get_logger().info("Command receive: speed {} angle {}".format(self.speed,self.angle))
        # self.information_flag=1
        self.Send_message()
        

def main(args=None):
    rclpy.init(args=args)
    envoie_serie_node = SerialCommunication()
    rclpy.spin(envoie_serie_node)
    envoie_serie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

