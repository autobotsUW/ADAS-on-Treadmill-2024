import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32MultiArray
from tkinter import * 

class Input(Node):

    def __init__(self):
        super().__init__('input_node')
        self.get_logger().info("Input Node started.\n")
        self.publisher_ = self.create_publisher(Int32MultiArray, 'input_position', 10)
        self.Xinput,self.Yinput=320,200
        self.send_input(self.Xinput,self.Yinput)
        self.create_root()

    def send_input(self,x,y):
        """
        Send the input to input_position topic
        """
        msg = Int32MultiArray()
        msg.data = [int(x),int(y)]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def create_root(self):
        """
        create the window to give input_position
        """
        root = Tk()
        root.title("ADAS on Treadmill : Input")
        root.option_add("*Font", "Arial 16")
        # root.geometry("800x400")

        label = Label(root, text="Give me a position:")
        label.grid(column=0, row=0,columnspan=2)

        # bouton_clear.grid (column = 2, row = 2,columnspan=2)
        label = Label(root, text="x:",justify='center')
        label.grid(column=0, row=1)

        Xinput = Spinbox(root, from_=0, to=640,width=5,justify='center',textvariable=IntVar(value=320))
        Xinput.grid(column=0, row=2)


        label = Label(root, text="y:",width=5,justify='center')
        label.grid(column=1, row=1)
        Yinput = Spinbox(root, from_=0, to=480,width=5,justify='center',textvariable=IntVar(value=200))
        Yinput.grid(column=1, row=2)

        bouton=Button(root, text="Set inuput", command=lambda:self.send_input(x= Xinput.get(),y=Yinput.get()))
        bouton.grid(column=0, row=3,columnspan=2)

        root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
