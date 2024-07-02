from tkinter import *
import random  # Pour générer des valeurs aléatoires

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Input(Node):

    def __init__(self):
        super().__init__('anomalies_input_node')
        self.get_logger().info("Anomalies input Node started.\n")
        self.publisher_ = self.create_publisher(String, 'anomalies', 10)
        self.create_root()

    def send_input(self):
        """
        Send the input to 'anomalies' topic
        """
        selected_type = self.topic_type.get()
        selected_method = self.topic_method.get()
        additional_text = self.text_entry.get()
        frequency=self.frequency.get()

        # Prepare message data based on selected type and method
        data = "{}|{}|{}|{}".format(selected_type,selected_method,frequency,additional_text)
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def create_root(self):
        """
        Create the window to input anomalies
        """

        # Main window
        root = Tk()
        root.title("Anomalies Input")
        root.option_add("*Font", "Arial 16")

        # Frame for type selection
        type_frame = Frame(root)
        type_frame.grid(row=0, column=0, padx=10, pady=10, sticky=W)

        # Label for type selection
        type_label = Label(type_frame, text="Select a topic:")
        type_label.grid(row=0, column=0, sticky=W)

        # Variable for selected type
        self.topic_type = StringVar()
        self.topic_type.set("car_position")  # Default: "car"

        # Radio buttons for types
        types = ["car_position", "input_position", "obstacles_position", "command"]
        for i, type in enumerate(types):
            rb = Radiobutton(type_frame, text=type, variable=self.topic_type, value=type)
            rb.grid(row=i+1, column=0, sticky=W)

        # Frame for method selection
        method_frame = Frame(root)
        method_frame.grid(row=0, column=1, padx=10, pady=10, sticky=W)

        # Label for method selection
        method_label = Label(method_frame, text="Select a type:")
        method_label.grid(row=0, column=0, sticky=W)

        # Variable for selected method
        self.topic_method = StringVar()
        self.topic_method.set("nothing")  # Default: "random"

        # Radio buttons for methods
        methods = ["nothing", "random", "add", "fixed"]
        for i, method in enumerate(methods):
            rb = Radiobutton(method_frame, text=method, variable=self.topic_method, value=method)
            rb.grid(row=i+1, column=0, sticky=W)

        # Text entry for additional text
        text_entry_label = Label(root, text="Anomalies:")
        text_entry_label.grid(row=1, column=0, padx=10, pady=10, sticky=W+E, columnspan=2)

        self.text_entry = Entry(root, width=30)
        self.text_entry.grid(row=2, column=0, padx=10, pady=10, sticky=W+E, columnspan=2)

        # Label and Spinbox for frequency selection
        frequency_label = Label(root,  justify='center', text="Frequency:")
        frequency_label.grid(row=3, column=0, padx=10, pady=10, sticky=W)

        self.frequency = Spinbox(root, from_=1, to=100, width=5, justify='center', textvariable=IntVar(value=10))
        self.frequency.grid(row=3, column=1, padx=10, pady=10, sticky=E)

        # Button to send anomalies
        send_button = Button(root, text="Send anomalies", command=self.send_input)
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Input()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
