import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

class Treadmill(Node):

    def __init__(self):
        super().__init__('treadmill_control_node')
        self.get_logger().info("Treadmill interface started.\n")
        self.create_window()
        

    def send_to_treadmill(self,parameter, value):
        # Set Chrome options for headless mode
        chrome_options = Options()
        chrome_options.add_argument('--headless')  # Run Chrome in headless mode

        # Set up the WebDriver with the specified Chrome options
        driver = webdriver.Chrome(options=chrome_options)

        try:
            # Open the target webpage
            driver.get('http://192.168.124.16/GSP.htm')  # Replace with the actual URL of the page

            # Locate the D900 input element and set its value
            d900_input = driver.find_element(By.NAME, 'D900')
            d900_input.clear()  # Clear existing text
            d900_input.send_keys(str(parameter))  # Enter the new value

            # Locate the D901 input element and set its value
            d901_input = driver.find_element(By.NAME, 'D901')
            d901_input.clear()  # Clear existing text
            d901_input.send_keys(str(value))  # Enter the new value

            # Locate the "Write" button and click it
            write_button = driver.find_element(By.XPATH, '//input[@type="submit" and @value="Write"]')
            write_button.click()
            # print('Send {} {}'.format(parameter,value))
            self.get_logger().info('Send to treadmill {} {}'.format(parameter,value))

        finally:
            # Close the WebDriver
            driver.quit()

    def update_speed(self,val):
        self.send_to_treadmill(61, val)
        self.label_speed.config(text="Speed : {:.2f} m/s".format(int(val)*0.00541))
        if int(val)==0:
            self.send_to_treadmill(65,0)

    def create_window(self):
        global root
        root = tk.Tk()
        root.title("ADAS on Treadmill : Control the treadmill")
        root.option_add("*Font", "Arial 16")

        # Frame for Start/Stop buttons
        frame_buttons = tk.Frame(root, padx=20, pady=20)
        frame_buttons.pack()

        start_button = tk.Button(frame_buttons, text="Start", command=lambda: self.send_to_treadmill(65, 97))
        start_button.grid(column=0, row=0, padx=10)

        stop_button = tk.Button(frame_buttons, text="Stop", command=lambda: self.send_to_treadmill(65, 0))
        stop_button.grid(column=1, row=0, padx=10)

        # Frame for Scale
        frame_scale = tk.Frame(root, padx=20, pady=20)
        frame_scale.pack()

        self.label_speed = tk.Label(frame_scale, text="Speed : {:.2f} m/s".format(0))
        self.label_speed.grid(column=0, row=0, columnspan=2, pady=10)

        scale = tk.Scale(frame_scale, from_=0, to=600, orient='horizontal', command=self.update_speed, length=400)
        scale.grid(column=0, row=1, columnspan=2, pady=10)
        # scale.after(100,self.treadmill_sub_function)

        # Frame for Other Parameters
        frame_parameters = tk.Frame(root, padx=20, pady=20)
        frame_parameters.pack()

        label_parameters = tk.Label(frame_parameters, text="Other parameters:")
        label_parameters.grid(column=0, row=0, columnspan=2, pady=10)

        label_number = tk.Label(frame_parameters, text="Number:", justify='center')
        label_number.grid(column=0, row=1, pady=5)

        Xinput = tk.Spinbox(frame_parameters, from_=0, to=1000, width=5, justify='center', textvariable=tk.IntVar(value=0))
        Xinput.grid(column=0, row=2, pady=5)

        label_value = tk.Label(frame_parameters, text="Value:", width=5, justify='center')
        label_value.grid(column=1, row=1, pady=5)

        Yinput = tk.Spinbox(frame_parameters, from_=0, to=1000, width=5, justify='center', textvariable=tk.IntVar(value=0))
        Yinput.grid(column=1, row=2, pady=5)

        button_set_input = tk.Button(frame_parameters, text="Set input", command=lambda: self.send_to_treadmill(Xinput.get(), Yinput.get()))
        button_set_input.grid(column=0, row=3, columnspan=2, pady=10)

        root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    node = Treadmill()
    # node.mainloop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


