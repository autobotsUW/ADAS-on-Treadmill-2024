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
        self.get_logger().info("Treadmill control Node started.\n")
        self.subscription = self.create_subscription(String,'treadmill',self.treadmill_sub_function,1)
        self.subscription  # prevent unused variable warning
        
    def treadmill_sub_function(self,msg):
        # self.get_logger().info("Treadmill sub function.\n")
        if msg.data=='STOP':
            self.send_to_treadmill(65,0)
        elif msg.data=='START':
            self.send_to_treadmill(61,200)
            self.send_to_treadmill(65,97)

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

def main(args=None):
    rclpy.init(args=args)

    node = Treadmill()
    # node.mainloop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


