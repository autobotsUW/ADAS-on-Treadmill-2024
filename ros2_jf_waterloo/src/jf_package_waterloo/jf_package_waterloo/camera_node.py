# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time

from std_msgs.msg import String,Float32MultiArray,Int32MultiArray




class Camera(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera Node started.\n")
        self.publisher_car = self.create_publisher(Int32MultiArray, 'car_position', 10)
        self.publisher_obstacles = self.create_publisher(Int32MultiArray, 'obstacles_position', 10)
        self.error_pub = self.create_publisher(String, 'error', 10)
        self.Xinput=320
        self.Yinput=240
        self.window=False
        self.display=False
        self.launch_camera()
    
    def launch_camera(self):
        """
        Open the camera, find the position of car, and the obstacles and send to topic car_position and obstacles_position
        """
        # Parameter of camera 640*480 and 30 FPS
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        cap.set(cv2.CAP_PROP_FPS,30)


        while True:
            # read a picture from the camera
            ret,frame=cap.read()
        
            if not ret:
                # If problem with the camera we stop the treadmill
                self.get_logger().info("Can't receive frame (stream end?).")
                msg=String()
                msg.data='camera connection'
                self.error_pub.publish(msg)
            else:
                # Search cars and obstacles
                treadmill,car,obstacles=self.find_the_car(frame[40:453,:])

                # Send the informations of cars [x,y,angle,height,width]
                if len(car)>1:
                    msg=Int32MultiArray()
                    car[2]=car[2]-treadmill[2]
                    msg.data = [int(0)]+[int(i) for i in car]
                    self.publisher_car.publish(msg)
                    # self.get_logger().info("Car detected in {} angle:{:.1f}".format(car[0:2],car[2]))
                else:
                    # If no car
                    msg=Int32MultiArray()
                    msg.data = []
                    self.publisher_car.publish(msg)
                    # self.get_logger().info("Car not detected")
                
                # Send the informations of obstacles [x,y,radius]
                if len(obstacles)>0:
                    msg=Int32MultiArray()
                    L=[]
                    for obstacle in obstacles:
                        # self.get_logger().info(str(obstacle))
                        if len(car)>1:
                            if (car[0]-obstacle[0])**2+(car[1]-obstacle[1])**2>obstacle[2]**2:
                                L+=[int(i) for i in obstacle]
                        else:
                            L+=[int(i) for i in obstacle]
                    msg.data=L
                    self.publisher_obstacles.publish(msg)
                    # self.get_logger().info("Obstacles detected in {}".format(L))
                else:
                    # If no obtsacle
                    msg=Int32MultiArray()
                    msg.data = []
                    self.publisher_obstacles.publish(msg)
                    # self.get_logger().info("No obstacles detected") 
                    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def initialize_window(self,title="Image"):
        """
        Initializes a display
        """
        self.window=True
        cv2.namedWindow(title)
        
    def show_image(self,img, title="Image"):
        """
        Display the camera picture after image processing
        """
        if not self.window:
            self.initialize_window(title)
        cv2.circle(img, (self.Xinput,self.Yinput), 5, (0, 0, 255), -1)
        cv2.imshow(title, img)
        cv2.waitKey(1)

    def find_the_car(self,img):
        """
        Search car and obstacles in the image
        Input: image with a good size
        Output: information of tradmill, cars and obstacles 
        """
        # Crop and resize the image
        color_img = cv2.resize(img[:, :], (640, 480))

        # Convert to grayscale
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Start measuring time
        start_time = time.time()

        # Apply a threshold to get a binary image
        ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)

        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        car = []
        treadmill=[]
        Lobstacle=[]
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 800:  # Filter small contours
                # Fit a rotated rectangle to the contour
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                width,height=rect[1]
                if width>200 and height>200:
                    # This is the treadmill
                    if self.display:
                        cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
                    # Calculate the center of the rectangle
                    center = (int(640-rect[0][0]), int(rect[0][1]))
                    angle = rect[2]
                    if height<width:
                        angle+=90
                    treadmill=list(center)+[angle]
                
                elif 0.8<width/height<1.2 and 800<area<2800:
                    # This is an obstacle
                    # Calculate the center of the rectangle
                    center = (int(640-rect[0][0]), int(rect[0][1]))
                    radius=int(max(width,height)/2)+1
                    if self.display:
                        cv2.circle(color_img, center, radius, (0, 0, 255), 3)
                    Lobstacle.append(list(center)+[radius])
                elif area>3000:
                    # This is a car
                    # Calculate the center of the rectangle
                    center = (int(640-rect[0][0]), int(rect[0][1]))
                    
                    # Get the angle of the rectangle
                    angle = rect[2]
                    if height<width:
                        angle+=90
                        car=list(center)+[angle,width,height]
                    else:
                        car=list(center)+[angle,height,width]
                    # print("Angle: {:.2f} degrees".format(angle))
                    
                    if self.display:
                        # Display
                        # Draw the rotated rectangle on the color image
                        cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)
                        
                        # Draw a point at the center
                        cv2.circle(color_img, center, 5, (0, 0, 255), -1)
                        # print("Center coordinates: {}".format(center))
                        
                        # Calculate the length of the line to draw
                        line_length = 100  # You can adjust this value

                        # Calculate the end points of the orthogonal line
                        orthogonal_angle_rad = np.deg2rad(angle + 90)  # Adjust by 90 degrees
                        x_end = int(center[0] + line_length * np.cos(orthogonal_angle_rad))
                        y_end = int(center[1] + line_length * np.sin(orthogonal_angle_rad))
                        x_start = int(center[0] - line_length * np.cos(orthogonal_angle_rad))
                        y_start = int(center[1] - line_length * np.sin(orthogonal_angle_rad))

                        # Draw the orthogonal line through the center with the calculated angle
                        cv2.line(color_img, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)   

        # Display the image with rectangles, center points, and orthogonal lines
        if self.display:
            self.show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
        #  cv2.imwrite("output.jpg", color_img)
        end_time = time.time()
        processing_time = end_time - start_time
        print("Processing time: {:.6f} seconds".format(processing_time))
        return treadmill, car,Lobstacle

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
