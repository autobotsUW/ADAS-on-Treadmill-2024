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
import apriltag

from std_msgs.msg import String,Float32MultiArray,Int32MultiArray




class Camera(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera Node started.\n")
        self.publisher_car = self.create_publisher(Int32MultiArray, 'car_position', 10)
        self.publisher_treadmill = self.create_publisher(Int32MultiArray, 'treadmill_position', 10)
        self.publisher_obstacles = self.create_publisher(Int32MultiArray, 'obstacles_position', 10)
        self.error_pub = self.create_publisher(String, 'error', 10)
        self.Xinput=320
        self.Yinput=240
        self.window=False
        self.display=False  #True to display the image from the camera
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        self.launch_camera()
    
    def launch_camera(self):
        """
        Open the camera, find the position of car, and the obstacles and send to topic car_position and obstacles_position
        """
        # Parameter of camera 
        cap = cv2.VideoCapture(0)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        cap.set(cv2.CAP_PROP_FPS,30)


        while True:
            start_time = time.time()
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
                treadmill,car,obstacles=self.find_the_car(frame[20:420,65:783])
                if len(treadmill)>1:
                    msg=Int32MultiArray()
                    msg.data = [int(i) for i in treadmill]
                    self.publisher_treadmill.publish(msg)

                # Send the informations of cars [number,x,y,angle]
                if len(car)>1:
                    for i in range(0,len(car),6):
                            car[i+3]=car[i+3]-treadmill[2]
                    msg=Int32MultiArray()
                    msg.data = [int(i) for i in car]
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
            end_time=time.time()
            processing_time = end_time - start_time
            # self.get_logger().info("Processing time: {:.3f} seconds".format(processing_time))
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

    def find_the_car(self,color_img):
        """
        Search car and obstacles in the image
        Input: image with a good size
        Output: information of tradmill, cars and obstacles 
        """
        L=np.shape(color_img)[1]
        # Convert to grayscale
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Start measuring time
        start_time = time.time()
        
        # Search april tags
        tags = self.detector.detect(gray_img)
        car=[]
        for tag in tags:
        #    H = tag.homography
        #          # Normaliser la matrice d'homographie
        #    H = H / H[2, 2]
        #    # Extraire les vecteurs de rotation
        #    r1 = H[:, 0]
        #    r2 = H[:, 1]
        #    # Calculer l'angle de rotation
        #    angle_radians = np.arctan2(r2[1], r2[0])
        #    angle_degrees = np.degrees(angle_radians)+180
            angle_degrees=0
            car+=[tag.tag_id]+[int(L-tag.center[0])]+[int(tag.center[1])]+[int(angle_degrees)]+[0,0]
            if self.display:
                cv2.circle(color_img, [int(tag.center[0]),int(tag.center[1])], 5, (0, 0, 255), -1)
            # self.get_logger().info(str(car))
            
            # for corner in tag.corners:
            #    cv2.circle(color_img, [int(corner[0]),int(corner[1])], 5, (0, 0, 255), -1)
            # line_length = 100  # You can adjust this value

            # # Calculate the end points of the orthogonal line
            # orthogonal_angle_rad = np.deg2rad(angle_degrees)  # Adjust by 90 degrees
            # x_end = int(tag.center[0] + line_length * np.cos(orthogonal_angle_rad))
            # y_end = int(tag.center[1] + line_length * np.sin(orthogonal_angle_rad))
            # x_start = int(tag.center[0])
            # y_start = int(tag.center[1])

            # Draw the orthogonal line through the center with the calculated angle
            # cv2.line(color_img, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)    


        # Apply a threshold to get a binary image
        ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)
        # cv2.imwrite("binaire.jpg", binary_img)

        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                    if self.display:
                        cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
                    # Calculate the center of the rectangle
                    center = (int(rect[0][0]), int(rect[0][1]))
                    angle = rect[2]
                    if height<width:
                        angle+=90
                    treadmill=list(center)+[angle]
                
                elif 0.7<width/height<1.3 and 1000<area<2800:
                    print(area)
                    # cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
                    # Calculate the center of the rectangle
                    center = (int(L-rect[0][0]), int(rect[0][1]))
                    radius=int(max(width,height)/2)+1
                    if self.display:
                        cv2.circle(color_img, center, radius, (0, 0, 255), 3)
                    Lobstacle.append(list(center)+[radius])
                
                elif 5000>area>3000:
                    # this is a car
                                          

                    # Calculate the center of the rectangle
                    center = (int(rect[0][0]), int(rect[0][1]))
                    if self.display:
                        cv2.circle(color_img, center, 5, (0, 255, 0), -1)
                        # Draw the rotated rectangle on the color image
                        cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)  
                    # Get the angle of the rectangle
                    angle = rect[2]
                    # self.get_logger().info('{} {} {}'.format(height<width,height,width))
                    if height<width:
                        angle+=90
                    else:
                        height,width=width,height

                    
                    for i in range(0,len(car),6):
                        if (((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5<25:
                            car[i+3]=angle
                            car[i+4]=width
                            car[i+5]=height
                            # self.get_logger().info(str(car))
                            if self.display:
                                # Calculate the length of the line to draw
                                line_length = 100  # You can adjust this value

                                # Calculate the end points of the orthogonal line
                                orthogonal_angle_rad = np.deg2rad(angle+90)  # Adjust by 90 degrees
                                x_end = int(center[0] + line_length * np.cos(orthogonal_angle_rad))
                                y_end = int(center[1] + line_length * np.sin(orthogonal_angle_rad))
                                x_start = int(center[0])
                                y_start = int(center[1])

                                # Draw the orthogonal line through the center with the calculated angle
                                cv2.line(color_img, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)    

                    
        # Display the image with rectangles, center points, and orthogonal lines
        if self.display:
            self.show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
        #  cv2.imwrite("output.jpg", color_img)
        end_time = time.time()
        processing_time = end_time - start_time
        # self.get_logger().info("Processing time: {:.3f} seconds".format(processing_time))
        return treadmill, car,Lobstacle

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
