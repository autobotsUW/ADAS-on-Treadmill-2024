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
from datetime import datetime
import os

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
        self.display=True  #True to display the image from the camera
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
                # file_name = os.path.expanduser('~/ADAS-on-Treadmill-2024/data/{}.png'.format(datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]))
                # cv2.imwrite(file_name, frame)
                # Search cars and obstacles
                treadmill,car,obstacles=self.find_the_car(frame[20:430,10:792])
                if len(treadmill)>1:
                    msg=Int32MultiArray()
                    msg.data = [int(i) for i in treadmill]
                    self.publisher_treadmill.publish(msg)

                # Send the informations of cars [number,x,y,angle,width,height]
                if len(car)>1:
                    for i in range(0,len(car),6):
                        car[i+3]=car[i+3]-treadmill[2]
                        if car[i+3]>80:
                            car[i+3]-=90
                            # self.get_logger().info("Error angle car {}".format(car))
                            # file_name=os.path.expanduser('~/ADAS-on-Treadmill-2024/Error {}.jpg'.format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
                            # cv2.imwrite(file_name, frame)
                    msg=Int32MultiArray()
                    msg.data = [int(i) for i in car]
                    self.publisher_car.publish(msg)
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
        # cv2.circle(img, (self.Xinput,self.Yinput), 5, (0, 0, 255), -1)
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
            H = tag.homography
            # Normaliser la matrice d'homographie
            H = H / H[2, 2]
            # Extraire les vecteurs de rotation
            r1 = H[:, 0]
            r2 = H[:, 1]
            # Calculer l'angle de rotation
            angle_radians = np.arctan2(r2[1], r2[0])
            angle_degrees = np.degrees(angle_radians)+180
            car+=[tag.tag_id]+[int(L-tag.center[0])]+[int(tag.center[1])]+[int(angle_degrees)]+[0,0]
            if self.display:
                cv2.circle(color_img, [int(tag.center[0]),int(tag.center[1])], 5, (0, 0, 255), -1) 
                line_length = 100  # You can adjust this value
                # Calculate the end points of the orthogonal line
                orthogonal_angle_rad = np.deg2rad(angle_degrees)  # Adjust by 90 degrees
                x_end = int(tag.center[0] + line_length * np.cos(orthogonal_angle_rad))
                y_end = int(tag.center[1] + line_length * np.sin(orthogonal_angle_rad))
                x_start = int(tag.center[0])
                y_start = int(tag.center[1])
                # Draw the orthogonal line through the center with the calculated angle
                cv2.line(color_img, (x_start, y_start), (x_end, y_end), (0, 0, 255), 3)    
            

        # Apply a threshold to get a binary image
        ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)
        # cv2.imwrite("2-binaire_line.jpg", binary_img)

        edges_img = cv2.Canny(binary_img, 50, 150)

        # Détecter les lignes avec la transformée de Hough
        lines = cv2.HoughLinesP(edges_img, 1, np.pi/180, 100, minLineLength=300, maxLineGap=700)
        Llines=[]
        # self.get_logger().info("{}".format(lines))
        # Dessiner les lignes détectées sur l'image d'origine
        Ly=[]
        if lines is not None:
            for line in lines:
                    x1, y1, x2, y2 = line[0]
                    y=int((y1+y2)/2)
                    Ly.append(y)
        i=0
        j=0
        Ly.sort()
        print(len(Ly))
        while i+j+1<len(Ly):
            while i+j+1<len(Ly) and abs(Ly[i+j]-Ly[i+j+1])<20:
                j+=1
            y_mid=sum(Ly[i:i+j+1])//len(Ly[i:i+j+1])
            Llines.append(y_mid)
            cv2.line(binary_img, (0, y_mid), (L, y_mid), (255, 255, 255), 15)
            if self.display:
                cv2.line(color_img, (0, y_mid), (L, y_mid), (255, 0, 0), 2)
            i+=j+1
            j=0
                
        # print(len(Llines))
        Llines.sort()
        #    cv2.imwrite("3-binaire_line2.jpg", binary_img)
        kernel = np.ones((5,5), np.uint8) 
        binary_img = cv2.erode(binary_img, kernel, iterations=6)
        #    cv2.imwrite("4-erode.jpg", binary_img)
        binary_img = cv2.dilate(binary_img, kernel, iterations=6)
        #    cv2.imwrite("5-dilate.jpg", binary_img)

        # Find contours in the binary image
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        treadmill=[np.shape(color_img)[1]//2,np.shape(color_img)[0]//2,90]
        Lobstacle=[]
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area>800:
                # Fit a rotated rectangle to the contour
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                width,height=rect[1]
                isAnObstacle=True

                if width>200 and height>200 and area>5e5:
                    if self.display:
                        cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
                    # Calculate the center of the rectangle
                    center = (int(rect[0][0]), int(rect[0][1]))
                    angle = rect[2]
                    if height<width:
                        angle+=90
                    treadmill=list(center)+[abs(angle)]
                    isAnObstacle=False
                    
                elif 8000>area>2000:
                    # This is a car
                    # Calculate the center of the rectangle
                    center = (int(rect[0][0]), int(rect[0][1]))
                    # Get the angle of the rectangle
                    angle = rect[2]
                    if height<width:
                        angle+=90
                    else:
                        height,width=width,height
                    
                    # Add the angle, the width, and the height to the list
                    for i in range(0,len(car),6):
                        # self.get_logger().info("Distance: {} ".format((((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5))
                        if (((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5<40:
                            isAnObstacle=False
                            # if not(70<angle<110):
                            #     self.get_logger().info("Error angle: {} ".format((((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5))
                            car[i+3]=angle
                            car[i+4]=width
                            car[i+5]=height
                        
                            if self.display:
                                cv2.circle(color_img, center, 5, (0, 255, 0), -1)
                                # Draw the rotated rectangle on the color image
                                cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)  
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

                if area<1e4 and isAnObstacle==True:
                    # Calculate the center of the rectangle
                    center = (int(L-rect[0][0]), int(rect[0][1]))
                    radius=int(max(width,height)/2)+1
                    for i in range(0,len(car),6):
                        self.get_logger().info("Distance: {} {}".format((((car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5,radius))
                        if (((car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5>radius:
                    
                            if self.display:
                                center2 = (int(rect[0][0]), int(rect[0][1]))
                                cv2.circle(color_img, center2, radius, (0, 0, 255), 3)
                            Lobstacle.append(list(center)+[radius])
            else:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                width,height=rect[1]
                cv2.drawContours(binary_img, [box], 0, (0, 255, 255), 3)

                    
        # Display the image with rectangles, center points, and orthogonal lines
        if self.display:
            self.show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
            # self.show_image(binary_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
        #  cv2.imwrite("output.jpg", color_img)
        treadmill+=Llines
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
