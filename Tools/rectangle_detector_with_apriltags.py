import cv2
import numpy as np
import time
import apriltag

window=False

def initialize_window(title="Image"):
    global window
    window=True
    cv2.namedWindow(title)
    
def show_image(img, title="Image"):
    if not window:
        initialize_window(title)
    cv2.imshow(title, img)
    cv2.waitKey(1)

# Function to find the car in the image
def find_the_car(color_img):
   L=np.shape(color_img)[1]
   # Convert to grayscale
   gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

   # Start measuring time
   start_time = time.time()
   
   # Search april tags
   tags = detector.detect(gray_img)
   car=[]
   print("Nombre de tag: {}".format(len(tags)))
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
   cv2.imwrite("2-binaire_line.jpg", binary_img)
   
   edges_img = cv2.Canny(binary_img, 50, 150)
   
   # Détecter les lignes avec la transformée de Hough
   lines = cv2.HoughLinesP(edges_img, 1, np.pi/180, 200, minLineLength=200, maxLineGap=700)
   Llines=[]
   # self.get_logger().info("{}".format(lines))
   # Dessiner les lignes détectées sur l'image d'origine
   Ly=[]
   if lines is not None:
      for line in lines:
            x1, y1, x2, y2 = line[0]
            y=int((y1+y2)/2)
            Ly.append(y)
            # Ld=[abs(y-yi) for yi in Llines]
            # Ld.sort()
            # if len(Llines)==0 or Ld[0]>40:
            #    Llines.append(y)
            #    cv2.line(binary_img, (0, y), (L, y), (255, 255, 255), 18)
            #    cv2.line(color_img, (0, y), (L, y), (255, 0, 0), 2)
   i=0
   j=0
   Ly.sort()
   print(len(Ly))
   while i+j+1<len(Ly):
      while i+j+1<len(Ly) and abs(Ly[i+j]-Ly[i+j+1])<20:
         j+=1
      y_mid=sum(Ly[i:i+j+1])//len(Ly[i:i+j+1])
      Llines.append(y_mid)
      cv2.line(binary_img, (0, y_mid), (L, y_mid), (255, 255, 255), 10)
      cv2.line(color_img, (0, y_mid), (L, y_mid), (255, 0, 0), 2)
      i+=j+1
      j=0
  
       

   print(len(Llines))
   Llines.sort()
   cv2.imwrite("3-binaire_line2.jpg", binary_img)
   kernel = np.ones((5,5), np.uint8) 
   binary_img = cv2.dilate(binary_img, kernel, iterations=1)
   show_image(binary_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   binary_img = cv2.erode(binary_img, kernel, iterations=7)
   cv2.imwrite("4-erode.jpg", binary_img)
   binary_img = cv2.dilate(binary_img, kernel, iterations=6)
   cv2.imwrite("5-dilate.jpg", binary_img)

   # Find contours in the binary image
   contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   treadmill=[]
   Lobstacle=[]
   for cnt in contours:
      area = cv2.contourArea(cnt)
      if area>500:
            # Fit a rotated rectangle to the contour
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            width,height=rect[1]
            isAnObstacle=True

            if width>200 and height>200 and area>5e5:
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
                  # print("Distance: {} ".format((((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5))
                  if (((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5<40:
                        isAnObstacle=False
                        # if not(70<angle<110):
                        #     self.get_logger().info("Error angle: {} ".format((((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5))
                        car[i+3]=angle
                        car[i+4]=width
                        car[i+5]=height
                  
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
   # show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   # show_image(binary_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   cv2.imwrite("6-line_detection.png", color_img)
   end_time = time.time()
   processing_time = end_time - start_time
   print("Processing time: {:.6f} seconds".format(processing_time))
   return Llines, car,Lobstacle

cap = cv2.VideoCapture(0,cv2.CAP_GSTREAMER)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

cap.set(cv2.CAP_PROP_SETTINGS, 1)
cap.set(cv2.CAP_PROP_FPS,30)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
i=0
while True:
   start_time=time.time()
   ret, frame = cap.read()
   print(ret)
   
   if not ret:
      break
   # show_image(frame[20:430,10:792], "Rotated Rectangles, Center Points, and Orthogonal Lines")
   # cv2.imwrite("0-img_camera.png", frame[20:430,10:792])
   # cv2.imwrite("1-img_camera_resize.png", frame)
   # assert False
   print(find_the_car(frame[20:430,10:792]))
   # assert False
      
   if (cv2.waitKey(1) & 0xFF == ord('q')) and i>10 :
      break

   end_time = time.time()
   processing_time = end_time - start_time
   # print("End time: {:.6f} seconds".format(processing_time))
   time.sleep(0.1)
   # i+=1
cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
print('Fin')


# import cv2
# import time

# # Initialisation du VideoWriter
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output_video.avi', fourcc, 20.0, (640, 480))

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS,15)

# # Boucle de traitement des images et enregistrement de la vidéo
# while True:
#    start_time = time.time()
#    ret, frame = cap.read()
   
#    if not ret:
#       break
   
#    # Votre traitement d'image ici
#    treadmill, car, Lobstacle = find_the_car(frame[30:450,:])
   
#    # Dessiner des rectangles ou effectuer d'autres traitements sur le cadre si nécessaire
#    # Ensuite, écrire le cadre dans le fichier de sortie
#    out.write(frame)
   
#    cv2.imshow('Video', frame)
   
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#       break
   
#    end_time = time.time()
#    processing_time = end_time - start_time
#    print("Temps de traitement: {:.6f} secondes".format(processing_time))

# # Libérer les ressources
# cap.release()
# out.release()
# cv2.destroyAllWindows()
