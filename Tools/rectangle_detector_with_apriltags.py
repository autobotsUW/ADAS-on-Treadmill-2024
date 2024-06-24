import cv2
import numpy as np
import time
import apriltag

window=False

def get_surrounding_average(image, x, y, window_size=3):
   half_window = window_size // 2
   start_x = max(x - half_window, 0)
   start_y = max(y - half_window, 0)
   end_x = min(x + half_window + 1, image.shape[1])
   end_y = min(y + half_window + 1, image.shape[0])

   region = image[start_x:end_x, start_y:end_y]
   size=region.shape[0]*region.shape[1]
   avg=[np.sum(region[:,:,k])/size for k in range(3)]
   return np.array(avg)

def pixel_color(pixel):

   lower_red = np.array([90, 100, 120])
   upper_red = np.array([118, 128, 140])
   lower_purple = np.array([108, 128, 140])
   upper_purple = np.array([170, 175, 185])    # upper_purple = np.array([130, 140, 150])
   
   if cv2.inRange(np.array([[pixel]]), lower_red, upper_red) == 255:
      return 'red'
   if cv2.inRange(np.array([[pixel]]), lower_purple, upper_purple) == 255:
      return 'purple'
   return 'other'




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
      car+=[tag.tag_id]+[int(L-tag.center[0])]+[int(tag.center[1])]+[int(angle_degrees)]
      cv2.circle(color_img, [int(tag.center[0]),int(tag.center[1])], 5, (0, 0, 255), -1)
      
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
               cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
               # Calculate the center of the rectangle
               center = (int(rect[0][0]), int(rect[0][1]))
               angle = rect[2]
               if height<width:
                  angle-=90
               treadmill=list(center)+[angle]
           
         elif 0.7<width/height<1.3 and 1000<area<2800:
            print(area)
            # cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
            # Calculate the center of the rectangle
            center = (int(rect[0][0]), int(rect[0][1]))
            radius=int(max(width,height)/2)+1
            cv2.circle(color_img, center, radius, (0, 0, 255), 3)
            Lobstacle.append(list(center)+[radius])
         
         elif 5000>area>3000:
               # this is a car
               # Draw the rotated rectangle on the color image
               cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)                        

               # Calculate the center of the rectangle
               center = (int(rect[0][0]), int(rect[0][1]))
               cv2.circle(color_img, center, 5, (0, 255, 0), -1)
               # Get the angle of the rectangle
               angle = rect[2]
               if height<width:
                  angle+=90
               
               for i in range(0,len(car),4):
                  # print(car[i])
                  # print((((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5)
                  if (((L-car[i+1])-center[0])**2+(car[i+2]-center[1])**2)**0.5<25:
                     car[i+3]=angle

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
   show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   #  cv2.imwrite("output.jpg", color_img)
   end_time = time.time()
   processing_time = end_time - start_time
   print("Processing time: {:.6f} seconds".format(processing_time))
   return treadmill, car,Lobstacle


cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS,30)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
i=0
while True:
   start_time=time.time()
   ret, frame = cap.read()
   # cv2.imwrite("output.png", frame)
   
   if not ret:
      break
   print(find_the_car(frame[20:420,65:783]))
      
   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

   end_time = time.time()
   processing_time = end_time - start_time
   # print("End time: {:.6f} seconds".format(processing_time))
cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()


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
