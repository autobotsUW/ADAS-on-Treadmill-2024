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
      # H = tag.homography
      #       # Normaliser la matrice d'homographie
      # H = H / H[2, 2]
      # # Extraire les vecteurs de rotation
      # r1 = H[:, 0]
      # r2 = H[:, 1]
      # # Calculer l'angle de rotation
      # angle_radians = np.arctan2(r2[1], r2[0])
      # angle_degrees = np.degrees(angle_radians)+180

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

      # # Draw the orthogonal line through the center with the calculated angle
      # cv2.line(color_img, (x_start, y_start), (x_end, y_end), (0, 0, 255), 3)    


   # Apply a threshold to get a binary image
   ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)
   cv2.imwrite("2-binaire_line.jpg", binary_img)
   
   edges_img = cv2.Canny(binary_img, 50, 150)
   
   # Détecter les lignes avec la transformée de Hough
   lines = cv2.HoughLinesP(edges_img, 1, np.pi/180, 100, minLineLength=400, maxLineGap=400)
   Llines=[]
   # Dessiner les lignes détectées sur l'image d'origine
   for line in lines:
      x1, y1, x2, y2 = line[0]
      y=int((y1+y2)/2)
      Ld=[abs(y-yi) for yi in Llines]
      Ld.sort()
      if len(Llines)==0 or Ld[0]>40:
         Llines.append(y)
         cv2.line(color_img, (0, y), (L, y), (255, 0, 0), 2)
         cv2.line(binary_img, (0, y), (L, y), (255, 255, 255), 18)
   # print(len(Llines))
   Llines.sort()
   cv2.imwrite("3-binaire_line2.jpg", binary_img)
   kernel = np.ones((5,5), np.uint8) 
   binary_img = cv2.erode(binary_img, kernel, iterations=6)
   cv2.imwrite("4-erode.jpg", binary_img)
   binary_img = cv2.dilate(binary_img, kernel, iterations=6)
   cv2.imwrite("5-dilate.jpg", binary_img)

   # Find contours in the binary image
   contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   treadmill=[]
   Lobstacle=[]
   for cnt in contours:
      area = cv2.contourArea(cnt)
      # Fit a rotated rectangle to the contour
      rect = cv2.minAreaRect(cnt)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      width,height=rect[1]
      if width>200 and height>200 and area>5e5:
            # treadmill
            cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
            # Calculate the center of the rectangle
            center = (int(rect[0][0]), int(rect[0][1]))
            angle = rect[2]
            if height<width:
               angle-=90
            treadmill=list(center)+[abs(angle)]

      elif area<2200:
         print(area)
         # cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
         # Calculate the center of the rectangle
         center = (int(rect[0][0]), int(rect[0][1]))
         radius=int(max(width,height)/2)+1
         cv2.circle(color_img, center, radius, (0, 0, 255), 3)
         Lobstacle.append(list(center)+[radius])
      
      elif 8000>area>2200:
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
            else:
               height,width=width,height

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
      else: 
          print(area)

            
   # Display the image with rectangles, center points, and orthogonal lines
   show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   # show_image(binary_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   cv2.imwrite("6-line_detection.png", color_img)
   end_time = time.time()
   processing_time = end_time - start_time
   print("Processing time: {:.6f} seconds".format(processing_time))
   return Llines, car,Lobstacle


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
   cv2.imwrite("0-img_camera.png", frame)
   cv2.imwrite("1-img_camera_resize.png", frame[48:425,54:760])
   # assert False
   
   if not ret:
      break
   print(find_the_car(frame[48:425,54:769]))
   # assert False
      
   if (cv2.waitKey(1) & 0xFF == ord('q')) or i>10:
      break

   end_time = time.time()
   processing_time = end_time - start_time
   # print("End time: {:.6f} seconds".format(processing_time))
   time.sleep(0.5)
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
