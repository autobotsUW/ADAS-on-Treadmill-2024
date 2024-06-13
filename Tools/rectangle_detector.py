import cv2
import numpy as np
import time

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
def find_the_car(img):
    # Crop and resize the image
    color_img = cv2.resize(img[:, :], (640, 480))

    # Convert to grayscale
    gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

    # Start measuring time
    start_time = time.time()

    # Apply a threshold to get a binary image
    ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)
    cv2.imwrite("binaire.jpg", binary_img)

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
            # print(width,height,width/height)
            # print(area)
            if width>200 and height>200:
               cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
               # Calculate the center of the rectangle
               center = (int(rect[0][0]), int(rect[0][1]))
               angle = rect[2]
               if height<width:
                  angle+=90
               treadmill=list(center)+[angle]
               
               # cv2.arrowedLine(color_img, center, [center[0]+50,center[1]], (0,0,0), 2)
               # cv2.arrowedLine(color_img, center, [center[0],center[1]+50], (0,0,0), 2)
               
               
               # cv2.putText(color_img, 'x', [center[0]+40,center[1]+20], cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)
               # cv2.putText(color_img, 'y', [center[0]+5,center[1]+50], cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)
               
            elif 0.7<width/height<1.3 and 800<area<2800:
               
               # cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
               # Calculate the center of the rectangle
               center = (int(rect[0][0]), int(rect[0][1]))
               radius=int(max(width,height)/2)+1
               cv2.circle(color_img, center, radius, (0, 0, 255), 3)
               Lobstacle.append(list(center)+[radius])
            elif area>3000:
               # Draw the rotated rectangle on the color image
               

               # Calculate the center of the rectangle
               center = (int(rect[0][0]), int(rect[0][1]))

               # Draw a point at the center
               # cv2.circle(color_img, center, 5, (0, 0, 255), -1)
               # print("Center coordinates: {}".format(center))

               # Get the angle of the rectangle
               angle = rect[2]
               if height<width:
                  angle+=90
               car=list(center)+[angle]+[color_img[center]]
               # print(color_img[center])
               # fichier=open('violet.txt','a')
               # fichier.write(str(color_img[center]))
               # fichier.close()


               # car_color=pixel_color(color_img[center])
               car_color=pixel_color(get_surrounding_average(color_img,center[0],center[1]))

               if car_color=='red':
                  cv2.drawContours(color_img, [box], 0, (0, 0, 255), 3)
               elif car_color=='purple':
                  cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
               else:
                  print(color_img[center])
                  cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)
               # print("Angle: {:.2f} degrees".format(angle))
               
               # Calculate the length of the line to draw
               line_length = 100  # You can adjust this value

               # Calculate the end points of the orthogonal line
               orthogonal_angle_rad = np.deg2rad(angle + 90)  # Adjust by 90 degrees
               x_end = int(center[0] + line_length * np.cos(orthogonal_angle_rad))
               y_end = int(center[1] + line_length * np.sin(orthogonal_angle_rad))
               x_start = int(center[0] - line_length * np.cos(orthogonal_angle_rad))
               y_start = int(center[1] - line_length * np.sin(orthogonal_angle_rad))

               # Draw the orthogonal line through the center with the calculated angle
               # cv2.line(color_img, (x_start, y_start), (x_end, y_end), (255, 0, 0), 2)    

    # Display the image with rectangles, center points, and orthogonal lines
    show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
   #  cv2.imwrite("output.jpg", color_img)
    end_time = time.time()
    processing_time = end_time - start_time
   #  print("Processing time: {:.6f} seconds".format(processing_time))
    return treadmill, car,Lobstacle
fichier=open('violet.txt','w')
fichier.close()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS,30)
i=0
while True:
   start_time=time.time()
   ret, frame = cap.read()
   cv2.imwrite("output.png", frame)
   
   if not ret:
      break
   find_the_car(frame[30:450,:])
      
   if cv2.waitKey(1) & 0xFF == ord('q'):
      break
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
