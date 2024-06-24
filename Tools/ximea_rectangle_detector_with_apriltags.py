import numpy as np
import time
import apriltag
from ximea import xiapi
import cv2 


window=False
# Initialize XIMEA camera
cam = xiapi.Camera()

# Open the camera device
print('Opening camera...')
cam.open_device()
cam.set_param('exposure',10000.0)

# Set camera parameters (example: exposure, width, height, FPS)
# cam.set_exposure(10000)  # Example: Set exposure to 10000 microseconds
# cam.set_width(1280)       # Example: Set width to 1280 pixels
# cam.set_height(720)       # Example: Set height to 720 pixels
# cam.set_framerate(30)     # Example: Set framerate to 30 FPS

# Start acquisition
print('Starting data acquisition...')
cam.start_acquisition()

# AprilTag detector setup
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

def get_surrounding_average(image, x, y, window_size=3):
    half_window = window_size // 2
    start_x = max(x - half_window, 0)
    start_y = max(y - half_window, 0)
    end_x = min(x + half_window + 1, image.shape[1])
    end_y = min(y + half_window + 1, image.shape[0])
    region = image[start_y:end_y, start_x:end_x]
    avg = np.mean(region, axis=(0, 1))
    return avg

def pixel_color(pixel):
    lower_red = np.array([90, 100, 120])
    upper_red = np.array([118, 128, 140])
    lower_purple = np.array([108, 128, 140])
    upper_purple = np.array([170, 175, 185])
    
    if cv2.inRange(np.array([[pixel]]), lower_red, upper_red) == 255:
        return 'red'
    if cv2.inRange(np.array([[pixel]]), lower_purple, upper_purple) == 255:
        return 'purple'
    return 'other'

def find_the_car(color_img):
    L = np.shape(color_img)[1]
    # gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    gray_img=color_img
    tags = detector.detect(gray_img)
    car = []
    print("Nombre de tag: {}".format(len(tags)))
    
    for tag in tags:
        angle_degrees = 0
        car += [tag.tag_id] + [int(L - tag.center[0])] + [int(tag.center[1])] + [int(angle_degrees)]
        cv2.circle(color_img, [int(tag.center[0]), int(tag.center[1])], 5, (0, 0, 255), -1)
    
    ret, binary_img = cv2.threshold(gray_img, 50, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    treadmill = []
    Lobstacle = []
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        if area > 800:  
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            width, height = rect[1]
            
            if width > 200 and height > 200:
                cv2.drawContours(color_img, [box], 0, (255, 0, 0), 3)
                center = (int(rect[0][0]), int(rect[0][1]))
                angle = rect[2]
                
                if height < width:
                    angle -= 90
                
                treadmill = list(center) + [angle]
            
            elif 0.7 < width / height < 1.3 and 1000 < area < 2800:
                print(area)
                center = (int(rect[0][0]), int(rect[0][1]))
                radius = int(max(width, height) / 2) + 1
                cv2.circle(color_img, center, radius, (0, 0, 255), 3)
                Lobstacle.append(list(center) + [radius])
            
            elif 5000 > area > 3000:
                cv2.drawContours(color_img, [box], 0, (0, 255, 0), 3)
                center = (int(rect[0][0]), int(rect[0][1]))
                cv2.circle(color_img, center, 5, (0, 255, 0), -1)
                angle = rect[2]
                
                if height < width:
                    angle += 90
                
                for i in range(0, len(car), 4):
                    if (((L - car[i + 1]) - center[0]) ** 2 + (car[i + 2] - center[1]) ** 2) ** 0.5 < 25:
                        car[i + 3] = angle
                        line_length = 100
                        orthogonal_angle_rad = np.deg2rad(angle + 90)
                        x_end = int(center[0] + line_length * np.cos(orthogonal_angle_rad))
                        y_end = int(center[1] + line_length * np.sin(orthogonal_angle_rad))
                        x_start = int(center[0])
                        y_start = int(center[1])
                        cv2.line(color_img, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
    
    # show_image(color_img, "Rotated Rectangles, Center Points, and Orthogonal Lines")
    show_image(binary_img,'Image binaire')
    cv2.imwrite("IRbin.jpg", binary_img)
    end_time = time.time()
    processing_time = end_time - start_time
    print("Processing time: {:.6f} seconds".format(processing_time))
    
    return treadmill, car, Lobstacle

def initialize_window(title="Image"):
    global window
    window = True
    cv2.namedWindow(title)

def show_image(img, title="Image"):
    if not window:
        initialize_window(title)
    cv2.imshow(title, img)
    cv2.waitKey(1)

i = 0
while True:
    start_time = time.time()
    img = xiapi.Image()

    # Get a new image from the XIMEA camera
    cam.get_image(img)

    # Convert the image data to a format usable by OpenCV (assuming BGR format)
    img_data = img.get_image_data_numpy()

    cv2.imwrite("IR.jpg", img_data)
    # Process the image to find the car and other objects
    treadmill, car, Lobstacle = find_the_car(img_data)

    # Display the processed image with detections
    # show_image(img_data, "Processed Image")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    end_time = time.time()
    processing_time = end_time - start_time
    print("End time: {:.6f} seconds".format(processing_time))
    break

# Stop acquisition and release resources
cam.stop_acquisition()
cam.close_device()
cv2.destroyAllWindows()
