import apriltag
import cv2
import time 
import numpy as np
start = time.time()
image = cv2.imread("test.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
tags = detector.detect(gray)

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
    angle_degrees = int(np.degrees(angle_radians))

    print(f"Tag ID: {tag.tag_id},Position: {tag.center}, Angle: {angle_degrees:.2f} degrees")   
    car+=[tag.tag_id]+[int(640-tag.center[0])]+[int(tag.center[1])]+[angle_degrees]
stop=time.time()
print(car)
print("Processing time {:.3f}".format(stop-start))
