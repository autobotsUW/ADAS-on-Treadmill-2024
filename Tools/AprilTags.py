import apriltag
import cv2
image = cv2.imread("test.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
tags = detector.detect(gray)

for tag in tags:
    print(tag.tag_id)   