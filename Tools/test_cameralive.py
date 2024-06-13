import cv2

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

cap = cv2.VideoCapture(0)
assert cap.isOpened()
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS,30)
i=0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    show_image(frame[48:465,:],'ADAS on a Treadmill')
    cv2.imwrite("ADAS{}.jpg".format(i),frame[45:465,:])
    i+=1
    print(i)
    if cv2.waitKey(1) & 0xFF == ord('q') or i>=10:
        break
    
cap.release()
cv2.imshow('Zones', cap)
cv2.waitKey(0)
cv2.destroyAllWindows()
