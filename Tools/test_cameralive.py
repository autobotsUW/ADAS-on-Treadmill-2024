# import cv2

# window=False

# def initialize_window(title="Image"):
#     global window
#     window=True
#     cv2.namedWindow(title)
    
# def show_image(img, title="Image"):
#     if not window:
#         initialize_window(title)
#     cv2.imshow(title, img)
#     cv2.waitKey(1)

# cap = cv2.VideoCapture(0)
# assert cap.isOpened()
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS,30)
# i=0

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
#     show_image(frame[48:465,:],'ADAS on a Treadmill')
#     cv2.imwrite("ADAS{}.jpg".format(i),frame[45:465,:])
#     i+=1
#     print(i)
#     if cv2.waitKey(1) & 0xFF == ord('q') or i>=10:
#         break
    
# cap.release()
# cv2.imshow('Zones', cap)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



from ximea import xiapi

# Initialize XIMEA camera
cam = xiapi.Camera()

try:
    # Open the camera device
    print('Opening camera...')
    cam.open_device()

    # Set camera parameters (exposure, resolution, etc.)
    cam.set_exposure(10000)  # Example: Set exposure to 10000 microseconds
    cam.set_width(640)       # Example: Set width to 640 pixels
    cam.set_height(480)      # Example: Set height to 480 pixels

    # Start acquisition
    print('Starting data acquisition...')
    cam.start_acquisition()

    i = 0

    while True:
        # Get a new image from the camera
        img = xiapi.Image()
        cam.get_image(img)

        # Convert image data to bytes
        data_raw = img.get_image_data_raw()

        # Transform data to list (if needed)
        # data = list(data_raw)

        # Example: Save the image to a file (adjust filename as needed)
        filename = f"ADAS{i}.jpg"
        with open(filename, 'wb') as f:
            f.write(data_raw)

        print(f"Saved image {filename}")

        i += 1

        # Exit loop after capturing 10 images
        if i >= 10:
            break

    # Stop acquisition
    print('Stopping acquisition...')
    cam.stop_acquisition()

finally:
    # Close the camera device
    cam.close_device()

print('Done.')

