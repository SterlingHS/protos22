from tkinter import E
import cv2
import numpy as np

cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("Cannot open camera")
    exit()

while(True):
    # Capture frame-by-frame
    ret, frame = cam.read()

    # convert image to HSV
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow("gray", hsv_img)

    # Mask the colors we want to keep
    yellowlower_bound = np.array([20, 100, 100])
    yellowupper_bound = np.array([30, 255, 255])
    purplelower_bound = np.array([129, 50, 70])
    purpleupper_bound = np.array([158, 255, 255])

    yellow = yellowupper_bound, yellowlower_bound
    purple = purplelower_bound, purpleupper_bound

    binary_imgyellow = cv2.inRange(hsv_img, yellowlower_bound, yellowupper_bound)
    cv2.imshow('yellow', binary_imgyellow)

    binary_imgpurple = cv2.inRange(hsv_img, purplelower_bound, purpleupper_bound)
    cv2.imshow('purple', binary_imgpurple)

    #cv2.imshow('frame', frame)
    
    #   img = cv2.imread('circles.png', cv2.IMREAD_COLOR)
    # Convert to gray-scale
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image to reduce noise
    img_blur_yellow = cv2.medianBlur(binary_imgyellow, 3)
    img_blur_purple = cv2.medianBlur(binary_imgpurple, 3)
    # img_blur = cv2.GaussianBlur(binary_img, 7)
    # Apply hough transform on the image
    cv2.imshow('blur yellow', img_blur_yellow)
    

    contours, hierarchy = cv2.findContours(binary_imgyellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    output_img = np.copy(im)
    x_list = []
    y_list = []
    height, width, channel = im.shape
    
    for contour in contours:

        #Ignore small contours that could be because of noise-bad thresholding
        if cv2.contourArea(contour) < 1000:
            continue
        print(cv2.contourArea(contour))
        #cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = -1)

        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        center = tuple([int(dim) for dim in center]) # Convert to int so we can draw

        # Draw rectangle and circle
        cv2.drawContours(output_img, [cv2.boxPoints(rect).astype(int)], -1, color = (0, 0, 255), thickness = -1)

        x_list.append((center[0] - width / 2) / (width / 2))
        y_list.append((center[1] - width / 2) / (width / 2))

    print(x_list)
    cv2.imshow('Contour', output_img)
    #cv2.waitKey(0)
    input("Stop here.")
    cv2.destroyAllWindows()


    cv2.imshow('detected circles', frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    if cv2.waitKey (1) == ord('q'):
        break

    # Read image 
# img = cv2.imread('lanes.jpg', cv2.IMREAD_COLOR) # road.png is the filename
# Convert the image to gray-scale
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Find the edges in the image using canny detector
img_blur = cv2.medianBlur(binary_imgyellow, 3)
img_blur = cv2.medianBlur(binary_imgpurple, 3)
edges = cv2.Canny("gray", 50, 200, apertureSize = 3)
# Detect points that form a line
lines = cv2.HoughLinesP(edges, 1, np.pi/180, max_slider=3, minLineLength=10, maxLineGap=250)
# Draw lines on the image
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(binary_imgyellow, (x1, y1), (x2, y2), (0, 255, 255), 3)
# Show result
cv2.imshow("Result Image", binary_imgyellow)

input("Press any key to continue...")