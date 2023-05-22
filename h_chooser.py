import cv2
import numpy as np

# Create a window
cv2.namedWindow('image')
cap = cv2.VideoCapture(0)

while(1):
    image = cap.read()[1]
    image = cv2.resize(image, (640, 480))
    image = cv2.flip(image, -1)

    lower = np.array([70, 42, 0])
    upper = np.array([139, 255, 255])

    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower, upper)

    # Find contours
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # get biggest contour
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
        # Print area of contour
        cv2.putText(image, str(h), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the original image
    cv2.imshow('image', image)
    cv2.waitKey(1)

    

cv2.destroyAllWindows()