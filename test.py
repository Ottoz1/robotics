import cv2
import numpy as np

def find_countours(lower, upper, image):
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    binary_img = cv2.inRange(imgHSV, lower, upper)

    # Get contours and only keep the one with the largest area
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("No contours found between the given HSV color bounds")

    # Remove all contours except the largest one
    largest_contour_index = 0
    largest_area = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            largest_area = area
            largest_contour_index = i

    # Remove all other contours from the image
    for i in range(len(contours)):
        if i != largest_contour_index:
            cv2.drawContours(binary_img, contours, i, (0, 0, 0), -1)

    largest_contour = contours[largest_contour_index]
    return binary_img, largest_contour

def find_number_within_areaOfInterest(binary_image, aot):
    # find the contours of the cropped image
    contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the second largest contour
    largest_contour_index = 0
    largest_area = 0
    second_largest_area = 0
    second_largest_index = 0
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            second_largest_area = largest_area
            second_largest_index = largest_contour_index
            largest_area = area
            largest_contour_index = i
        elif area > second_largest_area:
            second_largest_area = area
            second_largest_index = i

    return contours[second_largest_index]

lower = np.array([90, 42, 0])
upper = np.array([171, 255, 255])
image = cv2.imread("img/4.jpg")

image = cv2.resize(image, (1280, 720))
binary_img, largest_contour = find_countours(lower, upper, image)

number_cont = find_number_within_areaOfInterest(binary_img, largest_contour)

# Draw the largest contour on the image
cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 3)
cv2.drawContours(image, [number_cont], -1, (0, 0, 255), 3)

# Draw a bounding box around the largest contour
x, y, w, h = cv2.boundingRect(largest_contour)
cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

print("Area of the number: ", cv2.contourArea(number_cont))
print("Width of the boudning box: ", w)

cv2.imshow("binary_img", image)
cv2.waitKey(0)

quit()

# Read a video feed
cap = cv2.VideoCapture("vid/2.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (1280, 720))
    binary_img, largest_contour = find_countours(lower, upper, frame)
    num_count = find_number_within_areaOfInterest(binary_img, largest_contour)

    # Draw the largest contour on the image
    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
    cv2.drawContours(frame, [num_count], -1, (0, 0, 255), 3)

    cv2.imshow("binary_img", frame)
    cv2.waitKey(10)

