import cv2

image = cv2.imread("road.jpg", 1)
cv2.imshow("ROAD", image)
k = cv2.waitKey()
