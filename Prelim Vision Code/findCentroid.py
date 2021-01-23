import cv2
import numpy as np

# read image through command line
img = cv2.imread("/home/seth/robotics_club/nrc-avc-2020/prelimVisionCode/rubix.jpg")
 
# convert the image to grayscale
gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
# convert the grayscale image to binary image
ret,thresh = cv2.threshold(gray_image,127,255,0)
 
# find contours in the binary image
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for c in contours:
   # calculate moments for each contour
	area = cv2.contourArea(c)

	if(area > 10000):
		(x,y),radius = cv2.minEnclosingCircle(c)
		center = (int(x),int(y))
		radius = int(radius)
		img = cv2.circle(img,center,radius,(0,255,0),2)
		img = cv2.putText(img, "centroid", (int(x) - 25, int(y) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
 
# display the image
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.imshow("image", img)
cv2.waitKey(0)

cv2.destroyAllWindows()

