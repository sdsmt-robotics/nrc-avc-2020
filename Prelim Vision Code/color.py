import cv2
import numpy as np

cap = cv2.VideoCapture(0)

upper = 255
lower = 100

# set the bgr val of the color you are trying to select
#bgrVal = np.uint8([[[66,74,38 ]]])
bgrVal = np.uint8([[[65,50,215 ]]])
#bgrVal = np.uint8([[[170,110,50 ]]])

hsvVal = cv2.cvtColor(bgrVal,cv2.COLOR_BGR2HSV)
print ("hsvVal = ", hsvVal)

# define range of blue color in HSV
hsvUPPER = np.array([hsvVal[0,0,0] + 10, upper, upper])
hsvLOWER = np.array([hsvVal[0,0,0] - 10, lower, lower])

print ("hsvUPPER = ", hsvUPPER)
print ("hsvLOWER = ", hsvLOWER)

while(1):
	
	# Take each frame
	_, frame = cap.read()

	# Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, hsvLOWER, hsvUPPER)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)

	cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()

