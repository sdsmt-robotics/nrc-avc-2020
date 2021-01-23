import cv2
import numpy as np

#start video capture
cap = cv2.VideoCapture(0)

#code to ouput a video of whats going on
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))


#upper and lower values for the hsv color range
upper = 255
lower = 100

# set the bgr val of the color you are trying to select
#bgrVal = np.uint8([[[66,74,38 ]]]) #green
bgrVal = np.uint8([[[65,50,250 ]]]) #red
#bgrVal = np.uint8([[[170,110,50 ]]]) #blue

#convert the bgr val to hsv val
hsvVal = cv2.cvtColor(bgrVal,cv2.COLOR_BGR2HSV)
print ("hsvVal = ", hsvVal)

# define range of the color in HSV
hsvUPPER = np.array([hsvVal[0,0,0] + 15, upper, upper])
hsvLOWER = np.array([hsvVal[0,0,0] - 15, lower, lower])

print ("hsvUPPER = ", hsvUPPER)
print ("hsvLOWER = ", hsvLOWER)

while(1):
	
	# Take each frame
	_, frame = cap.read()

	# Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, hsvLOWER, hsvUPPER)
 
	# find contours in the binary image
	contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for c in contours:

		area = cv2.contourArea(c)

		if(area > 1000):
			(x,y),radius = cv2.minEnclosingCircle(c)
			center = (int(x),int(y))
			radius = int(radius)

			mask = cv2.circle(mask,center,radius,(0,255,0),2)
			mask = cv2.putText(mask, "centroid", (int(x) - 25, int(y) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			frame = cv2.circle(frame,center,radius,(0,255,0),2)
			frame = cv2.putText(frame, "centroid", (int(x) - 25, int(y) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

	#used for saving video
	#out.write(frame)
	cv2.imshow('mask',mask)
	cv2.imshow('frame',frame)

#######################################################################################################
## end of findCentroid.py code ########################################################################
#######################################################################################################

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cap.release()

#used for saving video
#out.release()

cv2.destroyAllWindows()

