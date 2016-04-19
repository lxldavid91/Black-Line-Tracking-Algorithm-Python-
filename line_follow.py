
Xianglong Lu 16/4/14

# stands for detailed commends

#Hardware: Raspberry Pi 2/3 + Raspberry pi camera module

#The goals of this code are:
#1.turning the image from color to grey, extract the black line by adjusting the threshold,
#then we find the line. Finally we calculate the mass center of this line in camera's region of interest,
# Then calculate angle between the oriantation of the robot and mass center of black line.

#2. Feedback the thetae to arduino (through serial port) for the purpose of controlling the 
#robot's steering servo. In this case, robot can roughly track this black line.

#3. roughly calculate fps of the pi camera, which is super important


#import all the modules it needs. time and math modules have been installed already. 
#cv2 and numpy are ready when you installed opencv. Other modules are supposed to be installed 
#in python individually.

import cv2
from numpy import linalg as LA 
import numpy as np 
import io 
import picamera 
import serial 
import matplotlib.pyplot as plt 
import pylab as plab 
import time
import math

#Here we start the code
#In this case, raspberry pi and arduino uno are communicating through serial port (the blue 
    #USB cable). Here we define port name is ttyACM0 and baud rate is 9600 

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.write('0 \n')

# Here we are trying to get the image(vedio stream or just jpeg images). The following lines 
#help us get the images or vedio stream and store them in frame. This this the way to setup
# a pi camera 

def getImage():
        cap.capture(stream, format = 'jpeg', use_video_port = True)
        frame = np.fromstring(stream.getvalue(), dtype = np.uint8)
        stream.seek(0)
        frame = cv2.imdecode(frame,1)
        return frame

cap = picamera.PiCamera()

#flip the image horizontally or vertically if necessary

cap.vflip = True
cap.hflip = True
#resolution is set to 320x240 to get higher fps
cap.resolution = (320,240)


stream = io.BytesIO()

end = '\n'
comma = ','

#initialize angle thetae
#thetae is the angle between the oriantation of the robot and mass center of black line
#in region of interest

thetae_p = 0


#main function begins:

while(1):

    #start counting time(for fps calculating)
    start_time = time.time()

    frame = getImage()
    frame1 = np.array(frame)
    #roi mean the region of interest, we do not need the whole frame of what camera captures
    # we just need the area that we are interested in 

    roi = frame1[50:100,50:200]
    #first number is horizontal
    #second number is vertical
    
    # Convert BGR to GRAY
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Threshold the HSV image to get only blue colors
    ret, output2 = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    output2 = cv2.GaussianBlur(output2,(5,5),0)
    #roi = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    #output2 = cv2.inRange(roi,np.array((10,26,33)),np.array((10,26,35)))
    erode = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    dilate = cv2.getStructuringElement(cv2.MORPH_RECT,(6,6))
    
    # Erode and dilate 
    output2 = cv2.erode(output2, erode, iterations = 3)
    output2 = cv2.dilate(output2, dilate, iterations = 5)
        
    #output2 is the contour
    cv2.imshow('out', output2)    
    # Finding contours
    _,contours,_= cv2.findContours(output2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        
    

    #More Info: http://stackoverflow.com/questions/16538774/
    #ealing-with-contours-and-bounding-rectangle-in-opencv-2-4-python-2-7

    areas = [cv2.contourArea(c) for c in contours]
    
    #if not not areas:

        max_index = np.argmax(areas)
        cnt = contours[max_index]
    
        cv2.drawContours(roi, [cnt], 0, (0,0,255), 2)

        m1 = cv2.moments(contours[max_index])

        #To calculate the mass center of black line

        u1 = int(m1['m10']/m1['m00'])
        #u1 is mass center horizontal
        v1 = int(m1['m01']/m1['m00'])
        #v1 is mass center vertical

        #u1 v1 can be printed here(in python command line)
        str1 = "u1 is %d"%u1
        str2 = "v1 is %d"%v1
        #print str1
        #print str2
        
        
        # To calculate thetae every iteration
        thetae = int((math.atan2(u1-320,150))*180/3.1416)
        
        
        # update thetae
        thetae_p = thetae
       
               
        #print thetae
        print "The vision feedback theta is %d"%(thetae)
        
        
        
        sthetae = str(thetae)
        string = sthetae + end
        ser.write(string)

       

    else:
        ser.write('0 \n')
        print 'a'
        
    #show frame and roi windows in real time

    cv2.imshow('frame',frame)
    cv2.imshow('roi', roi)
    
    # It's a way to roughly calculate fps of the camera,using pi 3 and pi camera, it should 
    #be around 10Hz.

    print("%s Hertz"%(1/(time.time() - start_time)))
    

    #wait esc to kill all process
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

ser.write('0 \n')
ser.close()
cap.close()

