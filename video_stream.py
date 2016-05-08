
import io
import subprocess
import cv2
import time

f =io.open('video.h264','rb')



counter=0
temp = ""
while(True):
    red= f.read()
    w = io.open(str(counter)+'temp.h264','wb')
    w.write(red) 
    w.close()#-loglevel panic
    subprocess.call("ffmpeg -loglevel panic -y -i "+str(counter)+"temp.h264 im_0"+str(counter)+".mp4", shell=True)
    time.sleep(1)
    cap = cv2.VideoCapture("im_0"+str(counter)+".mp4")
    counter+=1
    ret, img = cap.read() 
    while(ret):
        cv2.imshow('frame',img) 
        ret, img = cap.read()
        
        
        if cv2.waitKey(1) and 0xFF == ord('q'):
            break

    
  
            
        
        
#
#import numpy as np
#import cv2 
#from cv2 import cv  
#cap = cv2.VideoCapture() 
#cap.set(cv.CV_CAP_PROP_FOURCC, cv.CV_FOURCC('h','2','6','4'))  
#cwi=cap.open('video.h264')  
#counter = 0  
#while(cap.isOpened()):      
#    ret, frame = cap.read()
#        counter += 1
#        if counter % 30 == 0:
#            cv2.imshow('frame', frame)  
#        if cv2.waitKey(1) and 0xFF == ord('q'):
#            break  
#cap.release() 
#cv2.destroyAllWindows()






