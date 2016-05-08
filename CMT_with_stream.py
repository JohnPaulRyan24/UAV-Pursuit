#!/usr/bin/env python


import cv2


import time
import io
import subprocess
import CMT
import numpy as np
import util


CMT = CMT.CMT()
f =io.open('video.h264','rb')
if(True):
    #Use some initial image for right now
    im0 = cv2.imread('init.png')
    im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
    im_draw = np.copy(im0)

    #Just some initial box coordinates
    tl = (450, 240)
    br = (730, 400)
    print 'using', tl, br, 'as init bb'

    
    CMT.initialise(im_gray0, tl, br)
    
    
    counter=0
    frame = 1
    
    #Here we read frames from f and write them to a temp file
    red= f.read()
    w = io.open(str(counter)+'temp.h264','wb')
    w.write(red) 
    w.close()#-loglevel panic
    
    
    #This converts the h264 ti mp4
    subprocess.call("ffmpeg -loglevel panic -y -i "+str(counter)+"temp.h264 im_0"+str(counter)+".mp4", shell=True)
    
    #We always sleep between conversion and capture (do we need to?)
    time.sleep(1)
    cap = cv2.VideoCapture("im_0"+str(counter)+".mp4")
    counter+=1
    while True:
        # Read image
        status, im = cap.read()
        
        #If we've run out of frames for that mp4...
        if not status:
            cap.release()
            red= f.read() #Get more frames!!
            w = io.open(str(counter)+'temp.h264','wb')
            w.write(red) 
            w.close()#-loglevel panic
            subprocess.call("ffmpeg -loglevel panic -y -i "+str(counter)+"temp.h264 im_0"+str(counter)+".mp4", shell=True)
            time.sleep(1)
            cap = cv2.VideoCapture("im_0"+str(counter)+".mp4")
            counter+=1
            status, im = cap.read() 
            
        im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im_draw = np.copy(im)

        tic = time.time()
        CMT.process_frame(im_gray)
        toc = time.time()

        # Display results

        # Draw updated estimate
        if CMT.has_result:

            cv2.line(im_draw, CMT.tl, CMT.tr, (255, 0, 0), 4)
            cv2.line(im_draw, CMT.tr, CMT.br, (255, 0, 0), 4)
            cv2.line(im_draw, CMT.br, CMT.bl, (255, 0, 0), 4)
            cv2.line(im_draw, CMT.bl, CMT.tl, (255, 0, 0), 4)

        util.draw_keypoints(CMT.tracked_keypoints, im_draw, (255, 255, 255))
        # this is from simplescale
        util.draw_keypoints(CMT.votes[:, :2], im_draw)  # blue
        util.draw_keypoints(CMT.outliers[:, :2], im_draw, (0, 0, 255))

        cv2.imshow('main', im_draw)

        # Check key input
        k = cv2.waitKey(1)
        key = chr(k & 255)
        if key == 'q':
            break
        
        # Remember image
        im_prev = im_gray

        # Advance frame number
        frame += 1

