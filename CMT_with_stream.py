#!/usr/bin/env python


import cv2
import argparse
import os.path
import time
import io
import subprocess
import CMT
import numpy
import util
import numpy.matlib
from numpy.linalg import inv

# Kalman Filter Setup

def kalman_predict(x,P,F,Q):
    x = F*x;        #predicted state
    P = F*P*F.transpose() + Q; #predicted estimate covariance
    res = [x, P];
    return res;

def kalman_update(x,P,z,H,R):
    y = z - H*x;    #measurement error/innovation
    S = H*P*H.transpose() + R; #measurement error/innovation covariance
    K = P*H.transpose()*inv(S); #optimal Kalman gain
    x = x + K*y; #updated state estimate
    P = (numpy.matlib.identity(x.shape[0]) - K*H)*P; #updated estimate covariance

    res = [x, P];
    return res;

# Set up CMT parsing/necessary for writing csv data

parser = argparse.ArgumentParser(description='Track an object.')
parser.add_argument('--output-dir', dest='output', help='Specify a directory for output data.')

args = parser.parse_args()

if args.output is not None:
    if not os.path.exists(args.output):
        os.mkdir(args.output)
    elif not os.path.isdir(args.output):
        raise Exception(args.output + ' exists, but is not a directory')

# define the filter

x = numpy.matlib.zeros((6,1));
F = numpy.matrix('1 0 0 0 1 0; 0 1 0 0 0 1; 0 0 1 0 1 0; 0 0 0 1 0 1; 0 0 0 0 1 0; 0 0 0 0 0 1');
Q = numpy.matrix('0.25 0 0 0 0.5 0; 0 0.25 0 0 0 0.5; 0 0 0.25 0 0.5 0; 0 0 0 0.25 0 0.5; 0.5 0 0.5 0 1 0; 0 0.5 0 0.5 0 1');
Q[:] = [a*0.01 for a in Q];
H = numpy.matrix('1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0');
R = numpy.matlib.identity(4);
R[:] = [b*42.25 for b in R];
P = numpy.matlib.identity(6);
P[:] = [c*10000 for c in P];

n = 1;

CMT = CMT.CMT()
f =io.open('video.h264','rb')
if(True):

    counter = 0

    #Here we read frames from f and write them to a temp file
    red = f.read()
    w = io.open(str(counter)+'temp.h264','wb')
    w.write(red) 
    w.close()#-loglevel panic
    
    #This converts the h264 ti mp4
    subprocess.call("ffmpeg -loglevel panic -y -i "+str(counter)+"temp.h264 vid_files/im_0"+str(counter)+".mp4", shell=True)

    # Is this necessary?
    time.sleep(2)
    print "here"
    cap = cv2.VideoCapture("vid_files/im_00.mp4")
    print "here"
    status, im0 = cap.read()
    im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
    im_draw = numpy.copy(im0)

    tl, br = util.get_rect(im_draw)

    print "t1: " + str(tl) + " br: " + str(br)

    CMT.initialise(im_gray0, tl, br)
    
    frame = 1
    
    #We always sleep between conversion and capture (do we need to?)
    counter+=1
    while True:
        # Read image
        status, im = cap.read()
        print "Status: " + str(status) + " of Frame #" + str(frame)
        
        #If we've run out of frames for that mp4...
        if not status or counter == 1:
            cap.release()
            red= f.read() #Get more frames!!
            w = io.open(str(counter)+'temp.h264','wb')
            w.write(red) 
            w.close()#-loglevel panic
            subprocess.call("ffmpeg -loglevel panic -y -i "+str(counter)+"temp.h264 im_0"+str(counter)+".mp4", shell=True)
            # Is this necessary?
            time.sleep(1)
            cap = cv2.VideoCapture("im_0"+str(counter)+".mp4")
            counter+=1
            status, im = cap.read() 
            
        im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im_draw = numpy.copy(im)

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

        # Draw output image

        cv2.imwrite('{0}/output_{1:08d}.png'.format(args.output, frame), im_draw)

        with open('{0}/bbox_{1:08d}.csv'.format(args.output, frame), 'w') as z:
            z.write('x y \n')
            numpy.savetxt(z, numpy.array((CMT.tl, CMT.tr, CMT.br, CMT.bl, CMT.tl)), fmt='%.2f')

        # Draw Kalman Filter here instead
        number = '%08d'%(n);
        filename = ('output/bbox_' + number + '.csv');
        if not os.path.isfile(filename):
            break;

        # read in the detected object's bounding box dimensions
        fid = open(filename);
        fid.readline(); #ignore the header
        detections = [];
        for i in fid:
            detections.append(i.strip().split(' '));
        fid.close();

        meas_x1 = float(detections[0][0]);
        meas_x2 = float(detections[2][0]);
        meas_y1 = float(detections[0][1]);
        meas_y2 = float(detections[2][1]);
        z = numpy.matrix([[meas_x1], [meas_x2], [meas_y1], [meas_y2]]);

        # step 1: predict
        result = kalman_predict(x,P,F,Q);
        x = result[0];
        P = result[1];

        # step 2: update (if measurement exists)
        if all(j>0 for j in z):
            result = kalman_update(x,P,z,H,R); 
            x = result[0];
            P = result[1];

        est_z = H*x;
        est_x1 = est_z[0];
        est_x2 = est_z[1];
        est_y1 = est_z[2];
        est_y2 = est_z[3];

        # draw a bounding box around the detected object
        imgname = 'output/output_' + number + '.png';
        img = cv2.imread(imgname, 1);

        if all(k > 0 for k in est_z) and (est_x2>est_x1) and (est_y2>est_y1):
            cv2.rectangle(img, (est_x1, est_y1), (est_x2, est_y2), (0,255,0), 2);

        cv2.imshow('KalmanFilter',img);
        cv2.waitKey(1);

        n += 1;


        #cv2.imshow('main', im_draw)

        # Check key input
        k = cv2.waitKey(1)
        key = chr(k & 255)
        if key == 'q':
            break
        
        # Remember image
        im_prev = im_gray

        # Advance frame number
        frame += 1

f.close()
