import cv2
import numpy
import os.path
import numpy.matlib
from numpy.linalg import inv

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
while True:
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

    n = n + 1;
    
cv2.destroyAllWindows();

