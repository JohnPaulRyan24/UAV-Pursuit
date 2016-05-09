# UAV Pursuit of a Moving Target

This is a project done for the Robotics Motion Planning class CS480-009 at NYU in Spring 2016.
By Jacqueline Abalo, Andrew Klingelhofer, and John Ryan

# Requirements

* Parrot Bebop Drone
* Node.js Bebop Drone Library: https://github.com/hybridgroup/node-bebop
* Python (tested on Macbook Pro mid-2012 with Python 2.7.10)
* OpenCV for Python
* CMT for Python: https://github.com/gnebehay/CMT
* Node.js
* ffmpeg

# Steps to Run

1. Download or clone Node.js Bebop Drone Library: https://github.com/hybridgroup/node-bebop
2. Download or clone CMT for Python: https://github.com/gnebehay/CMT
3. ```brew install opencv``` and ```brew install ffmpeg```
4. ```brew install node``` --- it should install node and npm (check with node -v and npm -v)
5. Download or clone this repository
6. Move contents of CMT to 'node-bebop' repo's directory 'node_modules/node-bebop/examples'
7. Move contents of this repository to same place as 5
8. Open two terminal windows in directory '{path_to_directory}/node_modules/node-bebop/examples'
9. In one terminal window, run ```node video.js``` (this will be a different file for the finished product)
10. In the second terminal window, run ```python CMT_with_stream.py```
11. A window should appear with the first image taken by the drone
12. Click once on the window to create initial bounding box point, then click again to create full bounding box
13. The program should run and you should see a blue and green box (blue = CMT, green = Kalman filter)

# Issues

We still have a few issues.

- ```OpenCV Error: Assertion failed (scn == 3 || scn == 4) in cvtColor``` - We believe this error comes when there isn't enough light on the target object
- This runs incredibly slowly, partly due to the fact that we have to continuously ```ffmpeg``` the .h264 files we receive from the drone. We haven't been able to figure out a way around this. After just a short time running ```python CMT_with_stream.py``` the video lags behind real life by up to a minute. We'll hopefully find a work around, but worst case, this still will show the drone does what we expect (even with a delay). 
