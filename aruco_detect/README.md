# **Aruco Detector Package** 

Give the pose and ID of one Aruco Qr Code 

## Install
You need to install opencv-python 4.6.0.66 and opencv-contrib-python 4.6.0.66 (cf install file)

In src/aruco_test.py line 80, you have to change the topic of the suscriber message image.

And line 115 and 116, you have to give your own Camera Matrix parameters.

To launch Aruco Detector :
``` bash
bash roslaunch aruco_detect simu.launch 

```

Topic published :

"/bluerov_camera_aruco" : publish the image with a box around the Aruco Marker

"/bluerov_pose_aruco" : publish the pose of the Marker (x,y,z translation and yaw,pitch,roll)

"/id_qr_code_aruco" : publish the ID of the Aruco Marker

"/is_qrcode" : publish a boolean if an Aruco marker is detect or not



