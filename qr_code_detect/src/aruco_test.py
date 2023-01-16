import numpy as np
import cv2
import cv2.aruco as aruco


cap = cv2.VideoCapture(-1)  # Get the camera sourcedef track(matrix_coefficients, distortion_coefficients):

if not cap.isOpened():
    raise IOError("Cannot open webcam")

#distortion_coefficients = np.array([[0.17440838, -0.29731616, -0.00603996, 0.00652948, -0.37835933]]) #np.array([[-2.80866188e-01,  6.67909837e+00,  1.95197392e-02, -7.43758521e-04, -4.23088890e+01]])
#matrix_coefficients = np.array([[664.3672707, 0., 330.49979219], [0., 664.00515129, 228.44945711], [0., 0., 1.]]) #np.array([[643.87316344, 0., 305.88396889], [0., 651.79367093, 282.07739857], [0., 0., 1.]])
distortion_coefficients = np.array([[2.33357717e-01, -1.35951296e+00,  1.50406527e-02,  7.74104380e-05, 2.05875036e+00]])
matrix_coefficients = np.array([[626.73695036, 0., 314.73551481], [0., 624.98104834, 247.11903925], [0., 0., 1.]])
while True:
    ret, frame = cap.read()
    # operations on the frame come here
    cv2.imshow('test', frame)
    cv2.waitKey(1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)  # Use 5x5 dictionary to find markers
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters# lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)
