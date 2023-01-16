import cv2
import numpy as np
import time

def detec_(image):

    try:
        img1 = image
        kernel = np.ones((5,5),np.uint8)
        ret, bw = cv2.threshold(img1, 127, 255, cv2.THRESH_BINARY)
        hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        # on effectue un masque avec les valeurs ci-dessous recuperee sur internet
        # pour ne garder que les lignes jaunes
        lower = np.array([30,0, 230], dtype=np.uint8)
        upper = np.array([255, 20, 255], dtype=np.uint8)
        seg0 = cv2.inRange(hsv, lower, upper)
        closing = cv2.morphologyEx(seg0, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        #dilation = cv2.dilate(opening,kernel,iterations = 1)
        # on affiche l'image
        #cv2.imshow('test', seg0)
        #cv2.waitKey(3)

        sum = 0  # la somme des positions en x des pixels blancs
        cnt = 0  # le nombre de pixels blancs
        return opening
    except:
        pass


def forme(image_non_traitee, bin):

    # Read image.
    #img = cv2.imread("/home/potinl/Documents/prj_nao/vs-ik-full-td-20221122-UE52-VS-IK-V-REP-V3_6_2-18_04/UE52-VS-IK/imgs/out_11212.ppm")
    img = image_non_traitee
    # Convert to grayscale.
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    for l in range(len(gray)):
        for c in range(len(gray[0])):
            if bin[l,c] == 0:
                gray[l,c] = 0
            else:
                gray[l,c] = 255
    # Apply Hough transform on the blurred image.
    #detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,param2 = 30, minRadius = 1, maxRadius = 40)
    detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,25,
                            param1=300,param2=0.8,minRadius=3,maxRadius=15)
    # Draw circles that are detected.
    if detected_circles is not None:

         # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        return detected_circles

cap = cv2.VideoCapture('video_expedition.mp4')
while (True):
    # stoquer l'image issue de la vidéo à l'instant t dans la variable "frame"
    ret, frame = cap.read()
    cv2.imshow('output', frame)
	

cap.release()
cv2.destroyAllWindows()