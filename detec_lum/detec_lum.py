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
    detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1.5, 40, param1 = 250,param2 = 0.9, minRadius = 3, maxRadius = 10)
    #detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,25,
    #                        param1=300,param2=0.8,minRadius=3,maxRadius=15)
    #detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,30,
    #                        param1=20,param2=0.8,minRadius=3,maxRadius=15)
    
    # Draw circles that are detected.
    if detected_circles is not None:

         # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        return detected_circles

cap = cv2.VideoCapture('video_expedition.mp4')
i = 0
echelle = 0.5

while True:
    ret, frame = cap.read()
    dim = [int(frame.shape[1]*echelle), int(frame.shape[0]*echelle)]
    frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    new_image1 = detec_(frame)
    number_of_white_pix = np.sum(new_image1 == 255)
    if number_of_white_pix > 100:
        t = time.time()
        cercles1 = forme(frame, new_image1)
        if cercles1 is not None:
            for pt in cercles1[0]:
                a, b, r = pt[0], pt[1], pt[2]
                cv2.circle(frame, (a, b), r, (0, 0, 255), 2)
    #cv2.imshow('output', frame)
    cv2.imshow('output', frame)
    #if cercles1 is not None:
    #    print(time.time())
    #    cv2.waitKey(1000)
    #else:
    cv2.waitKey(1)

	

#quiter le programme et fermer toutes les fenêtres ouvertes
cap.release()
cv2.destroyAllWindows()

"""
image1 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere7.png")
new_image1 = detec_(image1)
cercles1 = forme(image1, new_image1)
blank = np.zeros((new_image1.shape), np.uint8)
print(cercles1)
try:
    for pt in cercles1[0]:
        #print("aaaa")
        a, b, r = pt[0], pt[1], pt[2]
        #print(a,b,r)
        cv2.circle(blank, (a, b), r, (255, 0, 0), 2)
        cv2.circle(image1, (a, b), r, (0, 0, 255), 2)
except:
    pass
image2 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere2.png")
new_image2 = detec_(image2)
cercles2 = forme(image2, new_image2)
print(cercles2)
try:
    for pt in cercles2[0]:
        a, b, r = pt[0], pt[1], pt[2]
        cv2.circle(image2, (a, b), r, (0, 0, 255), 2)
except:
    pass
image3 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere3.png")
new_image3 = detec_(image3)
cercles3 = forme(image3, new_image3)
try:
    for pt in cercles3[0]:
        a, b, r = pt[0], pt[1], pt[2]
        cv2.circle(image3, (a, b), r, (0, 0, 255), 2)
except:
    pass
image4 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere4.png")
new_image4 = detec_(image4)
cercles4 = forme(image4, new_image4)
try:
    for pt in cercles4[0]:
        a, b, r = pt[0], pt[1], pt[2]
        cv2.circle(image4, (a, b), r, (0, 0, 255), 2)
except:
    pass
image5 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere5.png")
new_image5 = detec_(image5)
cercles5 = forme(image5, new_image5)
try:
    for pt in cercles5[0]:
        a, b, r = pt[0], pt[1], pt[2]
        cv2.circle(image5, (a, b), r, (0, 0, 255), 2)
except:
    pass
image6 = cv2.imread("/home/potinl/PycharmProjects/Niche-sauvage/detec_lum/img/lumiere6.png")
new_image6 = detec_(image6)
cercles6 = forme(image6, new_image6)
try:
    for pt in cercles6[0]:
        a, b, r = pt[0], pt[1], pt[2]
        cv2.circle(image6, (a, b), r, (0, 0, 255), 2)
except:
    pass

cv2.imshow("lumiere1", image1)
cv2.imshow("lumiere2", image2)
cv2.imshow("lumiere3", image3)
cv2.imshow("lumiere4", image4)
cv2.imshow("lumiere5", image5)
cv2.imshow("lumiere6", image6)

cv2.imshow("binaire_ lumiere1", new_image1)
#cv2.imshow("binaire_ lumiere2", new_image2)
#cv2.imshow("binaire_ lumiere3", new_image3)
#cv2.imshow("binaire_ lumiere4", new_image4)
#cv2.imshow("binaire_ lumiere5", new_image5)
#cv2.imshow("binaire_ lumiere6", new_image6)

cv2.waitKey(0)
cv2.destroyAllWindows()


# reste à faire: 
# estimation position par rapport aux lumières
# utilisation de la direction des 3 lumières ? pour savoir la position du robot sur un plan horizontal
# droite montant de gauche vers droite, on est à gauche, droite descendant de gauche vers droite, on est à droite
# utilisation de la 4e lumière pour bien se positionner en hauteur
# limitation du nombre de lumière (lumière saturante)
# 
"""