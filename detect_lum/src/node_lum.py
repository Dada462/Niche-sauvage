#!/usr/bin/env python3
# # license removed for brevity
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import time
from bluerov_msgs.msg import CommandBluerov
from std_msgs.msg import Bool
from cv_bridge import CvBridge


def detec_(image):

    try:
        img1 = image
        kernel = np.ones((2,2),np.uint8)
        kernel2 = np.ones((3,3),np.uint8)
        #ret, bw = cv2.threshold(img1, 127, 255, cv2.THRESH_BINARY)
        hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        # on effectue un masque avec les valeurs ci-dessous recuperee sur internet
        # pour ne garder que les lignes jaunes
        lower = np.array([0,0, 220], dtype=np.uint8)
        upper = np.array([255, 20, 255], dtype=np.uint8)
        seg0 = cv2.inRange(hsv, lower, upper)
        closing = cv2.morphologyEx(seg0, cv2.MORPH_CLOSE, kernel)
        #opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        dilation = cv2.dilate(closing,kernel2,iterations = 1)
        # on affiche l'image
        #cv2.imshow('test', seg0)
        #cv2.waitKey(3)

        sum = 0  # la somme des positions en x des pixels blancs
        cnt = 0  # le nombre de pixels blancs
        return dilation
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
    detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1.2, 20, param1 = 250,param2 = 0.5, minRadius = 1, maxRadius = 10)
    #detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,25,
    #                        param1=300,param2=0.8,minRadius=3,maxRadius=15)
    #detected_circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,30,
    #                        param1=20,param2=0.8,minRadius=3,maxRadius=15)
    
    # Draw circles that are detected.
    if detected_circles is not None:

         # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        return detected_circles

def consigne_verticale(cercles, dim, nb_lum):
    consigne = 0 # Initialise la consigne à 0
    if nb_lum == 4: # Si il y a 4 cercles détectés
        cercle_gauche = 0 # Initialise le cercle le plus à gauche
        x_min = 100000000 # Initialise la variable x_min à une valeur très grande
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            if c[0] < x_min: # Si la coordonnée x du cercle est plus petite que x_min
                cercle_gauche = c # Ce cercle devient le cercle le plus à gauche
        consigne = (dim[1]/2) - cercle_gauche[1] # Calcule la consigne en comparant la coordonnée y du cercle le plus à gauche avec le milieu de l'image
        if consigne < 100: # Si la consigne est inférieure à 100
            consigne = 0 # La consigne est remise à 0
    elif nb_lum < 4: # Si il y a moins de 4 cercles détectés
        v_moy = 0 # Initialise la valeur moyenne à 0
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            v_moy += c[1] # Ajoute la coordonnée y du cercle à la valeur moyenne
        v_moy = v_moy/len(cercles) # Calcule la valeur moyenne en divisant la somme des coordonnées y par le nombre de cercles
        if abs(v_moy-(dim[1]/2)) > 50: # Si la différence entre la valeur moyenne et le milieu de l'image est supérieure à 50
            consigne = (dim[1]/2) - v_moy # Calcule la consigne en comparant la valeur moyenne avec le milieu de l'image
    return consigne # Retourne la consigne


def consigne_horizontale(cercles, dim, nb_lum):
    consigne = 0 # Initialise la consigne à 0
    if nb_lum == 4: # Si il y a 4 cercles détectés
        cercle_gauche = 0 # Initialise le cercle le plus à gauche
        x_min = 100000000 # Initialise la variable x_min à une valeur très grande
        pos_x_moy = 0 # Initialise la position x moyenne à 0
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            if c[0] < x_min: # Si la coordonnée x du cercle est plus petite que x_min
                cercle_gauche = c # Ce cercle devient le cercle le plus à gauche
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            if (c != cercle_gauche).any(): # Si le cercle est différent du cercle le plus à gauche
                pos_x_moy += c[0] # Ajoute la coordonnée x du cercle à la position x moyenne
        pos_x_moy = pos_x_moy/3 # Calcule la position x moyenne en divisant la somme des coordonnées x par 3
        consigne = (dim[0]/2) - pos_x_moy # Calcule la consigne en comparant la position x moyenne avec le milieu de l'image
        if consigne < 100: # Si la consigne est inférieure à 100
            consigne = 0 # La consigne est remise à 0
    elif nb_lum < 4: # Si il y a moins de 4 cercles détectés
        h_moy = 0 # Initialise la valeur moyenne à 0
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            h_moy += c[1] # Ajoute la coordonnée y du cercle à la valeur moyenne
        h_moy = h_moy/len(cercles) # Calcule la valeur moyenne en divisant la somme des coordonnées y par le nombre de cercles
        if abs(h_moy-(dim[0]/2)) > 50: # Si la différence entre la valeur moyenne et le milieu de l'image est supérieure à 50
            consigne = (dim[0]/2) - h_moy # Calcule la consigne en comparant la valeur moyenne avec le milieu de l'image
    return consigne # Retourne la consigne


def consigne_rotation(cercles, nb_lum):
    consigne = 0 # Initialise la consigne à 0
    if nb_lum == 4: # Si il y a 4 cercles détectés
        cercle_gauche = 0 # Initialise le cercle le plus à gauche
        cercles_hauts = [] # Initialise une liste pour les cercles les plus hauts
        x_min = 100000000 # Initialise la variable x_min à une valeur très grande
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            if c[0] < x_min: # Si la coordonnée x du cercle est plus petite que x_min
                cercle_gauche = c # Ce cercle devient le cercle le plus à gauche
        for c in cercles: # Pour chaque cercle dans la liste de cercles
            if (c != cercle_gauche).any(): # Si le cercle est différent du cercle le plus à gauche
                cercles_hauts.append(c) # Ajoute le cercle à la liste des cercles les plus hauts
        cercles_hauts = sorted(cercles_hauts, key=lambda item: (item[0], item[1])) # Trie les cercles les plus hauts en fonction de leur coordonnée x
        if cercles_hauts[0][1]-cercles_hauts[1][1] > 20 and cercles_hauts[1][1]-cercles_hauts[2][1] > 20:
            consigne = -100 # virage à gauche
        elif cercles_hauts[0][1]- cercles_hauts[1][1] <-20 and cercles_hauts[1][1]-cercles_hauts[2][1] < -20:
            consigne = 100 #virage à droite
    if 1 < nb_lum < 4:
        l_cercle = sorted(cercles, key=lambda item: (item[0], item[1]))
        diff = 0
        for i in range(len(l_cercle)-1):
            diff += l_cercle[i][1] - l_cercle[i+1][1]
        if abs(diff) > 50:
            if diff < 0:
                consigne = 50
            if diff > 0:
                consigne = -50
    return consigne

# cap = cv2.VideoCapture("detec_lum/img/expedition_niche_.mp4")

# i = 0
# echelle = 0.5

# if (cap.isOpened()== False): 
#     print("Error opening video stream or file")

# while(cap.isOpened()):
#     ret, frame = cap.read()
#     dim = [int(frame.shape[1]*echelle), int(frame.shape[0]*echelle)]
#     frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
#     new_image1 = detec_(frame)
#     number_of_white_pix = np.sum(new_image1 == 255)
#     if number_of_white_pix > 100:
#         t = time.time()
#         cercles1 = forme(frame, new_image1)
#         if cercles1 is not None:
#             for pt in cercles1[0]:
#                 a, b, r = pt[0], pt[1], pt[2]
#                 cv2.circle(frame, (a, b), r, (0, 0, 255), 2)
#                 nb_cercles = len(cercles1[0])
#                 cons_vert = consigne_verticale(cercles1[0], dim, nb_cercles)
#                 cons_hor = consigne_horizontale(cercles1[0], dim, nb_cercles)
#                 cons_rot = consigne_rotation(cercles1[0], nb_cercles)
#                 print("ver", cons_vert, "hor", cons_hor, "rot", cons_rot)
    
#     cv2.imshow('output', frame)
#     cv2.waitKey(2)

# #quiter le programme et fermer toutes les fenêtres ouvertes
# cap.release()
# cv2.destroyAllWindows()

command = CommandBluerov()
is_lights = Bool()

def image_callback(msg):

    global command
    global is_lights
    global msg_lum
    global iter
    iter +=1
    if iter %20 == 0:
        cons_vert = 0
        cons_hor = 0
        cons_rot = 0
        echelle = 0.8
        br = CvBridge()
        msg1 = br.imgmsg_to_cv2(msg)
        dim = [int(msg1.shape[1]*echelle), int(msg1.shape[0]*echelle)] # --only-pkg-with-deps
        msg1 = cv2.resize(msg1, dim, interpolation=cv2.INTER_AREA)
        new_image1 = detec_(msg1)
        #cv2.imshow('aaaa', new_image1)
        #cv2.waitKey(1)
        number_of_white_pix = np.sum(new_image1 == 255)
        if number_of_white_pix > 100:
            t = time.time()
            cercles1 = forme(msg1, new_image1)
            if cercles1 is not None:
                for pt in cercles1[0]:
                    a, b, r = pt[0], pt[1], pt[2]
                    cv2.circle(msg1, (a, b), r, (0, 0, 255), 2)
                    nb_cercles = len(cercles1[0])
                    cons_vert = consigne_verticale(cercles1[0], dim, nb_cercles)
                    cons_hor = consigne_horizontale(cercles1[0], dim, nb_cercles)
                    cons_rot = consigne_rotation(cercles1[0], nb_cercles)
                    print("ver", cons_vert, "hor", cons_hor, "rot", cons_rot)
                    if nb_cercles > 0 :
                        is_lights.data = True
                    else :
                        is_lights.data = False
        br2 = CvBridge()
        msg_lum = br2.cv2_to_imgmsg(msg1, "bgr8")
        #command.pose.position.y = cons_hor/4
        #command.pose.position.z = cons_vert/4
        #command.pose.orientation.z = cons_rot/4


def talker():
    rospy.init_node('detec_lum', anonymous=True)
    pub_command = rospy.Publisher('command_lights', CommandBluerov, queue_size=10)
    pub_is_lights = rospy.Publisher('is_lights', Bool, queue_size=10)
    pub_detection_lumiere = rospy.Publisher('visu_lum', Image, queue_size=10)
    rospy.Subscriber("bluerov_camera", Image, image_callback) 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        global command
        global msg_lum
        pub_command.publish(command)
        pub_is_lights.publish(is_lights)
        pub_detection_lumiere.publish(msg_lum)
        rate.sleep()


if __name__ == '__main__':
    msg_lum = Image()
    iter = 0
    try:
        talker()
    except rospy.ROSInterruptException:
        pass