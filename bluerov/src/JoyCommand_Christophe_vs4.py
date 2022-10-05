#!/usr/bin/env python3
# Mon code python3



import os

import rospy
import numpy
import time
#import pygame
import time as t
from std_msgs.msg import String
from std_msgs.msg import Float64  
from std_msgs.msg import UInt32
from std_msgs.msg import UInt16 
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Header

from std_msgs.msg import Float32MultiArray

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
import mavros_msgs

import struct

from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure

import mavros
from mavros import command

# from class_video import Video
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

from scipy.spatial.transform import Rotation

Phi = 0
Psy = 0
Theta = 0
dPhi = 0
dPsy = 0
dTheta = 0

cap = 0
cap_d = 0
depUp = 1500 
pression = 0
altitude=0
altitude_d=1.5
stable_on=False
stable_cap_on = False
pressionair=1.00265
button = [0,0,0,0,0,0,0,0,0,0,0]  # A, B,Y,Z, LH , RH , back, start, ?, L3, R3
Jaxes = [0,0,0,0,0,0,0,0]   # (gauche gauche(1)/droite(-1), gauche haut(1)/bas(-1), ? , droite gauche(1)/droite(-1), droite haut(1)/bas(-1), ?, flèches gauche(1)/droite(-1), flèches haut(1)/bas(-1))
#channel = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
frame_id = 0


            #msg[0] = depUp  # pitch (eleve l'avant du rov)
            #msg[1] = depUp  # roll (rotation sur axe x, penche sur cote)
            #msg[2] = depUp # elevation
            #msg[3] = depUp  # yaw (rotation sur axe z)
            #msg[4] = depUp  # avant/arriere
            #msg[5] = depUp  # lateral gauche/droite
            
# Pitch 2    (peu etre considere comme elevation ?)
# Roll 3 
# Throttle 4 (marche avant)
# Yaw 5      (orientation, 1500 == maintien cap)
# Forward 6 
# Lateral 7   (ne pas utiliser : trajectoire nonlinéaire à cause de l'ajout du radar)
# Reserved 8  (rien ne se passe)

# https://discuss.bluerobotics.com/t/ros-support-for-bluerov2/1550/13



def callback_compass(data):
    global cap 
    cap = data.data

    
def callback_joy(data):  
    global button
    global Jaxes
    global frame_id
    button = data.buttons
    Jaxes = data.axes
    frame_id = data.header.seq
  
    
def callback_press(data):
    global pression
    pression = data.fluid_pressure/100000

    
def callback_alt(data):
    global altitude
    altitude = -data.data

        
altitudeprec=altitude     
  
  
def callback_IMU(data):
        global Phi  # Roll
        global Theta # pitch
        global Psy   # yaw
        global dPhi
        global dTheta
        global dPsy
        W = data.orientation.w
        X = data.orientation.x
        Y = data.orientation.y
        Z = data.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion de quaternions en matrice de rotation
        Phi, Theta, Psy = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") #Roulis, Tangage, Lacet 
        dPhi=data.angular_velocity.x
        dTheta=data.angular_velocity.y
        dPsy=data.angular_velocity.z
       
  
       	
def listener():

	 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
 
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, callback_compass)
    
    rospy.Subscriber("joy", Joy, callback_joy)
    
    # rospy.Subscriber("/mavros/imu/static_pressure", FluidPressure, callback_press)

    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, callback_alt)

    rospy.Subscriber("/mavros/imu/data", Imu, callback_IMU)
    
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
     
 
def ROV_movement(msg0):
    
    #global depUp


    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)       
    head = Header()
    channel = [1500]*18 # [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    channel[8] = 1000
   # channel[msg0[0]] = msg0[1]
    for i in range(9):
        channel[i] = msg0[i]
        if abs(channel[i])>2000:
            channel[i] = 2000*numpy.sign(channel[i]) 

    msg = OverrideRCIn()
    msg.channels = channel 
    #print(channel)
    


    pub.publish(msg)
#    rate = rospy.Rate(20) # 20hz  
#    rate.sleep()
 

def publisher_enregistrement(msg1):
    pub = rospy.Publisher('enregistrement_camera', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    msg11 = Float32MultiArray(data = msg1)
    pub.publish(msg11)




  
if __name__ == '__main__':


    Test1 = 0


    
  ##  rospy.init_node('MyProgram', anonymous=True)
    rospy.init_node('MyProgram', anonymous=True, disable_signals=True)
    
    print('Control ROV par Joystick. Pensez à armer.')
    
    val_pwm = 100

    
    vit = 0
    msg = [1500]*9
    test_arm = 0
    timeprec=time.time()
    frame_id_old = 0
    msg1 = [0]*2
    pwm_light = 1000

    adresse_ip = 'pi@192.168.2.2'
    password = 'companion'    
#    Affichage_video = 0
#    if Affichage_video == 1:
#        video = Video(5600)
    
    while not rospy.is_shutdown():
        

#    #### Afficher la camera
#            # Wait for the next frame
#        if Affichage_video == 1:
#            if not video.frame_available():
#                    continue
#
#            frame = video.frame()
#            cv2.imshow('frame3', frame)
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break


        
        # stabilisation cap : enregistrement du cap        
        if Test1 == 1:
           # t.sleep(0.25) # t.sleep(0.5)
            listener()
            cap_d = cap
            stable_cap_on = True  
            Test1 = 0
        else:
            listener()
            
                    
        #print(abs(pression-pressionair)*10)
        #print(pression)
      
      
#        print('profondeur = ', altitude)
                                      
        
        if frame_id_old != frame_id:
        
            frame_id_old = frame_id
            
            if (button[7] == 1)&(test_arm == 0):
            #    command.arming(True) 
                
                armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                armService(True)
                print('armed...') 
                test_arm = 1
            elif (button[7] == 1)&(test_arm == 1):
                test_arm = 0
            #    command.arming(False) 
                armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                armService(False)
                print('disarmed.')           
                           

                          
    #################################
    #button = [0,0,0,0,0,0,0,0,0,0,0]   A, B,X,Y, LB , RB , back, start, ?, L3, R3
    #Jaxes = [0,0,0,0,0,0,0,0]   # (gauche gauche(1)/droite(-1), gauche haut(1)/bas(-1), ? , droite gauche(1)/droite(-1), droite haut(1)/bas(-1), ?, flèches gauche(1)/droite(-1), flèches haut(1)/bas(-1))





            # inclinaison de la camera
            if Jaxes[2]<0:  
                msg[7] = 1700 
           #     print('bouton LT')
                           
            elif Jaxes[5]<0:
                msg[7] = 1300
           #     print('bouton RT')  
            else:
                msg[7] = 1500
                                
            
            # accelerer/revenir au minimum
            if button[3]!= 0:  # bouton Y : accelerer   
               val_pwm = val_pwm + 100
               print('pwm = ', val_pwm)

            if button[0]!= 0:  # bouton A : revenir au minimum   
               val_pwm = 100
               print('pwm = ', val_pwm)
               
                   
            # tourne gauche/droite
            if Jaxes[6] != 0:    # fleche droite/gauche
                rot=int(-val_pwm*Jaxes[6]+1500 )
                msg[5] = rot 
       #         if rot > 1500:
       #             print('-->')
       #         elif rot<1500:
       #             print('<--')
       #         print(rot)
            else:
                msg[5] = 1500 
                
            # rotation gauche/droite
            if Jaxes[0] != 0:    # joystick droite/gauche
                rot=int(-100*Jaxes[0]+1500 )
                msg[3] = rot 
            #    if rot > 1500:
            #        print('rot -->')
            #    elif rot<1500:
            #        print('rot <--')
            #    print(rot)
                stable_cap_on = False
              #  cap_d = cap
                
            else:
                msg[3] = 1500 
                if stable_cap_on == False:
                    Test1 = 1
                
                          
           # marche avant/arrière     
            if Jaxes[7] != 0:  # fleche haut/bas
                vit = int(val_pwm*Jaxes[7]+1500 )
                msg[4] = vit 
    #            if vit>1500:
    #                print('^')
    #                print('|') 
    #            elif vit<1500:
    #                print('|')
    #                print('v')  
    #            print('vit = ',vit)
            else:
                msg[4] = 1500
                       
              
            # monter/ descendre
            depUp = 1500
            if button[4] != 0:  # monter lentement
                depUp = int(val_pwm+1500 )

            if button[5] != 0: # descendre lentement
                depUp = int(-val_pwm+1500 )        

                
            if Jaxes[4] != 0:  # monter/descendre rapidement
                depUp = int(200*Jaxes[4]+1500 ) 

            
            msg[2] = depUp # elevation

                
            if (Jaxes[4] != 0)|(button[4] != 0)|(button[5] != 0):
                stable_on = False  #Désactivation de la régulation en profondeur
               # t.sleep(1)
            else:
                if stable_on == False:
                    altitude_obj = altitude
                    stable_on = True   #Activation de la régulation en profondeur
                   # t.sleep(1)
        
            


                       
        # stabilisation
        if stable_on:
       #     print('stabilisation profondeur a ', altitude_obj)
            if numpy.sign(altitude-altitude_obj) < 0:
                 depUp = int(200*(2/3.14159*numpy.arctan(0.2*(altitude-altitude_obj)+0.05*((altitude-altitudeprec)/(time.time()-timeprec))))+1500+numpy.sign(altitude-altitude_obj)*50)
            else:
                 depUp = int(200*(2/3.14159*numpy.arctan(2*(altitude-altitude_obj)))+1500+numpy.sign(altitude-altitude_obj)*50)
       #     print('Puissance : {}  altitude : {}  altitude désirée : {}'.format( depUp, altitude, altitude_obj))
            msg[2] = depUp 
            
        altitudeprec=altitude
        timeprec=time.time()
                
        # stabilisation du cap
        if stable_cap_on:
 #           print('stabilisation cap a ', int(cap_d))
 
            cap_d0 = cap_d
            if abs(cap_d - cap)>280   :  
                if cap_d > 180:
                    cap_d = cap_d - 360
                if cap > 180:
                    cap = cap - 360
            
            msg[3] = -int(numpy.arctan((cap - cap_d)/180*3.14)*150) + 1500
            cap_d = cap_d0
        

        Test77 = 0
        
        if Test77 == 1:
            print('stabilisation roll and pitch')
            ###### stabilisation en roulis/roll
            #print('Roll = ',Phi*180/3.14) 
            Phi0 = numpy.sign(Phi)*min([3.14/4, max([0,abs(Phi)-5*3.14/180])])          
            msg[1] = -int(1.1*numpy.arctan(Phi0)*val_pwm) + 1500

     
            ###### stabilisation en pitch
            #print('Pitch = ', Theta*180/3.14)
            Theta0 = numpy.sign(Theta)*min([3.14/4, max([0,abs(Theta)-5*3.14/180]) ])           
            msg[0] = int(numpy.arctan(Theta0)*150) + 1500 




        if (button[1]!=0):
                
            file2 = '/home/pi/lights/test.txt'
        
            # pour le bluerov 1
            pwm_light = pwm_light+100
            msg1[1] = 0 
            if (pwm_light > 1800):
            	msg1[1] = 2  # communication a l'affichage
            if (pwm_light > 1900):
                pwm_light = 1000
                msg1[1] = 0

            # on ecrit la valeur que l'on veut dans le fihcier test.txt      
            msg_lum = str(pwm_light) # 'test'
            file2 = '/home/pi/lights/pwm_light.txt'
            file3 = '/home/pi/pwm_light.txt'
#            cmd = 'sshpass -p '+password+' ssh '+ adresse_ip +' "echo '+ msg_lum +' > '+file2+'"'
#            os.system(cmd)
            cmd = 'sshpass -p '+password+' ssh '+ adresse_ip +' "echo '+ msg_lum +' > '+file3+'"'
            os.system(cmd)    

        
        # enregistrement
        if (button[6]!=0)&(msg1[0] == 0):
            msg1[0] = 1
            print('enregistrement: début')
        elif (button[6]!=0)&(msg1[0] == 1):
            msg1[0] = 0
            print('enregistrement: fin')
        publisher_enregistrement(msg1)
                    

            
            
        msg[8] = pwm_light


        
        ROV_movement(msg)
        
                
        # remettre à zero les inputs des boutons
        button = [0,0,0,0,0,0,0,0,0,0,0]
        Jaxes = [0,0,0,0,0,0,0,0]
        t.sleep(0.1)
        
        



