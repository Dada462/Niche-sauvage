# -*- coding: utf-8 -*-
#©  Hélène Thomas - novembre 2017 WTFPL

import cv2
   
i=1;
cap = cv2.VideoCapture(0) 
while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   
    cv2.imshow('frame',gray)
    key = cv2.waitKey(1) & 0xFF
    if  key == ord('q'):
        break
    if key == ord('s'):
        nom='calibration_camera/mire_'+str(i)+'.png'
        #XXX est à remplacer par un nom de répertoire valide !
        cv2.imwrite(nom,frame)
        i=i+1;
   
cap.release()
cv2.destroyAllWindows()



