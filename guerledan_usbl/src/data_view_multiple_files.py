import numpy as np
import matplotlib.pyplot as plt
import glob
import os
print(os.getcwd())


values=['northing', 'easting', 'depth', 'azimith', 'elevation', 'range', 'Local depth']
data_fusioned=[]
for filename in glob.glob('/home/remi/catkin_ws/src/Niche-sauvage/guerledan_usbl/logs/October_*.dat'):
    f = open(filename, "r").read()
    print(filename)
    data=f.split('\n')
    data.pop(0)
    n=len(data)
    converted_data=[]
    for d in data:
        l=[]
        dsplit=d.split(',')
        for k in dsplit:
            l.append(float(k))
        converted_data.append(l)
    data_fusioned=data_fusioned+converted_data
data_fusioned=np.array(data_fusioned)
logs={}
for i in range(len(values)):
    # print(i)
    logs[values[i]]=data_fusioned[:,i]

fig,ax=plt.subplots(figsize=(8,7))
xlim=np.array([min(logs['northing']),max(logs['northing'])])
ylim=np.array([min(logs['easting']),max(logs['easting'])])
# guerledan_lake_sat_img = plt.imread("/home/remi/catkin_ws/src/Niche-sauvage/guerledan_usbl/logs/Map2.jpg")

for i in range(len(logs['northing'])):
    if i%4==0:
        ax.clear()
        # ax.imshow(guerledan_lake_sat_img, extent=[*(xlim*16/9), *ylim])
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.plot(logs['northing'][:i],logs['easting'][:i],color='#f11a1a')
        plt.pause(1e-6)
plt.show()

'''
USBL: affichage vecteur entre usbl
cap robot
profondeur
'''
