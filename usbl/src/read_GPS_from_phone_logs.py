import numpy as np
import matplotlib.pyplot as plt
import glob
import pyproj

values=['systemNanoTime (in ns)','utcMilliTime (in ms)','latitude (in deg)','longitude (in deg)','altitude (in m)','speed (in m/s)','bearing (in deg)','azimuth (in deg)','pitch (in deg)','roll (in deg)','gyrx (in rad/s)','gyry (in rad/s)','gyrz (in rad/s)','accx (in m/s2)','accy (in m/s2)','accz (in m/s2)']
data_fusioned=[]
for filename in glob.glob('logs/1/GPS_phone.txt'):
    f = open(filename, "r").read()
    print(filename)
    data=f.split('\n')
    data.pop(0)
    n=len(data)
    converted_data=[]
    for d in data:
        l=[]
        dsplit=d.split(';')
        for k in dsplit:
            l.append(float(k))
        converted_data.append(l)
    data_fusioned=data_fusioned+converted_data
data_fusioned=np.array(data_fusioned)
data_fusioned=data_fusioned[(data_fusioned[:,2]!=0)]
data_fusioned=data_fusioned[(data_fusioned[:,3]!=0)]
data_fusioned[:,2]=data_fusioned[:,2]
data_fusioned[:,3]=data_fusioned[:,3]
logs={}

for i in range(len(values)):
    logs[values[i]]=data_fusioned[:,i]

# Define a projection
wgs84=pyproj.CRS("EPSG:4326") # LatLon with WGS84 datum used by GPS units and Google Earth 
lam93=pyproj.CRS("EPSG:2154") # Lambert93, commonly used in France

X,Y = pyproj.transform(wgs84, lam93, logs['longitude (in deg)'], logs['latitude (in deg)'])
X,Y=X-X[0],Y-Y[0]

fig,ax=plt.subplots(figsize=(8,7))
xlim=np.array([min(X),max(X)])
ylim=np.array([min(Y),max(Y)])
# guerledan_lake_sat_img = plt.imread("logs/Map2.jpg")

for i in range(len(X)):
    if i%100==0:
        ax.clear()
        # ax.imshow(guerledan_lake_sat_img, extent=[*(xlim*16/9), *ylim])
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        plt.plot(X[:i],Y[:i],color='#f11a1a')
        plt.pause(1e-6)
plt.show()
