import numpy as np
import matplotlib.pyplot as plt

f = open("logs/1/combined.dat", "r").read()
data=f.split('\n')
data.pop(0)
n=len(data)
X=np.linspace(0,1,n)
Y=[]
values=['northing', 'easting', 'depth', 'azimith', 'elevation', 'range', 'Local depth']
for d in data:
    l=[]
    for k in d.split(','):
        l.append(float(k))
    Y.append(l)

Y=np.array(Y)
logs={}
for i in range(len(values)):
    logs[values[i]]=Y[:,i]

print(logs['range'].shape)
plt.scatter(logs['northing'],logs['easting'])
plt.show()
