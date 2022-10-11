import numpy as np
import matplotlib.pyplot as plt

f = open("src/logs_test_10_October.dat", "r").read()
data=f.split('\n')
data.pop(0)
n=len(data)
X=np.linspace(0,1,n)
Y=[]
for d in data:
    l=[]
    for k in d.split(','):
        l.append(float(k))
    Y.append(l)
Y=np.array(Y)
plt.plot(X,Y[:,1]/10)
plt.show()