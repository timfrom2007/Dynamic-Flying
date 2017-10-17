import matplotlib.pyplot as plt
import numpy as np
import cmath
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('Distace')
ax.set_ylabel('RSSI')

g_rssi = []
m_rssi = []
k_rssi = []
rssi = []
i=0
file = open('rssi.txt')

for line in file:
    if(i==0):
        g_rssi = line.split(" ")
    if(i==1):
        m_rssi = line.split(" ")
    if(i==2):
        k_rssi = line.split(" ")
    if(i==3):
        rssi = line.split(" ")
    i += 1

le = len(rssi)
arr = np.arange(le)

ax.plot(arr,rssi[0:le], lw=2, label='RSSI')
ax.plot(arr,g_rssi[0:le], lw=1, label='Gauss')
ax.plot(arr,m_rssi[0:le], lw=1, label='Median')
ax.plot(arr,k_rssi[0:le], lw=1, label='Kalman') 

ax.legend()

plt.show()
