from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.set_xlabel('lat')
ax.set_xlim(0.393480,0.393430)
ax.set_ylabel('long')
ax.set_ylim(1.988933, 1.988983)
#ax.set_zlabel('alt')
#ax.set_zlim(0, 3)

lat = []
long = []
alt = []

f = open('result.txt', 'r')
for line in f:
    line = line.split(' ')
    lat.append(float(line[0]))
    long.append(float(line[1]))
    alt.append(float(line[2]))
    print (line)
    
ax.plot(lat, long, lw=2, label='UAV Path')    
plt.plot([0.393446],[1.988958],'o', label='UAV')  #出發點
plt.plot([0.393460],[1.988941],'o', label='TARGET')  #Target



#plt.plot([0.393454],[1.988964],'o', label='phone')  #手機

#plt.plot([0.393455],[1.988963],'o', label='DRSSI')  #RSSI 定位
#plt.plot([0.393455],[1.988964],'o', label='RSSI_N')  #RSSI(改) 定位
#plt.plot([0.393453],[1.988963],'o', label='Tril')  #Tri 定位

ax.legend()


plt.show()

#plt.savefig("filename.png",dpi=300,format="png")  #存成圖檔
