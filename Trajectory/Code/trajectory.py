from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import math


##########################

#Follow Target1 or Target2
#Comment out Target1 if you want to show Target2, vice versa

##########################



fig = plt.figure()
ax = fig.gca(projection='3d')

toDegree = 180 / math.pi

ax.set_xlabel('lat')
ax.set_xlim(round(0.436484*toDegree, 4),round(0.436492*toDegree, 4))
ax.set_ylabel('long')
ax.set_ylim(round(2.1211985*toDegree, 4), round(2.121208*toDegree, 4))
#ax.set_zlabel('alt')
#ax.set_zlim(0, 3)

lat = []
long = []
alt = []

predictLat = []
predictLon = []
distError = []
time = []

#with open('target1.txt', 'r') as rfile:  #0.43648972424_2.12120737396 Target1
with open('target2.txt', 'r') as rfile: #0.43648975113_2.12119927893 Target2

    for line in rfile:
        line = line.split(' ')
        lat.append(float(line[0])*toDegree)
        long.append(float(line[1])*toDegree)
        alt.append(float(line[2])*toDegree)

        predictLat.append(float(line[5])*toDegree)
        predictLon.append(float(line[6])*toDegree)
        #distError.append(float(line[7])*toDegree)
        time.append(int(time[8]))
    
ax.plot(lat, long, lw=2, label='UAV Path') #Draw Flying Path   
plt.plot([0.43648689681*toDegree],[2.12120451162*toDegree],'o', label='UAV')  #出發點


#plt.plot([0.43648972424*toDegree],[2.12120737396*toDegree],'o', label='TARGET')  #Target1
plt.plot([0.43648975113*toDegree],[2.12119927893*toDegree],'o', label='TARGET')  #Target2


#Draw Predict Point
for i in range(len(predictLat)):
    if predictLon[i] != 0.0:
        plt.plot([predictLat[i]],[predictLon[i]],'yo')  #Target


ax.legend()
plt.show()
