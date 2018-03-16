import matplotlib.pyplot as plt
import numpy as np
import cmath
import sys

fig = plt.figure()
ax = fig.add_subplot(111)

ax.set_xlabel('Time (second)')
ax.set_xlim(0,100)
ax.set_ylabel('Location Error(m)')
ax.set_ylim(0,45)

distError = []
time = []

distError2 = []
time2 = []

with open('target1.txt', 'r') as rfile:  #0.43648972424_2.12120737396 Target1
    for line in rfile:
        line = line.split(' ')
        distError.append(float(line[7]))
        time.append(int(line[8]))
        
with open('target2.txt', 'r') as rfile: #0.43648975113_2.12119927893 Target2

    for line in rfile:
        line = line.split(' ')
        distError2.append(float(line[7]))
        time2.append(int(line[8]))

ax.plot(time,distError, lw=1, label='Lat:25.0090189-Lon:121.53623') #Target1
#ax.plot(time2,distError2, lw=1, label='Lat:25.0090205-Lon:121.53576') #Target2


ax.legend()
plt.show()

