import matplotlib.pyplot as plt
import numpy as np
import cmath
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('Distance error')
ax.set_ylabel('probability')

err = []
file = open('error_to_points.txt')
for line in file:
    err.append(line)
    #print (line)


array_length = len(err)
for i in range(array_length):
    err[i] = err[i].rstrip('\n')
    err[i] = float(err[i])


sorted_data = np.sort(err)

yvals=np.arange(len(sorted_data))/float(len(sorted_data)-1)

plt.plot(sorted_data,yvals)

plt.show()
