import matplotlib.pyplot as plt
import numpy as np
import cmath
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('flightMove')
ax.set_ylabel('Error')

ax.set_xlim(1, 100)
ax.set_ylim(0, 300)

err = []
file = open('distance_error.txt')
for line in file:
    err.append(line)
    #print (line)


array_length = len(err)
for i in range(array_length):
    err[i] = err[i].rstrip('\n')
    err[i] = float(err[i])



ax.plot(err,lw=3)
plt.show()
