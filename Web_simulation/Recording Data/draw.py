import matplotlib.pyplot as plt
import numpy as np
import cmath
import sys

#   input format : python3 same_compare.py $1
#
#   $1 : fileName
#

fileName  = sys.argv[1]; #FileName

fig = plt.figure()
ax = fig.add_subplot(111)


if __name__ == "__main__":
    
    with open(fileName) as csvfile:
        ax.set_xlabel('Time (second)')
        ax.set_ylabel(csvfile.readline().split(",")[0]) #Y Ray Label Name

        path = []
        data = []
        for line in csvfile: #Check each Path's Value
            row = line.split("\n")[0].split(",")
            del(row[-1])
            path.append(row[0])
            data.append(row[1:])

    leng = np.arange(0, len(data[0]))

    # Each path, direction decision, or weight distribution method.
    # You can add a comment to hide the value.
    ax.plot(leng,data[2], lw=1, label='Dynamic Paht Planning')  #Linear Based, Region Based, Dynamic Path Planning
    ax.plot(leng,data[0], lw=1, label=path[0])  #2Leaf
    ax.plot(leng,data[1], lw=1, label=path[1])  #Half_Circle
    ax.plot(leng,data[3], lw=1, label=path[3])  #Linear(Region_Target)
    ax.plot(leng,data[4], lw=1, label=path[4])  #Random
    ax.plot(leng,data[5], lw=1, label=path[5])  #Log-Distance based
    ax.plot(leng,data[6], lw=1, label=path[6])  #Target-Based
    ax.plot(leng,data[7], lw=1, label=path[7])  #Target-Region_10m
    ax.plot(leng,data[8], lw=1, label=path[8])  #Triangulation

    ax.legend()
    plt.show()

