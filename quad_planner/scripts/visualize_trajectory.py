#!/usr/bin/env python

import sys
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


filename = sys.argv[1]

data = np.loadtxt(filename)
fig = plt.figure()
ax = fig.gca(projection='3d')
cmap = plt.get_cmap('jet')
plt.hold('on')
for i in xrange(1, data.shape[0]):
    cmap_index = float(i) / data.shape[0]
    if i == 1:
        markersize = 16
    else:
        markersize = 6
    ax.plot(data[(i-1):(i+1),0], data[(i-1):(i+1),1], data[(i-1):(i+1),2], '.-', color=cmap(cmap_index), markersize=markersize)
plt.hold('off')
plt.show()
