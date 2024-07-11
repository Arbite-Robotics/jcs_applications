#! /usr/bin/env python
#
# Plot some data
#
import matplotlib.pyplot as plt
import pandas as pd
import math
import sys

def normalise_angle_2pi(angle):
    while (angle > 2.0*math.pi):
        angle -= 2.0*math.pi
    while (angle < 0.0):
        angle += 2.0*math.pi
    return angle


if (len(sys.argv) != 2):
    print ("Missing path to data")
    sys.exit(2)

print ("data " + sys.argv[1])
data_rotating = pd.read_csv(sys.argv[1])


data_rotating['timestamp'] = data_rotating['timestamp_ns'].div(1e9)

# Normalise host_th to range [0, 2pi]
data_rotating['host_th_m'] = data_rotating['host_th_m'].apply(normalise_angle_2pi)

# If required: invert the host signal
data_rotating['host_th_m'] = 2.0*math.pi - data_rotating['host_th_m']

print ("ave motor current: " + str(data_rotating['i_mot_abs'].mean()))
print ("ave bus current: " + str(data_rotating['i_bus'].mean()))

# data_fixed.plot('timestamp_ns', ['t_0', 't_1', 't_hs'])
data_rotating.plot('timestamp', ['t_0', 't_1', 't_hs'])

plt.show()