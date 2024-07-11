#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import pandas as pd
import glob
import os

if (len(sys.argv) < 2):
	print ("Missing path to data")
	sys.exit(2)

data = []
filenames = []
for fname in sorted(glob.glob(sys.argv[1] + '*.csv')):
	print (fname)
	data.append( pd.read_csv(fname))
	# Get just the filenames into a list
	filenames.append(os.path.split(fname)[1])


ax = plt.gca()
for d in data:
	d["t_s"] = (d["timestamp_ns"] - d["timestamp_ns"].iloc[0]) / 1e9
	d.plot("t_s", ["t_ave"], ax=ax)
	# d.plot("t_s", ["t_hs"], ax=ax)
	# d.plot("t_s", ["i_d"], ax=ax)

plt.legend(filenames, loc='best')
plt.show()


