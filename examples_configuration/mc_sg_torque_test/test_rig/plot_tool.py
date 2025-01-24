#! /usr/bin/env python
import matplotlib.pyplot as plt
import sys
import pandas as pd

# Plot tool for plotting the output of a CSV as writted by joc_tool host oscilloscope

print ("data " + sys.argv[1])
data = pd.read_csv(sys.argv[1])

# Plot current against temperature
# p = data.plot("t", ["mc_0::i_q", "mc_0::t_ave"])

# Plot various things
# p = data.plot("t", ["mc_0::i_q", "sg_0::sg_an_0"])
# p = data.plot("t", "mc_0::i_q")
p = data.plot("t", "sg_0::sg_an_0")


plt.legend(loc='best')
plt.show()
