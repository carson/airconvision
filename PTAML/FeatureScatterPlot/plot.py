import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm 

if len(sys.argv) < 2:
    sys.stderr.write('Usage: sys.argv[0] \n')
    sys.exit(1)

#txt_files = glob.iglob("./*.txt")

data = np.loadtxt(sys.argv[1])
x = data[:,0]
y = data[:,1]

plt.scatter(x, y, alpha=0.05, cmap=cm.Paired)

#gridsize=50
#plt.hexbin(x, y, gridsize=gridsize, cmap=cm.jet, bins=None)
#plt.axis([x.min(), x.max(), y.min(), y.max()])

plt.show()

