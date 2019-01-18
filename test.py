import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

#plt.plot([1,2,3,4])
#plt.savefig('test.png')

import time 
import datetime

t=time.time()
print(t)
print (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')) 
