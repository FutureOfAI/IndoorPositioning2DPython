import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

#plt.plot([1,2,3,4])
#plt.savefig('test.png')

import time
import datetime

t=time.time()
#print(t)
#print (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')) 
import threading 
from time import sleep, ctime

def loop():
    
    global timer
    print("123")
    t = time.time()
    print (int(round(t * 1000)))

    timer = threading.Timer(1, loop)
    timer.start() 
    #time.sleep(1)
try:
    
    timer = threading.Timer(1, loop) 
    timer.start()

except:
    print "Error: unable to start thread"

