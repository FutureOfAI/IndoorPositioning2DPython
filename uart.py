import serial 
import time 
import binascii 
import numpy as np

distance=[0]*10000
ser = serial.Serial("/dev/ttyAMA0", 57600,bytesize = 8,parity = 'N',stopbits = 1)
if ser.isOpen == False:
    ser.open() 
n=0
A16=0
A17=0
A18=0
A19=0
sum=0
#ser.write(b"DistanceOutON")

try:
    while True:
	ser.write("PostionOutON"+"\r"+"\n")
        #ser.write("DistanceOutON"+"\r"+"\n")

        size = ser.inWaiting() 
        if size != 0:
            response = ser.read(size) 
            print binascii.hexlify(response)
            #print binascii.hexlify(response[5]),binascii.hexlify(response[6])
            #A16=int(binascii.hexlify(response[13]),16)*256+int(binascii.hexlify(response[14]),16)
            #A17=int(binascii.hexlify(response[15]),16)*256+int(binascii.hexlify(response[16]),16)
            #A18=int(binascii.hexlify(response[17]),16)*256+int(binascii.hexlify(response[18]),16)
            #A19=int(binascii.hexlify(response[19]),16)*256+int(binascii.hexlify(response[20]),16)
            x=int(binascii.hexlify(response[1]),16)*256+int(binascii.hexlify(response[2]),16)
            y=int(binascii.hexlify(response[3]),16)*256+int(binascii.hexlify(response[4]),16)
            g=int(binascii.hexlify(response[6]),16)

	
	    #A16=float(A16)/1024*100
            #A17=float(A17)/1024*100
            #A18=float(A18)/1024*100
            #A19=float(A19)/1024*100
            x=float(x)/1024*100
            y=float(y)/1024*100
            #g=float(g)/1024*100
	   
	    print x , y , g
	    distance[n]=A18
	    sum=sum+distance[n]
	    if n==39:
	        #print sum/40
		sum=0
		n=0
	    #print a
	    #print distance[n]
	    #print n
            ser.flushInput() 
            n=n+1
        time.sleep(0.1) 
except KeyboardInterrupt:
    ser.close()

