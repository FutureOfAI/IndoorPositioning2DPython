"""
This python script is used to configure the DW1000 chip as an anchor for ranging functionalities. It must be used in conjunction with the RangingTAG script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


import DW1000
import monotonic
import DW1000Constants as C
#import imu
 

import RPi.GPIO as GPIO
import time
import numpy as np
import math
import os

#os.system("python ./kts/scripts/imu.py")

n_23=0
n_25=0
n_26=0
n_27=0


X_2D=np.array([0,0.9,0.17,11.8,7.78,11.7])
X_2D=X_2D.reshape([3,2])
Y_2D=(4.65,6.75)


DistanceFinish_Flag=0
Position_Flag=0
Same_tag_flag=0


lastActivity = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA =32 
data = [0] * LEN_DATA

LEN_TAG=4
tag=[0]*LEN_TAG

timePollAckSentTS = 0
timePollAckReceivedTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
REPLY_DELAY_TIME_US = 7000 
########################## TAG #######################
lastPoll = 0
expectedMsgID = C.POLL_ACK
POLL_RANGE_FREQ = 1000 # the distance between the tag and the anchor will be estimated every second.

TimePollSentTS = 0
TimeRangeSentTS = 0
TimePollAckReceivedTS = 0


def ResetInactive():
    """
    This function restarts the default polling operation when the device is deemed inactive.
    """
    global expectedMsgID
    print("Reset inactive")
    expectedMsgID = C.POLL_ACK
    transmitPoll()
    noteActivity()

def transmitPoll():
    """
    This function sends the polling message which is the first transaction to enable ranging functionalities.
    It checks if an anchor is operational.
    """
    global data, lastPoll
    while (millis() - lastPoll < POLL_RANGE_FREQ):
        pass
    DW1000.newTransmit()
    data[0] = C.POLL
    data[16] = 28
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()
    lastPoll = millis()


def transmitRange():
    """
    This function sends the range message containing the timestamps used to calculate the range between the devices.
    """
    global data, timeRangeSentTS
    DW1000.newTransmit()
    data[0] = C.RANGE
    timeRangeSentTS = DW1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
    DW1000.setTimeStamp(data, timePollSentTS, 1)
    DW1000.setTimeStamp(data, timePollAckReceivedTS, 6)
    DW1000.setTimeStamp(data, timeRangeSentTS, 11)
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()

#####################################################


########################   EKF   #########################
pi=3.14
r2d = (180/pi)
d2r = (pi/180)
g =9.8

def millis():
    """
    This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
    is used to avoid having the chip stuck in an undesirable state.
    """    
    return int(round(monotonic.monotonic() * C.MILLISECONDS))


def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sentAck variable as True so the loop can continue.
    """            
    global sentAck
    sentAck = True


def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received receivedAck as True so the loop can continue.
    """       
    global receivedAck
    receivedAck = True


def noteActivity():
    """
    This function records the time of the last activity so we can know if the device is inactive or not.
    """        
    global lastActivity
    lastActivity = millis()


def resetInactive():
    """
    This function restarts the default polling operation when the device is deemed inactive.
    """    
    global expectedMsgId
        
    expectedMsgId = C.POLL
    receiver()
    noteActivity()


def transmitPollAck():
    """
    This function sends the polling acknowledge message which is used to confirm the reception of the polling message. 
    """        
    global data
    DW1000.newTransmit()
    data[0] = C.POLL_ACK
    DW1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def transmitRangeAcknowledge():
    """
    This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
    """
    global data
    DW1000.newTransmit()
    data[0] = C.RANGE_REPORT
    
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def transmitRangeFailed():
    """
    This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
    """    
    global data
    DW1000.newTransmit()
    data[0] = C.RANGE_FAILED
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """    
    global data
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()


def computeRangeAsymmetric():
    """
    This is the function which calculates the timestamp used to determine the range between the devices.
    """
    global timeComputedRangeTS
    round1 = DW1000.wrapTimestamp(timePollAckReceivedTS - timePollSentTS)
    reply1 = DW1000.wrapTimestamp(timePollAckSentTS - timePollReceivedTS)
    round2 = DW1000.wrapTimestamp(timeRangeReceivedTS - timePollAckSentTS)
    reply2 = DW1000.wrapTimestamp(timeRangeSentTS - timePollAckReceivedTS)
    timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)



def spread_2D():


    ######Tag-------Anchor-------Distance#####
    r0=((X_2D[0,0]-Y_2D[0])**2 + (X_2D[0,1]-Y_2D[1])**2)**(0.5) #23
    r1=((X_2D[1,0]-Y_2D[0])**2 + (X_2D[1,1]-Y_2D[1])**2)**(0.5) #25
    r2=((X_2D[2,0]-Y_2D[0])**2 + (X_2D[2,1]-Y_2D[1])**2)**(0.5) #27

    print(r0, r1, r2)
    A=np.array([2*(X_2D[0,0]-X_2D[1,0]),2*(X_2D[0,1]-X_2D[1,1]),2*(X_2D[0,0]-X_2D[2,0]),2*(X_2D[0,1]-X_2D[2,1])])
    A=A.reshape([2,2])


    #b=np.array([r1**2-r0**2+X[0,0]**2-X[1,0]**2+X[0,1]**2-X[1,1]**2+X[0,2]**2-X[1,2]**2,r2**2-r0**2+X[0,0]**2-X[2,0]**2+X[0,1]**2-X[2,1]**2+X[0,2]**2-X[2,2]**2,r3**2-r0**2+X[0,0]**2-X[3,0]**2+X[0,1]**2-X[3,$
    #b=np.array([6.172**2-4.75**2+X[0,0]**2-X[1,0]**2+X[0,1]**2-X[1,1]**2+X[0,2]**2-X[1,2]**2,9.874**2-4.75**2+X[0,0]**2-X[2,0]**2+X[0,1]**2-X[2,1]**2+X[0,2]**2-X[2,2]**2,9.437**2-4.75**2+X[0,0]**2-X[3,0]**2$
    b=np.array([tag[1]**2-tag[0]**2+X_2D[0,0]**2-X_2D[1,0]**2+X_2D[0,1]**2-X_2D[1,1]**2,tag[3]**2-tag[0]**2+X_2D[0,0]**2-X_2D[2,0]**2+X_2D[0,1]**2-X_2D[2,1]**2])
    #print(b)
    B=np.linalg.inv(A)
    #print(B)
    Solution_Anc=B.dot(b)
    #print(Y)
    print("Real_Position:%.2f  %.2f  "%(Y_2D[0],Y_2D[1]))
    print("Position:%.2f  %.2f  "%(Solution_Anc[0],Solution_Anc[1]))





def loop():
    
    global sentAck,n_23,n_25,n_26,n_27, receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId,expectedMsgID, timeRangeSentTS,Same_tag_flag,DistanceFinish_Flag,Position_Flag

    if Position_Flag==0:

        if (sentAck == False and receivedAck == False):
            if ((millis() - lastActivity) > C.RESET_PERIOD):
                resetInactive()
       	    return
    	if sentAck:
            sentAck = False
            msgId = data[0]
            if          Same_tag_flag == data[16]:

                if msgId == C.POLL_ACK:
            	    timePollAckSentTS = DW1000.getTransmitTimestamp()
            	    noteActivity()

    	if receivedAck:
            receivedAck = False
            data = DW1000.getData(LEN_DATA)
            msgId = data[0]
            print(data[16])		      
        
            if msgId == C.POLL:
            	DistanceFinish_Flag =1
            	Same_tag_flag = data[16]
            	protocolFailed = False
            	timePollReceivedTS = DW1000.getReceiveTimestamp()
            	#print(timePollReceivedTS)
            	expectedMsgId = C.RANGE
            	transmitPollAck()
            	noteActivity()

            elif msgId == C.RANGE :
            	if (DistanceFinish_Flag == 1 and Same_tag_flag == data[16]):
                    DistanceFinish_Flag = 0

                    timeRangeReceivedTS = DW1000.getReceiveTimestamp()
                    expectedMsgId = C.POLL
            	    if protocolFailed == False:
                    	timePollSentTS = DW1000.getTimeStamp(data, 1)
                    	timePollAckReceivedTS = DW1000.getTimeStamp(data, 6)
                    	timeRangeSentTS = DW1000.getTimeStamp(data, 11)
                    	computeRangeAsymmetric()
                    	transmitRangeAcknowledge()
                    	distance = (timeComputedRangeTS % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
                    	#os.system("python ./DW1000RangingTAG.py")

                    
                    	if data[16]==23:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance1: %.2f m" %(distance))
                            n_23=n_23+1
                            tag[0]=((n_23-1)*tag[0]+distance)/n_23
                            print(n_23)
                    	if data[16]==25:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance2: %.2f m" %(distance))
                            n_25=n_25+1
                            tag[1]=((n_25-1)*tag[1]+distance)/n_25

                        if data[16]==26:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance3: %.2f m" %(distance))
                            n_26=n_26+1
                            tag[2]=((n_26-1)*tag[2]+distance)/n_26

                        if data[16]==27:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance4: %.2f m" %(distance))
                            n_27=n_27+1
                            tag[3]=((n_27-1)*tag[3]+distance)/n_27



                        if tag[0] !=0 and tag[1]!=0 and tag[2] !=0 and tag[3]!=0:
                            print(tag)
                            spread_2D()
                            #print("success")
                        
                if n_23 >=5 and n_25 >=5 and n_26 >=5 and n_27 >=5:
	            	#print("transmit TAG")

                    	#print(n_23)
	 	    	#print(timePollReceivedTS)
		    	#timePollReceivedTS = DW1000.getReceiveTimestamp()
                 #os.system("python ./DW1000RangingTAG.py")
		    #import DW1000RangingTAG
                    Position_Flag=1
		
                else:
                    transmitRangeFailed()

                noteActivity()
    if Position_Flag==1:
        #timePollSentTS =0
	#DW1000.registerCallback("handleSent", handleSent)

        if (sentAck == False and receivedAck == False):
            if ((millis() - lastActivity) > C.RESET_PERIOD):
                ResetInactive()
            return

    	if sentAck :

            sentAck = False

            msgID = data[0]
            #print(data[16],"sucess")
            if data[16]==28:
                if msgID == C.POLL :
                    #print("1")
           	
                    timePollSentTS = DW1000.getTransmitTimestamp()

		    #print(timePollSentTS)
                elif msgID == C.RANGE :
                    timeRangeSentTS = DW1000.getTransmitTimestamp()
                    noteActivity()


        if receivedAck:

            receivedAck = False
            data = DW1000.getData(LEN_DATA)
            msgID = data[0]
            if data[16]==28:
                print(data[16])

            	if (msgID == C.POLL_ACK):
                    timePollAckReceivedTS = DW1000.getReceiveTimestamp()
                    expectedMsgID = C.RANGE_REPORT
                    transmitRange()
                    noteActivity()
            	elif msgID == C.RANGE_REPORT:
                    expectedMsgID = C.POLL_ACK
                    transmitPoll()
                    noteActivity()
            	elif msgID == C.RANGE_FAILED:
                    expectedMsgID = C.POLL_ACK
                    transmitPoll()
                    noteActivity()
        time.sleep(0.1)



try:
    PIN_RST = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_RST,GPIO.IN)
    PIN_IRQ = 19
    PIN_SS = 27
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized")
    print("############### ANCHOR ##############")

    DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY) 
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
    

    receiver()
    noteActivity()
    #os.system("nano ./DW1000RangingTAG.py.save.2")
    #os.system("python ./DW1000RangingTAG.py")
    #print("success",fusionPose)
    #print(E.g)   
    while 1: 
        #os.system("python ./DW1000RangingTAG.py")

        loop()

except KeyboardInterrupt:
    DW1000.close()
