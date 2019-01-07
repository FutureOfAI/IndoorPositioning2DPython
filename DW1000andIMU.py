import DW1000
import monotonic
import DW1000Constants as C
import RPi.GPIO as GPIO
import time
import numpy as np
import math
import os
import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import operator
import socket
import threading
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import pylab

n_ekf=0
n_ekf_start=0
##### distance count of each Tag #####
n_23=0
n_25=0
n_26=0
n_27=0
######################################

###### 2D Tag position  #####
X_2D=np.array([0,0.9,0.17,11.8,7.78,11.7]).reshape([3,2])
Y_2D=(4.65,6.75)
#########################

###### 3D Tag position  #####
X=np.array([0,0.9,2.75,0.17,11.8,2.72,7.78,11.7,2.5,7.65,0.13,2.7]).reshape([4,3])
Y=(0,5.64,2.73)

##### Flag #####
DistanceFinish_Flag=0
Position_Flag=0
Same_tag_flag=0
###############

###### Anchor variable #####
lastActivity = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA =25 
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
##########################

######## TAG #########
lastPoll = 0
expectedMsgID = C.POLL_ACK
POLL_RANGE_FREQ = 1000 # the distance between the tag and the anchor will be estimated every second.
TimePollSentTS = 0
TimeRangeSentTS = 0
TimePollAckReceivedTS = 0
######################

###### imu ######
IMU_IP = "127.0.0.2"
IMU_PORT = 5005

MON_IP = "127.0.0.5"
MON_PORT = 5005

SETTINGS_FILE = "RTIMULib"

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

# offsets
yawoff = 0.0
pitchoff = 0.0
rolloff = 0.0

# timers

t_print = time.time()
t_damp = time.time()
t_fail = time.time()
t_fail_timer = 0.0
t_shutdown = 0


if (not imu.IMUInit()):
    hack = time.time()
    imu_sentence = "$IIXDR,IMU_FAILED_TO_INITIALIZE*7C"
    if (hack - t_print) > 1.0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(imu_sentence, (IMU_IP, IMU_PORT))
        t_print = hack
        t_shutdown += 1
        if t_shutdown > 9:
            sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()

f = open('mag', 'r')
magnetic_deviation = float(f.readline())
f.close()
###################################

####### EKF #######

r2d = (180/math.pi)
d2r = (math.pi/180)
threshold_c =0.55
g =9.8
scale_factor_err = -0.0512
bx0=1*0.1*g
by0=-0.1*g
bz0=-4.97*d2r
xverr = 0.1
yverr = -0.1
xperr = 0.5
yperr = -0.5
xaerr = 0
yaerr = 0
psierr = 1
err_factor =1
radiosensor_err_factor = 1.0
sig_x_r=radiosensor_err_factor*0.1             
sig_y_r=radiosensor_err_factor*0.1
sig_xr=err_factor*0.1*9.8/3600
sig_yr=err_factor*0.1*9.8/3600
sig_bx=err_factor*0.1*9.8
sig_by=err_factor*0.1*9.8
sig_arw_0 = 1*0.02                 
sig_rrw_0 = 0.02/3600
psi_true=0
psi=[0]*5
Imu=[0]*5
dwm=[0]*5
state = 1
select = 1
select_d = 1
u = 0
a=[0]*3
d=[0]*4
d_1=[0]*4
a_1=[0]*3
axm_h_1=0
aym_h_1=0
wzm_h_1=0
psi_h_1=0
bx_h_1=0
by_h_1=0
bz_h_1=0
xvm_Nh_1=0
yvm_Nh_1=0
xpm_Nh_1=0
ypm_Nh_1=0
xam_Nh_1=0
yam_Nh_1=0
xvm_Nh = 0
yvm_Nh = 0
xam_Nh = 0
yam_Nh = 0
psi_h = 0
bx_h = 0
by_h = 0
bz_h = 0
axm_h = 0
aym_h = 0
wzm_h = 0
R1m_h = 0
R2m_h = 0
R3m_h = 0
R4m_h = 0
xpm_Nh = 0
ypm_Nh = 0
xpm_Nh_2 = 0
ypm_Nh_2 = 0
zpm_Nh_2 = 0
end_count = 0
dt =0.005
EKF_Solution_Anc=[0]*2

def spread_2D():

    global Solution_Anc,tag
    ######Tag-------Anchor-------Distance#####
    r0=((X_2D[0,0]-Y_2D[0])**2 + (X_2D[0,1]-Y_2D[1])**2)**(0.5) #23
    r1=((X_2D[1,0]-Y_2D[0])**2 + (X_2D[1,1]-Y_2D[1])**2)**(0.5) #25
    r2=((X_2D[2,0]-Y_2D[0])**2 + (X_2D[2,1]-Y_2D[1])**2)**(0.5) #27

    #print(r0, r1, r2)
    A=np.array([2*(X_2D[0,0]-X_2D[1,0]),2*(X_2D[0,1]-X_2D[1,1]),2*(X_2D[0,0]-X_2D[2,0]),2*(X_2D[0,1]-X_2D[2,1])])
    A=A.reshape([2,2])

    #b=np.array([r1**2-r0**2+X[0,0]**2-X[1,0]**2+X[0,1]**2-X[1,1]**2+X[0,2]**2-X[1,2]**2,r2**2-r0**2+X[0,0]**2-X[2,0]**2+X[0,1]**2-X[2,1]**2+X[0,2]**2-X[2,2]**2,r3**2-r0**2+X[0,0]**2-X[3,0]**2+X[0,1]**2-X[3,$
    #b=np.array([6.172**2-4.75**2+X[0,0]**2-X[1,0]**2+X[0,1]**2-X[1,1]**2+X[0,2]**2-X[1,2]**2,9.874**2-4.75**2+X[0,0]**2-X[2,0]**2+X[0,1]**2-X[2,1]**2+X[0,2]**2-X[2,2]**2,9.437**2-4.75**2+X[0,0]**2-X[3,0]**2$
    b=np.array([tag[1]**2-tag[0]**2+X_2D[0,0]**2-X_2D[1,0]**2+X_2D[0,1]**2-X_2D[1,1]**2,tag[3]**2-tag[0]**2+X_2D[0,0]**2-X_2D[2,0]**2+X_2D[0,1]**2-X_2D[2,1]**2])

    B=np.linalg.inv(A)

    Solution_Anc=B.dot(b)


    #print("Real_Position:%.2f  %.2f  "%(Y_2D[0],Y_2D[1]))
    print("2D_Position:%.2f  %.2f  "%(Solution_Anc[0],Solution_Anc[1]))

def EKF_start():
    t1=threading.Thread(target=EKF_message)     
    #t2=threading.Thread(target=PSI_message) 
    t1.start()
    #t2.start()

def EKF_message():
    global P00_z,tag,fusionPose
    EKF_New()
    
    Imu[0]=-fusionPose[0]
    Imu[1]=-fusionPose[1]
    Imu[2]=-fusionPose[2]*d2r

        
    dwm[0]=tag[0]
    dwm[1]=tag[1]
    dwm[2]=tag[3]
    dwm[3]=tag[2]
    EKF_Update()
    time.sleep(0.01)

def PSI_message():
    
    if state ==2:
        if psi_h*180/3.14 <0:
            psi_true=-(psi_h*180/3.14)
            psi_true=(psi_true%360)
        else:
            psi_true = (psi_h*(180/3.14))
            psi_true =-(psi_true%360)
    psi[1]=psi_true
    ##$$$$$$$$$$$$$$$$$##
    time.sleep(0.1)
   



def EKF_New():
    global P00_z,R,F_z,Q_z,Xz_h
    P00_z = np.zeros([8,8])
    R=np.zeros([4,4])
    H=np.zeros([4,8])
    Xz_h=np.zeros([8,1])
    tmp=np.zeros([8,8])
    tmp_1=np.zeros([8,8])
    tmpYX=np.zeros([1,8])
##### covariance matrix(P00_z=phi_z*P00_z*(phi_z')+Q_z*dt) #####
    P00_z[0,0] = xperr**2
    P00_z[2,2] = 1*bx0**2
    P00_z[3,3] = yperr**2
    P00_z[5,5] = 1*by0**2
    P00_z[6,6] = (1*psierr*d2r)**2
    P00_z[7,7] =100* bz0**2

##### Xz_h(xz_h=phi_z*xz_h) #####
    Xz_h[7,0] = bz0
    
##### Q matrix -predict(P00_z=phi_z*P00_z*(phi_z')+Q_z*dt) #####
    Q_z=np.zeros([8,8])   
##### R matrix -measurement(H*P00_z*H'+R) #####
    R=np.array([sig_x_r**2,0,0,0,0,sig_y_r**2,0,0,0,0,sig_x_r**2,0,0,0,0,sig_y_r**2])*100
    R=R.reshape([4,4])
##### Kalman Filter Gain(K_z = P00_z*H'/(H*P00_z*H'+R)) #####
    K_z=np.zeros([8,4]) 
##### Measurement covariance update #####
    S=np.zeros([4,4]) 
##### F matrix(P = (I+F*dt)*P*(I+F*dt)' + Q) #####
    F_z=np.zeros([8,8]) 
    H_D=np.zeros([4,8])

    

def EKF_Update():
    ##### IMU_MPU9250 #####
    global n_ekf,dwm,Imu,state,xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,xam_Nh,yam_Nh,wzm_h,bx_h,by_h,bz_h,psi_h,end_count,P00_z,tmp,tmp_1,tmp_YX,u,select,count1,select_d,EKF_Solution_Anc,R,Xz_h,F_z,Q_z,H_D,bx_h_1,by_h_1,bz_h_1,psi_h_1,xpm_Nh_1,ypm_Nh_1,xvm_Nh_1,yvm_Nh_1,xam_Nh_1,yam_Nh_1
    a[0]=0 #acc_x
    a[1]=0 #acc_y
    a[2]=0 #gyro_z
    
    d[0]=dwm[0] #distance1
    d[1]=dwm[1] #distance2
    d[2]=dwm[2] #distance3
    d[3]=dwm[3] #distance4
    #print("aaaaaa",a)    
    #print("d",d)
    for i in range(0,4):
        if d[i]>150000:
            d[i]=d_1[i]
    spread_2D()
    xpm_Nh_2=Solution_Anc[0]
    ypm_Nh_2=Solution_Anc[1]
    
    if state == 1:

        bz_h = bz0
	       
        if d[0]>0 and d[1]>0 and d[2]>0 and d[3]>0 :

            xpm_Nh = xpm_Nh_2
            ypm_Nh = ypm_Nh_2
            xpm_Nh_1 = xpm_Nh
            ypm_Nh_1 = ypm_Nh
	    #print("1111111111111",xpm_Nh,ypm_Nh)
            state = 2
	    #print (xpm_Nh,ypm_Nh)
    if state == 2:
	#print("2222222")
	for j in range(0,2):
            bx_h = bx_h_1
            by_h = by_h_1
            bz_h = bz_h_1
            axm_h = a[0]-bx_h  #acc_x
            aym_h = a[1]-by_h  #acc_y
            axm_h_1 = a_1[0]- bx_h_1
            aym_h_1 = a_1[1]- by_h_1
            wzm_h_1 = (1-scale_factor_err)*(a_1[2] - bz_h_1)
            wzm_h = (1-scale_factor_err)*(a[2] - bz_h_1)
            psi_h = psi_h_1 +(wzm_h_1+wzm_h)*dt/2.0
            xam_Nh = np.cos(psi_h_1)*axm_h_1 - np.sin(psi_h_1)*aym_h_1 - wzm_h_1*yvm_Nh_1
            yam_Nh = np.sin(psi_h_1)*axm_h_1 + np.cos(psi_h_1)*aym_h_1 + wzm_h_1*xvm_Nh_1
            xvm_Nh = xvm_Nh_1 + (xam_Nh+xam_Nh_1)*dt/2
            yvm_Nh = yvm_Nh_1 + (yam_Nh+yam_Nh_1)*dt/2
            xpm_Nh = xpm_Nh_1 + (xvm_Nh+xvm_Nh_1)*dt/2
            ypm_Nh = ypm_Nh_1 + (yvm_Nh+yvm_Nh_1)*dt/2
            #print("test",xpm_Nh,ypm_Nh,xvm_Nh,xvm_Nh_1)   


            F_z[0,1] = 1
            F_z[1,2] = -np.cos(psi_h_1)
            F_z[1,4] = -wzm_h_1
            F_z[1,5] = np.sin(psi_h_1)
            F_z[1,6] = 0*(-np.sin(psi_h_1)*(axm_h_1)-np.cos(psi_h_1)*(aym_h_1))
            F_z[1,7] = 0*(yvm_Nh_1)
            F_z[3,4] = 1
            F_z[4,1] = wzm_h_1
            F_z[4,2] = -np.sin(psi_h_1)
            F_z[4,5] = -np.cos(psi_h_1)
            F_z[4,6] = 0*(np.cos(psi_h_1)*(axm_h_1)-np.sin(psi_h_1)*(aym_h_1))    
            F_z[4,7] = 0*(-xvm_Nh_1)
            F_z[6,7]= -1
            I=np.eye(8)
            phi_z = I+(F_z*dt)
            Q_z[1,1] = (np.cos(psi_h_1))*(np.cos(psi_h_1))*(sig_bx**2) + (np.sin(psi_h_1))*(np.sin(psi_h_1))*(sig_by**2) + (yvm_Nh_1*yvm_Nh_1)*(sig_arw_0**2)
            Q_z[2,2] = sig_xr**2
            Q_z[4,4] = (np.sin(psi_h_1))*(np.sin(psi_h_1))*(sig_bx**2) + (np.cos(psi_h_1))*(np.cos(psi_h_1))*(sig_by**2) + (xvm_Nh_1*xvm_Nh_1)*(sig_arw_0**2)
            Q_z[5,5] = sig_yr**2
            Q_z[6,6] = 1*sig_arw_0**2
            Q_z[7,7] = 1*sig_rrw_0**2  
            Xz_h=phi_z.dot(Xz_h)
	    #print("789",P00_z)
            if select==1:
	    	tmp_1=phi_z.dot(P00_z)
	    else:
	    	tmp_1=phi_z.dot(tmp)	
	    #print(tmp_1)
            tmp=tmp_1.dot(phi_z.T)+Q_z*dt
            #print("1",tmp)	
            if select ==1:
                a[0]=0 #acc_x
                a[1]=0 #acc_y
                a[2]=0 #gyro_z
                a_1[0]=a[0]
            	a_1[1]=a[1]
            	a_1[2]=a[2]
            	bx_h_1 = bx_h
            	by_h_1 = by_h
            	bz_h_1 = bz_h
            	xam_Nh_1 = xam_Nh
            	yam_Nh_1 = yam_Nh
            	xvm_Nh_1 = xvm_Nh
            	yvm_Nh_1 = yvm_Nh
            	xpm_Nh_1 = xpm_Nh
            	ypm_Nh_1 = ypm_Nh
	    	#print("test",xpm_Nh,ypm_Nh)
            	psi_h_1 = psi_h
            	wzm_h_1 = wzm_h
            	u = u +1
            	select = 2     
        ###### IMU Estimate four Distances #####
	"""
    	R1m_h = ((X[0,0]-xpm_Nh)**2+(X[0,1]-ypm_Nh)**2)**(0.5)
    	R2m_h = ((X[1,0]-xpm_Nh)**2+(X[1,1]-ypm_Nh)**2)**(0.5)
    	R3m_h = ((X[2,0]-xpm_Nh)**2+(X[2,1]-ypm_Nh)**2)**(0.5)
    	R4m_h = ((X[3,0]-xpm_Nh)**2+(X[3,1]-ypm_Nh)**2)**(0.5)
    	"""

    	R1m_h = ((X[0,0]-xpm_Nh)**2+(X[0,1]-ypm_Nh)**2+(X[0,2]-1.32)**2)**(0.5)
    	R2m_h = ((X[1,0]-xpm_Nh)**2+(X[1,1]-ypm_Nh)**2+(X[1,2]-1.32)**2)**(0.5)
    	R3m_h = ((X[2,0]-xpm_Nh)**2+(X[2,1]-ypm_Nh)**2+(X[2,2]-1.32)**2)**(0.5)
    	R4m_h = ((X[3,0]-xpm_Nh)**2+(X[3,1]-ypm_Nh)**2+(X[3,2]-1.32)**2)**(0.5)    
    	
    	#print("imu",R1m_h,R2m_h,R3m_h,R4m_h)
    	##### H Matrix Residual Calculator #####
    	r1_partial_x = 0
    	r1_partial_y = 0
    	r2_partial_x = 0 
    	r2_partial_y = 0 
    	r3_partial_x = 0 
    	r3_partial_y = 0 
    	r4_partial_x = 0 
    	r4_partial_y = 0
    	r1_partial_x =-(X[0,0]-xpm_Nh)/R1m_h
    	r1_partial_y =-(X[0,1]-ypm_Nh)/R1m_h
    	r2_partial_x =-(X[1,0]-xpm_Nh)/R2m_h 
    	r2_partial_y =-(X[1,1]-ypm_Nh)/R2m_h
    	r3_partial_x =-(X[2,0]-xpm_Nh)/R3m_h
    	r3_partial_y =-(X[2,1]-ypm_Nh)/R3m_h
    	r4_partial_x =-(X[3,0]-xpm_Nh)/R4m_h 
    	r4_partial_y =-(X[3,1]-ypm_Nh)/R4m_h
    	##### H Matrix Data #####
    	H=np.array([r1_partial_x,0,0,r1_partial_y,0,0,0,0,r2_partial_x,0,0,r2_partial_y,0,0,0,0,r3_partial_x,0,0,r3_partial_y,0,0,0,0,r4_partial_x,0,0,r4_partial_y,0,0,0,0])
    	H=H.reshape([4,8])

    	###### Kalman_Filter_update_8_4_radio ######
    	zxm_z=[0]*4
    	##### Real Distance - Esitmate Distance #####
    	zxm_z[0] = d[0]-R1m_h
    	zxm_z[1] = d[1]-R2m_h
    	zxm_z[2] = d[2]-R3m_h
    	zxm_z[3] = d[3]-R4m_h
    	#print("dddddd",zxm_z)
    
    	#print("##################",d[0],d[1],d[2],d[3])
    	##### Mu Martix Data #####
    
    	Y=np.array([zxm_z[0],zxm_z[1],zxm_z[2],zxm_z[3]])
    	k=0
    	l=0
    	z_update=np.zeros([8,1])
    	H_D=np.zeros([1,8])
    	K_z_help=[0]*1
    	tmpXY=np.zeros([8,1])
    	Mu_z=Y.reshape([4,1])
	K_z=np.zeros([8,1])
    	I=np.eye(8)
	n_h=0
        for i in range(0,4):
            if end_count> 15 :
                if (zxm_z[i]*zxm_z[i])**0.5< threshold_c:
                    for j in range(0,8):

                        H_D[0,j]=H[i,j]
                    if select_d==1:
                        tmpYX=H_D.dot(tmp)
                    else:
                        tmp=P00_z.dot(I)
                        tmpYX=H_D.dot(tmp)
		    K_z_help=tmpYX.dot(H_D.T)
                    K_z_help[0]=1/(K_z_help[0]+R[i,i])
                    tmpXY=tmp.dot(H_D.T)
                    K_z=tmpXY*(K_z_help)
                    P00_z=(I-K_z.dot(H_D)).dot(tmp)
		    #print("sucess")
	    	else:
		    for j in range(0,8):

                        H_D[0,j]=H[i,j]

                    K_z=np.zeros([8,1])
		    P00_z=(I-K_z.dot(H_D)).dot(tmp)
                    count1 = count1+1
	    else:
		for j in range(0,8):
                #for i in range(0,3):

                   H_D[0,j]=H[i,j]
		   #print(H_D)
                if select_d==1: 
                    tmpYX=H_D.dot(tmp)

                else:
		    n_h=n_h+1
		    #print(n_h)
                    tmp=P00_z.dot(I)
                    tmpYX=H_D.dot(tmp)

		K_z_help=tmpYX.dot(H_D.T)
                K_z_help[0]=1/(K_z_help[0]+R[i,i])
                tmpXY=tmp.dot(H_D.T)
                K_z=tmpXY*(K_z_help)
		#print(K_z)
                P00_z=(I-K_z.dot(H_D)).dot(tmp)
		#print(H_D)
            z_update=K_z*Y[i]+z_update
	    select_d = 0
        #print(z_update) 
    	xpm_Nh=xpm_Nh+z_update[0,0]
    	xvm_Nh=xvm_Nh+z_update[1,0]
    	bx_h = bx_h + z_update[2,0]
    	ypm_Nh=ypm_Nh+z_update[3,0]
    	yvm_Nh=yvm_Nh+z_update[4,0]
    	by_h= by_h + z_update[5,0]
    	psi_h=psi_h+z_update[6,0]
    	bz_h=bz_h+z_update[7,0] 
    	Xz_h=np.zeros([8,1])
    a_1[0] = a[0]
    a_1[1] = a[1]
    a_1[2] = a[2]
    bx_h_1 = bx_h
    by_h_1 = by_h
    bz_h_1 = bz_h
    xam_Nh_1 = xam_Nh
    yam_Nh_1 = yam_Nh
    xvm_Nh_1 = xvm_Nh
    yvm_Nh_1 = yvm_Nh
    xpm_Nh_1 = xpm_Nh
    ypm_Nh_1 = ypm_Nh
    psi_h_1 = psi_h
    wzm_h_1 = wzm_h
    d_1[0]=d[0]#distance1
    d_1[1]=d[1]#distance2
    d_1[2]=d[2]#distance3
    d_1[3]=d[3]#distance4

    
    u= u+1
    select = 1
    select_d = 1
    count1 = 0
    end_count = end_count+1
    
    ##$$$$$$$$$$$$$$$$$$$$$##
    EKF_Solution_Anc[0]=xpm_Nh
    EKF_Solution_Anc[1]=ypm_Nh
    print("EKF_Position:",EKF_Solution_Anc[0],EKF_Solution_Anc[1])
    #print("EKF_Position:%.2f  %.2f  "%(EKF_Solution_Anc[0],EKF_Solution_Anc[1]))
    #ekf_x=np.arange(0,180)
    #ekf_y=np.sin(ekf_x * np.pi / 180.0)

    pylab.scatter(EKF_Solution_Anc[0],EKF_Solution_Anc[1],s=0.5)
    pylab.xlim(4.6,5)
    pylab.ylim(6.7,7.0)
    pylab.xlabel("x-axis") 
    pylab.ylabel("y-axis") 
    pylab.title("EKF Position",fontsize=24) 
    n_ekf=n_ekf+1

    if n_ekf>20:
	#print("-----plot-----")
	
	pylab.savefig('ekf5_finish.png')	
    	#print("EKF_Position:%.2f  %.2f  "%(EKF_Solution_Anc[0],EKF_Solution_Anc[1]))
    



def Tag_resetInactive():
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

################################################
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

def Anchor_resetInactive():
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
"""
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
    
    B=np.linalg.inv(A)
   
    Solution_Anc=B.dot(b)
    

    print("Real_Position:%.2f  %.2f  "%(Y_2D[0],Y_2D[1]))
    print("Position:%.2f  %.2f  "%(Solution_Anc[0],Solution_Anc[1])) 
"""



def loop():
    
    global sentAck,n_ekf_start,n_23,n_25,n_26,n_27, receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId,expectedMsgID, timeRangeSentTS,Same_tag_flag,DistanceFinish_Flag,Position_Flag,EKF_start,EKF_message,EKF_New,EKF_Update

    if Position_Flag==0:
        
        if imu.IMURead():
            global fusionPose
            Data = imu.getIMUData()
            fusionPose = Data["accel"]
            #print("fffff",fusionPose)
	    time.sleep(poll_interval*1.0/1000.0)
            #time.sleep(0.1)


        if (sentAck == False and receivedAck == False):
            if ((millis() - lastActivity) > C.RESET_PERIOD):
                Anchor_resetInactive()
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
            #print(data[16])		      
        
            if msgId == C.POLL:
            	DistanceFinish_Flag =1
            	Same_tag_flag = data[16]
            	protocolFailed = False
		timePollReceivedTS = DW1000.getReceiveTimestamp()
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
            		
        
        
                    
                    	if data[16]==23:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance1: %.2f m" %(distance))
                            n_23=n_23+1
			    if distance <12:
                            #tag[0]=((n_23-1)*tag[0]+distance)/n_23
			    	tag[0]=distance
                            #print(n_23)
                    	if data[16]==25:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance2: %.2f m" %(distance))
                            n_25=n_25+1
                            if distance <12:
                            #tag[1]=((n_25-1)*tag[1]+distance)/n_25
			    	tag[1]=distance	
                        if data[16]==26:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance3: %.2f m" %(distance))
                            n_26=n_26+1
                            if distance <12:
                            #tag[2]=((n_26-1)*tag[2]+distance)/n_26
			    	tag[2]=distance
                        if data[16]==27:
                            print("Tag: %.2d"%(data[16]))
                            print("Distance4: %.2f m" %(distance))
                            n_27=n_27+1
                            if distance <12:
                            #tag[3]=((n_27-1)*tag[3]+distance)/n_27
			    	tag[3]=distance


                        if tag[0] !=0 and tag[1]!=0 and tag[2] !=0 and tag[3]!=0:
                            #print("tag",tag)
                            #spread_2D()
			    EKF_start()
			    n_ekf_start=n_ekf_start+1
			    print(n_ekf_start)
		   	    	
                            #print("success")
                        
                if n_23 >=5 and n_25 >=5 and n_26 >=5 and n_27 >=5:
	            #print("transmit TAG")
                    #os.system("python ./DW1000RangingTAG.py")
                    Position_Flag=0

                else:
                    transmitRangeFailed()

                noteActivity()
    if Position_Flag==1:
        
        if (sentAck == False and receivedAck == False):
            if ((millis() - lastActivity) > C.RESET_PERIOD):
                Tag_resetInactive()
            return

    	if sentAck :

            sentAck = False

            msgID = data[0]
            
            if data[16]==28:
                if msgID == C.POLL :
                    timePollSentTS = DW1000.getTransmitTimestamp()
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
    PIN_IRQ = 19
    PIN_SS = 27
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_RST,GPIO.IN)
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
    
    while 1: 
        loop()

except KeyboardInterrupt:
    DW1000.close()

