
import random,threading,time 
from Queue import Queue 
import RTIMU 
import numpy as np 
import DW1000 
import monotonic 
import DW1000Constants as C 
import RPi.GPIO as GPIO 
import math 
import datetime
import Adafruit_CharLCD as LCD

SETTINGS_FILE = "RTIMULib"
s = RTIMU.Settings(SETTINGS_FILE) 
imu = RTIMU.RTIMU(s)

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
#IMU thread
# Raspberry Pi pin configuration:
lcd_rs        = 6  # Note this might need to be changed to 21 for older revision Pi's.
lcd_en        = 22
lcd_d4        = 25
lcd_d5        = 24
lcd_d6        = 23
lcd_d7        = 18
# Alternatively specify a 16x2 LCD.
lcd_columns = 16
lcd_rows    = 2

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows)

imu_count=0
total=400

uwb_array=np.zeros([4,total])
imu_array=np.zeros([10,total])
imu_array=np.zeros([10,total])
ekf_array=np.zeros([2,total])
ls_array=np.zeros([2,total])
bias_array=np.zeros([3,total])
grid_array=[0]*400
Grid=[0]*1
n_23=0
n_24=0
n_25=0
n_26=0
n_27=0
n_29=0
sum_x=0
sum_y=0

Position_Flag=0
DistanceFinish_Flag=0
Same_tag_flag=0
lastActivity = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA =25
data = [0] * LEN_DATA
LEN_TAG=6
tag=[0]*LEN_TAG
timePollAckSentTS = 0
timePollAckReceivedTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
REPLY_DELAY_TIME_US = 7000 

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
DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
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

###### Grid_1_2D_Tag_position  #####
X_2D_1=np.array([7.65,0.13,0,0.9,0,5.64]).reshape([3,2])
#Y_2D=(4.76,6.67)

###### Grid_2_2D_Tag_position  #####
X_2D_2=np.array([0,5.64,0.17,11.8,7.78,11.7]).reshape([3,2])
Y_2D=(4.76,6.23)

##### Car test #####
#X_2D=np.array([0,0,0,1,0.82,1,0.82,0]).reshape([4,2])

#########################
###### 3D Tag position  #####
X=np.array([7.65,0.13,2.7,0,0.9,2.75,0,5.64,2.73,0.17,11.8,2.72,7.78,11.7,2.5,11.04,5.61,2.74]).reshape([6,3])
Y=(0,5.64,2.73)
def Grid_1_spread_2D():

    global Solution_Anc
    ######Tag-------Anchor-------Distance#####
    r0=((X_2D_1[0,0]-Y_2D[0])**2 + (X_2D_1[0,1]-Y_2D[1])**2)**(0.5) #26
    r1=((X_2D_1[1,0]-Y_2D[0])**2 + (X_2D_1[1,1]-Y_2D[1])**2)**(0.5) #23
    r2=((X_2D_1[2,0]-Y_2D[0])**2 + (X_2D_1[2,1]-Y_2D[1])**2)**(0.5) #24
    print(r0,r1,r2)

    A=np.array([2*(X_2D_1[0,0]-X_2D_1[1,0]),2*(X_2D_1[0,1]-X_2D_1[1,1]),2*(X_2D_1[0,0]-X_2D_1[2,0]),2*(X_2D_1[0,1]-X_2D_1[2,1])])
    A=A.reshape([2,2])
    b=np.array([tag[0]**2-tag[2]**2+X_2D_1[0,0]**2-X_2D_1[1,0]**2+X_2D_1[0,1]**2-X_2D_1[1,1]**2,tag[4]**2-tag[2]**2+X_2D_1[0,0]**2-X_2D_1[2,0]**2+X_2D_1[0,1]**2-X_2D_1[2,1]**2])
    B=np.linalg.inv(A)
    Solution_Anc=B.dot(b)
    #print("Real_Position:%.2f  %.2f  "%(Y_2D[0],Y_2D[1]))
    print("2D_Position:%.2f  %.2f  "%(Solution_Anc[0],Solution_Anc[1]))
def Grid_2_spread_2D():

    global Solution_Anc
    ######Tag-------Anchor-------Distance#####
    r0=((X_2D_2[0,0]-Y_2D[0])**2 + (X_2D_2[0,1]-Y_2D[1])**2)**(0.5) #24
    r1=((X_2D_2[1,0]-Y_2D[0])**2 + (X_2D_2[1,1]-Y_2D[1])**2)**(0.5) #25
    r2=((X_2D_2[2,0]-Y_2D[0])**2 + (X_2D_2[2,1]-Y_2D[1])**2)**(0.5) #27
    print(r0,r1,r2)

    A=np.array([2*(X_2D_2[0,0]-X_2D_2[1,0]),2*(X_2D_2[0,1]-X_2D_2[1,1]),2*(X_2D_2[0,0]-X_2D_2[2,0]),2*(X_2D_2[0,1]-X_2D_2[2,1])])
    A=A.reshape([2,2])
    b=np.array([tag[1]**2-tag[4]**2+X_2D_2[0,0]**2-X_2D_2[1,0]**2+X_2D_2[0,1]**2-X_2D_2[1,1]**2,tag[3]**2-tag[4]**2+X_2D_2[0,0]**2-X_2D_2[2,0]**2+X_2D_2[0,1]**2-X_2D_2[2,1]**2])
    B=np.linalg.inv(A)
    Solution_Anc=B.dot(b)
    #print("Real_Position:%.2f  %.2f  "%(Y_2D[0],Y_2D[1]))
    print("2D_Position:%.2f  %.2f  "%(Solution_Anc[0],Solution_Anc[1]))

def EKF_message():
    global P00_z,tag,accel,gyro
    EKF_New()

    Imu[0]=accel[0]
    Imu[1]=-accel[1]
    Imu[2]=gyro[2]
    if sum([tag[2],tag[0],tag[4]])<sum([tag[4],tag[1],tag[3]]):

        dwm[0]=tag[2]
        dwm[1]=tag[0]
        dwm[2]=tag[4]
        dwm[3]=tag[5]
    else:
        dwm[0]=tag[4]
        dwm[1]=tag[1]
        dwm[2]=tag[3]
        dwm[3]=tag[5]

    
    print(tag[2],tag[0],tag[4],tag[1],tag[3],tag[5])
    if (tag[0] and tag[1] and tag[2] and tag[3] and tag[4] and tag[5])!=0:
        EKF_Update()
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
    global sum_x,sum_y,bias_array,imu_array_uwb_array,n_ekf,dwm,Imu,state,xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,xam_Nh,yam_Nh,wzm_h,bx_h,by_h,bz_h,psi_h,end_count,P00_z,tmp,tmp_1,tmp_YX,u,select,count1,select_d,Solution_Anc,EKF_Solution_Anc,R,Xz_h,F_z,Q_z,H_D,bx_h_1,by_h_1,bz_h_1,psi_h_1,xpm_Nh_1,ypm_Nh_1,xvm_Nh_1,yvm_Nh_1,xam_Nh_1,yam_Nh_1

    starttime = datetime.datetime.now()
    a[0]=Imu[0] #acc_x
    a[1]=Imu[1] #acc_y
    a[2]=Imu[2] #gyro_z

    d[0]=dwm[0] #distance1
    d[1]=dwm[1] #distance2
    d[2]=dwm[2] #distance3
    d[3]=dwm[3] #distance4

    for i in range(0,4):
        if d[i]>150000:
            d[i]=d_1[i]
    if sum([tag[2],tag[0],tag[4]])<sum([tag[4],tag[1],tag[3]]):

        Grid_1_spread_2D()
        Grid=1
    else:
	Grid_2_spread_2D()
	Grid=2
    xpm_Nh_2=Solution_Anc[0]
    ypm_Nh_2=Solution_Anc[1]

    if state == 1:
        bz_h = bz0
        if d[0]>0 and d[1]>0 and d[2]>0 and d[3]>0: 
            xpm_Nh = xpm_Nh_2
            ypm_Nh = ypm_Nh_2
            xpm_Nh_1 = xpm_Nh
            ypm_Nh_1 = ypm_Nh
            state = 2
    if state == 2:
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

            if select==1:
	    	tmp_1=phi_z.dot(P00_z)
	    else:
	    	tmp_1=phi_z.dot(tmp)

            tmp=tmp_1.dot(phi_z.T)+Q_z*dt

            if select ==1:
                a[0]=Imu[0] #acc_x
                a[1]=Imu[1] #acc_y
                a[2]=Imu[2] #gyro_z
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
            	psi_h_1 = psi_h
            	wzm_h_1 = wzm_h
            	u = u +1
            	select = 2
	if Grid==1:
            ###### IMU Estimate four Distances #####
            R1m_h = ((X[0,0]-xpm_Nh)**2+(X[0,1]-ypm_Nh)**2+(X[0,2]-1.32)**2)**(0.5)
            R2m_h = ((X[1,0]-xpm_Nh)**2+(X[1,1]-ypm_Nh)**2+(X[1,2]-1.32)**2)**(0.5)
            R3m_h = ((X[2,0]-xpm_Nh)**2+(X[2,1]-ypm_Nh)**2+(X[2,2]-1.32)**2)**(0.5)
            R4m_h = ((X[5,0]-xpm_Nh)**2+(X[5,1]-ypm_Nh)**2+(X[5,2]-1.32)**2)**(0.5)
            ##### H Matrix Residual Calculator #####
            r1_partial_x =-(X[0,0]-xpm_Nh)/R1m_h
            r1_partial_y =-(X[0,1]-ypm_Nh)/R1m_h
            r2_partial_x =-(X[1,0]-xpm_Nh)/R2m_h
            r2_partial_y =-(X[1,1]-ypm_Nh)/R2m_h
            r3_partial_x =-(X[2,0]-xpm_Nh)/R3m_h
            r3_partial_y =-(X[2,1]-ypm_Nh)/R3m_h
            r4_partial_x =-(X[5,0]-xpm_Nh)/R4m_h
            r4_partial_y =-(X[5,1]-ypm_Nh)/R4m_h
        if Grid==2:
            ###### IMU Estimate four Distances #####
            R1m_h = ((X[2,0]-xpm_Nh)**2+(X[2,1]-ypm_Nh)**2+(X[2,2]-1.32)**2)**(0.5)
            R2m_h = ((X[3,0]-xpm_Nh)**2+(X[3,1]-ypm_Nh)**2+(X[3,2]-1.32)**2)**(0.5)
            R3m_h = ((X[4,0]-xpm_Nh)**2+(X[4,1]-ypm_Nh)**2+(X[4,2]-1.32)**2)**(0.5)
            R4m_h = ((X[5,0]-xpm_Nh)**2+(X[5,1]-ypm_Nh)**2+(X[5,2]-1.32)**2)**(0.5)
            ##### H Matrix Residual Calculator #####
            r1_partial_x =-(X[2,0]-xpm_Nh)/R1m_h
            r1_partial_y =-(X[2,1]-ypm_Nh)/R1m_h
            r2_partial_x =-(X[3,0]-xpm_Nh)/R2m_h
            r2_partial_y =-(X[3,1]-ypm_Nh)/R2m_h
            r3_partial_x =-(X[4,0]-xpm_Nh)/R3m_h
            r3_partial_y =-(X[4,1]-ypm_Nh)/R3m_h
            r4_partial_x =-(X[5,0]-xpm_Nh)/R4m_h
            r4_partial_y =-(X[5,1]-ypm_Nh)/R4m_h



	"""  
        R1m_h = ((X_2D[0,0]-xpm_Nh)**2+(X_2D[0,1]-ypm_Nh)**2)**(0.5)
        R2m_h = ((X_2D[1,0]-xpm_Nh)**2+(X_2D[1,1]-ypm_Nh)**2)**(0.5)
        R3m_h = ((X_2D[2,0]-xpm_Nh)**2+(X_2D[2,1]-ypm_Nh)**2)**(0.5)
        R4m_h = ((X_2D[3,0]-xpm_Nh)**2+(X_2D[3,1]-ypm_Nh)**2)**(0.5)
        ##### H Matrix Residual Calculator #####
        r1_partial_x =-(X_2D[0,0]-xpm_Nh)/R1m_h
        r1_partial_y =-(X_2D[0,1]-ypm_Nh)/R1m_h
        r2_partial_x =-(X_2D[1,0]-xpm_Nh)/R2m_h
        r2_partial_y =-(X_2D[1,1]-ypm_Nh)/R2m_h
        r3_partial_x =-(X_2D[2,0]-xpm_Nh)/R3m_h
        r3_partial_y =-(X_2D[2,1]-ypm_Nh)/R3m_h
        r4_partial_x =-(X_2D[3,0]-xpm_Nh)/R4m_h
        r4_partial_y =-(X_2D[3,1]-ypm_Nh)/R4m_h
	"""


    	##### H Matrix Data #####
    	H=np.array([r1_partial_x,0,0,r1_partial_y,0,0,0,0,r2_partial_x,0,0,r2_partial_y,0,0,0,0,r3_partial_x,0,0,r3_partial_y,0,0,0,0,r4_partial_x,0,0,r4_partial_y,0,0,0,0])
        #H=np.array([r1_partial_x,0,0,r1_partial_y,0,0,0,0,r2_partial_x,0,0,r2_partial_y,0,0,0,0,r3_partial_x,0,0,r3_partial_y,0,0,0,0,0,0,0,0,0,0,0,0])
    	H=H.reshape([4,8])

    	###### Kalman_Filter_update_8_4_radio ######
    	zxm_z=[0]*4
    	##### Real Distance - Esitmate Distance #####
    	zxm_z[0] = d[0]-R1m_h
    	zxm_z[1] = d[1]-R2m_h
    	zxm_z[2] = d[2]-R3m_h
    	zxm_z[3] = d[3]-R4m_h
        #zxm_z[3] = 0

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

	    	else:
		    for j in range(0,8):

                        H_D[0,j]=H[i,j]

                    K_z=np.zeros([8,1])
		    P00_z=(I-K_z.dot(H_D)).dot(tmp)
                    count1 = count1+1
	    else:
		for j in range(0,8):


                   H_D[0,j]=H[i,j]
                if select_d==1:
                    tmpYX=H_D.dot(tmp)

                else:
		    n_h=n_h+1
                    tmp=P00_z.dot(I)
                    tmpYX=H_D.dot(tmp)

		K_z_help=tmpYX.dot(H_D.T)
                K_z_help[0]=1/(K_z_help[0]+R[i,i])
                tmpXY=tmp.dot(H_D.T)
                K_z=tmpXY*(K_z_help)
                P00_z=(I-K_z.dot(H_D)).dot(tmp)

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



    EKF_Solution_Anc[0]=xpm_Nh
    EKF_Solution_Anc[1]=ypm_Nh


    print("EKF_Position:%.2f  %.2f  "%(EKF_Solution_Anc[0],EKF_Solution_Anc[1]))
    print(Grid)
    ekf_array[0,end_count]=EKF_Solution_Anc[0]
    ekf_array[1,end_count]=EKF_Solution_Anc[1]
    ls_array[0,end_count]=Solution_Anc[0]
    ls_array[1,end_count]=Solution_Anc[1]
    imu_array[0,end_count]=accel[0]
    imu_array[1,end_count]=accel[1]
    imu_array[2,end_count]=accel[2]
    imu_array[3,end_count]=gyro[0]
    imu_array[4,end_count]=gyro[1]
    imu_array[5,end_count]=gyro[2]
    imu_array[6,end_count]=compass[0]
    imu_array[7,end_count]=compass[1]
    imu_array[8,end_count]=compass[2]
    uwb_array[0,end_count]=tag[0]
    uwb_array[1,end_count]=tag[1]
    uwb_array[2,end_count]=tag[3]
    uwb_array[3,end_count]=tag[2]
    grid_array[end_count]=Grid
    #bias[0,end_count]=bx_h
    #bias[1,end_count]=by_h
    #bias[2,end_count]=bz_h
    sum_x=(sum_x+ekf_array[0,end_count])
    sum_y=(sum_y+ekf_array[1,end_count])
    
    bias_array[0,end_count]=bx_h
    bias_array[1,end_count]=by_h
    bias_array[2,end_count]=bz_h
    #print(bx_h,by_h,bz_h)
    if end_count==4:
        sum_x=sum_x/5
        sum_y=sum_y/5
        print "average",sum_x,sum_y
        lcd_show="X:%.2f"%(float(sum_x))+"  m" + '\n' + "Y:%.2f"%(float(sum_y))+"  m"
        lcd.clear()
        lcd.message(lcd_show)

        sum_x=0
	sum_y=0
	end_count=0
    #lcd_show="X:%.2f"%(float(ekf_array[0,end_count]))+"  m" + '\n' + "Y:%.2f"%(float(ekf_array[1,end_count]))+"  m"
    #lcd.clear()
    #lcd.message(lcd_show)


    endtime = datetime.datetime.now()
    #print("time: %.3f s"% (float(((endtime-starttime).microseconds)*(10**(-6)))))
    print(end_count)
    end_count = end_count+1
    if end_count==400:
        print("--------------------------------------------------save--------------------------------------------")
        print("--------------------------------------------------save--------------------------------------------")
        print("--------------------------------------------------save--------------------------------------------")
        np.savetxt('Distance_data_pos_0318.csv',(ekf_array) ,delimiter=',')
        #np.savetxt('Grid_straight_0220_5.csv',(grid_array) ,delimiter=',')

        #np.savetxt('LS_data_0202.csv',(ls_array) ,delimiter=',')
        #np.savetxt('EKF_data_0202.csv',(ekf_array) ,delimiter=',')
        #np.savetxt('IMU_data_0202.csv',(imu_array) ,delimiter=',')
        #np.savetxt('UWB_data_0202.csv',(uwb_array) ,delimiter=',')


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

DW1000.registerCallback("handleSent", handleSent)
DW1000.registerCallback("handleReceived", handleReceived)


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
    DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY) 
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)       
    expectedMsgId = C.POLL
    print("run")
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
receiver()
noteActivity()
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

def loop():
    global uwb_array,gyro_array,gyro_count,sentAck,n_ekf_start,start,n_23,n_24,n_25,n_26,n_27,n_29,receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId,expectedMsgID, timeRangeSentTS,Same_tag_flag,DistanceFinish_Flag,Position_Flag,EKF_start,EKF_message,EKF_New,EKF_Update
    
    if Position_Flag==0:
        if sentAck == False and receivedAck == False:
            if ((millis() - lastActivity) > C.RESET_PERIOD):
                Anchor_resetInactive()
       	    return
    	if sentAck:
            #print("1")
            sentAck = False
            msgId = data[0]
            if Same_tag_flag == data[16]:
                if msgId == C.POLL_ACK:
            	    timePollAckSentTS = DW1000.getTransmitTimestamp()
            	    noteActivity()

    	if receivedAck:
            receivedAck = False
            data = DW1000.getData(LEN_DATA)
            msgId = data[0]
            if msgId == C.POLL:
                #print("2")
            	DistanceFinish_Flag =1
            	Same_tag_flag = data[16]
            	protocolFailed = False
		timePollReceivedTS = DW1000.getReceiveTimestamp()
            	expectedMsgId = C.RANGE
            	transmitPollAck()
            	noteActivity()
            elif msgId == C.RANGE :
                #print("3")
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
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance1: %.2f m" %(distance))
                            #print("[%s]"%(time.ctime(time.time())))
                            #t = time.time()
                            #print (int(round(t * 1000)))

                            if distance <12 : #and distance>5:
                                tag[0]=distance
                                #uwb_array[0,n_23]=tag[0]
                                #print(n_23)
                                n_23+=1
                        if data[16]==24:
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance1: %.2f m" %(distance))
                            #print("[%s]"%(time.ctime(time.time())))
                            #t = time.time()
                            #print (int(round(t * 1000)))

                            if distance <12 : #and distance>5:
                                tag[4]=distance
                                #uwb_array[0,n_24]=tag[4]
                                #print(n_24)
                                n_24+=1

                        if data[16]==25:
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance2: %.2f m" %(distance))
                            #print("[%s]"%(time.ctime(time.time())))

                            if distance <12: #and distance>5:
                                tag[1]=distance
                                #uwb_array[2,n_25]=tag[1]
                                #print(n_25)
                                n_25+=1
                        if data[16]==26:
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance3: %.2f m" %(distance))
                            #t = time.time()
                            #print (int(round(t * 1000)))

                            if distance <12 and distance>5:
                                tag[2]=distance
                                #uwb_array[3,n_26]=tag[2]
                                #print(n_26)
                                n_26+=1
                        if data[16]==27:
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance4: %.2f m" %(distance))

                            if distance <12 and distance >5:
                                tag[3]=distance
                                #uwb_array[4,n_27]=tag[3]
                                #print(n_27)
                                n_27+=1
                        if data[16]==29:
                            #print("Tag: %.2d"%(data[16]))
                            #print("Distance: %.2f m" %(distance))
			    
                            if distance <12 :
                                tag[5]=distance
                                uwb_array[0,n_29]=tag[5]
                                print(n_29)
                                n_29+=1
			if  n_29==200:
	                    print("******************************save*********************************************")
           	            np.savetxt('distance_toto_0318_1.csv',(uwb_array) , delimiter=',')

                else:
                    transmitRangeFailed()
                noteActivity()
class IMU_data(threading.Thread):
    def __init__(self, t_name, queue):
        threading.Thread.__init__(self,name=t_name)
        self.data=queue
    def run(self):
	global imu_count,imu_array,accel,gyro,compass
        while 1:
	    if imu.IMURead():
            	Data = imu.getIMUData()
            	accel = Data["accel"]
            	gyro = Data["gyro"]
            	compass = Data["compass"]
		""" 
                imu_array[0,imu_count]=accel[0]
		imu_array[1,imu_count]=accel[1]
                imu_array[2,imu_count]=accel[2]
                imu_array[3,imu_count]=gyro[0]
                imu_array[4,imu_count]=gyro[1]
                imu_array[5,imu_count]=gyro[2]
                imu_array[6,imu_count]=compass[0]
                imu_array[7,imu_count]=compass[1]
                imu_array[8,imu_count]=compass[2]
                imu_array[9,0]=imu_array[5,imu_count]*0.01
                if imu_count>=1:
                    imu_array[9,imu_count]=imu_array[9,imu_count-1]+imu_array[5,imu_count]*0.01
                #print (imu_array[9,imu_count],imu_count)
                imu_count+=1


	    	if imu_count==1000:
		    print("******************************save*********************************************")
		    np.savetxt('9_State_data.csv',(imu_array) , delimiter=',')
		"""
            time.sleep(0.01)


#Consumer thread
class Consumer_even(threading.Thread):
    def __init__(self,t_name,queue):
        threading.Thread.__init__(self,name=t_name)
        self.data=queue
    def run(self):
        while 1:
            loop()
            time.sleep(0.01)



class Consumer_odd(threading.Thread):
    def __init__(self,t_name,queue):
        threading.Thread.__init__(self, name=t_name)
        self.data=queue
    def run(self):
	global accel,gyro
        while 1:
            EKF_message()
            time.sleep(0.1)


#Main thread
def main():
    queue = Queue()
    imu_data = IMU_data('Pro.', queue)
    consumer_even = Consumer_even('Con_even.', queue)
    consumer_odd = Consumer_odd('Con_odd.',queue)
    imu_data.start()
    consumer_even.start()
    consumer_odd.start()
    imu_data.join()
    consumer_even.join()
    consumer_odd.join()
    print ('All threads terminate!')


if __name__ == '__main__':
    main()
