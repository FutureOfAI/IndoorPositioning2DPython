
import DW1000
import monotonic
import imu
import DW1000RangingAnchor as Anc 
import DW1000Constants as C
import RPi.GPIO as GPIO
import numpy as np
import math


pi=3.14
r2d = (180/pi)
d2r = (pi/180)
g =9.8
scale_factor_err = -0.0512
#print(A.n_23)



for ii in range(0,1000):
#  profile_flag = 1
#   if (profile_flag ==1):

    """
    xr1 = 0
    yr1 = 0.9
    h_1 = 2.75

    xr2 = 0.17   
    yr2 = 11.8 
    h_2 = 2.72 

    xr3 = 7.78
    yr3 = 11.7
    h_3 = 2.5

    xr4 = 7.65
    yr4 = 0.13
    h_4 = 2.7
    """
    dt = 0.01
    T = 39.91
    t0 =np.arange(0,T,dt)
    t0_1=t0.reshape([3991,1])
    n = len(t0)
    m=t0_1.shape
    t00 = t0
    fn = 0*0.05
    psi_0 = 1*0*d2r
    wn = 0
    wz=[0]*n
    psi=[0]*n
    ft=1*0.5
    wt=2*pi*ft
    radius = (0.5*g)/(pi**2)

    #Need to convert into body frame (B-frame) for accelerometer sensings and optical
    #flow sensing since they are mounted on the body frame
    def trajectory1():
        global x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N
        x_p_N= radius*np.sin(wt*t0)*np.cos(psi_0)+6.0
        x_v_N = radius*wt*np.cos(wt*t0)*np.cos(psi_0)
        x_a_N= -radius*wt**2*np.sin(wt*t0)*np.cos(psi_0)
        y_p_N= radius*np.sin(wt*t0)*np.sin(psi_0)+4.53
        y_v_N= radius*wt*np.cos(wt*t0)*np.sin(psi_0)
        y_a_N= -radius*wt**2*np.sin(wt*t0)*np.sin(psi_0)

   
    trajectory1()
    x_v_B = np.zeros(n)
    y_v_B = np.zeros(n)
    x_a_B = np.zeros(n)
    y_a_B = np.zeros(n)
    for i in range(0,n):
    	x_a_B[i] = np.array([np.cos(psi[i]),np.sin(psi[i])]).dot((np.array([x_a_N[i]+wz[i]*y_v_N[i],y_a_N[i]-wz[i]*x_v_N[i]])).reshape([2,1]))
        y_a_B[i] = np.array([-np.sin(psi[i]),np.cos(psi[i])]).dot((np.array([x_a_N[i]+wz[i]*y_v_N[i],y_a_N[i]-wz[i]*x_v_N[i]])).reshape([2,1]))
        x_v_B[1] = np.array([np.cos(psi[1]),np.sin(psi[1])]).dot((np.array([x_v_N[1],y_v_N[1]])).reshape([2,1]))
        y_v_B[1] = np.array([-np.sin(psi[1]),np.cos(psi[1])]).dot((np.array([x_v_N[1],y_v_N[1]])).reshape([2,1]))    
    for i in range(1,n):
        x_v_B[i] = x_v_B[i-1] + (x_a_B[i] + x_a_B[i-1])*dt/2
    	y_v_B[i] = y_v_B[i-1] + (y_a_B[i] + y_a_B[i-1])*dt/2

    # Define inertial sensor parameters "accelerate mpu9150"
    # ========================================================
    # gyroscope
    ####   fotmat=1   ####
    bz0=1*-4.37*d2r  

    #gyroscope(bias & noise)
    sig_arw_0 = 1*0.02                 
    sig_rrw_0 = 0.02/3600
    def Biasbg1():
	sig_rrw=sig_rrw_0*d2r
	nrrw=np.random.normal(0,sig_rrw,n)
	bz=np.zeros(m)
	bz[1]=bz0
	for i in range(0,n-1):
            bz[i+1]=bz[i]+nrrw[i]*dt
    Biasbg1()		
    #accelerometer (biases and noises)
    bx0=1*0.1*g
    by0=-0.1*g
    err_factor = 1.0
    sig_xr_0 = err_factor*0.1*g/3600
    sig_yr_0 = err_factor*0.1*g/3600         
 
    #accelerate (noise)
    sig_bx_0 = err_factor*0.1*g
    sig_by_0 = err_factor*0.1*g

    #accelerate (calculator bias)
    def Biasba1():
	global bx
    	sig_ar =sig_xr_0
    	nba=np.random.normal(0,sig_ar,n)
    	bx=np.zeros(m)
	bx[0]=bx0
	for i in range(0,n-1):
    	    bx[i+1]=bx[i]+nba[i]*dt
    Biasba1()	
    def Biasbp1():
	global by
    	sig_pr=sig_yr_0
	npb=np.random.normal(0,sig_pr,n)
	by=np.zeros(m)
	by[1]=by0
	for i in range(0,n-1):
            by[i+1]=by[i]+npb[i]*dt
    Biasbp1()		
    def transform_m():
    	sig_bx=sig_bx_0
	sig_by=sig_by_0
	nx=np.random.normal(0,sig_bx,n)
	ny=np.random.normal(0,sig_by,n)
	axm=np.zeros(m)
	aym=np.zeros(m)
	for i in range(0,n):
    	    axm[i]=x_a_B[i]+bx[i]+nx[i]
    	    aym[i]=y_a_B[i]+by[i]+ny[i]	

    transform_m()		
    # optical flow sensor model - in B-frame
    sig_bx=sig_bx_0 #(noise)
    sig_by=sig_by_0 #(noise)
    sig_xr=sig_xr_0 #(bias)
    sig_yr=sig_yr_0 #(bias)
    radiosensor_err_factor = 1.0
    sig_x_r=radiosensor_err_factor*0.1             
    sig_y_r=radiosensor_err_factor*0.1
    delta_t = dt                                 
    delta_s = 2*delta_t 
	
    def propagate_step():
	global sensor_step,propagation_step
    	t_simulation = T           
	simulation_step = t_simulation/delta_t
	sensor_step = t_simulation/delta_s
	propagation_step = simulation_step/sensor_step
    propagate_step()	
    # Define the initial conditions for the inertial sensors and the navigation
    # states
    #===============================================================
    # Introduce initial position and velocity estimate error in N-frame
    xverr = 0.1
    yverr = -0.1
    xperr = 0.5
    yperr = -0.5
    xaerr = 0
    yaerr = 0
    psierr = 1
	
    def initial_estimate_value8_radio():
	axm_h=np.zeros(m)
	aym_h=np.zeros(m)
	by_h=np.zeros(m)
	bx_h=np.zeros(m)
	xvm_Nh=np.zeros(m)
	yvm_Nh=np.zeros(m)
	xpm_Nh=np.zeros(m)
	ypm_Nh=np.zeros(m)
	xam_Nh=np.zeros(m)
	yam_Nh=np.zeros(m)
	#wzm_h=np.zeros(m)
	psi_h=np.zeros(m)
	bz_h=np.zeros(m)
	#axm_h[1]=axm[1]
	#aym_h[1]=aym[1]
	by_h[1]=0*9.8
	bx_h[1]=0*9.8
	xvm_Nh[1]=0
	yvm_Nh[1]=0
	#xpm_Nh[1]=position_x
	#ypm_Nh[1]=position_y
	xam_Nh[1]=0*x_a_N[1]
	yam_Nh[1]=0*y_a_N[1]
	#wzm_h[1]=wzm[1]
	psi_h[1]=0*d2r
	bz_h[1]=bz0
    initial_estimate_value8_radio()

    # Define the initial conditions for the 8-state Kalman Filter
    def define_initial_condition_8():
    	xz_h=np.zeros([8,1])
    	xz_h[2,0] = 0*bx0
    	xz_h[5,0] = 0*by0
    	xz_h[7,0] = bz0
    	P00_z = np.zeros([8,8])
    	P00_z[0,0] = xperr**2
	P00_z[2,2] = 1*bx0**2
	P00_z[3,3] = yperr**2
	P00_z[5,5] = 1*by0**2
	P00_z[6,6] = (1*psierr*d2r)**2
	P00_z[7,7] =100* bz0**2

    define_initial_condition_8()


    # for 8-state filter

    F_z=np.zeros([8,8])
    F_z[0,1] = 1
    F_z[3,4] = 1
    F_z[6,7] = -1
    Q_z=np.zeros([8,8])
    l=1
    k=2-1  #python
    for i in np.arange(0,sensor_step):
	for j in np.arange(0,propagation_step):
            k=1+j+(i-1)*propagation_step
       	    #bx_h[k]=bx_h[k-1]
            #by_h[k]=by_h[k-1]
            #bz_h[k]=bz_h[k-1]
    """
    def inertial_navigation_computation8_radio():
        #axm_h[k]=axm[k]-bx_h[k]
	#aym_h[k]=aym[k]-by_h[k]
	#axm_h[k-1]=axm[k-1]-bx_h[k-1]
	#aym_h[k-1]=aym[k-1]-by_h[k-1]
    	#wzm_h[k-1] = (1-scale_factor_err)*(wzm[k-1] - bz_h[k-1])
	#wzm_h[k] = (1-scale_factor_err)*(wzm[k] - bz_h[k-1])
	#psi_h[k] = psi_h[k-1] + (wzm_h[k-1]+wzm_h[k])*dt/2
	#if psi_h[k]>= 2*pi:
            #psi_h[k] = psi_h[k] - 2*pi
	#else:
            #psi_h[k] = psi_h[k]
        #xam_Nh[k] = np.cos(psi_h[k-1]).dot(axm_h[k-1])-np.sin(psi_h[k-1]).dot(aym_h[k-1])-wzm_h[k-1].dot(yvm_Nh[k-1])
	#yam_Nh[k] = np.sin(psi_h[k-1]).dot(axm_h[k-1])+np.cos(psi_h[k-1]).dot(aym_h[k-1])+wzm_h[k-1].dot(xvm_Nh[k-1])
	#xvm_Nh[k] = xvm_Nh[k-1]+(xam_Nh[k]+xam_Nh[k-1])*dt/2
	#yvm_Nh[k] = yvm_Nh[k-1]+(yam_Nh[k]+yam_Nh[k-1])*dt/2
	#xpm_Nh[k] = xpm_Nh[k-1]+(xvm_Nh[k]+xvm_Nh[k-1])*dt/2
	#ypm_Nh[k] = ypm_Nh[k-1]+(yvm_Nh[k]+yvm_Nh[k-1])*dt/2

    #inertial_navigation_computation8_radio()
    """
    def define_Dymamic_equation8_radio():
        #F_z[1,2] = -np.cos(psi_h[k-1])
	#F_z[1,4] = -wzm_h[k-1]
	#F_z[1,5] = np.sin(psi_h[k-1])
	#F_z[1,6] = 1*(-np.sin(psi_h[k-1]).dot(axm_h[k-1])-np.cos(psi_h[k-1]).dot(aym_h[k-1]))
	#F_z[1,7] = 1*(yvm_Nh[k-1])
	#F_z[4,1] = wzm_h[k-1]
	#F_z[4,2] = -np.sin(psi_h[k-1])
	#F_z[4,5] = -np.cos(psi_h[k-1])
	#F_z[4,6] = 1*(np.cos(psi_h[k-1]).dot(axm_h[k-1])-np.sin(psi_h[k-1]).dot(aym_h[k-1]))	
	#F_z[4,7] = 1*(-xvm_Nh[k-1])
        I=np.eye(8)
        #phi_z = I+(F_z*dt)
	#Q_z[1,1] = ((np.cos(psi_h[k-1]))**2)*sig_bx**2 + ((np.sin(psi_h[k-1]))**2)*sig_by**2 + yvm_Nh[k-1].dot(yvm_Nh[k-1])*sig_arw_0**2
	#Q_z[2,2] = sig_xr**2
	#Q_z[4,4] = ((np.sin(psi_h[k-1]))**2)*sig_bx**2 + ((np.cos(psi_h[k-1]))**2)*sig_by**2 + xvm_Nh[k-1].dot(xvm_Nh[k-1])*sig_arw_0**2
	#Q_z[5,5] = sig_yr**2
	#Q_z[6,6] = 1*sig_arw_0**2
	#Q_z[7,7] = 1*sig_rrw_0**2

    define_Dymamic_equation8_radio()  

    #def Kalman_Filter_estimate1_radio():
    	#xz_h=phi_z.dot(xz_h)
    	#P00_z=phi_z.dot(P00_z).dot(phi_z.T)+Q_z*dt

        #Kalman_Filter_estimate1_radio()
    """
    def radio_discrete_8_4_EKF():
        R1m_h[k] = sqrt((xr1-xpm_Nh[k])**2+(yr1-ypm_Nh[k])**2+(h_1-1.32)**2)
	R2m_h[k] = sqrt((xr2-xpm_Nh[k])**2+(yr2-ypm_Nh[k])**2+(h_2-1.32)**2)
	R3m_h[k] = sqrt((xr3-xpm_Nh[k])**2+(yr3-ypm_Nh[k])**2+(h_3-1.32)**2)
	R4m_h[k] = sqrt((xr4-xpm_Nh[k])**2+(yr4-ypm_Nh[k])**2+(h_4-1.32)**2)
        r1_partial_x =-(xr1-xpm_Nh[k])/R1m_h[k]
	r1_partial_y =-(yr1-ypm_Nh[k])/R1m_h[k]
	r2_partial_x =-(xr2-xpm_Nh[k])/R2m_h[k] 
	r2_partial_y =-(yr2-ypm_Nh[k])/R2m_h[k]
	r3_partial_x =-(xr3-xpm_Nh[k])/R3m_h[k]
	r3_partial_y =-(yr3-ypm_Nh[k])/R3m_h[k]
	r4_partial_x =-(xr4-xpm_Nh[k])/R4m_h[k] 
	r4_partial_y =-(yr4-ypm_Nh[k])/R4m_h[k]
  	H=np.array([r1_partial_x,0,0,r1_partial_y,0,0,0,0,r2_partial_x,0,0,r2_partial_y,0,0,0,0,r3_partial_x,0,0,r3_partial_y,0,0,0,0,r4_partial_x,0,0,r4_partial_y,0,0,0,0])
        H=H.reshape([4,8])
        R=100*[sig_x_r**2,0,0,0,0,sig_y_r**2,0,0,0,0,sig_x_r**2,0,0,0,0,sig_y_r**2]
        R=R.shape([4,4])
    """
    #radio_discrete_8_4_EKF()
    c=0.5
    """
    def Kalman_Filter_update_8_4_radio_v1():
	zxm_z1[k] = R1m[k]-R1m_h.T[k] 
	zxm_z2[k] = R2m[k]-R2m_h.T[k]
	zxm_z3[k] = R3m[k]-R3m_h.T[k] 
	zxm_z4[k] = R4m[k]-R4m_h.T[k]
	Mu_z=np.array([zxm_z1[k],zxm_z2[k],zxm_z3[k],zxm_z4[k]])
	Mu_z=Mu_z.reshape([4,1])
	z_update =np.zeros([8,1])		    	
	K_z=P00_z.dot(H.T)/(H.dot(P00_z).dot(H.T)+R)
	z_update = K_z.dot(Mu_z)
    	I=np.eye(8)
	P00_z=(I-K_z.dot(H)).dot(P00_z)   
    """
    #Kalman_Filter_update_8_4_radio_v1()	
    """
    def upon_radiosensor_measurement_8_2():
	xpm_Nh[k]=xpm_Nh[k]+z_update[0]
	xvm_Nh[k]=xvm_Nh[k]+z_update[1]
	bx_h[k] = bx_h[k] + z_update[2]
	ypm_Nh[k]=ypm_Nh[k]+z_update[3]
	yvm_Nh[k]=yvm_Nh[k]+z_update[4]
	by_h[k] = by_h[k] + z_update[5]
	psi_h[k]=psi_h[k]+z_update[6]
	bz_h[k]=bz_h[k]+z_update[7]	
    """
    #upon_radiosensor_measurement_8_2()
    xz_h=np.zeros([8,1])
    l = l+1
    m1 = 12500
    m2 = k-1
    """
    bxerrave = mean((bx(m1:m2)-bx_h(m1:m2))/g);
    byerrave = mean((by(m1:m2)-by_h(m1:m2))/g);
    bzerrave = mean((bz(m1:m2)-bz_h(m1:m2))*r2d);
    bxerrstd = std((bx(m1:m2)-bx_h(m1:m2))/g);
    byerrstd = std((by(m1:m2)-by_h(m1:m2))/g);
    bzerrstd = std((bz(m1:m2)-bz_h(m1:m2))*r2d);
    bxerr(ii) = abs(bxerrave) + 3*bxerrstd;
    byerr(ii) = abs(byerrave) + 3*byerrstd;
    bzerr(ii) = abs(bzerrave) + 3*bzerrstd;
    """




"""
for k1 in range(1,100):
    xperr[k1] = xperrave[k1] + 3*xperrstd[k1]
    yperr[k1] = yperrave[k1] + 3*yperrstd[k1]
"""
