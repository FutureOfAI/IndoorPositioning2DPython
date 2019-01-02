import serial
import math
import operator
import time
import socket
import os.path
import os

DST_IP = "127.0.0.3"
DST_PORT = 5005

MON_IP = "127.0.0.6"
MON_PORT = 5005

mon = 0

# initialize for VDR
five = 0
setx_total = 0
setx_run = [0] * 5
sety_total = 0
sety_run = [0] * 5
drift_total = 0
drift_run = [0] * 5

# initialize for VLW
vlwfirst = 1
vlwinit = 0.0

# initialize log
log = time.time()
f = open('dstraw', 'w')
f.write("Last 60 Seconds DST Raw Input:\r\n")
f.close()

ser = serial.Serial('/dev/dst', 4800, timeout=5)

while True:

    # health monitor
    hack = time.time()
    if hack - mon > .5:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(str(hack), (MON_IP, MON_PORT))
        mon = hack

    # log raw input
    dst_raw = ser.readline()
    f = open('dstraw', 'a')
    f.write(dst_raw)
    f.close()
    if hack - log > 60:
        os.remove('dstraw')
        f = open('dstraw', 'w')
        f.write("Last 60 Seconds DST Raw Input:\r\n")
        f.close()
        log = hack
    
    # checking to see if it's a valid NMEA sentence
    if "*" in dst_raw:
        dst_split = dst_raw.split('*')
        dst_sentence = dst_split[0].strip('$')
        cs0 = dst_split[1][:-2]
        cs = format(reduce(operator.xor,map(ord,dst_sentence),0),'X')
        if len(cs) == 1:
            cs = "0" + cs

        # if it is a valid NMEA sentence
        if cs0 == cs:
            dst_vars = dst_sentence.split(',')
            title = dst_vars[0]

            # depth sentence
            if title == "SDDPT":

                # roll and pitch from imu
                try:
                    f = open('imu_bus', 'r')
                    line = f.readline()
                    f.close()
                    imu_split = line.split(',')
                    imu_hack = float(imu_split[0])
                    roll = float(imu_split[2])
                    pitch = float(imu_split[3])
                    
                except ValueError:
                    time.sleep(.03)
                    f = open('imu_bus', 'r')
                    line = f.readline()
                    f.close()
                    imu_split = line.split(',')
                    imu_hack = float(imu_split[0])
                    roll = float(imu_split[2])
                    pitch = float(imu_split[3])

                # correct depth for 23 degree offset from centerline, but if roll/pitch
                # are from the last 3 seconds, correct depth for attitude
                depth = round(float(dst_vars[1])*math.cos(math.radians(23)),1)
                if time.time() - imu_hack < 3.0:
                   depth = round(float(dst_vars[1])*math.cos(math.radians(23-roll))*math.cos(math.radians(pitch)),1)

                # assemble the sentence with .5 meter offset from waterline
                dpt = "SDDPT," + str(depth) + ",0.50"
                dptcs = format(reduce(operator.xor,map(ord,dpt),0),'X')
                if len(dptcs) == 1:
                    dptcs = "0" + dptcs
                sddpt = "$" + dpt + "*" + dptcs + "\r\n"

                # to kplex
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(sddpt, (DST_IP, DST_PORT))

            # mean water temp sentence
            elif title == "YXMTW":

                # write to bus
                mtw = dst_vars[1]
		f = open('dst_bus', 'w')
		f.write(mtw)
		f.close()

                # to kplex
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(dst_raw, (DST_IP, DST_PORT))

            # vessel waterspeed sentence and current set and drift
            elif title == "VWVHW":

                # heading and roll from imu
                try:
                    f = open('imu_bus', 'r')
                    line = f.readline()
                    f.close()
                    imu_split = line.split(',')
                    imu_hack = float(imu_split[0])
                    heading = float(imu_split[1])
                    roll = float(imu_split[2])
                except ValueError:
                    f.close()
                    time.sleep(.03)
                    f = open('imu_bus', 'r')
                    line = f.readline()
                    f.close()
                    imu_split = line.split(',')
                    imu_hack = float(imu_split[0])
                    heading = float(imu_split[1])
                    roll = float(imu_split[2])                

                # course and groundspeed from gps
                try:
                    f = open('gps_bus', 'r')
                    line = f.readline()
                    gps_split = line.split(',')
                    f.close()
                    gps_hack = float(gps_split[0])
                    valid = gps_split[1]
                    course = float(gps_split[2])
                    groundspeed = float(gps_split[3])
                except ValueError:
                    time.sleep(.03)
                    f = open('gps_bus', 'r')
                    line = f.readline()
                    f.close()
                    gps_split = line.split(',')
                    gps_hack = float(gps_split[0])
                    valid = gps_split[1]
                    course = float(gps_split[2])
                    groundspeed = float(gps_split[3])

                # calculate corrected waterspeed from heel and velocity
                waterspeed = float(dst_vars[5])
                sensor_angle = 23.0
                if time.time() - imu_hack < 3.0:
                    sensor_angle = math.fabs(23.0-roll)
                five_knot_correction = -.02 * sensor_angle
                ten_knot_correction = sensor_angle * (.035 - .0065 * sensor_angle) - 2
                if sensor_angle > 10.0:
                    ten_knot_correction = -.03 * sensor_angle - 2
                if waterspeed < 5.0:
                    waterspeed = round(waterspeed + (five_knot_correction * waterspeed / 5), 1)
                else:
                    waterspeed = round((waterspeed + (waterspeed * (ten_knot_correction * (waterspeed - 5) - 2 * five_knot_correction * (waterspeed - 10))) / 50),1)

                # assemble the sentence
                vhw = "VWVHW,"
                if time.time() - imu_hack < 3.0:
                    vhw = vhw + str(int(heading))
                else: vhw = vhw + ''
                vhw = vhw + ",T,,M," + str(waterspeed) + ",N,,K"
                vhwcs = format(reduce(operator.xor,map(ord,vhw),0),'X')
                if len(vhwcs) == 1:
                    vhwcs = "0" + vhwcs
                vwvhw = "$" + vhw + "*" + vhwcs + "\r\n"

                # to kplex
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(vwvhw, (DST_IP, DST_PORT))

                # VDR set and drift
                if time.time() - imu_hack < 3.0 and time.time() - gps_hack < 3.0 and valid == "A":
                    
                    heading_radians = math.radians(heading)
                    course_radians = math.radians(course)
                    set0 = course_radians - heading_radians
                    if set0 < 0:
                        set0 = set0 + 2 * math.pi
                    set_relative = math.pi - math.atan2(groundspeed * math.sin(set0), waterspeed - groundspeed * math.cos(set0))
                    if waterspeed == 0 and groundspeed == 0:
                        set_relative = set0
                    set_radians = heading_radians + set_relative
                    if set_radians > (2 * math.pi):
                        set_radians = set_radians - (2 * math.pi)
                    drift = math.sqrt(pow(waterspeed,2) + pow(groundspeed,2) - 2 * waterspeed * groundspeed * math.cos(set0))
                    
                    # dampen out set and drift over the last five readings
                    setx_total = setx_total - setx_run[five]
                    setx_run[five] = math.cos(set_radians)
                    setx_total = setx_total + setx_run[five]
                    setx_ave = setx_total / 5
                    sety_total = sety_total - sety_run[five]
                    sety_run[five] = math.sin(set_radians)
                    sety_total = sety_total + sety_run[five]
                    sety_ave = sety_total / 5
                    set_radians = math.atan2(sety_ave, setx_ave)
                    if set_radians < 0:
                        set_radians = set_radians + 2 * math.pi
                    set_true = math.degrees(set_radians)
                    set_apparent = set_true - heading
                    if set_apparent < 0:
                        set_apparent = set_apparent + 360

                    drift_total = drift_total - drift_run[five]
                    drift_run[five] = drift
                    drift_total = drift_total + drift_run[five]
                    drift = drift_total / 5
                    five = five + 1
                    if five > 4:
                        five = 0
                    
                    # assemble the sentence
                    vdr = "IIVDR," + str(int(set_true)) + ",T,,M," + str(round(drift,1)) + ",N"
                    vdrcs = format(reduce(operator.xor,map(ord,vdr),0),'X')
                    if len(vdrcs) == 1:
                        vdrcs = "0" + vdrcs
                    iivdr = "$" + vdr + "*" + vdrcs + "\r\n"

                    # to kplex
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.sendto(iivdr, (DST_IP, DST_PORT))

            # voyage log sentence
            elif title == "VWVLW":

                # calculate present trip total vs overall total
                if vlwfirst == 1:
                    vlwinit = dst_vars[1]
                    vlwfirst = 0
                trip = float(dst_vars[1]) - float(vlwinit)
                vlw = "VWVLW," + dst_vars[1] + ",N," + str(trip) + ",N"
                cs = format(reduce(operator.xor,map(ord,vlw),0),'X')
                if len(cs) == 1:
                    cs = "0" + cs
                vwvlw = "$" + vlw + "*" + cs  + "\r\n"

                # to kplex
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(vwvlw, (DST_IP, DST_PORT))
	  
	    # if it's any other valid NMEA sentence
            else:

                # to kplex
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(dst_raw, (DST_IP, DST_PORT))
