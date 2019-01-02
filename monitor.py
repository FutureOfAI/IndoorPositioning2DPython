#!/usr/bin/env python

import serial
import operator
import time
import os
import sys
import socket
import select

# change the working directory to the scripts folder
os.chdir("home/pi/kts/scripts")

# log and begin instrument scripts
log = open('log', 'w')
log.write("Monitor Initialized\r\n")
log.write("Starting GPS...\r\n")
log.close()
os.system("python gps.py &")
log = open('log', 'a')
log.write("Starting IMU...\r\n")
log.close()
os.system("python imu.py &")
log = open('log', 'a')
log.write("Starting DST...\r\n")
log.close()
os.system("python dst.py &")
log = open('log', 'a')
log.write("Starting BME...\r\n")
log.close()
os.system("python bme.py &")
log = open('log', 'a')
log.write("Starting kplex...\r\n")
log.close()
os.system("sudo kplex &")

GPS_IP = "127.0.0.4"
GPS_PORT = 5005
gpssock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gpssock.bind((GPS_IP, GPS_PORT))

IMU_IP = "127.0.0.5"
IMU_PORT = 5005
imusock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
imusock.bind((IMU_IP, IMU_PORT))

DST_IP = "127.0.0.6"
DST_PORT = 5005
dstsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dstsock.bind((DST_IP, DST_PORT))

BME_IP = "127.0.0.8"
BME_PORT = 5005
bmesock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
bmesock.bind((BME_IP, BME_PORT))

log = open('log', 'a')
log.write("Starting Loop\r\n------------------------\r\n")
log.close()

gps_hack = time.time()
imu_hack = time.time()
dst_hack = time.time()

while True:

    # monitor gps.py
    gpsready = select.select([gpssock], [], [], .1)
    if gpsready [0]:
        data, addr = gpssock.recvfrom(1024)
        gps_hack = float(data)
    if time.time() - gps_hack > 10.0:
        log = open('log', 'a')
        log.write("Restarting GPS...\r\n")
        log.close()
        os.system("pkill -9 -f gps.py")
        os.system("python gps.py &")
        gps_hack = time.time()        

    # monitor imu.py
    imuready = select.select([imusock], [], [], .1)
    if imuready [0]:
        data, addr = imusock.recvfrom(1024)
        imu_hack = float(data)
    if time.time() - imu_hack > 10.0:
        log = open('log', 'a')
        log.write("Restarting IMU...\r\n")
        log.close()
        os.system("pkill -9 -f imu.py")
        os.system("python imu.py &")
        imu_hack = time.time()

    # monitor dst.py
    dstready = select.select([dstsock], [], [], .1)
    if dstready [0]:
        data, addr = dstsock.recvfrom(1024)
        dst_hack = float(data)
    if time.time() - dst_hack > 10.0:
        log = open('log', 'a')
        log.write("Restarting DST...\r\n")
        log.close()
        os.system("pkill -9 -f dst.py")
        os.system("python dst.py &")
        dst_hack = time.time()

    # monitor bme.py
    bmeready = select.select([bmesock], [], [], .1)
    if bmeready [0]:
        data, addr = bmesock.recvfrom(1024)
        bme_hack = float(data)
    if time.time() - bme_hack > 60:
        log = open('log', 'a')
        log.write("Restarting BME...\r\n")
        log.close()
        os.system("pkill -9 -f bme.py")
        os.system("python bme.py &")
        bme_hack = time.time()
