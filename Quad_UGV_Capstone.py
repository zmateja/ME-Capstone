from struct import*

import socket                   #UDP Communication
import time
import serial                   #Serial Communication

UDP_IP=""                       #Local IP Port
UDP_IP_PC="192.168.10.1"        #PC IP Address
UDP_PORT=25000                  #IP Port to Receive message from PC

MotorL=0
MotorR=0
AUX1=0
AUX2=0
RPMs=[0,0,1,0]                  #Initation

sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT))
sock.setblocking(0)             #Set up a non-blocking UDP Port receiving message from PC


time.sleep(5)    #Time to open Serial port. Very Important!

while 1:
    try:                                        #Try to receive Motor Commands from PC
        data,PCaddr=sock.recvfrom(8)            # CHANGE (ZACH)
        RevData=unpack('hhhh',data)
    except:                                     #Catch if the communication misclick
        RevData=[MotorL,MotorR,AUX1,AUX2]

    FlagData=unpack('hh',Quaddata)
    Red_LED_flag=FlagData[0]
    Green_LED_flag=FlagData[1]                  #Unpack LED Flags
    if RevData[2]==1:
        MotorL=0
        MotorR=0
        AUX1=1
        AUX2=0                                  #E-stop
    else:
        MotorL=RevData[0]
        MotorR=RevData[1]
        AUX1=RevData[2]
        AUX2=RevData[3]

