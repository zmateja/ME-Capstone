#   UGV Complete Code V1.0
#   Jonathan Nikolaidis and Zach Mateja
#   Villanova University HAV Lab
#   01/29/19
#   UGV/QUAD V1
#   Debugging purpose


import RPI.GPIO as GPIO
import time, threading

GPIO.setmode(GPIO.BCM)
pin1 = 1
pin2 = 2
pin3 = 3
pin4 = 4

counter_L = 0
lastcounter_L = 0
Dcounter_L = 0
RPM_L = 0.0
e_L = 0.0
e_sum_L = 0.0
RPM_controller_L = 0.0
PWM_controller_L = 0.0

counter_R = 0
lastcounter_R = 0
Dcounter_R = 0
RPM_R = 0.0
e_R = 0.0
e_sum_R = 0.0
RPM_controller_R = 0.0
PWM_controller_R = 0.0

Kp = 15.0
Ki = 0.6

buf[255]
buf_offset = 0
MOTOR_L = 0
MOTOR_R = 0
AUX1 = 0
AUX2 = 0

# ***********************************************************************************************************
#                                        Timer Interrupt Setup Start
# ***********************************************************************************************************

# turn on the timer clock in the power management controller https://forum.arduino.cc/index.php?topic=130423.15
pmc_set_writeprotect(false) # disable write protection for pmc registers
pmc_enable_periph_clk(ID_TC1)  # enable peripheral clock TC1

# we want wavesel 01 with RC
# TC_Configure(clock ,channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
TC_Configure(TC0, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4)
#  TC_SetRC(TC2, 1, 656000);
#  TC_SetRC(TC0, 1, 13120);                  # 50Hz
#  TC_SetRC(TC0, 1, 6560);                  # 100Hz
TC_SetRC(TC0, 1, 2624)  # 250Hz
TC_Start(TC0, 1)

# enable timer interrupts on the timer
#TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS  # IER = interrupt enable register
#TC0->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS  # IDR = interrupt disable register

# Enable the interrupt in the nested vector interrupt controller
# TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1
NVIC_EnableIRQ(TC1_IRQn)
# ***********************************************************************************************************
#                                          Timer Interrupt Setup End
# ***********************************************************************************************************

# External Interrupt Setup for left and right encoders
# Each interrupt (L and R) will execute at the rising edge of the incoming signal
# Define our motor pins as outputs

GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pin3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(pin4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(pin1, GPIO.RISING, callback=ei_r)
GPIO.add_event_Detect(pin2, GPIO.RISING, callback=ei_l)

def ei_l():
    if GPIO.INPUT(pin3):
        counter_L+=1
    elif GPIO.INPUT(pin3)==GPIO.LOW:
        counter_L-=1

def ei_r():
    if GPIO.INPUT(pin4):
        counter_R+=1
    elif GPIO.INPUT(pin4)==GPIO.LOW:
        counter_R-=1


void loop()
{
# ***********************************************************************************************************
#                                        Communications Start
#*********************************************************************************************************** 
  #Read RPM from Serial
  static char* channels[4] = {0, 0, 0, 0};
  int bytesAvail = Serial.available();
  while (bytesAvail > 0)
  {
    char mc = Serial.read();
    buf[buf_offset] = mc;

    if (mc == '@')
    {
      buf[buf_offset] = '\0';       #Replace current byte in buffer with NULL terminator
      char *str = strtok(buf, "*"); #Find subset of buf that ends with a '*'
      char *ch = strtok(str, ",");  #Find first subset of str terminated by ','
      channels[0] = ch;             #Save value for motor command

      for (int i = 1; i < 4; i++)   #Finds remaining subsets of str
      {
        char *ch = strtok(NULL, ",");
        channels[i] = ch;
      }

      MOTOR_L = atof(channels[0]);   #Convert motor command to floats
      MOTOR_R = atof(channels[1]);
      AUX1 = atof(channels[2]);
      AUX2 = atof(channels[3]);

      buf_offset = 0;               #Reset buffer index
      }

    else if (mc != '\r')            #If current byte has data, increment buf_offset
    {
      buf[buf_offset++] = mc;
    }

    bytesAvail--;                   #Decrement bytesAvail
  }

# ***********************************************************************************************************
#                                            Communications Finish
#  *********************************************************************************************************** 

#      HOW THE SEND COMMUNICATION PROTOCOL WORKS:
#      If we successfully read from the Pi, we set flag = 1. If we have a successful
#      read then we send back a string to the Pi.
#      "*" signifies the start of the string and "*@" denotes the end.
#    The flag is then reset and we wait for the next successful read

}

#***********************************************************************************************************
#                                         Timer Interrupt Function Start
#*********************************************************************************************************** 
# This part of the code will execute at the frequency selected in the timer interrupt setup

def tc1_handler:
	TC_GetStatus(TC0, 1);

	if AUX1 == 1:                              #If vehicle is e-stopped, reset all of the counters and errors
		counter_L = 0
		lastcounter_L = 0
		e_L = 0
		e_sum_L = 0
		MOTOR_L=0

		counter_R = 0
		lastcounter_R = 0
		e_R = 0
		e_sum_R = 0
		MOTOR_R=0
	break

	#Calculate Left RPM
	Dcounter_L = counter_L - lastcounter_L
	RPM_L = Dcounter_L * 3.0/ 4          #  500hz / 400 clicks per second / 50 gear ratio * 60 seconds
	lastcounter_L = counter_L

	#Calculate Right RPM
	Dcounter_R = counter_R - lastcounter_R
	RPM_R = Dcounter_R * 3.0/ 4          #  500hz / 400 clicks per second / 50 gear ratio * 60 seconds
	lastcounter_R = counter_R

	#PI Controller Left
	#The integral saturation filters makes sure our error sum doesn't go off to infinity if our wheels get stuck on something.
	e_L = MOTOR_L - RPM_L
	e_sum_L = e_sum_L + e_L
	if e_sum_L > 5000:
		e_sum_L = 5000 # Integral Saturation Filter

	if e_sum_L < -5000:
		e_sum_L = -5000

	RPM_controller_L = Kp * e_L + Ki * e_sum_L

	if RPM_controller_L > 0:
		PWM_controller_L = (0.0099 * RPM_controller_L*RPM_controller_L - 0.1735*RPM_controller_L)*(7.2/12)
	else:
		PWM_controller_L = (-0.0099 * RPM_controller_L*RPM_controller_L  - 0.1735*RPM_controller_L)*(7.2/12)

	#PI Controller Right
	e_R = MOTOR_R - RPM_R
	e_sum_R = e_sum_R + e_R
	if e_sum_R > 5000:
		e_sum_R = 5000 # Integral Saturation Filter

	if e_sum_R < -5000:
		e_sum_R = -5000

	RPM_controller_R = Kp * e_R + Ki * e_sum_R

	if RPM_controller_R > 0:
		PWM_controller_R = (0.0099 * RPM_controller_R*RPM_controller_R - 0.1735*RPM_controller_R)*(7.2/12)
	else:
		PWM_controller_R = (-0.0099 * RPM_controller_R*RPM_controller_R - 0.1735*RPM_controller_R)*(7.2/12)

	#Motor Input Saturation Filter
	#PWM signals range from 0 to 255 so we saturate our motor controller output to accomodate that.
	#We account for the negative PWM values using the if/else cases below.

	#range is from 0 to 100 for Python's duty cycle
	if PWM_controller_L > 255:
		PWM_controller_L = 100

	if PWM_controller_L < -255:
		PWM_controller_L = -100

	if -255 <= PWM_controller_L <= 255
		PWM_controller_L = (PWM_controller_L/255)*100

	if PWM_controller_R > 255:
		PWM_controller_R = 100

	if PWM_controller_R < -255:
		PWM_controller_R  = 100

	if -255 <= PWM_controller_R <= 255
		PWM_controller_R = (PWM_controller_R/255)*100


	#Left Motor Commands
	#If desired motor speed (from the Pi) is given as a zero, we are going to bypass the PI controller.
	#The PI controller will constantly try to apply current to the motor if we don't do this which will
	#1) waste power and 2) make an annoying high-pitch noise.
	#https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
	p1 =GPIO.PWM(channel(10), frequency) # change channel(10) to the correct pin
	p2 =GPIO.PWM(channel(3), frequency)
	p3 =GPIO.PWM(channel(9), frequency)
	p4 =GPIO.PWM(channel(5), frequency)


	if MOTOR_L == 0:
		p1.start(0)
		p2.start(0)
	else:
		if PWM_controller_L > 0:
			p1.start(0)
			p1.start(abs(PWM_controller_L))
		elif PWM_controller_L == 0:
			p1.start(0)
			p2.start(0)
		elif PWM_controller_L < 0:
			p2.start(0)
			p2.start(abs(PWM_controller_L))

	#Right Motor Commands
	if MOTOR_R == 0:
		p3.start(0)
		p4.start(0)
	else:
		if PWM_controller_R > 0:
			p3.start(0)
			p4.start(abs(PWM_controller_R))
		elif PWM_controller_R == 0:
			p3.start(0)
			p4.start(0)
		elif PWM_controller_R < 0:
			p4.start(0)
			p3.start(abs(PWM_controller_R))
	print(time.ctime())
	threading.Timer(.004, tc1_handler).start()

#***********************************************************************************************************
#                                         Timer Interrupt Function Finish
#*********************************************************************************************************** 


#***********************************************************************************************************
#                                         External Interrupt Start
#*********************************************************************************************************** 
# This is where we read in the pulses from the encoder channels.
# The encoder is passed to a decoder chip which outputs two channels:
#      1) a square wave which corresponds to the #clicks
#      2) a HIGH-LOW signal which identifies the motor's direction
# We look at the first channel to determine when our encoder reads a click. This channel
# signals an interrupt every time we detect a rising edge (see External Interrupt Setup).
# Once we determine the encoder read a click, we ask it if it's going forward or backwards.

	
def ei_l():
    if GPIO.INPUT(pin3):
        counter_L+=1
    elif GPIO.INPUT(pin3)==GPIO.LOW:
        counter_L-=1

def ei_r():
    if GPIO.INPUT(pin4):
        counter_R+=1
    elif GPIO.INPUT(pin4)==GPIO.LOW:
        counter_R-=1
# ***********************************************************************************************************
#                                         External Interrupt Finish
# *********************************************************************************************************** 


