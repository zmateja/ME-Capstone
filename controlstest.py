import RPi.GPIO as GPIO

import gpiozero
import time, threading
try:
	GPIO.setwarnings(False)
	
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
	
	Kp = 3
	Ki = 0.1
	
	MOTOR_L = 80.0
	MOTOR_R = 80.0

	AUX1 = 0
	AUX2 = 0
	
	pin1 = 4
	pin2 = 17
	pin3 = 2
	pin4 = 3
	
	frequency = 20

	left_motor = gpiozero.Motor("BOARD12","BOARD13", None, True, None)
	right_motor = gpiozero.Motor("BOARD8","BOARD10", None, True, None)
	
	GPIO.setup(pin1, GPIO.IN)
	GPIO.setup(pin2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(pin3, GPIO.IN)
	GPIO.setup(pin4, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	
	
	def ei_l(pin4):
	    global counter_L
	
	    if(GPIO.input(pin3) == True):
	        counter_L-=1
	    else:
	        counter_L+=1

	def ei_r(pin2):
	    global counter_R
	
	    if(GPIO.input(pin1) == True):
	        counter_R-=1
	    else:
	        counter_R+=1
	
	
	GPIO.add_event_detect(pin4, GPIO.RISING, callback=ei_l)
	GPIO.add_event_detect(pin2, GPIO.RISING, callback=ei_r)
	
	def timer_interrupt():
	    
	    global counter_L
	    global lastcounter_L
	    global Dcounter_L
	    global RPM_L
	    global e_L
	    global e_sum_L
	    global RPM_controller_L
	    global PWM_controller_L

	    global counter_R
	    global lastcounter_R
	    global Dcounter_R
	    global RPM_R
	    global e_R
	    global e_sum_R
	    global RPM_controller_R
	    global PWM_controller_R
		
	    global MOTOR_L
	    global MOTOR_R
	
	    if AUX1 == 1:                              #If vehicle is e-stopped, reset all of the counters and errors
	        counter_L = 0
	        lastcounter_L = 0
	        e_L = 0
	        e_sum_L = 0
	        MOTOR_L=0
	        
	    else:
	        #Calculate Left RPM
	        Dcounter_L = counter_L - lastcounter_L
	        RPM_L = (Dcounter_L*frequency*60.0)/5000    #  500hz / 400 clicks per second / 50 gear ratio * 60 seconds
	        lastcounter_L = counter_L
		print("Left:")
	        print(RPM_L)
	
		#PI Controller Left
		#The integral saturation filters makes sure our error sum doesn't go off to infinity if our wheels get stuck on something.
		e_L = MOTOR_L - RPM_L
		e_sum_L = e_sum_L + e_L
		if e_sum_L > 5000:
			e_sum_L = 5000 # Integral Saturation Filter
	
		if e_sum_L < -5000:
			e_sum_L = -5000
		#print(e_sum_L)

		RPM_controller_L = Kp * e_L + Ki * e_sum_L
	
		if RPM_controller_L > 0:
			PWM_controller_L = (0.0099 * RPM_controller_L*RPM_controller_L - 0.1735*RPM_controller_L)*(7.2/12)
		else:
			PWM_controller_L = (-0.0099 * RPM_controller_L*RPM_controller_L  - 0.1735*RPM_controller_L)*(7.2/12)
	
		#print(PWM_controller_L)
	
		#Motor Input Saturation Filter
		#PWM signals range from 0 to 255 so we saturate our motor controller output to accomodate that.
		#We account for the negative PWM values using the if/else cases below.
	
		#range is from 0 to 100 for Python's duty cycle
		if PWM_controller_L > 255:
			PWM_controller_L = 1.0
	
		elif PWM_controller_L < -255:
			PWM_controller_L = -1.0
	
		else:
			PWM_controller_L = (PWM_controller_L/255.0)
	
		#Left Motor Commands
		#If desired motor speed (from the Pi) is given as a zero, we are going to bypass the PI controller.
		#The PI controller will constantly try to apply current to the motor if we don't do this which will
		#1) waste power and 2) make an annoying high-pitch noise.
		#https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
		
		#print(PWM_controller_L)
	
		if MOTOR_L == 0:
			left_motor.stop()

		else:
			if PWM_controller_L > 0:
				left_motor.forward(PWM_controller_L)
			elif PWM_controller_L == 0:
				left_motor.stop()
			else:
				left_motor.backward(abs(PWM_controller_L))

	    if AUX2 == 1:                              #If vehicle is e-stopped, reset all of the counters and errors
	        counter_R = 0
	        lastcounter_R = 0
	        e_R = 0
	        e_sum_R = 0
	        MOTOR_R=0
	        
	    else:
	        #Calculate Left RPM
	        Dcounter_R = counter_R - lastcounter_R
	        RPM_R = (Dcounter_R*frequency*60)/5000.0       #  500hz / 400 clicks per second / 50 gear ratio * 60 seconds
	        lastcounter_R = counter_R
		
		print("Right:")
		print(RPM_R)
	
		#PI Controller Left
		#The integral saturation filters makes sure our error sum doesn't go off to infinity if our wheels get stuck on something.
		e_R = MOTOR_R - RPM_R
		e_sum_R = e_sum_R + e_R
		if e_sum_R > 5000:
			e_sum_R = 5000 # Integral Saturation Filter
	
		if e_sum_R < -5000:
			e_sum_R = -5000
		#print(e_sum_R)
		#print(e_R)
		RPM_controller_R = Kp * e_R + Ki * e_sum_R
	
		if RPM_controller_R > 0:
			PWM_controller_R = (0.0099 * RPM_controller_R*RPM_controller_R - 0.1735*RPM_controller_R)*(7.2/12)
		else:
			PWM_controller_R = (-0.0099 * RPM_controller_R*RPM_controller_R  - 0.1735*RPM_controller_R)*(7.2/12)
	
		#print(PWM_controller_R)
	
		#Motor Input Saturation Filter
		#PWM signals range from 0 to 255 so we saturate our motor controller output to accomodate that.
		#We account for the negative PWM values using the if/else cases below.
	
		#range is from 0 to 1.0 for Python's duty cycle
		if PWM_controller_R > 255:
			PWM_controller_R = 1.0
	
		elif PWM_controller_R < -255:
			PWM_controller_R = -1.0
	
		else:
			PWM_controller_R = (PWM_controller_R/255.0)
	
		#Left Motor Commands
		#If desired motor speed (from the Pi) is given as a zero, we are going to bypass the PI controller.
		#The PI controller will constantly try to apply current to the motor if we don't do this which will
		#1) waste power and 2) make an annoying high-pitch noise.
		#https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
		
	
		#print(PWM_controller_R)
	
		if MOTOR_R == 0:
			right_motor.stop()

		else:
			if PWM_controller_R > 0:
				right_motor.forward(PWM_controller_R)
			elif PWM_controller_R == 0:
				right_motor.stop()
			else:
				right_motor.backward(abs(PWM_controller_R))
	    threading.Timer(1.0/frequency, timer_interrupt).start()
	threading.Timer(1.0/frequency, timer_interrupt).start()
	
	exit()
except KeyboardInterrupt:
	GPIO.cleanup()
	raise
