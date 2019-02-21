
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

GPIO.add_event_detect(pin1, GPIO.RISING, callback=EI_R)
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












