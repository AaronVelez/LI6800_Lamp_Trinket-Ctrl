# Name: main.py
# Created: 10/11/2020
# Authors: Juan de Dios Moreno González y Aarón I. Vélez Ramírez
# Universidad Nacional Autónoma de México

# Code for temperature control of LI6800 custum LED lamp

# Details:
# Runs in an Arduino-compatible Trinket board with an ARM Cortex M0+ processor
# It measures LED temperature using a thermistor
# It controls a 4-wire fan via a 25kHz PWM signal
# PWM duty cycle is set by a PID algorithm to control LED temperaure
# Temperature setpoint is encoded in an analog signal from LI6800 console
# Fan speed is read from fan tachometer
# Fan speed is encoded in an analog singal and sent to LI6800


##### Module imports
import board
import digitalio
import analogio
import time



##### Pin definitions
# 4-wire fan PWM control pin
Fan_PWM_pin = DigitalInOut(board.D4)
Fan_PWM_pin.direction = Direction.OUTPUT

# 4-wire fan tachometer 
Fan_Tach_pin = DigitalInOut(board.D3)
Fan_Tach_pin.direction = Direction.INPUT
Fan_Tach_pin.pull = Pull.UP #### NEEDS ATENTION. I DO NOT KNO WIF IT SHOULD BE UP OR DOWN!!!!!!!!!!!!!!!!!!!!

# Analog LED temperature setpoint
LED_Temp_Ctrl_pin = AnalogIn(board.A4)

# Analog LED temperature read
LED_Temp_pin = AnalogIn(board.A1)

# Analog fan speed
Fan_Speed_pin = AnalogOut(board.A0)



##### Constants definitions
# Reference voltge.
# The real board voltage might differ from the nominal 3.3V. For better measurements, enter thevalue measured with a calibrated multimeter.
Ref_voltage = 3.3

# Thermistor constants
# Juan, please add here your constants
Div_R = 3.32 # Value of the voltage divider resistor in kohms
# add all the constants for the termistor....

# PID gains
Kp = 0.00001 
Ki = 0.002
Kd = 0.000001



##### Variable definitions
Fan_PWM_duty-cycle = 0
Fan_Tach_rpm = 0
LED_Temp_Ctrl_voltage = 0.0
LED_Temp_voltage = 0.0
LED_Temp_Cdeg =0.0
Fan_Speed_voltage = 0.0



##### Functions
# PID function attributed to https://github.com/jckantor/CBE30338
def PID(Kp, Ki, Kd, MV_bar=0):
    # initialize stored data
    e_prev = 0
    t_prev = -100
    I = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, PV, SP = yield MV
        
        # PID calculations
        e = SP - PV
        
        P = Kp*e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)
        
        MV = MV_bar + P + I + D
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t


##### Setup
# Create and initialize PID control
PID_fan = PID(Kp, Ki, Kd)
PID_fan.send(None) 



##### Main Loop
##### It runs forever
while True:
  # Step 1. Read LED temperature setpoin defined by LI6800.
  # 1.1 Read analog voltage proportional to desired LED temperature at LED_Temp_Ctrl_pin
  # 1.2 Convert it to °C using constant (needs to be added to constants)
  
  
  # Step 2. Read LED temperature
  # 2.1 Read analog voltage at LED_Temp_pin
  # 2.2 Translate read voltage to Thermistor resistance
  # 2.3 Translate thermistor resistance to LED temperature


  # Step 3. Read fan speed and send it to LI6800
  # 3.1 Read digital pulses at Fan_Tach_pin
  # 3.2 Calculate fan speed in rpm
  # 3.3 Convert it to analog voltage value using constant (needs to be added to constants)
  # 3.4 Write analog voltage value to Fan_Speed_pin
  
  
  # Step 4. Run PID control algorithm.
  # It calculates the required fan speed (in PWM duty cycle values) to achive the desired LED temperature taking into consideration the current and near past LED temperature.
 
  
  # Step 5. Set new Fan speed
  # 5.1 Write to Fan_PWM_pin the new PWM duty cycle calculated by the PID algorithm
  
  
  # Step 6. Wait or no wait before executing the loop again (to be defined later).
