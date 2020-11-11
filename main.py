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



##### Main Loop


