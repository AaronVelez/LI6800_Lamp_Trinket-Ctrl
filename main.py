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
import math


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

a = [3.3538646E-03, 2.56544090E-04, 1.9243889E-06, 1.0969244E-07]
b = [3.3540154E-03, 2.5627725E-04, 2.0829210E-06, 7.3002306E-08]
c = [3.3539264E-03, 2.5609446E-04, 1.9621987E-06, 4.6045930E-08]
d = [3.3368620E-03, 2.4057263E-04, -2.6687093E-06, -4.0719355E-07]

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


  # Step 2. Read LED temperature
  # 2.1 Read analog voltage at LED_Temp_pin
  # 2.2 Translate read voltage to Thermistor resistance
  # 2.3 Translate thermistor resistance to LED temperature
Voltage_out = (LED_Temp_pin * Ref_voltage)/65535
R = (Voltage_out*3.32)/(Ref_voltage - Voltage_out)
## Borrar este print envetualmente.
print('Thermistor resistance: {} ohms'.format(R))
if R >= 68.600 and R < 3.274
   LED_Temp_Cdeg = (a1 + a2)*(math.ln(r))+(a3)*(math.ln(R))**2+(a4)*(math.ln(R))**3 - 273 
  elif R >= 3.274 and R < 0.36036: 
    LED_Temp_Cdeg = (b1 + b2)*(math.ln(R))+(b3)*(math.ln(R))**2+(b4)*(math.ln(R))**3 - 273
  elif R >= 0.36036 and R < 0.06831:
    LED_Temp_Cdeg = (c1+ c2*(math.ln(R))+(c3)*(math.ln(R))**2+(c4)*(math.ln(R))**3 - 273
  else:
    LED_Temp_Cdeg = (d1+ d2*(math.ln(R))+(d3)*(math.ln(R))**2+(d4)*(math.ln(R))**3 - 273
         
print ('Temperature: {} °C'.format(LED_Temp_Cdeg))


