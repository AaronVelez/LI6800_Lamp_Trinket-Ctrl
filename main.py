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


# Module imports
import board
import digitalio
import analogio
import time



# Set pin definitions
LED_Temp = AnalogIn(board.A1)



# Main Loop

import  board
import  analogio
import math

thermistor  = analogio.AnalogIn(board.A1)}
a = [3.3538646E-03, 2.56544090E-04, 1.9243889E-06, 1.0969244E-07]
b = [3.3540154E-03, 2.5627725E-04, 2.0829210E-06, 7.3002306E-08]
c = [3.3539264E-03, 2.5609446E-04, 1.9621987E-06, 4.6045930E-08]
d = [3.3368620E-03, 2.4057263E-04, -2.6687093E-06, -4.0719355E-07]
R = 1000 / (65535/thermistor.value - 1)
print('Thermistor resistance: {} ohms'.format(R))
if R >= 68.600 and r < 3.274
    T = (a1 + a2)*(math.ln(r))+(a3)*(math.ln(r))**2+(a4)*(math.ln(r))**3 
  elif r >= 3.274 and r < 0.36036: 
    T = (b1 + b2)*(math.ln(r))+(b3)*(math.ln(r))**2+(b4)*(math.ln(r))**3 
  elif r >= 0.36036 and < 0.06831:
    T = (c1+ c2*(math.ln(r))+(c3)*(math.ln(r))**2+(c4)*(math.ln(r))**3
  else:
    T = (d1+ d2*(math.ln(r))+(d3)*(math.ln(r))**2+(d4)*(math.ln(r))**3
         
T = T + 273
print ('Temperature: {} °C'.format(T))


