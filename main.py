# Name: main.py
# Created: 10/11/2020
# Authors: Juan de Dios Moreno González y Aarón I. Vélez Ramírez
# Universidad Nacional Autónoma de México
#
# Code for temperature control of LI6800 custum LED lamp
#
# Details:
# Runs in an Adafruit Trinket board with an ARM Cortex M0+ processor
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
Fan_max_rpm = 3000

# Thermistor parameters
# Juan, please add here your constants
Div_R = 3.32 # Value of the voltage divider resistor in kohms
# add all the constants for the termistor....
a = [3.3538646E-03, 2.56544090E-04, 1.9243889E-06, 1.0969244E-07]
b = [3.3540154E-03, 2.5627725E-04, 2.0829210E-06, 7.3002306E-08]
c = [3.3539264E-03, 2.5609446E-04, 1.9621987E-06, 4.6045930E-08]
d = [3.3368620E-03, 2.4057263E-04, -2.6687093E-06, -4.0719355E-07]

# PID gains and parameters
Kp = 0.00001 
Ki = 0.002
Kd = 0.000001
beta = 1
gamma = 0
N = 10
MV_min = 0
MV_max = 2**16




##### Variable definitions
Fan_PWM_duty-cycle_bits = 0     # In 16-bit format
Fan_Tach_rpm = 0
Fan_Tach_bits = 0               # In 16-bit format
Fan_Speed_voltage = 0.0
LED_Temp_Ctrl_voltage = 0.0
LED_Temp_Ctrl_Cdeg = 0.0
LED_Temp_voltage = 0.0
LED_Temp_Cdeg =0.0



##### Functions
# PID function attributed to secction "4.5 Realizable PID Control" https://github.com/jckantor/CBE30338
def PID(Kp, Ki, Kd, MV_bar=0, MV_min=0, MV_max=100, beta=1, gamma=0, N=10):

    # initial yield and return
    data = yield MV_bar
    t,  = data[0:3]
    
    P = Kp*(beta*SP - PV)
    MV = MV_bar + P
    MV = MV_min if MV < MV_min else MV_max if MV > MV_max else MV
    I = 0
    D = 0
    dI = 0
    
    S = Kd*(gamma*SP - PV)
    t_prev = t
    
    while True:
        # yield MV, wait for new t, SP, PV, TR
        data = yield MV, P, I, D, dI
        
        # see if a tracking data is being supplied
        if len(data) < 4:
            t, PV, SP = data
        else:
            t, PV, SP, TR = data
            d = MV - TR
            #I = TR - MV_bar - P - D
        
        # PID calculations
        P = Kp*(beta*SP - PV)
        eD = gamma*SP - PV
        D = N*Kp*(Kd*eD - S)/(Kd + N*Kp*(t - t_prev))
        
        # conditional integration
        dI = Ki*(SP - PV)*(t - t_prev)
        if (MV_bar + P + I + D + dI) > MV_max:
            dI = max(0, min(dI, MV_max - MV_bar - P - I - D))
        if (MV_bar + P + I + D + dI) < MV_min:
            dI += min(0, max(dI, MV_min - MV_bar - P - I - D))
        I += dI
        MV = MV_bar + P + I + D 
        
        # Clamp MV to range 0 to 100 for anti-reset windup
        MV = max(MV_min, min(MV_max, MV))
        
        # update stored data for next iteration
        S = D*(t - t_prev) + S
        t_prev = t

        

##### Setup
# Create and initialize PID control
PID_fan = PID(Kp, Ki, Kd, MV_min, MV_max, beta, gamma, N) # NEEDS TUNING!
PID_fan.send(None) 



##### Main Loop
##### It runs forever
while True:
  # Step 1. Read LED temperature setpoin defined by LI6800.
  # 1.1 Read analog voltage proportional to desired LED temperature at LED_Temp_Ctrl_pin
  # 1.2 Convert it to °C using constant (needs to be added to constants) and store it in LED_Temp_Ctrl_Cdeg
  
  
  # Step 2. Read LED temperature
  # 2.1 Read analog voltage at LED_Temp_pin and store it in LED_Temp_voltage
  # 2.2 Translate read voltage to Thermistor resistance
  # 2.3 Translate thermistor resistance to LED temperature and store it in LED_Temp_Cdeg 


  LED_Temp_voltage = (LED_Temp_pin * Ref_voltage)/65535
  R = (LED_Temp_voltage*3.32)/(Ref_voltage - LED_Temp_voltage)
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
  

  # Step 3. Read fan speed and send it to LI6800
  # 3.1 Read digital pulses at Fan_Tach_pin
  # 3.2 Calculate fan speed in rpm and store it in Fan_Tach_rpm
  # 3.3 Convert Fan_Tach_rpm into a 16-bit value, mapping the maximum fan speed (Fan_max_rpm constant) to 65,536, and store it in Fan_Tach_bits
  # 3.4 Convert Fan_Tach_rpm to an analog voltage value using constant (needs to be added to constants) and store it in Fan_Speed_voltage
  # 3.5 Write Fan_Speed_voltage value to Fan_Speed_pin, so LI6800 can read it
  
  
  # Step 4. Run PID control algorithm.
  # It calculates the required fan speed (in PWM duty cycle values IN 16-bit format) to achive the desired LED temperature.
    # It takes into consideration the current and near past LED temperature.
    t = time.monotonic()
    PV = LED_Temp_Cdeg
    SP = LED_Temp_Ctrl_Cdeg
    TR = Fan_Tach_bits
    # Given time (t) process variable (PV), setpoint (SP) and tracked MV (TR), it returns manipulated variable (MV) and P, I, D and d
    MV, P, I, D, d = PID_fan.send([t, PV, SP, TR])
    Fan_PWM_duty-cycle_bits = MV
    

  
  # Step 5. Set new Fan speed
  # 5.1 Write to Fan_PWM_pin the new PWM duty cycle in 16-bit format (Fan_PWM_duty-cycle_bits) calculated by the PID algorithm
  
  
  # Step 6. Wait or no wait before executing the loop again (to be defined later).
