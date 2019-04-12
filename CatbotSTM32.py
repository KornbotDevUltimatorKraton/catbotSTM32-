#Auther Mr.Chanapai chuadchum 
#Project Catbot for the experimental on the Quadped Robot 
# date 21/3/19 
import pyfirmata # Pyfirmata for the hardware control 
import math # Math function for the calculation on the robotkinematics and phsyc modelling 
from sklearn.linear_model import LinearRegression # Machine learning library 
import cv2 # Image processing function for the face recognition 
import scipy 
import nanpy as np 
import pandas as pd # data .csv 
import csv 
import os 
import sys  
import smbus # i2c interface library on the rpi 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  # Gyro scope address 
# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
try:
         # STM32F091RCT6  M0 microcontroller     
   hardware  = pyfirmata.ArduinoMega("/dev/ttyACM0")  # The hardware connection on the STM32 board on the catbot  
         # STM32F303k8t6
   hardware3 = pyfirmata.Arduino("/dev/ttyACM1")   
except: 
    print("Catbot Hardware serial connection fail !")
   # pwm pins setup 
Hydraulicpump = hardware.get_pin('d:3:p')   #PWM for the pins 3 to control the function of the pump accelerator and pressure 
     # Pressure control function on the catbot 
            #Servo setup 
FrontlegLeft = hardware.get_pin('d:22:s')  
FrontlegRight = hardware.get_pin('d:23:s')
Middlebody = hardware.get_pin('d:24:s')
BacklegLeft = hardware.get_pin('d:25:s')
BacklegRight = hardware.get_pin('d:26:s')
Taily = hardware.get_pin('d:30:s')   
Tailx = hardware.get_pin('d:31:s') 
def Hydrauliccontrol(Pressure):   
    Hydraulicpump.write(Pressure)  # Pressure control function for the hydraulic pump 
# Hydraulic solenoid class function on the system 
      # Chess function for the logic control  
def SolenoidHydraulicchess(slleft,slright,srleft,srright):
    if slleft == 1:  # Logic left on 
        hardware.digital[2].write(1)  # Logic high 
    if slright == 1: 
        hardware.digital[4].write(1) 
      # Bicept function for the logic control function hardware 
    if srleft == 1:  # Logic left on 
        hardware.digital[7].write(1) #  Logic high 
    if srright == 1: 
        hardware.digital[8].write(1) # 
    else:
          hardware.digital[2].write(0) # Logic low 
          hardware.digital[4].write(0)
          hardware.digital[7].write(0)# Logic low 
          hardware.digital[8].write(0)#
def SolenoidHydraulicBicept(slleft,slright,srleft,srright): # insert the logic input on the system function
    if slleft == 1:  # Logic left on 
        hardware.digital[9].write(1)  # Logic high 
    if slright == 1: 
        hardware.digital[10].write(1) 
      # Bicept function for the logic control function hardware 
    if srleft == 1:  # Logic left on 
        hardware.digital[11].write(1) #  Logic high 
    if srright == 1: 
        hardware.digital[12].write(1) # 
    else:
          hardware.digital[9].write(0) # Logic low 
          hardware.digital[10].write(0)
          hardware.digital[11].write(0)# Logic low 
          hardware.digital[12].write(0)#

def SolenoidHydraulicAbduct(slleft,slright,srleft,srright):
    if slleft == 1:  # Logic left on 
        hardware.digital[14].write(1)  # Logic high 
    if slright == 1: 
        hardware.digital[15].write(1) 
      # Bicept function for the logic control function hardware 
    if srleft == 1:  # Logic left on 
        hardware.digital[16].write(1) #  Logic high 
    if srright == 1: 
        hardware.digital[17].write(1) # 
    else:
          hardware.digital[14].write(0) # Logic low 
          hardware.digital[14].write(0)
          hardware.digital[16].write(0)# Logic low 
          hardware.digital[17].write(0)#
      # Bicept function for the logic control function hardware 
def SolenoidHydraulicRectusfemeris(slleft,slright,srleft,srright): # insert the logic input on the system function   
    if slleft == 1:  # Logic left on 
        hardware.digital[18].write(1)  # Logic high 
    if slright == 1: 
        hardware.digital[19].write(1) 
      # Bicept function for the logic control function hardware 
    if srleft == 1:  # Logic left on 
        hardware.digital[20].write(1) #  Logic high 
    if srright == 1: 
        hardware.digital[21].write(1) # 
    else:
          hardware.digital[18].write(0) # Logic low 
          hardware.digital[19].write(0)
          hardware.digital[20].write(0)# Logic low 
          hardware.digital[21].write(0)#
     # Position control function for the hydraulc Front leg  
def Frontleghydraulicpositionning(goalchl,goalchr,goalBil,goalBiR,AnglechessL,AnglechessR,AngleBiL,AngleBiR,Pressure):
          # All Left, Right chess     
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           # Left hydraulic ramp control valve 
      if AnglechessL < goalchl: 
            SolenoidHydraulicchess(1,0,0,1)  # Logic inverst 
      if AnglechessL > goalchl: 
            SolenoidHydraulicchess(0,1,1,0)
      if AnglechessL == goalchl: 
            SolenoidHydraulicchess(0,0,0,0)  # All off 
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          # Right hydraulic ramp control valv 
      if AnglechessR < goalchr: 
            SolenoidHydraulicchess(1,0,0,1)  # Logic inverst 
      if AnglechessR > goalchr: 
            SolenoidHydraulicchess(0,1,1,0)
      if AnglechessR == goalchr: 
            SolenoidHydraulicchess(0,0,0,0)  # All off 
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
         # All Left,Right Bicept 
         #Left 
      if AngleBiL < goalBil: 
            SolenoidHydraulicBicept(1,0,0,1)  # Logic inverst 
      if AngleBiL > goalBil: 
            SolenoidHydraulicBicept(0,1,1,0)
      if AngleBiL == goalBil: 
            SolenoidHydraulicBicept(0,0,0,0)  # All off 
            # Right 
      if AngleBiR < goalBiR: 
            SolenoidHydraulicBicept(1,0,0,1)  # Logic inverst 
      if AngleBiR > goalBiR: 
            SolenoidHydraulicBicept(0,1,1,0)
      if AnglechessR == goalchr: 
            SolenoidHydraulicBicept(0,0,0,0)  # All off 

     # Position control 
def Backleghydraulicpositionning(goalAbl,goalAbR,goalRecl,goalRecR,AngleAbL,AngleAbR,AngleRecL,AngleRecR,Pressure):  
       
            # All Left, Right Abduct     
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           # Left hydraulic ramp control valve 
      if AngleAbL < goalAbl: 
            SolenoidHydraulicAbduct(1,0,0,1)  # Logic inverst 
      if AngleAbL > goalAbl: 
            SolenoidHydraulicAbduct(0,1,1,0)
      if AngleAbL == goalAbl: 
            SolenoidHydraulicAbduct(0,0,0,0)  # All off 
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          # Right hydraulic ramp control valv 
      if AngleAbR < goalAbR: 
            SolenoidHydraulicAbduct(1,0,0,1)  # Logic inverst 
      if AngleAbR > goalAbR: 
            SolenoidHydraulicAbduct(0,1,1,0)
      if AngleAbR == goalAbR: 
            SolenoidHydraulicAbduct(0,0,0,0)  # All off 
      #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
         # All Left,Right Rectus femerus 
         #Left 
      if AngleRecL < goalRecl: 
            SolenoidHydraulicRectusfemeris(1,0,0,1)  # Logic inverst 
      if AngleRecL > goalRecl: 
            SolenoidHydraulicRectusfemeris(0,1,1,0)
      if AngleRecL == goalRecl: 
            SolenoidHydraulicRectusfemeris(0,0,0,0)  # All off 
            # Right 
      if AngleRecR < goalRecR: 
            SolenoidHydraulicRectusfemeris(1,0,0,1)  # Logic inverst 
      if AngleRecR > goalRecR: 
            SolenoidHydraulicRectusfemeris(0,1,1,0)
      if AngleRecR == goalRecR: 
            SolenoidHydraulicRectusfemeris(0,0,0,0)  # All off 
     # Control motion with the timing speed on the servo motor 
def Servomotioncontrol(AngleFr,AngleFl,AngleBl,AngleBr,AngleMid,AngleTX,AngleTY,timing): 
      FrontlegLeft.write(AngleFl) # Angle front left 
      FrontlegRight.write(AngleFr) 
      Middlebody.write(AngleMid)
      BacklegLeft.write(AngleBl)
      BacklegRight.write(AngleBr)
      Tailx.write(AngleTX)
      Taily.write(AngleTY)   
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
    #All function running on the looping condition
def AnimatedmotionAutomate(x,y,z,stannding,walking,running,sitdown): 
           # Every motion stabiliation function combination 
    if stannding == 1:   # unit function standing 
       Servomotioncontrol(90,90,0,0,90,0,90,500)   # Standing funtion 
    if walking == 1:     # unit function walking 
         for i in range(75,135): 
             
         for q in range(135,75,-1): 
             
    if running  == 1:    # unit function running 
         
while True: 
       
        # Servo motion control positioning and timing speed motion 
       
       #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          #Gyroscope sensor 
        bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
        address = 0x68       # via i2cdetect
 
         # Aktivieren, um das Modul ansprechen zu koennen
        bus.write_byte_data(address, power_mgmt_1, 0)
 
        print "Gyroskop"
        print "--------"
        gyroskop_xout = read_word_2c(0x43)
        gyroskop_yout = read_word_2c(0x45)
        gyroskop_zout = read_word_2c(0x47)
 
        print "gyroskop_xout: ", ("%5d" % gyroskop_xout), " skaliert: ", (gyroskop_xout / 131)
        print "gyroskop_yout: ", ("%5d" % gyroskop_yout), " skaliert: ", (gyroskop_yout / 131)
        print "gyroskop_zout: ", ("%5d" % gyroskop_zout), " skaliert: ", (gyroskop_zout / 131)
        print "Beschleunigungssensor"
        print "---------------------"
        beschleunigung_xout = read_word_2c(0x3b)
        beschleunigung_yout = read_word_2c(0x3d)
        beschleunigung_zout = read_word_2c(0x3f)
        beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
        beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
        print "beschleunigung_xout: ", ("%6d" % beschleunigung_xout), " skaliert: ", beschleunigung_xout_skaliert
        print "beschleunigung_yout: ", ("%6d" % beschleunigung_yout), " skaliert: ", beschleunigung_yout_skaliert
        print "beschleunigung_zout: ", ("%6d" % beschleunigung_zout), " skaliert: ", beschleunigung_zout_skaliert
        print "X Rotation: " , get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
        print "Y Rotation: " , get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
         #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
