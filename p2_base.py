#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import os
import math as m
from Robot import Robot

def Trayectoria8(miRobot,v,radio):

  w_modulo=v/radio
  
  #Rotar -90º (con v=0) y fijamos (-90º por segundo --> w= angulo / 1s)
  angulo=m.radians(-90)
  w=angulo # Para que lo gire en 1 segundo
  miRobot.setSpeed(0,w)
  time.sleep(angulo/w)
  
  #semicírculo +180º
  angulo=m.radians(180)
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w) 

  #circulo -360º
  angulo=m.radians(-360)
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w) 
  
  #semicirculo +180º
  angulo=m.radians(180)
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w) 
  
def TrayectoriaLoop(miRobot,v,big_radio,recta,alpha):
# tan(alpha)=big_radio/(a+b)
# tan(alpha)=small_radio/a
# small_radio=a * tan(alpha)
# a=(big_radio/tan(alpha)) - b
# siendo b=recta


  a=big_radio/m.tan(m.radians(alpha))-recta
  small_radio=a*m.tan(m.radians(alpha))
  
  #Rotar 90º (con v=0) y fijamos (90º por segundo --> w= angulo / 1s)
  angulo=m.radians(90)
  w=angulo # Para que lo gire en 1 segundo
  miRobot.setSpeed(0,w)
  time.sleep(angulo/w)

  #arco de -90º+alpha
  angulo=m.radians(-90+alpha)
  w_modulo=v/small_radio
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w)

  #tramo recto con w=0
  miRobot.setSpeed(v,0)
  time.sleep(recta/v)   

  #arco de -180º-2*alpha
  angulo=m.radians(-180-2*alpha)
  w_modulo=v/big_radio
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w)

  #tramo recto con w=0
  miRobot.setSpeed(v,0)
  time.sleep(recta/v)  

  #arco de -90º+alpha
  angulo=m.radians(-90+alpha)
  w_modulo=v/small_radio
  w=w_modulo*angulo/abs(angulo) #Darle el signo correcto a w
  miRobot.setSpeed(v,w)
  time.sleep(angulo/w)

  
def Girarsobresi(miRobot):
  w_modulo=v/radio
  miRobot.setSpeed(0,360/5)
  time.sleep(5)
  lectura=miRobot.readSpeed()
  print(lectura[0], " ,", m.degrees(lectura[1]))        
  miRobot.setSpeed(0,0)

def main(args):
  try:
    if args.type=="8":
      print("Trayectoria tipo 8")
      logFileName=os.getcwd() + "/LogT8.txt"
    elif args.type=="L":
      print("Trayectoria tipo loop")
      logFileName=os.getcwd() + "/LogTloop.txt"          
    else:
      print("Trayectoria desconocida.")
      logFileName=""    
    if args.radioD < 0:
      print('d must be a positive value')
      exit(1)
    if args.alpha <= 0:
      print('alpha must be a positive value')
      exit(1)
    if args.rDist <= 0:
      print('rDist must be a positive value')
      exit(1)           
     
    if args.file!="":
      logFileName=os.getcwd() + "/" + args.file
                
    
    # Instantiate Odometry. Default value will be 0,0,0
    if args.pos_ini:
      robot = Robot(init_position=args.pos_ini, logFileName=logFileName)
    else:
      robot = Robot(logFileName=logFileName)

    #print("X,Y,Theta value at the beginning from main X= %.2f, Y=%.2f, Theta=%.2f" %(robot.x.value, robot.y.value, robot.th.value))
    odoValues=robot.readOdometry()
    print("Odometry at the beginning from main X= %.2f, Y=%.2f, Theta=%.2f" % (odoValues[0],odoValues[1],odoValues[2]))

    # 1. launch updateOdometry Process()
    robot.startOdometry()
    tIni=time.perf_counter()
    print("Start : %s" % tIni )
    
    # 2. perform trajectory
    if args.type=="8":
      Trayectoria8(robot,args.velo,args.radioD)
    
    if args.type=="L":
      #miRobot,v,big_radio,recta,alpha
      TrayectoriaLoop(robot,args.velo,args.radioD,args.rDist,args.alpha)          
    
    tEnd=time.perf_counter()
    print("End : %s, Elapsed: %s" % (tEnd, tEnd-tIni) )



    # 3. wrap up and close stuff ...
    # This currently unconfigure the sensors, disable the motors,
    # and restore the LED to the control of the BrickPi3 firmware.
    robot.stopOdometry()


  except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
    robot.stopOdometry()


if __name__ == "__main__":

  # get and parse arguments passed to main
  # Add as many args as you need ...
  parser = argparse.ArgumentParser()
  parser.add_argument("-f", "--file", help="filename for log", default="")
  
  parser.add_argument("-t", "--type", help="Trayectory type, 8(default), L(loop)", default="8")
  
  parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory, or big radio for loop trajectory(mm)",
                      type=float, default=200.0)
                      
  parser.add_argument("-a", "--alpha", help="loop trajectory tangent angle (degrees)",
                      type=float, default=10.0)
                      
  parser.add_argument("-r", "--rDist", help="Straight section length for loop trajectory (mm)",
                      type=float, default=480.0)
                      
  parser.add_argument("-v", "--velo", help="Linear speed (mm/2)",
                      type=float, default=100.0)  
                      
  parser.add_argument("-p", "--pos_ini", nargs=3, help="Set odometry initial position", type=float)                
  args = parser.parse_args()
  
  main(args)
  
