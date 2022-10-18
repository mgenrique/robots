#!/usr/bin/python
# -*- coding: UTF-8 -*-
# De Maria
import argparse
import numpy as np
import time
import math as m
from Robot import Robot

def main(args):
  try: 
    if args.radioD < 0:
      print('d must be a positive value')
      exit(1)

    robot = Robot()

    print("X value at the beginning from main X= %.2f" %(robot.x.value))

    ##Trayectoria8():
    ###R=v/w -> v=R*w
    ###teta=w*t -> w=teta/t
    #robot.setSpeed(0,-(m.pi/2)/4)
    #time.sleep(4)
    #robot.setSpeed(200*m.pi/10,m.pi/10)
    #time.sleep(10)
    #robot.setSpeed(200*2*m.pi/20,-2*m.pi/20)
    #time.sleep(20)
    #robot.setSpeed(200*m.pi/10,m.pi/10)
    #time.sleep(10)
    #robot.setSpeed(0,0)
    
    #Trayectoria2
    a=50 
    d=100
    r1=110
    r2=300
    beta=m.acos(d/r1)
    print(beta)
    alfa1=m.pi-2*beta
    print(alfa1)
    alfa2=m.pi+2*beta
    print(alfa2)
    #con estos datos beta=25Âº aprox y distEjes=300mm
    robot.setSpeed(0,(m.pi/2)/4)
    time.sleep(4)
    robot.setSpeed(r1*(alfa1/2)/5,(-alfa1/2)/5)
    time.sleep(5)
    robot.setSpeed(r2/5,0)
    time.sleep(5)
    robot.setSpeed(r2*alfa2/20,-alfa2/20)
    time.sleep(20)
    robot.setSpeed(r2/5,0)
    time.sleep(5)
    robot.setSpeed(r1*(alfa1/2)/5,(-alfa1/2)/5)
    time.sleep(5)
    robot.setSpeed(0,0)
    
    ##Girarsobresi():
    #robot.setSpeed(0,360/5)
    #time.sleep(5)
    #lectura=robot.readSpeed()
    #print(lectura[0], " ,", m.degrees(lectura[1]))        
    #robot.setSpeed(0,0)
    
    

  except KeyboardInterrupt:
  # except the program gets interrupted by Ctrl+C on the keyboard.
  # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
    robot.stopOdometry()
    f.close()

if __name__ == "__main__":

  # get and parse arguments passed to main
  # Add as many args as you need ...
  parser = argparse.ArgumentParser()
  parser.add_argument("-f", "--file", help="filename for log", default="Log.txt")
  parser.add_argument("-t", "--type", help="Trayectory type, 8(default), L(loop)", default="8")
  parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory, or loop trajectory big radio (mm)",
                      type=float, default=400.0)
  parser.add_argument("-a", "--radioA", help="loop trajectory small radio(mm)",
                      type=float, default=200.0)
  parser.add_argument("-r", "--rDist", help="Distance between centers for loop trajectory (mm)",
                      type=float, default=800.0)                       
  parser.add_argument("-p", "--pos_ini", nargs=3, help="Set odometry initial position", type=float)
  args = parser.parse_args()

  main(args)
  
