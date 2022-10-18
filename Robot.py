#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import math as m

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

class Robot:
  def __init__(self, init_position=[0.0, 0.0, 0.0]):
    """
    Initialize basic robot params. \

    Initialize Motors and Sensors according to the set up in your robot
    """


    # Robot construction parameters
    self.R = 27 # radio mm
    self.L = 125 # distancia entre ruedas mm


    ##################################################
    # Motors and sensors setup

    # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
    self.BP = brickpi3.BrickPi3()
    #Motor A rueda izquierda, Motor B rueda derecha, Motor C pinzas
    self.PortL = self.BP.PORT_A
    self.PortR = self.BP.PORT_B

                
    # Configure sensors, for example a touch sensor.
    #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)
    try:
      # reset encoders
      self.BP.offset_motor_encoder(self.PortL, self.BP.get_motor_encoder(self.PortL)) # reset encoder L
      self.BP.offset_motor_encoder(self.PortR, self.BP.get_motor_encoder(self.PortR)) # reset encoder R
      #initialize variables self.encL y self.encR
      self.encL=self.BP.get_motor_encoder(self.PortL) #Read motor encoder L in degrees
      self.encR=self.BP.get_motor_encoder(self.PortR) #Read motor encoder R in degrees
    except IOError as error:
      sys.stdout.write(error)

    ##################################################
    # odometry shared memory values
    self.x = Value('d',0.0)
    self.y = Value('d',0.0)
    self.th = Value('d',0.0)
    self.finished = Value('b',1) # boolean to show if odometry updates are finished

    # if we want to block several instructions to be run together, we may want to use an explicit Lock
    self.lock_odometry = Lock()
    #self.lock_odometry.acquire()
    #print('hello world', i)
    #self.lock_odometry.release()

    # odometry update period
    self.P = 1.0



  def setSpeed(self, v,w):
    """ Ajusta la velocidad lineal y angular del robot 
        v en mm/s
        w entra en dps y se convierte inmediatamente a radianes
    """
    print("setting speed to %.2f %.2f" % (v, w))
    #w=m.radians(w)
    # compute the speed that should be set in each motor ...
    wL=(v-w*self.L/2)/self.R
    wR=(v+w*self.L/2)/self.R
    dpsL=m.degrees(wL)
    self.BP.set_motor_dps(self.PortL, dpsL)
    dpsR=m.degrees(wR)
    self.BP.set_motor_dps(self.PortR, dpsR)

  def readSpeed(self):
  
    """ Returns v,w"""
    status=self.BP.get_motor_status(self.PortR) #return [reply[4], speed, encoder, dps]
    dpsR=status[3]
    status=self.BP.get_motor_status(self.PortL) #return [reply[4], speed, encoder, dps]
    dpsL=status[3]        
    wR=m.radians(dpsR)
    wL=m.radians(dpsL)
    w=self.R/self.L*(wR-wL)
    v=self.R/2*(wR+wL)
    return v,w

  def readOdometry(self):
    """ Returns current value of odometry estimation """
    return self.x.value, self.y.value, self.th.value

  def startOdometry(self):
    """ This starts a new process/thread that will be updating the odometry periodically """
    self.finished.value = False
    self.p = Process(target=self.updateOdometry, args=(self.x, self.y, self.th, self.finished))
    self.p.start()
    print("PID: ", self.p.pid)
    # we don't really need to pass the shared params x, y, th, finished,
    # because they are part of the class, so visible within updateOdometry in any case,
    # but it's just to show an example of process receiving params

# You may want to pass additional shared variables besides the odometry values and stop flag
  def updateOdometry(self, x_odo, y_odo, th_odo, finished):
    """ To be filled ...  """
    try:
      # reset encoders
      self.BP.offset_motor_encoder(self.PortL, self.BP.get_motor_encoder(self.PortL)) # reset encoder L
      self.BP.offset_motor_encoder(self.PortR, self.BP.get_motor_encoder(self.PortR)) # reset encoder R      
    except IOError as error:
      #print(error)
      sys.stdout.write(error) 
      
    while not finished.value:
      # current processor time in a floating point value, in seconds
      tIni = time.clock()   
      ######## UPDATE FROM HERE with your code (following the suggested scheme) ########

      # compute updates
      try:
        sys.stdout.write("Reading encoder values .... \n")
        encL=self.BP.get_motor_encoder(self.PortL) #Read motor encoder L in degrees
        encR=self.BP.get_motor_encoder(self.PortR) #Read motor encoder R in degrees
        # reset encoders
        self.BP.offset_motor_encoder(self.PortL, encL) # reset encoder L
        self.BP.offset_motor_encoder(self.PortR, encR) # reset encoder R
        
      except IOError as error:
        sys.stdout.write(error)
      
      sys.stdout.write("Encoder (%s) increased (in degrees) L: %6d  R: %6d " % (type(encL), encL, encR))     

      delta_t=tIni-tEnd
      
      #wR=thetaR / delta_t     
      #wL=thetaL / delta_t
      #delta_sR=self.R*encR      
      #delta_sL=self.R*encL
      #delta_s=(delta_sR+delta_sL)/2
      delta_theta=m.radians(encR+encL)/2
      delta_s=self.R*delta_theta     
      deltaX=delta_s*m.cos(th_odo+delta_theta/2)
      deltaY=delta_s*m.sin(th_odo+delta_theta/2)          
 

      # to "lock" a whole set of operations, we can use a "mutex"
      self.lock_odometry.acquire()
      x_odo.value+=deltaX
      y_odo.value+=deltaY
      th_odo.value+=delta_theta
      self.lock_odometry.release()


      # save LOG
      # Need to decide when to store a log with the updated odometry ...

      sys.stdout.write("Update of odometry ...., X=  %d, Y=  %d, th=  %d \n" %(x_odo.value, y_odo.value, th_odo.value) )      

      ######## UPDATE UNTIL HERE with your code ########

      tEnd = time.clock()
      time.sleep(self.P - (tEnd-tIni))

    #print("Stopping odometry ... X= %d" %(x_odo.value))
    sys.stdout.write("Stopping odometry ... X=  %.2f, Y=  %.2f, th=  %.2f \n" %(x_odo.value, y_odo.value, th_odo.value))


  # Stop the odometry thread.
  def stopOdometry(self):
    self.finished.value = True
    self.BP.reset_all()
  

    
  

