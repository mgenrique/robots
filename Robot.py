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
  def __init__(self, init_position=[0.0, 0.0, 0.0], logFileName=""):
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
    
    # Si se ha especificado un nombre para el fichero se hace log  
    if logFileName!="":
      self.LogEnable=True
      #Configurar fichero para log odometry     
      print ("El log se realizará en ", logFileName)
      try:
        #self.f = open(logFileName, 'a') #mode 'a' append
        self.f = open(logFileName, 'w') #mode 'w' new
      except IOError as error:
        try:
          self.f = open(logFileName, 'w') #mode 'w' new
        except IOError as error:
          sys.stdout.write("No se ha podido crear el fichero para el log: ", logFileName)
          sys.stdout.write(error)
          self.LogEnable=False
    else:
      self.LogEnable=False
    ##################################################
    # odometry shared memory values
    self.x = Value('d',init_position[0])
    self.y = Value('d',init_position[1])
    self.th = Value('d',init_position[2])
    self.finished = Value('b',1) # boolean to show if odometry updates are finished

    # if we want to block several instructions to be run together, we may want to use an explicit Lock
    self.lock_odometry = Lock()
    #self.lock_odometry.acquire()
    #print('hello world', i)
    #self.lock_odometry.release()

    # odometry update period
    self.P = 0.1



  def setSpeed(self, v,w):
    """ Ajusta la velocidad lineal y angular del robot 
        v en mm/s
        w entra en dps y se convierte inmediatamente a radianes
    """
    print("setting speed to %.2f %.2f" % (v, w))
    # w=m.radians(w)
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
  def NormPiDegrees(self,Angulo_Grados):
    #Deja un angulo entre -180 y 180
    angle= int(Angulo_Grados) % 360 #resto división, puede que no funione si Angulo_Grados no es un entero
    if angle>180:
      angle=-360+angle
    elif angle<-180:
      angle=360+angle
    return angle
  def NormPiRadians(self, Angulo_radianes):
    #Deja un angulo entre -pi y pi
    if (Angulo_radianes>2*m.pi):
      vueltas=m.floor(Angulo_radianes/(2*m.pi))
      angle=Angulo_radianes-vueltas*2*m.pi
    elif (Angulo_radianes<-2*m.pi):
      vueltas=m.floor(Angulo_radianes/(-2*m.pi))
      angle=Angulo_radianes+vueltas*2*m.pi
    else:
      angle=Angulo_radianes
    #Asegurar que queda entre -pi y pi    
    if angle>m.pi:
      angle=-2*m.pi+angle
    elif angle<-1*m.pi:
      angle=2*m.pi+angle
    return angle
    
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
    try:
      # reset encoders
      self.BP.offset_motor_encoder(self.PortL, self.BP.get_motor_encoder(self.PortL)) # reset encoder L
      self.BP.offset_motor_encoder(self.PortR, self.BP.get_motor_encoder(self.PortR)) # reset encoder R
      encL_prev=self.BP.get_motor_encoder(self.PortL) #Read motor encoder L in degrees
      encR_prev=self.BP.get_motor_encoder(self.PortR) #Read motor encoder R in degrees      
    except IOError as error:
      #print(error)
      sys.stdout.write(error) 
    
    tEnd = time.perf_counter()
    if self.LogEnable:
      self.f.writelines("Tiempo; X(mm); Y(mm); Theta(grados)"+"\n")
    
    while not finished.value:
      try:
        # current processor time in a floating point value, in seconds
        tIni = time.perf_counter()
        delta_t=tIni-tEnd
        ######## UPDATE FROM HERE with your code (following the suggested scheme) ########

        # compute updates
        sys.stdout.write("Reading encoder values .... \n")
        encL_actual=self.BP.get_motor_encoder(self.PortL) #Read motor encoder L in degrees
        encR_actual=self.BP.get_motor_encoder(self.PortR) #Read motor encoder R in degrees
          

        
        encL=m.radians(encL_actual-encL_prev)
        encR=m.radians(encR_actual-encR_prev)
        encL_prev=encL_actual
        encR_prev=encR_actual
        sys.stdout.write("Encoder (%s) increased (in degrees) L: %6d  R: %6d " % (type(encL), encL, encR))
        
        
        #Tracción diferencial
        wR=encR / delta_t     
        wL=encL / delta_t       
        v=self.R*(wR+wL)/2
        w=self.R*(wR-wL)/self.L
        delta_theta=w*delta_t
        
        self.lock_odometry.acquire()      
        if self.P<0.5:
          #Procedimiento aproximado para angulos pequeños
          delta_s=v*delta_t
          deltaX=delta_s*m.cos(th_odo.value+delta_theta/2)
          deltaY=delta_s*m.sin(th_odo.value+delta_theta/2)          
        else:
          #Procedimiento exacto. Requiere mayor tiempo de computo
          if w==0:
            deltaX=delta_t*v*m.cos(th_odo.value)
            deltaY=delta_t*v*m.sin(th_odo.value)
          else:
            radio=v/w
            #deltaX=2*radio*m.sin(delta_theta/2)*m.cos(th_odo.value+delta_theta/2)
            #deltaY=2*radio*m.sin(delta_theta/2)*m.sin(th_odo.value+delta_theta/2)
            deltaX=radio*(m.sin(th_odo.value+delta_theta)-m.sin(th_odo.value))
            deltaY=-1*radio*(m.cos(th_odo.value+delta_theta)-m.cos(th_odo.value))
          
        # to "lock" a whole set of operations, we can use a "mutex"
        x_odo.value+=deltaX
        y_odo.value+=deltaY
        th_odo.value=self.NormPiRadians(th_odo.value+delta_theta)
        self.lock_odometry.release()


        # save LOG       
        if self.LogEnable:
          self.f.writelines(str(tIni) + "; " + str(x_odo.value) + "; " + str(y_odo.value) + "; " + str(m.degrees(th_odo.value)) +"\n")
        else:
          sys.stdout.write("Update of odometry ...., X=  %d, Y=  %d, th=  %d \n" %(x_odo.value, y_odo.value, m.degrees(th_odo.value)) )        

        ######## UPDATE UNTIL HERE with your code ########

        tEnd = time.perf_counter()
        time.sleep(self.P - (tEnd-tIni))
      except IOError as error:
        sys.stdout.write(error)
    ##############################
    #print("Stopping odometry ... X= %d" %(x_odo.value))
    sys.stdout.write("Stopping odometry ... X=  %.2f, Y=  %.2f, th=  %.2f \n" %(x_odo.value, y_odo.value, m.degrees(th_odo.value)))
    if self.LogEnable:
      self.f.close


  # Stop the odometry thread.
  def stopOdometry(self):
    self.finished.value = True
    self.BP.reset_all()

