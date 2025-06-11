import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import time
import math
from inverse_kinematics import solveIK
import gaits
from sim import Sim
from Joint_Controller.leg_controller import LegController

class Robot ():

   def __init__(self, gait = "si", useSim = True, useMotors = False):
      self.gait = gait
      self.startTime = time.time()
      self.prevGait = ""
      self.cycle = 0
      self.gaitTraj = None
      self.joint_positions = None
      self.useSim = useSim
      self.useMotors = useMotors
      
      self.GeneratePosition()
      # Initialize sim positions to start position
      if self.useSim:
         self.sim = Sim(self.joint_positions)
      if self.useMotors:
         self.leg_controller = LegController()
      
      self.last_update_time = time.time()


   def GeneratePosition(self):      
      if self.prevGait != self.gait:
         self.prevGait = self.gait
         self.startTime = time.time()
         
         match self.gait:
            case "forward" | "f":
               self.gaitTraj = gaits.Walk()
            case "rightWalk" | "rw":
               self.gaitTraj = gaits.WalkTurn(turnRight=True)
            case "leftWalk"|"lw":
               self.gaitTraj = gaits.WalkTurn(turnRight=False)
            case "backward"|"b":
               self.gaitTraj = gaits.WalkBackward()
            case "trot" | "t":
               self.gaitTraj = gaits.Trot()
            case "rightInPlaceNoRoll" | "rnr":
               self.gaitTraj = gaits.TurnInPlaceNoRoll(turnRight=True)
            case "leftInPlaceNoRoll"|"lnr":
               self.gaitTraj = gaits.TurnInPlaceNoRoll(turnRight=False)
            case "standing" | "s":
               self.gaitTraj = gaits.Stand()
            case "leftInPlaceWithRoll"|"lwr":
               self.gaitTraj = gaits.TurnInPlaceWithRoll()
            case "rightInPlaceWithRoll"|"rwr":
               self.gaitTraj = gaits.TurnInPlaceWithRoll()
            case "sit" | "si":
               self.gaitTraj = gaits.Sit()

      curTime = time.time() - self.startTime
      pos = self.gaitTraj.getPos(curTime)
      
      posFL = solveIK(pos[0])
      posFR = solveIK(pos[1])
      posBR = solveIK(pos[2], True)
      posBL = solveIK(pos[3], True)
      self.joint_positions = [posFL[0], posFR[0], posBR[0], posBL[0], 
                              posFL[1], posFR[1], posBR[1], posBL[1], 
                              posFL[2], posFR[2], posBR[2], posBL[2]]
      

   def update(self):
      self.GeneratePosition()
      if self.useSim:
         if not self.sim.update(self.joint_positions):
            raise Exception("Simulation closed")
      if self.useMotors:
         self.leg_controller.update(self.joint_positions)
      
      #track cycle time
      now = time.time()
      cycle_time = now - self.last_update_time
      print(f"Cycle time: {cycle_time:.4f} seconds")
      self.last_update_time = now
      
         
      