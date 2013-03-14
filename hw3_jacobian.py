#!/usr/bin/env python

PACKAGE_NAME = 'hw3'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

import collections
import Queue

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()


class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()
    
    #self.run_problem_jacobian()


  #######################################################
  # the usual initialization for openrave
  #######################################################
  def openrave_init(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW3 Viewer')
    self.env.Load('models/%s_jacobian.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]

    #set right wam as active manipulator
    with self.env:
      self.robot.SetActiveManipulator('right_wam');
      self.manip = self.robot.GetActiveManipulator()

      #set active indices to be right arm only
      self.robot.SetActiveDOFs(self.manip.GetArmIndices() )
      self.end_effector = self.manip.GetEndEffector()

  #######################################################
  # problem specific initialization
  #######################################################
  def problem_init(self):
    # load ikmodel
    self.ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not self.ikmodel.load():
      self.ikmodel.autogenerate()

    # move left arm out of way
    self.robot.SetDOFValues(np.array([4,2,0,-1,0,0,0]),self.robot.GetManipulator('left_wam').GetArmIndices() )


  #######################################################
  # Use jacobians to go forward in a straight line
  #######################################################
  def run_problem_jacobian(self):
    self.robot.GetController().Reset()

#    curr_hand_trans = self.manip.GetTransform()
#    curr_hand_trans[0:3,3] = np.array([0.6,-0.191,0.805])
#    ik_sols = self.manip.FindIKSolutions(curr_hand_trans, openravepy.IkFilterOptions.CheckEnvCollisions)
#    self.robot.SetActiveDOFValues(ik_sols[0])

    start_config = np.array([ 5.59316009, -0.78576424, -0.2 ,  2.15229081, -3.28525585, 1.37854482, -1.75801915])

    with self.env:
      self.robot.SetActiveDOFValues(start_config)

    # get the trajectory!
    traj = self.straight_line_jacobian(1.0)

    with self.env:
      self.robot.SetActiveDOFValues(start_config)
    
    self.robot.GetController().SetPath(traj)
    self.robot.WaitForController(0)


  #TODO
  #######################################################
  # From the current configuration, move straight forward
  # Move UP TO distance specified by input distance
  # Terminate if you get to an illegal configuration
  # RETURN: a trajectory to move the hand straight
  #######################################################
  def straight_line_jacobian(self, distance):
    #you may find the following useful
    #j_spatial = self.manip.CalculateJacobian()
    #j_rot = self.manip.CalculateRotationJacobian()
    #J = np.vstack((j_spatial, j_rot))
    #Jinv = np.linalg.pinv(J) #pseudo-inverse
    #quat = openravepy.quatFromRotationMatrix(trans)

    return None



  #######################################################
  # Converts list of points to a trajectory
  #######################################################
  def points_to_traj(self, points):
    traj = openravepy.RaveCreateTrajectory(self.env,'')
    traj.Init(self.robot.GetActiveConfigurationSpecification())
    for idx,point in enumerate(points):
      traj.Insert(idx,point)
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj,self.robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
    return traj


if __name__ == '__main__':
  robo = RoboHandler()
  #time.sleep(10000) #to keep the openrave window open
  
