#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for implementing a kalman filter on the range and angle to 
# the leader agent's fiducial.
# Modeled after https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

import numpy as np


FULL_THROTTLE_SPEED = 0.52 # m/s range reduction at full speed
SLOW_FROM_STEER = 5.0 # How much mismatched duty slows range decrease 
ANGLE_RATE = 132.0 # rotation degrees per second per duty disparity

# Holds the kalman filter state for the target
# State is a 4-element vector (R, theta, vR, vTheta), plus a 4x4 
# covariance matrix (sigR, sigTheta, sigvR, sigvTheta diagonal terms).
# We initalize them with values upon construction of the class.
class kalman_filter:
    def __init__(self, R, theta, vR, vTheta, sigR, sigTheta, sigvR, sigvTheta):
        
        self.stateVector = [R, theta, vR, vTheta]
        
        self.covMatrix = [[sigR, 0,        0,     0        ], \
                          [0,    sigTheta, 0,     0        ], \
                          [0,    0,        sigvR, 0        ], \
                          [0,    0,        0,     sigvTheta]]
                          
        # Process noise matrix. In our setup, this captures our estimate
        # of the uncertainty related to the motion of the lead agent.
        # In reality, this is a function of the time since the last measurement,
        # but since we have a somewhat stable framerate, we let it be constant.
        self.Q = [[0.003, 0, 0,    0], \
                  [0,    1, 0,    0], \
                  [0,    0, 0.003, 0], \
                  [0,    0, 0,    1]]
                  
        # State to sensor reading mapping matrix 
        # We measure range and angle directly, and don't measure range rate
        # or angle rate, so this is a 2x4 matrix as follows     
        self.H = [[1, 0, 0, 0], \
                  [0, 1, 0, 0]]
	

    # Takes the state of the target and projects it forward by dt seconds.
    # This should include a "Bu" control term for the PWM duty in the future.
    def project(self, dt, lastLeftDuty, lastRightDuty):
        updateMatrix = [[1, 0, dt, 0], \
                        [0, 1, 0, dt], \
                        [0, 0, 1,  0], \
                        [0, 0, 0,  1]]
                     
        applied_dR = -FULL_THROTTLE_SPEED * dt * (lastLeftDuty + lastRightDuty) / \
                     np.exp(np.abs(lastLeftDuty - lastRightDuty)*SLOW_FROM_STEER)
        applied_dTheta = (lastRightDuty - lastLeftDuty) * ANGLE_RATE * dt
        appliedControl = [applied_dR, applied_dTheta, 0, 0]
        
        self.stateVector = np.dot(updateMatrix, self.stateVector) + appliedControl
        self.covMatrix = np.dot(np.dot(updateMatrix, self.covMatrix), np.transpose(updateMatrix)) + self.Q
        

    # Updates the state using the measurement. Note that it is assumed we have
    # already projected the state forward at this point, so the update step is
    # literally just changing the state per the measurement.
    # K = P H' (H P H' + R)^-1
    # x = x + K (z - H x)
    # P = P - K H P
    # measurement is a 2x1 vector [R, theta]
    # msmtErrorMatrix is a 2x2 matrix [sigR, 0       ]
    #                                 [0,    sigTheta]
    def update(self, measurement, msmtErrorMatrix):
        # Create the Kalman gain matrix, which carries the relationship between
        # the error of the measurements and the existing covaraince matrix.
        # Only need the measurement error to calculate it.
    	K = np.dot(np.dot(self.covMatrix, np.transpose(self.H)), \
    		      np.linalg.inv(np.dot(np.dot(self.H, self.covMatrix), np.transpose(self.H)) \
    		             + msmtErrorMatrix) \
    		  )
  
        # Update the state vector. Simply the state we projected to plus the
        # Kalman-gain-weighted difference between the measurement and the 
        # projected state vector.
        self.stateVector = self.stateVector + \
                           np.dot(K, measurement - np.dot(self.H, self.stateVector))
    
        # Update the covariance matrix based on the kalman gain
        self.covMatrix = self.covMatrix - np.dot(K, np.dot(self.H, self.covMatrix))
        
    # Prints the state of the Kalman filter
    def printState(self):
    	print self.stateVector
    	
    # Prints the covariance of the Kalman filter
    def printCovariance(self):
        print self.covMatrix
