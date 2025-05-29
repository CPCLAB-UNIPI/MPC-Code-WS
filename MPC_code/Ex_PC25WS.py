# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 12:11:54 2016

@author: marcovaccari
"""
from __future__ import division
from past.utils import old_div
from casadi import *
from casadi.tools import *
from matplotlib import pylab as plt
import math
import scipy.linalg as scla
import numpy as np
from Utilities import*

### 1) Simulation Fundamentals

# 1.1) Simulation discretization parameters
Nsim =  # Simulation length

N =     # Horizon

h =  # Time step

# 3.1.2) Symbolic variables
xp = SX.sym("xp", ) # process state vector       
x = SX.sym("x", )  # model state vector          
u = SX.sym("u", )  # control vector              
y = SX.sym("y", )  # measured output vector      
d = SX.sym("d", )  # disturbance                     

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
### 2) Process and Model construction 

# 2.1) Process Parameters
tau = 3
CA0 = 2.0      # Feed concentration of A (kmol/m3)
T0 = 298.0     # Feed temperature (K)
Tm = 298.0      # Reference temperature (K)
km = 4.0e-3     # Pre-exponential factor (1/min)
EoR = 8000.0    # Activation energy over Gas constant (K)
deltaH = -300000.0  # Heat of reaction (kJ/kmol)
rho = 1000.0   # Density (kg/m3)
Cp = 4      # Heat capacity (J/g.K)
UA_V = 340     # Heat transfer coefficient * area/ volume (kJ/(m3*min*K)

# State map
def User_fxp_Cont(x,t,u,pxp,pxmp):
    """
    SUMMARY:
    It constructs the function fx_p for the non-linear case
        
    SYNTAX:
    assignment = User_fxp_Cont(x,t,u)
        
    ARGUMENTS:
    + x         - State variable     
    + t         - Current time
    + u         - Input variable  
        
    OUTPUTS:
    + fx_p      - Non-linear plant function
    """ 
        
    ## System Dynamics 
    
    return vertcat(dCAdt, dTdt)

Mx = 10 # Number of elements in each time step 

# Output map
def User_fyp(x,u,t,pyp,pymp):
    """
    SUMMARY:
    It constructs the function User_fyp for the non-linear case
    
    SYNTAX:
    assignment = User_fyp(x,t)
  
    ARGUMENTS:
    + x             - State variable
    + t             - Variable that indicate the current iteration
    
    OUTPUTS:
    + fy_p      - Non-linear plant function     
    """ 
    
    return fy_p

# White Noise
# R_wn = 1e-4*np.array([[1.0, 0.0], [0.0, 1.0]]) # Output white noise covariance matrix


# 2.2) Model Parameters
    
# State Map
def User_fxm_Cont(x,u,d,t,px):
    """
    SUMMARY:
    It constructs the function fx_model for the non-linear case
    
    SYNTAX:
    assignment = User_fxm_Cont(x,u,d,t)
  
    ARGUMENTS:
    + x,u,d         - State, input and disturbance variable
    + t             - Variable that indicate the real time
    
    OUTPUTS:
    + x_model       - Non-linear model function     
    """ 
     
  
    return vertcat(dCAdt, dTdt)

# Output Map
def User_fym(x,u,d,t,py):
    """
    SUMMARY:
    It constructs the function fy_m for the non-linear case
    
    SYNTAX:
    assignment = User_fym(x,u,d,t)
  
    ARGUMENTS:
    + x,d           - State and disturbance variable
    + t             - Variable that indicate the current iteration
    
    OUTPUTS:
    + fy_p      - Non-linear plant function     
    """ 
    
    
    return fy_model
    

# 2.3) Disturbance model for Offset-free control
offree = "no" 


# 2.4) Initial condition
x0_p = np.array([1., 400]) # plant
x0_m = np.array([1., 400]) # model
u0 = np.array([300])

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
### 3) State Estimation
 
# what is the minimum needed for estimating disturbance in LIN?


# what is the minimumneeded for estimating disturbance in NL?


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
### 4) Steady-state and dynamic optimizers

# 4.1) Setpoints
def defSP(t):
    """
    SUMMARY:
    It constructs the setpoints vectors for the steady-state optimisation 
    
    SYNTAX:
    assignment = defSP(t)
  
    ARGUMENTS:
    + t             - Variable that indicates the current time
    
    OUTPUTS:
    + ysp, usp, xsp - Input, output and state setpoint values      
    """ 
    
    return [ysp, usp, xsp]
    
# 4.2) Bounds constraints
## Input bounds
umin = 
umax = 

## State bounds
xmin = 
xmax = 

# ## Output bounds
ymin = 
ymax = 



# 4.3) Steady-state optimization : objective function
Qss =  #Output matrix
Rss = # Control matrix

# 4.4) Dynamic optimization : objective function 
Q = #
S = #np.array([[0.1, 0.0], [0.0, 0.1]])