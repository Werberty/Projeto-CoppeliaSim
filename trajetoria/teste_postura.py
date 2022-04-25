try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')


import time

import numpy as np

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')

    robotname = 'Manta'
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)    
    
    returnCode, steerHandle = sim.simxGetObjectHandle(clientID, 'steer_joint', sim.simx_opmode_oneshot_wait)
    returnCode, motorHandle = sim.simxGetObjectHandle(clientID, 'motor_joint', sim.simx_opmode_oneshot_wait)
    
    motor_torque = 60
    returnCode = sim.simxSetJointForce(clientID, motorHandle, motor_torque, sim.simx_opmode_oneshot_wait)
    
    motor_velocity = 12
    returnCode = sim.simxSetJointTargetVelocity(clientID, motorHandle, motor_velocity, sim.simx_opmode_oneshot_wait)
  
    returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    print(returnCode, robotPos)
    
    max_steer = np.deg2rad(20)
    
    # Estado objetivo é y = 0
    setpoint = 0
    
    error = 0
    lastError = 0
    lastTime = 0
    cumError = 0
    
    # kp = .4, kd = .5, ki = .01
    # Lembrar de habilitar o 'Real-time mode'
    t = 0
    lastTime = time.time()
    while t < 25:
        
        time.sleep(0.005)
        
        returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming + 10)
        #print(returnCode, robotPos)
        
        now = time.time()
        dt = now - lastTime
        
        # Error        
        error = setpoint - robotPos[1]
        dError = (error - lastError) / dt
        cumError += error * dt
               
        # Proporcional
        kp = .4
        up = kp*error
        
        # Derivativo
        kd = .5
        ud = kd*dError
        
        # Integral
        ki = .01
        ui = ki*cumError
    
        # Controller
        u = up + ud + ui
    
        # Limitando o valor para +/- max
        u = max(min(u, max_steer), -max_steer)
        
        returnCode = sim.simxSetJointTargetPosition(clientID, steerHandle, u, sim.simx_opmode_streaming + 10)

        t = t + dt        
        lastTime = now 
        lastError = error

    motor_velocity = 0
    returnCode = sim.simxSetJointTargetVelocity(clientID, motorHandle, motor_velocity, sim.simx_opmode_oneshot_wait)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    
print ('Program ended')