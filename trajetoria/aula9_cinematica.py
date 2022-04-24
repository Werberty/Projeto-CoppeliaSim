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


def Rz(theta):

    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

# Normalize angle to the range [-pi,pi)


def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi


print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True,
                         5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')

    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(
        clientID, robotname, sim.simx_opmode_oneshot_wait)

    returnCode, robotLeftMotorHandle = sim.simxGetObjectHandle(
        clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, robotRightMotorHandle = sim.simxGetObjectHandle(
        clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Goal configuration (x, y, theta)
    qgoal = np.array([2, 2, np.deg2rad(90)])
    # qgoal = np.array([-2, -4, np.deg2rad(180)])

    # Frame que representa o Goal
    returnCode, goalFrame = sim.simxGetObjectHandle(
        clientID, 'Goal', sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectPosition(
        clientID, goalFrame, -1, [qgoal[0], qgoal[1], 0], sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectOrientation(
        clientID, goalFrame, -1, [0, 0, qgoal[2]], sim.simx_opmode_oneshot_wait)

    # Específico do robô
    L = 0.331
    r = 0.09751
    maxv = 1.0
    maxw = np.deg2rad(45)

    rho = np.inf
    while rho > .05:

        returnCode, robotPos = sim.simxGetObjectPosition(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        returnCode, robotOri = sim.simxGetObjectOrientation(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

        dx, dy, dth = qgoal - robotConfig

        rho = np.sqrt(dx**2 + dy**2)
        alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy, dx))
        beta = normalizeAngle(qgoal[2] - np.arctan2(dy, dx))

        kr = 4 / 20
        ka = 8 / 20
        kb = -1.5 / 20

        # Alvo na parte de trás
        if abs(alpha) > np.pi/2:
            kr = -kr

            # Se não ajustar a direção muda
            alpha = normalizeAngle(alpha-np.pi)
            beta = normalizeAngle(beta-np.pi)

        v = kr*rho
        w = ka*alpha + kb*beta

        # Limit v,w to +/- max
        v = max(min(v, maxv), -maxv)
        w = max(min(w, maxw), -maxw)

        wr = ((2.0*v) + (w*L))/(2.0*r)
        wl = ((2.0*v) - (w*L))/(2.0*r)

        sim.simxSetJointTargetVelocity(
            clientID, robotRightMotorHandle, wr, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            clientID, robotLeftMotorHandle, wl, sim.simx_opmode_oneshot_wait)

    sim.simxSetJointTargetVelocity(
        clientID, robotRightMotorHandle, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, robotLeftMotorHandle, 0, sim.simx_opmode_oneshot_wait)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')

print('Program ended')
