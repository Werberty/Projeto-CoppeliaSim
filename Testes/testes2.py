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

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True,
                         5000, 5)  # Connect to CoppeliaSim


def line_side(pose, initPos, goalPos):
    xDiff1 = initPos[0]-goalPos[0]
    yDiff1 = goalPos[1]-initPos[1]
    xDiff2 = initPos[0]-pose[0]
    yDiff2 = initPos[1]-pose[1]
    dGoalIni = np.sqrt(xDiff1**2 + yDiff1**2)
    d = (xDiff2*yDiff1 + yDiff2*xDiff1)/dGoalIni
    return d
    # if d > 0:
    #     return 1
    # else:
    #     return -1


if clientID != -1:
    print('Connected to remote API server')

    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(
        clientID, robotname, sim.simx_opmode_oneshot_wait)

    returnCode, l_wheel = sim.simxGetObjectHandle(
        clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(
        clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Handles para os sonares
    returnCode, sonar_front = sim.simxGetObjectHandle(
        clientID, robotname + '_ultrasonicSensor5', sim.simx_opmode_oneshot_wait)
    returnCode, sonar_right = sim.simxGetObjectHandle(
        clientID, robotname + '_ultrasonicSensor7', sim.simx_opmode_oneshot_wait)
    
    # Posição final Goal
    qgoal = np.array([4, 4, np.deg2rad(45)])
    # qgoal = np.array([-2, -4, np.deg2rad(180)])

    # Frame que representa o Goal
    returnCode, goalFrame = sim.simxGetObjectHandle(
        clientID, 'Goal', sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectPosition(
        clientID, goalFrame, -1, [qgoal[0], qgoal[1], 0], sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectOrientation(
        clientID, goalFrame, -1, [0, 0, qgoal[2]], sim.simx_opmode_oneshot_wait)
    
    # Posição atual do robô
    returnCode, robotPos = sim.simxGetObjectPosition(
        clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    returnCode, robotOri = sim.simxGetObjectOrientation(
        clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    initPose = np.array([robotPos[0], robotPos[1], robotOri[2]])

    # Específico do robô
    L = 0.331
    r = 0.09751

    following = False

    t = 0
    # Lembrar de habilitar o 'Real-time mode'
    startTime = time.time()
    lastTime = startTime
    while t < 90:

        now = time.time()
        dt = now - lastTime

        # Fazendo leitura dos sensores
        returnCode, detected_front, point_front, *_ = sim.simxReadProximitySensor(
            clientID, sonar_front, sim.simx_opmode_oneshot_wait)
        returnCode, detected_right, point_right, *_ = sim.simxReadProximitySensor(
            clientID, sonar_right, sim.simx_opmode_oneshot_wait)
        
        # Posição atual do robô
        returnCode, robotPos = sim.simxGetObjectPosition(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        returnCode, robotOri = sim.simxGetObjectOrientation(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

        # Velocidades iniciais
        v = .4
        w = 0

        obstacle_in_front = (
            detected_front and np.linalg.norm(point_front) < .5)
        obstacle_in_right = (
            detected_right and np.linalg.norm(point_right) < .5)

        line = line_side(robotConfig, initPose, qgoal)

        # Controle
        if obstacle_in_front:
            v = 0.1
            w = np.deg2rad(30)
            following = True
        else:
            if obstacle_in_right:
                w = np.deg2rad(10)
            elif following:
                v = .4
                w = np.deg2rad(-30)
                if line < 0.1 and line > -0.1:
                    v = 0
                    w = 0
                    # following = False


        # Cinemática Inversa
        wr = ((2.0*v) + (w*L))/(2.0*r)
        wl = ((2.0*v) - (w*L))/(2.0*r)

        # Enviando velocidades
        sim.simxSetJointTargetVelocity(
            clientID, l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            clientID, r_wheel, wr, sim.simx_opmode_oneshot_wait)

        t = t + dt
        lastTime = now

    sim.simxSetJointTargetVelocity(
        clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')

print('Program ended')
