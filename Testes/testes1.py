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


def conectar():
    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19999, True, True,
                             5000, 5)  # Connect to CoppeliaSim
    return clientID


def iniciar(clientID):
    global robotLeftMotorHandle, robotRightMotorHandle, robotHandle

    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(
        clientID, robotname, sim.simx_opmode_oneshot_wait)

    returnCode, robotLeftMotorHandle = sim.simxGetObjectHandle(
        clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, robotRightMotorHandle = sim.simxGetObjectHandle(
        clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Handles para os sonares
    returnCode, sonar_front = sim.simxGetObjectHandle(
        clientID, robotname + '_ultrasonicSensor5',
        sim.simx_opmode_oneshot_wait)
    returnCode, sonar_right = sim.simxGetObjectHandle(
        clientID, robotname + '_ultrasonicSensor7',
        sim.simx_opmode_oneshot_wait)

    # Goal configuration (x, y, theta)
    qgoal = np.array([4, 4, np.deg2rad(45)])
    # qgoal = np.array([-2, -4, np.deg2rad(180)])

    # Frame que representa o Goal
    returnCode, goalFrame = sim.simxGetObjectHandle(
        clientID, 'Goal', sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectPosition(
        clientID, goalFrame, -1, [qgoal[0], qgoal[1], 0],
        sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectOrientation(
        clientID, goalFrame, -1, [0, 0, qgoal[2]],
        sim.simx_opmode_oneshot_wait)

    distancia = obter_distancia(clientID, sonar_front, sonar_right)
    robotConfig = obter_poseRobot(clientID, robotHandle)
    dx, dy, dth = qgoal - robotConfig
    print(f'Distancia: {distancia}\n\
Posição do robô: {robotConfig}\n\
Posição do Goal: {dx}, {dy}, {np.rad2deg(dth)}'
          )

    # go_to_goal(clientID, qgoal)
    

    # time.sleep(5)


def obter_distancia(clientID, sonar_front, sonar_right):
    # Fazendo leitura dos sensores
    returnCode, detected_front, point_front, *_ = sim.simxReadProximitySensor(
        clientID, sonar_front, sim.simx_opmode_oneshot_wait)
    # returnCode, detected_right, point_right, *_ = sim.simxReadProximitySensor(
    #     clientID, sonar_right, sim.simx_opmode_oneshot_wait)

    return [detected_front, np.linalg.norm(point_front)]


def obter_poseRobot(clientID, robotHandle):
    returnCode, robotPos = sim.simxGetObjectPosition(
        clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    returnCode, robotOri = sim.simxGetObjectOrientation(
        clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

    return robotConfig


def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi


def go_to_goal(clientID, qgoal):
    time.sleep(0.5)
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


def finalizar(clientID):
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)


if __name__ == '__main__':
    clientID = conectar()

    if clientID != -1:
        print('Connected to remote API server')
        iniciar(clientID)
        finalizar(clientID)
