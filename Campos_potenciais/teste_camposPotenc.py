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


def readSensorData(clientId=-1,
                   range_data_signal_id="hokuyo_range_data",
                   angle_data_signal_id="hokuyo_angle_data"):

    # the first call should be non-blocking to avoid getting out-of-sync angle data
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(
        clientId, range_data_signal_id, sim.simx_opmode_streaming)

    # the second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(
        clientId, angle_data_signal_id, sim.simx_opmode_blocking)

    # check the if both data were obtained correctly
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # unpack data from range and sensor messages
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    # return none in case were nothing was gotten from the simulator
    return None


def vetor_de_atracao():
    pass 


if clientID != -1:
    print('Connected to remote API server')

    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(
        clientID, robotname, sim.simx_opmode_oneshot_wait)

    returnCode, l_wheel = sim.simxGetObjectHandle(
        clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(
        clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Goal configuration (x, y, theta)
    qgoal = np.array([3, -3, np.deg2rad(90)])
    # qgoal = np.array([-2, -4, np.deg2rad(180)])

    # # Frame que representa o Goal
    # returnCode, goalFrame = sim.simxGetObjectHandle(
    #     clientID, 'Goal', sim.simx_opmode_oneshot_wait)
    # returnCode = sim.simxSetObjectPosition(
    #     clientID, goalFrame, -1, [qgoal[0], qgoal[1], 0], sim.simx_opmode_oneshot_wait)
    # returnCode = sim.simxSetObjectOrientation(
    #     clientID, goalFrame, -1, [0, 0, qgoal[2]], sim.simx_opmode_oneshot_wait)

    # Handle para os dados do LASER
    laser_range_data = "hokuyo_range_data"
    laser_angle_data = "hokuyo_angle_data"

    # Geralmente a primeira leitura é inválida (atenção ao Operation Mode)
    # Em loop até garantir que as leituras serão válidas
    returnCode = 1
    while returnCode != 0:
        returnCode, range_data = sim.simxGetStringSignal(
            clientID, laser_range_data, sim.simx_opmode_streaming + 10)

    # Específico do robô
    L = 0.331
    r = 0.09751
    maxv = 1.0
    maxw = np.deg2rad(45)

    t = 0
    # Lembrar de habilitar o 'Real-time mode'
    # startTime = time.time()
    # lastTime = startTime
    rho = np.inf
    while rho > 0.1:

        # now = time.time()
        # dt = now - lastTime

        returnCode, robotPos = sim.simxGetObjectPosition(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        returnCode, robotOri = sim.simxGetObjectOrientation(
            clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

        dx, dy = qgoal[:2] - robotConfig[:2]

        # Apenas para interromper o loop
        rho = np.sqrt(dx**2 + dy**2)

        # Fazendo leitura dos sensores
        raw_range_data, raw_angle_data = readSensorData(
            clientID, laser_range_data, laser_angle_data)
        laser_data = np.array([raw_angle_data, raw_range_data]).T

        # dx, dy = [np.cos(laser_data[0, 0]), np.sin(laser_data[0, 1])]

        # dx, dy = 0, 0
        # for laser in laser_data:
        #     x = laser[1] * np.cos(laser[0])
        #     y = laser[1] * np.sin(laser[0])
        #     dx += x
        #     dy += y
        #     # print(dx, dy)
        
        kr = 1 
        kt = 2 

        v = kr*(dx*np.cos(robotConfig[2]) + dy*np.sin(robotConfig[2]))
        w = kt*(np.arctan2(dy, dx) - robotConfig[2])

        # Limit v,w to +/- max
        v = max(min(v, maxv), -maxv)
        w = max(min(w, maxw), -maxw)

        # Cinemática Inversa
        wr = ((2.0*v) + (w*L))/(2.0*r)
        wl = ((2.0*v) - (w*L))/(2.0*r)

        # Enviando velocidades
        sim.simxSetJointTargetVelocity(
            clientID, l_wheel, wl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(
            clientID, r_wheel, wr, sim.simx_opmode_oneshot_wait)

        # t = t + dt
        # lastTime = now

    sim.simxSetJointTargetVelocity(
        clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')

print('Program ended')
