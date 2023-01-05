import cv2
import numpy as np
import imutils
import threading
import math
import time
import DobotDllType as dType

from multiprocessing import Process, Pipe

api = dType.load()

def dobot_get_start():
    
    dType.SetQueuedCmdClear(api)

    dType.SetHOMEParams(api, 200, 0, 20, 30, isQueued = 1) # x, y, z, r 
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1) # velocity[4], acceleration[4]
    dType.SetPTPCommonParams(api, 300, 300, isQueued = 1) # velocityRatio(속도율), accelerationRation(가속율)
   
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

    #dType.SetQueuedCmdStartExec(api)

    #dType.SetQueuedCmdClear(api)

def dobot_connect():

    CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound", 
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:",CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):
        dobot_get_start()

def move_dobot(of_x,of_y):
    
    offset_x = round(float(of_x))
    offset_y = round(float(of_y))

    last_index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, offset_x, offset_y, -10, 0, isQueued = 1)[0]

    dType.SetQueuedCmdStartExec(api)
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(1)

    dType.SetQueuedCmdStopExec(api)

    return offset_x, offset_y

def control_dobot(conn):

    list = conn.recv()

    of_x = list[0]
    of_y = list[1]

    x,y = move_dobot(of_x,of_y)

    conn.send([x,y])

def f1(conn):

    x = 200
    y = -45

    time.sleep(10)

    for i in range(9):

        
        conn.send([x, y+(i*10)])

    conn.close()

if __name__ == '__main__':

    dobot_connect()

    p_conn,c_conn = Pipe()
    p_conn2,c_conn2 = Pipe()

    time.sleep(10)

    p = Process(target=f1, args=(c_conn,))
    p.start()

    while(1):

        q = p_conn.recv()

        print(q)

        of_x = q[0]
        of_y = q[1]

        move_dobot(of_x,of_y)

        pose = dType.GetPose(api)
        print( "x: ", pose[0], "y: ", pose[1], "z: ", pose[2])
        print("\n")
