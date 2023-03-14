import threading
import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#load dll
api = dType.load()

#connect dobot
state = dType.ConnectDobot(api, "", 115200)[0]

print(state)

if(state == dType.DobotConnect.DobotConnect_NoError):
    #CLEAN commendqueued
    dType.SetQueuedCmdClear(api)
    #async motion params setting
    dType.SetHOMEParams(api,200, 150, 50, 0, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
    #async HOME
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    #Async PTPMotion
   
        
    #Start to Execute Command Queued
    dType.SetQueuedCmdStartExec(api)
    
    #wait for executing last command
    while lastindex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(1000)
       
    #stop to  execute command Queued
    dType.SetQueuedCmdStopExec(api)
        
    #disconnect Dobot
    dType.DisconnectDobot(api)
        