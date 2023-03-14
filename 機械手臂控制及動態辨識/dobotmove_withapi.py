import DobotDllType as dType
import requests
import time


CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):

    #Clean Command Queued
    dType.SetQueuedCmdClear(api)

    #Async Motion Params Setting
    dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)

    #Async Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    
time.sleep(10)
X_start = 250
Y_start = 0
Z_start = 50
R_start = 0
counter=0

while True:
    #global X_start, Y_start, Z_start, R_start, counter
    url = 'http://140.137.41.136:1380/A6230957/ControlMove/api/Controls/python'
    myobj1 = {'Value':998}
    x1 = requests.post(url, data = myobj1)
    x1_json = x1.json()
    print(x1_json['Result'])
    print(x1_json['Direction'])
    if (str(x1_json['Result'])=="Start"):        
        counter = counter+1
        if (x1_json['Direction']=='X+'):
            X_start=X_start+20
        elif (x1_json['Direction']=='X-'):
            X_start=X_start-20
        elif (x1_json['Direction']=='Y+'):
            Y_start=Y_start+20
        elif (x1_json['Direction']=='Y-'):
            Y_start=Y_start-20
        elif (x1_json['Direction']=='Z+'):
            Z_start=Z_start+20
        elif (x1_json['Direction']=='Z-'):
            Z_start=Z_start-20
        elif (x1_json['Direction']=='R+'):
            R_start=R_start+20
        elif (x1_json['Direction']=='R-'):
            R_start=R_start-20
        elif (x1_json['Direction']=='C'):
            dType.SetQueuedCmdClear(api)
            lastIndex = dType.SetHOMECmd(api, temp = 0, isQueued = 1)[0]
            X_start = 250
            Y_start = 0
            Z_start = 50
            R_start = 0
            
        """
        if (state == dType.DobotConnect.DobotConnect_NoError):

            #Clean Command Queued
            dType.SetQueuedCmdClear(api)
            
            #Async Motion Params Setting
            dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
            dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
            dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
            
            #Async Home
            dType.SetHOMECmd(api, temp = 0, isQueued = 1)
        """
        if (x1_json['Direction']!='C'):
            dType.SetQueuedCmdClear(api)
            #Async PTP Motion
            lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, X_start, Y_start, Z_start, R_start, isQueued = 1)[0]
            #dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 212, -151.8156 , 70, 0, 1)
        

        #Start to Execute Command Queued
        dType.SetQueuedCmdStartExec(api)
        
        #Wait for Executing Last Command 
        
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)            
            #Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)
        location = "X:"+str(X_start)+",Y:"+str(Y_start)+",Z:"+str(Z_start)+",R:"+str(R_start)
        myobj2 = {'Value':123, 'Location':location}
        x2 = requests.post(url, data = myobj2)
        x2_json = x2.json()
        if (x2_json['Result']=="機械手臂移動成功"):
            print(location)
        
    else:
        time.sleep(1)
    
    if(counter>20):
        break
    time.sleep(1)

#Disconnect Dobot
dType.DisconnectDobot(api)   
