import cv2 
import numpy as np
import time
import imutils

from time import sleep
#import math
import threading
import DobotDllType as dType
#import sys

#影像重心設定
X_Center = 120 #120
Y_Center = 140

#設置攝像頭編號
capture = cv2.VideoCapture(0)
#print("fps"+str(capture.get(5)))

#設定讀取之cfg檔及權重檔
#net = cv2.dnn.readNetFromDarknet("D:/test/color/cfg/yolov3.cfg","D:/test/color/cfg/weights/yolov3_3000.weights")
net = cv2.dnn_DetectionModel("D:/test/color/cfg/yolov4-custom.cfg","D:/test/color/cfg/weights/yolov4-custom_2500.weights")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
classes = [line.strip() for line in open("D:/test/color/cfg/color.names")]
colors = [(255,0,0),(0,255,0),(0,0,255),(0,255,255)]


#偵測物件之函式定義
def yolo_detect(frame):
    # forward propogation
    img = cv2.resize(frame, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape 
    blob = cv2.dnn.blobFromImage(img, 1/255.0, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # get detection boxes
    class_ids = []
    confidences = []
    boxes = []
    
    label = "None"
    x=""
    y=""
    w=""
    h=""
    
    for out in outs:
        for detection in out:
            tx, ty, tw, th, confidence = detection[0:5]
            scores = detection[5:]
            class_id = np.argmax(scores)  
            if confidence > 0.5:   
                center_x = int(tx * width)
                center_y = int(ty * height)
                w = int(tw * width)
                h = int(th * height)

                # 取得箱子方框座標
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                
    # draw boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.4)
    font = cv2.FONT_HERSHEY_COMPLEX
    
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            color = colors[class_ids[i]]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 1)
            cv2.putText(img, label, (x, y -5), font, 0.3, color, 1)
                                
    return img, label, x, y, w, h
    

#Dobot init 連接手臂
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#Load Dll
api = dType.load()

#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])
    
#亮度調整之函式定義
def adjust_gamma(image, gamma=1.0):

   invGamma = 1.0 / gamma
   table = np.array([((i / 255.0) ** invGamma) * 255
      for i in np.arange(0, 256)]).astype("uint8")

   return cv2.LUT(image, table)

#儲存欲執行指令之函式定義
def work(lastIndex):
    #Start to Execute Command Queued
    dType.SetQueuedCmdStartExec(api)    
    
    #Wait for Executing Last Command 
    while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:       
        dType.dSleep(100)

    dType.SetQueuedCmdClear(api)

#調整之函式定義    
def Dobot_adjust():
    dType.SetEMotor(api, 0, 1, 5000,1)
    dType.SetWAITCmd(api, 12.12, isQueued=1)
    work(lastIndex)
    
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 268.3032, 0 , 50, 0, 1)
    dType.SetWAITCmd(api, 1, isQueued=1)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 268.3032, 0 , 15, 0, 1)
    work(lastIndex)

#停止執行之函式定義
def dobot_stop():
    dType.SetEMotor(api, 0, 1, 0,1)
    lastIndex = dType.SetWAITCmd(api, 3, isQueued=1)
    dType.SetEMotor(api, 0, 1, 0,1)
    work(lastIndex)
 
color_state = "None"
color0 = "None"
x0 = ""
y0 = ""
w0 = ""
h0 = ""
co = 0

cob=0
cog=0
cor=0
coy=0

#手臂運轉之函式定義    
def Dobot_work(offx, offy,offr,color):
    color_state = color
    count=0
    """
    if color_state == "blue":
        count=cob
    if color_state == "green":
        count=cog
    if color_state == "red":
        count=cor
    if color_state == "yellow":
        count=coy
    """
    
    #控制手臂速度與加速度
    dType.SetPTPJointParamsEx(api,55,90,55,90,55,90,55,90,1) 
    #讓程式運行一秒
    dType.SetWAITCmd(api, 1, isQueued=1)
    #輸送帶以50的速度運轉
    dType.SetEMotor(api, 0, 1, 5000,1)
    #讓程式運行兩秒
    dType.SetWAITCmd(api, 0.6, isQueued=1)
    #手臂開始移動
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, offx, offy , 30, 0, 1)
    #移動到判斷的位置
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, offx, offy , 6.7259, 0, 1)
    #吸取
    dType.SetEndEffectorSuctionCup(api, 1,  1, isQueued=1)
    #邊吸邊動
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, offx, offy-30 , 4, 0, 1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, offx, offy-30 , 70, 0, 1)
    
    dType.SetWAITCmd(api, 0.5, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 212, -151.8156 , 70, 0, 1)
    dType.SetWAITCmd(api, 0.25, isQueued=1)
    
    #print("color_state = " + str(color_state))
    goal_x=0
    if(color_state == "yellow"):
        goal_x=140.0
    elif(color_state == "blue"):
        goal_x=10.0539
    elif(color_state == "red"):
        goal_x=190.8678
    elif(color_state == "green"):
        goal_x=70.0
    print("goal_x =" + str(goal_x))
    
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, goal_x, (-35*count)-212.8156 , 70, offr, 1)
    dType.SetWAITCmd(api, 2, isQueued=1)
    #放下
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, goal_x, (-35*count)-212.8156 , -56.2392, offr, 1)
    dType.SetWAITCmd(api, 2, isQueued=1)
    #放開
    dType.SetEndEffectorSuctionCup(api, 1,  0, isQueued=1)
    dType.SetWAITCmd(api, 3, isQueued=1)
    #抬起來
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, goal_x, (-35*count)-212.8156 , 70, offr, 1)
    dType.SetWAITCmd(api, 2, isQueued=1)
    #移回去
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 212, -151.8156 , 70, 0, 1)
    dType.SetWAITCmd(api, 0.5, isQueued=1)
    
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 268.3032, 0 , 50, 0, 1)   
    dType.SetEMotor(api, 0, 1, 5000,1)
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)
    
    work(lastIndex)
    
    sleep(0.5)
    print("==============================")
    
#手臂確認連接執行歸零及歸位
if (state == dType.DobotConnect.DobotConnect_NoError):

    #Clean Command Queued
    dType.SetQueuedCmdClear(api)


    dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued = 1)
    dType.SetPTPJointParams(api,200,200,200,200,200,200,200,200, isQueued = 1)
    dType.SetPTPCoordinateParams(api,200,200,200,200, isQueued = 1)
    dType.SetPTPJumpParams(api, 10, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
    
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)
    lastIndex = dType.SetWAITCmd(api, 20, isQueued=1)
    work(lastIndex)
    #sleep(0.5)

#顯示即時影像之函式定義
def image_show():
    global color0, x0, y0, w0, h0, co
    while True:
        tstart = time.time()
        ret, cap_input = capture.read()
        img, color, x, y, w, h = yolo_detect(cap_input)
        # 顯示畫面   
        cv2.imshow("Frame", imutils.resize(img, width=640))
        
        color0 = color
        if color!="None":
            if h/2.0+y <50 and co == 0:
                while True:
                    tend = time.time()
                    if tend - tstart >= 1.4:
                        #print("wait for %f sec."  % (tend - tstart))
                        break
                x0 = x
                y0 = y
                w0 = w
                h0 = h
                co = 1
        
        #tend = time.time()
        #print("Cost %f sec to identify." % (tend - tstart))
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            dobot_stop()
            co = 2
            break
    
#機械手臂執行進行分類之函示定義
def Dobot_execute():
    flag_adjust = False
    flag_start_work = False
    flag_debug = False
    flag_detect = False
    n1 = 0
    n0 = 0
    global color0, x0, y0, w0, h0, co,cob,cog,cor,coy
    
    while True:
        color = color0
        x = x0
        y = y0
        w = w0
        h = h0
        
        if flag_start_work == True:
            n1=n1+1      
            if color!="None" and x!= "":
                n1=0
                #取得顏色
                color_state = color
                
                #利用影像中目標物的矩，算出重心位置
                cX = w/2.0+x
                cY = h/2.0+y
                
                
                dY = cY-26.0
                #X_Center = 130 Y_Center = 140 

                
                #算出手臂預設位置與重心的差距
                
                if cY >= 49.0:
                    offy = -66.0
                elif cY >= 48.0 and cY < 49.0:
                    offy = -66.5
                elif cY >= 47.0 and cY < 48.0:
                    offy = -67.0
                elif cY >= 46.0 and cY < 47.0:
                    offy = -67.5
                elif cY >= 45.0 and cY < 46.0:
                    offy = -69.0
                elif cY >= 43.5 and cY < 45.0:##
                    offy = -71.25
                elif cY >= 42.0 and cY < 43.5:##
                    offy = -71.5
                elif cY >= 41.5 and cY < 42.0:##
                    offy = -71.75
                elif cY >= 40.5 and cY < 41.5:
                    offy = -72.0
                elif cY >= 39.0 and cY < 40.5:
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.65662756    #0.5053568 0.5001383
                    offy = -72.5
                elif cY>=35.5 and cY<39.0:##
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.64662756
                    offy = -87.5  #37+5  #35+3
                elif cY<35.5 and cY>=32.0:                   
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.63
                    offy = -89.5
                    
                elif cY<32.0 and cY>=28.0:                   
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.63
                    offy = -96.5
                elif cY<28.0 and cY>=25.0:
                    offy = -101.125
                elif cY<25.0 and cY>=22.5:
                    offy = -105.75
                elif cY<22.5 and cY>=22.0: #
                    offy = -110.5
                elif cY<22.0 and cY>=19.0: #<20
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.62
                    #offy = (cY-dY+(dY/3.5)-Y_Center)*0.63   #0.4857797
                    offy = -115.0
                else: #<19
                    offy = -120.0
                
                
                if color_state == "blue" and cY> 40.5:
                    offy = offy - 6
                
                if cX <= 100.0:
                    offy = offy - 10
                              
                 
                if(cX-X_Center) >= 14.5:
                    offx = (X_Center-cX)*1.14317147#0.653842022 #1.77821111     148-120
                    
                elif(cX-X_Center)<14.5 and (cX-X_Center)>=0:
                    offx = (X_Center-cX)*0.4921233*2+5
                elif(cX-X_Center)>-14.5 and (cX-X_Center)<0:
                    offx = (X_Center-cX)*0.5138767*2+5
                    
                else:
                    offx = (X_Center-cX)*1.712952 #0.7810596      #1.28467463   #0.6079283  #1.06201185   95-120=-的
                #算出手臂移動的距離
                offxa = 268.3032+offx
                offya = offy+63
                offra = 0
                if(cY < 50):
                    print("color: "+color_state)
                   # print("x: "+ str(x))
                   # print("y: "+ str(y))
                   # print("w: "+ str(w))
                   # print("h: "+ str(h))
                    print("cx: "+ str(cX))
                    print("cY: "+ str(cY))
                    print("offxa: "+ str(offxa))
                    print("offya: "+ str(offya))    
                    
                    Dobot_work(offxa,offya,offra,color_state)
                    flag_start_work = False
                    n0 = n0+1
                    color0 = "None"
                    x0 = ""
                    y0 = ""
                    w0 = ""
                    h0 = ""
                    co = 0
                    
                    if color_state == "blue":
                        cob = cob+1
                    if color_state == "green":
                        cog = cog+1
                    if color_state == "red":
                        cor = cor+1
                    if color_state == "yellow":
                        coy = coy+1
            
                n1=0
                color_state = "None"
                    
        else:
            n1=0
            color_state = "None"        
                
        if (flag_adjust == True):
            Dobot_adjust()
            flag_adjust = False
        
        flag_start_work = True
        
        if co == 2:
            dobot_stop()
            break
        

#持續運轉輸送帶
last=dType.SetEMotor(api, 0, 1, 5000,1)
work(last)

thread1 = threading.Thread(target = image_show)
thread2 = threading.Thread(target = Dobot_execute)

thread2.start()
thread1.start()
    
thread1.join()
thread2.join()

print("Escape hit, closing Frame...")
cv2.destroyAllWindows()
capture.release()
#ser.close()
cv2.destroyAllWindows()

print("disconnect...")
#Stop to Execute Command Queued
dType.SetQueuedCmdStopExec(api)
#Disconnect Dobot
dType.DisconnectDobot(api)
#dobot_stop()


