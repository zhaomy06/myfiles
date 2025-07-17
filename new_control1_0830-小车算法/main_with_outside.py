##2022/2/23日修改内容：
#1.修改了加速油门策略，注释掉了接近点位的处理，使油门可以正常运行
#2.在循迹算法主程序中发现转向输出单位为度°而不是百分比，造成小车输出转向角度限值，修复bug为原始 角度值/45*0.98 (树莓派算法角度范围为-45-45，stm32限制为-0.98-0.98)
#   以此缩放
#陈祺遥 2022年3月2日修改版本
import base64
import hashlib
import traceback
import serial
import sys
import os
import time
import re
import math
import threading
import Readline
import numpy as np
from straight_controller import Straight_controller,PI
from lqr_controller import LQR_controller
import configparser as cfg
import datetime
'''
import wiringpi
wiringpi.piHiPri(99)
import psutil
current_process = psutil.Process()
current_process.cpu_affinity([3])
'''



print('start running !!!')
fname='trajeo.txt'

#ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.4)
#serPC=serial.Serial("/dev/ttyAMA0",115200,timeout=2)
#设置串口，serpc为树莓派与4G模块接口，ser为与stm32小车之间通讯

ser=serial.Serial('/dev/ttyS0',115200,timeout=0.4)
serPC=serial.Serial('/dev/ttyS1',115200,timeout=5)
kk = re.compile(r'\d+\.?\d+|\d+|-\d+\.?\d+')#编译正则表达式，用以检索串口报文数据，可以加速处理串口报文
ser.write(bytes('Ready \r\n',encoding='UTF-8'))
send=0#文件发送标志位0为为发送，1为发送中
PCorder=''#pc指令变量
dataraw=''#存放stm32发送报文
data=''

tk=Straight_controller()#mode3用
lqr=LQR_controller()#mode2用

mode=1#mode 0录制，1 正常 ，2 回放 3.直线
tempstr=0

setting=cfg.ConfigParser()
setting.read('RobotPi.ini')
tk.kp=float(setting.get("PID","thr_p"))
tk.ki=float(setting.get("PID","thr_i"))
tk.kd=float(setting.get("PID","thr_d"))
tk.kSteer = float(setting.get("PID", "str_k"))
edgeX1=float(setting.get("edge", "edgeX1"))
edgeY1=float(setting.get("edge", "edgeY1"))
edgeX2 = float(setting.get("edge", "edgeX2"))
edgeY2 = float(setting.get("edge", "edgeY2"))
edgeX3 = float(setting.get("edge", "edgeX3"))
edgeY3 = float(setting.get("edge", "edgeY3"))
edgeX4 = float(setting.get("edge", "edgeX4"))
edgeY4 = float(setting.get("edge", "edgeY4"))


def pcinput():#pc上位机指令解析程序
    global mode 
    global send
    global tempstr
    global edgeX1
    global edgeX2
    global edgeX3    
    global edgeX4
    global edgeY1
    global edgeY2
    global edgeY3    
    global edgeY4
    while(1):
        serPC.flushInput()
        PCraw=serPC.readline()
        PCorder =PCraw.decode()[:-2] #去\r\n
        if PCorder != '':
            #print(PCorder)
            if send ==0:
                if PCorder=='track':
                    mode=2
                    lqr.lqr_mode=1
                    print('xunji')
                if PCorder=='normal':
               
                    if mode==2:
                         mode=1
                         time.sleep(0.5)
                    tk.thr=0
                    tk.steer=0
                    tk.brk=0
                    car_control(0,0,0)
                    print('normal')
                
                if PCorder=='sdstart':
                    send=1
                    data=''
                    serPC.flushOutput()
                    jarFile = open(fname,"wb+")
                    jarFile.close()
                    print('sendstart')
                # receve.readpart('trajeo.txt')
                if PCorder=='updata':
                    print("updata updata")
                    try:
                        preFile = np.loadtxt('trajeo.txt')
                    except:
                        print('err1')
                    
                    # try:
                    x = preFile[:, 0]
                    y = preFile[:, 1]
                    h = preFile[:, 2]
                    v = preFile[:, 3]
                    t = preFile[:, 4]
                    lqr.update_trajectory(x, y, h, v, t)

                    send=1
                    serPC.write(bytes("finish\n",encoding='UTF-8'))
                    time.sleep(0.2)
                    serPC.write(bytes("finish\n",encoding='UTF-8'))
                    print('finish')
                    send=0
                    # except:
                    #     print('updata err')
                if  PCorder[:5]=='Ready':
                    print('pidold'+str(tk.kp))
                    PID=kk.findall(PCorder)
                    lqr.ki=float(PID[1])
                    lqr.kd=float(PID[2])
                    lqr.kp=float(PID[0])
                    tk.kSteer=float(PID[3])
                    setting.set('PID','thr_p',PID[0])
                    setting.set('PID','thr_i',PID[1])
                    setting.set('PID','thr_d',PID[2])
                    setting.set('PID', 'str_k', PID[3])
                    with open('RobotPi.ini','w+') as f:# 创建配置文件
                        setting.write(f)
                    print('finsh PID '+str(PID))
                if PCorder[:6] == 'Linear':
                    mode = 3
                    r = r'[-0-9.0-9]+'
                    loca = re.findall(r, PCorder)
                    tk.Linearx2 = float(loca[0])
                    tk.Lineary2 = float(loca[1])
                    tk.goal_v = float(loca[2])
                    tk.model = float(loca[3])
                    print(f"have receive the data:x2 = {tk.Linearx2},y2 = {tk.Lineary2},goal_v = {tk.goal_v}")
                    # tk.xylinear(0, 0, loca[0], loca[1])               # 录入起点终点位置
                    print('Linear')
                    tk.thr = 0.0
                    tk.steer = 0.0
                    tk.brake = 0.2
                    car_control(th.thr, th.steer, tk.brake)
                    tk.i = 0
                if  PCorder[:4]=='Edge':
                    print('edge old'+str(edgeX1))
                    EdGe=kk.findall(PCorder)
                    edgeX1=float(EdGe[0])
                    edgeY1=float(EdGe[1])
                    edgeX2=float(EdGe[2])
                    edgeY2=float(EdGe[3])
                    edgeX3=float(EdGe[4])
                    edgeY3=float(EdGe[5])
                    edgeX4=float(EdGe[6])
                    edgeY4=float(EdGe[7])                                                            
                    setting.set("edge", "edgeX1", EdGe[0])
                    setting.set("edge", "edgeY1", EdGe[1])
                    setting.set("edge", "edgeX2", EdGe[2])
                    setting.set("edge", "edgeY2", EdGe[3])
                    setting.set("edge", "edgeX3", EdGe[4])
                    setting.set("edge", "edgeY3", EdGe[5])
                    setting.set("edge", "edgeX4", EdGe[6])
                    setting.set("edge", "edgeY4", EdGe[7])
                    with open('RobotPi.ini','w+') as f:# 创建配置文件
                        setting.write(f)
                    print('finsh Edge '+str(edgeX1))

                if PCorder=='reseti':    #重置循迹原点
                    tk.i=0
                if PCorder=='restart':    #重新启动树莓派
                    os.system('sudo reboot')
                if PCorder=='poweroff':    #关闭树莓派
                    os.system('sudo poweroff')                            
            else:
                if PCorder[:4]=='wait':
                    send=0
                if PCorder[:6]=='sdpart':
                    print('sdpart decode')
                    data=base64.b64decode(PCorder[6:])
                    jarFile = open(fname,"ab+")
                    jarFile.write(data)
                    jarFile.close()
                    print('save')
                    serPC.flushOutput()
                if PCorder[:6]=='sdfnsh':
                    print('sdfinish')
                    serPC.flushOutput()
                    #receve.read('trajeo.txt')
                    data=base64.b64decode(PCorder[6:])
                    jarFile = open(fname,"ab+")
                    jarFile.write(data)
                    jarFile.close()
                    sFile = open(fname,"rb").read()
                    md5=hashlib.md5(sFile).hexdigest()
                    print('finishsave'+md5)
                    serPC.write(bytes('MD5rec:'+md5+"\n", encoding='utf-8'))
                    send=0

            if PCorder=='emstop':   #紧急停止
                
                tempstr=0.5
                if mode==2:
                    mode=1
                    time.sleep(0.5)
                print('emstop')
                car_control(0,0.7,0)

            
readData = threading.Thread(target=pcinput)#创建pc串口指令接收线程
readData.setDaemon(True)#后台服务线程与主程序一起关闭
readData.start()
'''
def isOutside(X,Y):
    global edgeX1
    global edgeX2
    global edgeX3    
    global edgeX4
    global edgeY1
    global edgeY2
    global edgeY3    
    global edgeY4
    x1=edgeX1
    x2=edgeX2
    x3=edgeX3
    x4=edgeX4
    y1=edgeY1
    y2=edgeY2
    y3=edgeY3
    y4=edgeY4
    point=[X,Y]
    rangelist=[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
    lnglist = []
    latlist = []
    for i in range(len(rangelist) - 1):
        lnglist.append(rangelist[i][0])
        latlist.append(rangelist[i][1])
    maxlng = max(lnglist)
    minlng = min(lnglist)
    maxlat = max(latlist)
    minlat = min(latlist)
    if (point[0] > maxlng or point[0] < minlng or
            point[1] > maxlat or point[1] < minlat):
        return True
    count = 0
    point1 = rangelist[0]
    for i in range(1, len(rangelist)):
        point2 = rangelist[i]
            # 点与多边形顶点重合
        if (point[0] == point1[0] and point[1] == point1[1]) or (point[0] == point2[0] and point[1] == point2[1]):
                #"在顶点上"
            return True
            # 判断线段两端点是否在射线两侧 不在肯定不相交 射线（-∞，lat）（lng,lat）
        if (point1[1] < point[1] and point2[1] >= point[1]) or (point1[1] >= point[1] and point2[1] < point[1]):
                # 求线段与射线交点 再和lat比较
            point12lng = point2[0] - (point2[1] - point[1]) * (point2[0] - point1[0]) / (point2[1] - point1[1])
                # 点在多边形边上
            if (point12lng == point[0]):
                    #"点在多边形边上"
                return True
            if (point12lng < point[0]):
                count += 1
        point1 = point2
    if count % 2 == 0:
        return True
    else:
        return False
'''
def isOutside(X,Y):
    global edgeX1
    global edgeX2
    global edgeX3
    global edgeX4
    global edgeY1
    global edgeY2
    global edgeY3
    global edgeY4
    x1=edgeX1
    x2=edgeX2
    x3=edgeX3
    x4=edgeX4
    y1=edgeY1
    y2=edgeY2
    y3=edgeY3
    y4=edgeY4
    X -= x1
    x2 -= x1
    x3 -= x1
    x4 -= x1
    x1 -= x1
    Y -= y1
    y2 -= y1
    y3 -= y1
    y4 -= y1
    y1 -= y1
    s1 = areaCal(X, Y, x1, y1, x2, y2)
    s2 = areaCal(X, Y, x2, y2, x3, y3)
    s3 = areaCal(X, Y, x3, y3, x4, y4)
    s4 = areaCal(X, Y, x4, y4, x1, y1)
    S = areaCal(x1,y1,x2,y2,x3,y3) + areaCal(x3,y3,x4,y4,x1,y1)
    #self.logger.info("S"+str(S)+"A"+str((s1+s2+s3+s4))+"delta"+str((S-(s1+s2+s3+s4))))
    return abs(S-(s1+s2+s3+s4))>4

def areaCal(x1,y1,x2,y2,x3,y3):
    return 0.5*abs(x1*y2-x1*y3+x2*y3-x2*y1+x3*y1-x3*y2)



def restart_program():#程序重启函数（暂定还未使用）
  """Restarts the current program.
  Note: this function does not return. Any cleanup action (like
  saving data) must be done before calling this function."""
  python = sys.executable
  os.execl(python, python, * sys.argv)


def car_control(thr, brake, steer):  # 树莓派与小车stm32控制函数，分别为油门，制动，转向。0-1浮点数
    ser.write(bytes('$,'+str(round(thr, 3))+','+str(round(brake, 3))+','+str(round(steer, 3))+',\r\n', encoding='UTF-8'))


def clockwise_0_360_to_pi_negative_pi(angle):
    angle = -angle
    angle = angle/180*math.pi
    while angle >= math.pi:
        angle -= 2*math.pi
    while angle < - math.pi:
        angle += 2 * math.pi
    return angle

def CalXY(raw_x,raw_y,theta):
    unit_vec=[math.cos(theta),math.sin(theta)]
    diff_vec=unit_vec*lqr.lr
    new_pos=[raw_x,raw_y]+diff_vec
    return new_pos[0],new_pos[1]


f = None
i = 0
front_mo = None
xyv_pre = []
repeat_flag = False  # 停下之后初始化未考虑

# log 日志
now = datetime.datetime.now()
file_name = f'./log/log{now.strftime("%Y%m%d_%H%M%S")}.txt'
file_log = open(file_name, 'w', encoding='utf-8')

while 1:  # 业务主循环
    try:
        if send == 0:
            ser.flushInput()
            dataraw = ser.readline()
            serPC.write(dataraw)  # 报文转发至pc
        '''    else:
            print('waiting')
        '''
        info = dataraw.decode()
        xyv = kk.findall(info)

        if xyv == xyv_pre:
            repeat_flag = True  # 重复了
        else:
            repeat_flag = False
        xyv_pre = xyv

        # print(kk.findall(info))
        # print(xyv)
        if xyv:  # 若果xyv非空
            # xo = xyv[0]
            # yo = xyv[1]
            ho = str(clockwise_0_360_to_pi_negative_pi(float(xyv[2])))
            xo, yo = CalXY(xyv[0] , xyv[1] , ho)
            vo = xyv[3]
            mo = int(xyv[4])  # 模式(正常，录制，循迹)
            th = xyv[5]
            br = xyv[6]
            steer = xyv[7]
            soc = xyv[8]
            cur_time = time.time()  # 时间

            if mo == 0:
                if front_mo == mo:
                    if i % 5 == 0:
                        to = str(time.time())
                        s = xo + ' ' + yo + ' ' + ho + ' ' + vo + ' ' + to + '\n'
                        print(s)
                        f.write(s)
                    i += 1
                else:
                    f = open("trajeo.txt", 'w', encoding='utf-8')
                    f.close()
                    f = open("trajeo.txt", 'a', encoding='utf-8')
            elif mo != 0 and front_mo == 0:
                i = 0
                f.close()
            front_mo = mo

        if mode == 2:
            lqr.update_ego_state(float(xo), float(yo), float(ho), float(vo), cur_time, repeat_flag)
            lqr_mode = lqr.track()  # 循迹数据计算
            # print log
            log_list = lqr.log_list
            print("track ok")
            for item in log_list:
                file_log.write(str(item) + ' ')
            file_log.write('\n')
            print("wirte log ok")

            if lqr_mode == 0 and float(vo) <= 0.01:
                mode = 1
                file_log.close()
                car_control(0, 0.95, 0)
            # if isOutside(float(xo),float(yo)):
            #     tk.thr=0
            #     tk.brake=0.7
            #     tk.steer=0
            #     print('out')
            car_control(lqr.thr, lqr.brake, lqr.steer)
            # ser.write(bytes('$,'+str(round(tk.thr,3))+','+str(round(tk.brake,3))+','+str(round(tk.steer,3))+',\r\n',encoding='UTF-8'))
            # time.sleep(0.01)
            # 因为串口接收具有阻塞性所以在主循环内不再添加延时函数
        elif mode == 3:
            tk.update_data(float(xo), float(yo), float(ho), float(vo))
            if tk.i == 0:
                tk.Linearx1 = float(xo)
                tk.Lineary1 = float(yo)
                tk.xylinear(tk.Linearx1, tk.Lineary1, tk.Linearx2, tk.Lineary2)  # 录入起点终点位置
            tk.linear()                         # 计算直线行驶数据
            print('steer=' + str(tk.steer) + 'thr=' + str(tk.thr))
            # if isOutside(float(xo), float(yo)):
            #     tk.thr = 0
            #     tk.brake = 0.7
            #     tk.steer = 0
            #     print('out of side')
            car_control(tk.thr, tk.brake, tk.steer)

    except Exception as err:  # 异常处理
        time.sleep(0.1)
        traceback.print_exc()
        print('err')





        '''
        xInfos = re.search(r'xo:',info)
        xInfo = xInfos.span()
        yInfo = re.search(r'yo:',info).span()#match1=re.search(r"hello","222hello word").span()=3,8 自行理解
        hInfo = re.search(r'ho:',info).span()#记录标题的位置，方便随后提取文本
        vInfo = re.search(r'vo:',info).span()
        mInfo = re.search(r'mo:',info).span()
        tInfo = re.search(r'th:',info).span()
        bInfo = re.search(r'br:',info).span()
        sInfo = re.search(r'st:',info).span()
        socInfo = re.search(r'soc:',info).span()
        xo=info[xInfo[1]:(yInfo[0]-1)]
        yo=info[yInfo[1]:(hInfo[0]-1)]
        ho=info[hInfo[1]:(vInfo[0]-1)]
        vo=round(1.3*float(info[vInfo[1]:(mInfo[0]-1)]),2)#round返回浮点数x的四舍五入值
        mo=info[mInfo[1]:(tInfo[0]-1)]
        th=info[tInfo[1]:(bInfo[0]-1)]
        br=info[bInfo[1]:(sInfo[0]-1)]
        t=info[sInfo[1]:(socInfo[0]-1)]
        soc=info[socInfo[1]:(socInfo[1]+5)]
        #mode=int(mo)
        '''
