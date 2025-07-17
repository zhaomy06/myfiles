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
from straight_controller import algorithm,PI
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

ser=serial.Serial('/dev/ttySC1',115200,timeout=0.4)
serPC=serial.Serial('/dev/ttySC0',115200,timeout=5)
kk = re.compile(r'\d+\.?\d+|\d+|-\d+\.?\d+')#编译正则表达式，用以检索串口报文数据，可以加速处理串口报文
ser.write(bytes('Ready \r\n',encoding='UTF-8'))
send=0#文件发送标志位0为为发送，1为发送中
PCorder=''#pc指令变量
dataraw=''#存放stm32发送报文
data=''

tk=algorithm()#建立一个轨迹跟踪类
mode=1#mode 0录制，1 正常 ，2 回放
tempstr=0

def pcinput():#pc上位机指令解析程序
    global mode 
    global send
    global tempstr
    while(1):
        serPC.flushInput()
        PCraw=serPC.readline()
        PCorder =PCraw.decode()[:-2] #去\r\n
        if PCorder != '':
            #print(PCorder)
            if send ==0:
                if PCorder=='track':
                    mode=2
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
                #receve.readpart('trajeo.txt')            
                if PCorder=='updata':
                    try:
                        preFile=np.loadtxt('trajeo.txt')
                    except:
                        print('err1')
                    
                    try:
                        x=preFile[:,0]
                        y=preFile[:,1]
                        v=preFile[:,2]
                        tk.update_predata(x,y,v)
                        send=1
                        serPC.write(bytes("finish\n",encoding='UTF-8'))
                        time.sleep(0.2)
                        serPC.write(bytes("finish\n",encoding='UTF-8'))
                        print('finish')
                        send=0
                    except:
                        print('updata err')                
                
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
            if  PCorder[:5]=='Ready':
                print('pid'+str(tk.kSteer))
                PID=kk.findall(PCorder)
                tk.ki=float(PID[0])
                tk.kd=float(PID[1])
                tk.kp=float(PID[2])
                tk.kSteer=float(PID[3])
                print('finsh PID '+str(PID))
readData = threading.Thread(target=pcinput)#创建pc串口指令接收线程
readData.setDaemon(True)#后台服务线程与主程序一起关闭
readData.start()

def restart_program():#程序重启函数（暂定还未使用）
  """Restarts the current program.
  Note: this function does not return. Any cleanup action (like
  saving data) must be done before calling this function."""
  python = sys.executable
  os.execl(python, python, * sys.argv)
def car_control(thr,brake,steer):#树莓派与小车stm32控制函数，分别为油门，制动，转向。0-1浮点数
    ser.write(bytes('$,'+str(round(thr,3))+','+str(round(brake,3))+','+str(round(steer,3))+',\r\n',encoding='UTF-8'))

while(1):#业务主循环
    try:

        if send==0:
            ser.flushInput()
            dataraw = ser.readline()
            serPC.write(dataraw)#报文转发至pc
        '''    else:
            print('waiting')
        '''
        info=dataraw.decode()
        xyv=kk.findall(info)
        #print(kk.findall(info))

        #print(xyv)
        if xyv!=[]:
            xo=xyv[0]
            yo=xyv[1]
            ho=xyv[2]
            vo=xyv[3]
            mo=xyv[4]
            th=xyv[5]
            br=xyv[6]
            st=xyv[7]
            soc=xyv[8]
        if mode==2:                         #寻迹模式下
            tk.update_data(float(xo),float(yo),float(vo),float(ho))
            tk.track()#循迹数据计算
            print('steer='+str(tk.steer)+'thr='+str(tk.thr))
            car_control(tk.thr,tk.brake,tk.steer)
            #ser.write(bytes('$,'+str(round(tk.thr,3))+','+str(round(tk.brake,3))+','+str(round(tk.steer,3))+',\r\n',encoding='UTF-8'))
            #time.sleep(0.01)
            #因为串口接收具有阻塞性所以在主循环内不再添加延时函数
        
    except Exception as err:#异常处理
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