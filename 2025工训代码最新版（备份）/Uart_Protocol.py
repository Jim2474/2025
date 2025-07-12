from maix import time,uart,pinmap,camera,display,image,nn,app

#发送测试数据类
class TTest():
    def __init__(self) -> None:#默认实例 无返回值
        #发送标识位
        self.Tflag=0

        #发送数据内容
        self.test_number0=0
        self.test_number1=0
        self.test_number2=0

TTest0=TTest()  #类的实例
TTest0.Tflag=1
TTest0.test_number0=0
TTest0.test_number1=1
TTest0.test_number2=2

#串口发送函数
def DataTransmit(head,tail,Tdata,serial):
    data= () #定义一个元组储存数据 元组内的元素不可变
    if Tdata.Tflag==1:
        data=bytes([
            head,
            Tdata.test_number0,
            Tdata.test_number1,
            Tdata.test_number2,
            tail
        ])
    elif Tdata.Tdata==2:
         data=bytes([
            head,
           
            tail
        ])
    if data!=():
        serial.write(data)    


#接受测试数据类
class RTest():
    def __init__(self) -> None:
        self.Rflag=0
        self.Tflag=0
        
        self.test_number0=0
        self.test_number1=0
        self.test_number2=0
        self.test_number3=0
        self.test_number4=0
        self.test_number5=0
        

RTest0=RTest()
RTest0.Rflag=1
RTest0.Tflag=1

#串口接受函数
def ReceivedData(head,tail,lenth,serial):
    BufData=serial.read(40)
    print(f"收到数据：{BufData}")
    if BufData and len(BufData)>=lenth:
        
        if BufData[0]==head and BufData[lenth-1]==tail:    #判断帧头帧尾
            
            return BufData
        else:
            return None
    
#接收数据解析函数
def ParseData(RData,BufData):
    print("解析解析解析")
    if BufData!=None:

        if RData.Rflag==1:
            RData.test_number0=BufData[1]
            RData.test_number1=BufData[2]
            RData.test_number2=BufData[3]
            RData.test_number3=BufData[4]
            RData.test_number4=BufData[5]
            RData.test_number5=BufData[6]  

        elif RData.Rflag==2:
            pass



#参数分别是：接收解析数据的实例，帧头帧尾，接收数据长度，接收串口
def ParseString(RData,head,tail,lenth,serial):   
    BufData=serial.read(100).decode('utf-8')  # 读取字符串并解码  
    if BufData and len(BufData)>=lenth:#接收足够的数据长度则开始判断数据
        #拆分字符串，用逗号隔开
        parts = BufData.strip().split(',')       
        if parts[0]==str(head) and parts[-1].strip()==str(tail):#若帧头帧尾没问题 同时分离了空格
            RData.test_number0=int(parts[1]) 
            RData.test_number1=int(parts[2])
            RData.test_number2=int(parts[3])
            return BufData
    else:
        return None 
             
#软件定时器
class Timer():
    def __init__(self) -> None:
        #计时数
        self.count=0
        #计数器上线
        self.count_max=0
        #计时完成标志
        self.count_flag=0



def TimerStart(Timer):
    if Timer.count<Timer.count_max:
        
        Timer.count_flag=0
        Timer.count+=1
        #print(f"{Timer.count}未到max")
    else:
        #计数完成 标志位置1
        Timer.count_flag=1
        Timer.count=0

    #50ms发一次
    #TimerStart(timer0)
   # if timer0.count_flag==1:        
      #  #ParseData(RTest0,ReceivedData(0xAA,0XBB,5,serial))
     #   ParseString(RTest0,'!','#',5,serial)       
       # DataTransmit(0xCC,0XDD,RTest0,serial)
      #  result1 = f"!,{RTest0.test_number0}, {RTest0.test_number1}, {RTest0.test_number2},#"
        
      #  serial.write(result1.encode('utf-8') )
      #  print(result1)


