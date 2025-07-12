from maix import time,uart,pinmap,camera,display,image,nn,app
import Object_Detect
import global_vars
from Object_Detect  import System,My_FindQR,My_ObjectDetect,Myfind_blobs
#from Uart_Protocol import TTest,RTest,Timer,TimerStart #串口发送接收的类



########yolo模型识别类的实例 参数设定在里面
System_Qrcode=  System(init_mode="qrcode")
System_Detect1= System(init_mode="yolo")
System_Detect2= System(init_mode="Find_blobs")

dis = display.Display()
cam=camera.Camera(320,240)
# 创建函数字典
function_dict = {
    "find_qr": My_FindQR,
    "object_detect": My_ObjectDetect,
    "Find_blobs":Myfind_blobs
}


global_vars.mode = "object_detect"  # 定义初始模式
qr_detected = False  # 标志是否识别到二维码
timec=0
while not app.need_exit():   
    
    timec=timec+1
    if timec==10:
        timec=0
        print(f"当前模式是：{global_vars.mode}")
    if global_vars.mode == "find_qr":       
        function_dict["find_qr"](System_Qrcode,dis,cam)  # 执行找二维码功能
        
    elif global_vars.mode == "object_detect":
       function_dict["object_detect"](System_Detect1,dis,cam)

    elif global_vars.mode == "Find_blobs":
        function_dict["Find_blobs"](System_Detect2,dis,cam)
