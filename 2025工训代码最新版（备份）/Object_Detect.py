from maix import time,uart,pinmap,camera,display,image,nn,app
from concurrent.futures import ThreadPoolExecutor
import Uart_Protocol,threading
from Uart_Protocol import TTest,RTest,Timer,TimerStart,ParseData,ReceivedData#串口发送接收的类
import global_vars
import re,math

# 添加卡尔曼滤波器类
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance  # 过程噪声方差
        self.measurement_variance = measurement_variance  # 测量噪声方差
        self.estimate = initial_value  # 初始状态估计
        self.estimate_error = 1  # 初始估计误差协方差
        
    def update(self, measurement):
        # 预测步骤
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        
        # 更新步骤
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

##将串口实例化放在这里 方便后面发送
devices = uart.list_devices()
serial=uart.UART(devices[0],115200,uart.BITS.BITS_8,uart.PARITY.PARITY_NONE,uart.STOP.STOP_1)

# 初始化卡尔曼滤波器实例
# X方向：由于范围更大（±160），设置更大的过程噪声方差和较小的测量噪声方差
kf_x = KalmanFilter(process_variance=0.8, measurement_variance=2.5)

# Y方向：范围相对较小（±120），可以设置稍小的过程噪声方差
kf_y = KalmanFilter(process_variance=0.6, measurement_variance=2.0)

########### 中断回调函数 模拟stm32 设置不同中断号  uart4 uart5 

def uart_receive_it(serial : uart.UART, data : bytes):
    print(f"进入中断{data}")

    # 检查数据包长度、帧头和帧尾
    if len(data) == 8 and data[0] == 0xAA and data[7] == 0xBB:
        print(f"收到的数据是：{data[0]},{data[1]}")
        # 解析有效数据
        global_vars.test_number0 = data[1]
        global_vars.test_number1 = data[2]
        global_vars.test_number2 = data[3]
        global_vars.test_number3 = data[4]
        global_vars.test_number4 = data[5]
        global_vars.test_number5 = data[6]
        print(f"有效数据: {global_vars.Rdata.test_number0}, {global_vars.Rdata.test_number1}, {global_vars.Rdata.test_number2}, {global_vars.Rdata.test_number3}, {global_vars.Rdata.test_number4}, {global_vars.Rdata.test_number5}")
    else:
        print("Invalid data packet")
        #return    
            

    #if global_vars.uart5_flag==1:
        global_vars.uart5_flag=0
        print("%%%%%%%%%%%%%%%%%%%%%%%进入切换模式的中断")
        global_vars.uart5_data = data#模式切换应答 !B#
        global_vars.response=global_vars.uart5_data #抓取应答 !A# 这个在waitforacnow处理了 意思是逻辑函数在那里，完全不影响
        print(global_vars.uart5_data)
        if global_vars.uart5_data:
            try:
                global_vars.uart5_data = global_vars.uart5_data.decode('utf-8').strip()  # 解码并去除两端空白字符
                print(f"uart5_data is :{global_vars.uart5_data}")
                match = re.match(r"!([a-zA-Z0-9])#", global_vars.uart5_data)
                if match:
                    content = match.group(1)
                    print(f"Extracted content: {content}")
                    if content == "B":  # 如果接收到的应答
                        global_vars.material_timer={'red':0,'green':0,'blue':0}#字典关键字指向三个计时器时间

                        #if global_vars.switch_priority_flag == 0:
                           # global_vars.switch_priority_flag = 1  # 切换到第二次抓取
                        global_vars.mode = "Find_blobs"
                        print(f"切换到{global_vars.mode}.")
                        print(f'########@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
                        print(f'清零后的数据{global_vars.material_timer}')
                       #elif global_vars.switch_priority_flag == 2:  # 第二轮抓取完成
                           # global_vars.mode = "object_detect"
                            #print("第二轮抓取完成，进入物体识别模式")
                    else:
                        print(f"Unexpected content: {content}")
                else:
                    print("Invalid format")
            except UnicodeDecodeError:
                print("Failed to decode response1")
            except Exception as e:
                print(f"An error occurred1: {e}")



serial.set_received_callback(uart_receive_it)   #串口接收回调函数
grad_count=0
timer0=Timer()
timer0.count_max=30#100000
global_vars.Rdata.Rflag=1
global_vars.Rdata.Tflag=1

""""
暂存单片机端应答代码
uint8_t ack_message[]="ACK";
//发送应答信息  暂存于此  HAL_UART_Transmit_DMA(&huart4, ack_message, sizeof(ack_message) - 1);

"""

"""""

###########初始化的类

"""""
class System:
    def __init__(self,init_mode) :#设置不同的初始化模式，默认值是"yolo"
        if init_mode =="qrcode":
            self.init_qrcode_mode()
        elif init_mode =="yolo":            
            self.init_yolo_mode()
        elif init_mode =="Find_blobs":
            self.init_FindBlobs_mode()           
        #else :
            #raise ValueError("Unknown mode!!!!!")

    def init_qrcode_mode(self):#扫码初始化
        #self.cam = camera.Camera(320,240)
        #self.dis = display.Display()  ####有bug 所以只能放在main函数实例化，然后传参
        self.last_err_x_pos=0
        self.last_err_y_pos=0

    def init_yolo_mode(self):   #yolo模型识别初始化

        self.detector = nn.YOLO11(model="/root/models/yolo11n.mud", dual_buff = True)
        #self.detector=nn.YOLOv5(model="/root/models/best.mud", dual_buff = True)
        #self.detector=nn.YOLO11(model="/root/models/sehuan.mud", dual_buff = True)
        #self.cam = camera.Camera(self.detector.input_width(), self.detector.input_height(), self.detector.input_format())
        #self.dis = display.Display()

        self.image_width = self.detector.input_width()
        self.image_height =self.detector.input_height()
        self.last_err_x_pos=0
        self.last_err_y_pos=0


    def init_FindBlobs_mode(self):
        #self.cam = camera.Camera(320,240)
        self.detector = nn.YOLO11(model="/root/models/yolo11n.mud", dual_buff = True)
        #self.detector=nn.YOLOv5(model="/root/models/best.mud", dual_buff = True)
        #self.detector=nn.YOLO11(model="/root/models/yolov11n.mud", dual_buff = True)
        #self.cam = camera.Camera(self.detector.input_width(), self.detector.input_height(), self.detector.input_format())
        #self.dis = display.Display()

        self.image_width = self.detector.input_width()
        self.image_height =self.detector.input_height()
        self.last_err_x_pos=0
        self.last_err_y_pos=0
        


   # def get_img(self):
       # return self.cam.read()

    def get_detector(self):
        return self.detector

"""""
############################################################################################
####################################模型识别函数###############################################

"""""
"""
        My_ObjectDetect 函数用于从摄像头读取图像，并使用检测器检测图像中的对象。检测到的对象会在图像上绘制矩形框和十字线，
        计算对象中心与图像中心的误差，并进行一阶滤波。最后将误差和对象宽度通过串口发送出去，并显示处理后的图像。
        参数:
        detect1: 包含检测器和图像尺寸等信息的对象。
        dis: 显示图像的对象。
        cam: 摄像头对象，用于读取图像。
        返回:
        无返回值。
"""
def My_ObjectDetect(detect1,dis,cam):
    img = cam.read()
    img = img.lens_corr(strength=1.5)	# 调整strength的值直到画面不再畸变

    objs = detect1.detector.detect(img, conf_th = 0.65, iou_th = 0.45)
    color_map_2 = {1: 'red', 2: 'green', 3: 'blue'}  # yolo模型识别的颜色映射

    if len(objs) == 0:
        dis.show(img)
        global_vars.valid_data1 = 0
        result1 = f"!,0, 0, {global_vars.valid_data1},#"
        serial.write(result1.encode('utf-8'))
        return

    # 计算屏幕中心点坐标
    center_x = detect1.image_width / 2
    center_y = detect1.image_height / 2
    
    # 存储所有检测到的物体及其到中心点的距离
    detected_objects = []
    
    for obj in objs:
        # 计算物体中心点
        obj_center_x = obj.x + obj.w / 2
        obj_center_y = obj.y + obj.h / 2
        
        # 计算到屏幕中心的距离
        distance = ((obj_center_x - center_x) ** 2 + (obj_center_y - center_y) ** 2) ** 0.5
        
        detected_objects.append({
            'obj': obj,
            'distance': distance,
            'center_x': obj_center_x,
            'center_y': obj_center_y
        })
        
        # 绘制矩形框和十字线
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        img.draw_cross(int(obj_center_x), int(obj_center_y), color=image.COLOR_GREEN)
        
        # 显示标签和置信度
        msg = f'{detect1.detector.labels[obj.class_id]}: {obj.score:.2f}'
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
    
    # 找到距离中心最近的物体
    #nearest_obj = min(detected_objects, key=lambda x: x['distance'])
    #obj = nearest_obj['obj']
    
    # 在最近物体的中心画一个蓝色的十字线
    #img.draw_cross(int(nearest_obj['center_x']), int(nearest_obj['center_y']), color=image.COLOR_BLUE, size=15)
    img.draw_cross(int(obj_center_x), int(obj_center_y), color=image.COLOR_BLUE, size=15)

    # 计算最近物体的误差并使用卡尔曼滤波
    err_x_pos = center_x-  obj_center_x #- nearest_obj['center_x']
    err_x_pos = kf_x.update(err_x_pos)  # 应用卡尔曼滤波

    err_y_pos = center_y - obj_center_y#- nearest_obj['center_y']
    err_y_pos = kf_y.update(err_y_pos)  # 应用卡尔曼滤波
    
    #计算偏移角度
    angle = math.atan2(err_y_pos, err_x_pos) * 180 / math.pi
    angle=round(angle,2)
    print(f"angle: {angle:.2f}")
    ttttt=f'{angle}'
    #img.draw_string(obj.x, obj.y+30, ttttt, color=image.COLOR_RED)


    # 四舍五入误差值
    r_err_x_pos = round(err_x_pos, 2)
    r_err_y_pos = round(err_y_pos, 2)
    
    # 获取最近物体的颜色
    #color_str = color_map_2.get(int(detect1.detector.labels[obj.class_id]), 'unknown')
    # 将颜色字符串转换为对应的数字
    #color_num_map = {'red': 1, 'green': 2, 'blue': 3, 'unknown': 0}
    #global_vars.valid_data1 = color_num_map[color_str]
    
    # 发送结果
    result1 = f"!,{r_err_x_pos}, {r_err_y_pos}, {global_vars.valid_data1},#"
    serial.write(result1.encode('utf-8'))
    img.draw_string(obj.x, obj.y+15, result1, color=image.COLOR_RED)
    print(result1)
    
    dis.show(img)

"""""
##########################################################################################
########################################识别二维码#########################################

"""""

"""
    等待接收应答(ACK),
    如果在指定时间内收到应答则返回True.超时则返回False
"""
# 创建一个线程池，最大同时运行 3 个线程
executor = ThreadPoolExecutor(max_workers=3)

# 全局事件，用于主线程等待应答的通知
ack_event = threading.Event()

"""
等待从串口接收ACK应答信号。
参数:
timeout (int): 等待ACK的超时时间,单位为秒,默认为3秒。
返回:
bool: 如果在超时时间内接收到ACK应答信号,返回True,否则返回False。
功能:
该函数通过串口读取数据,并检查是否接收到ACK应答信号。如果在指定的超时时间内接收到ACK应答信号,
则返回True:如果超时未接收到应答信号,则返回False。

"""
def wait_for_ack(timeout=10):

    start_time = time.time()
    
    #while time.time() - start_time < timeout:       
       # response = serial.read(64)  # 读取串口数据
       # global_vars.response=response
    global_vars.uart4_flag=1   
    if global_vars.response:  # 检查串口是否有数据
            try:
                global_vars.response = global_vars.response.decode('utf-8').strip()  # 解码并去除两端空白字符
                print(f"response is :{global_vars.response}")
                
                # 使用正则表达式匹配 !a# 格式的字符串
                match = re.match(r"!([a-zA-Z0-9])#", global_vars.response)
                if match:
                    content = match.group(1)
                    print(f"Extracted content: {content}")
                    if content == "X":  # 如果接收到的应答是 "ACK"
                        print("Received ACK from the MCU.")
                        return True  # 返回True表示接收到应答
                    else:
                        print(f"Unexpected content: {content}")
                else:
                    print("Invalid format")
            except UnicodeDecodeError:
                print("Failed to decode response")
            except Exception as e:
                print(f"An error occurred: {e}")
    #print("Timeout waiting for ACK.")
    return False  # 超时未收到应答，返回False

"""
# 定义一个用于等待ACK的函数,放入子线程中处理
    防止二维码识别阻塞
"""
def wait_for_ack_thread():
    global ack_event
    result =  True#wait_for_ack() 
    if result:
        ack_event.set()  # 如果接收到ACK，通知主线程
        print("接收到ack")
    else:
        print("子线程超时未收到ack")  # 超时也通知主线程 

"""
#######  #扫二维码并应答
"""
def My_FindQR(detect1,dis,cam):
    global    qr_detected # 引用全局变量
    img = cam.read()  
    qrcodes = img.find_qrcodes()   
    for qr in qrcodes:
        corners = qr.corners()       
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1], image.COLOR_RED)
            img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)
 

 # 使用正则表达式提取两个三位数
        match = re.match(r"(\d{3})\+(\d{3})", qr.payload())
        if match:  
            num1 = match.group(1)
            num2 = match.group(2)
            global_vars.receive_qrcode=1
            result1 = f"!,{num1},{num2},{global_vars.receive_qrcode},#"
            for _ in range(10):  # 连发3次数据
                serial.write(result1.encode('utf-8'))
                print(result1)
        else:
            print("QR Code 格式不正确")

        # 启动一个子线程来等待ACK应答
        ack_event.clear()  # 清除事件状态
        future = executor.submit(wait_for_ack_thread)
       # thread.start()

        # 等待子线程的通知（最多等待5秒，避免死锁） #############################这里暂时不太好，最坏的情况会超时3s
        ack_received = ack_event.wait(timeout=3)

        
        if ack_received:
            global_vars.mode = "Find_blobs"  # 切换模式为找物体
            print(f"Mode switched to {global_vars.mode}.")
        else:
            print("mode has not been switched")
        
    dis.show(img)

     

"""####################################################################################################
#############################寻找色块,按照扫码得到的优先级抓取物料#########################################
"""
global_vars.QRmaterial_order =['red', 'green', 'blue']#扫码得到的抓取顺序

materials=[] #储存检测到的物料
grad_count=0                                #记得每次清零 在扫码后
material_center=160
grabbed_materials = set()  # 用于记录已抓取的物料，避免重复抓取
priority_index=0 

Catch_Time=0.5 #在抓取位置0.5s后再抓取


"""
########获得当前y最大的物料
"""
def Get_ymin_material(materials):
    if not materials:
        return None
    return max(materials,key=lambda x:x[1])#返回y值最小的物料 要用变量接受 因为我想知道key

"""
##########在中心区域0.5秒后才可以抓取  这个时间还要再考虑一下该设置多久 能不能默认舍弃第一次抓取呢 这样更能避免误判
"""
def Check_material_in_center(material_center, xcenter, material_color):
    #global material_timer
     
    #threshold = 300              # 设定中心区域阈值

    # 判断物料是否接近中心区域（x 坐标差小于 50）
    #if abs(xcenter - material_center) <= threshold:#只赋值一次 避免一直不满足条件 
        # 如果物料在中心区域，检查 material_timer 是否已记录该物料的时间
    if global_vars.material_timer[material_color] == 0:
            # 记录进入中心区域的时间
        global_vars.material_timer[material_color] = time.time()
        #print(f"material_timer:{material_timer}")   #   ！！！！！！！！！！！！！！！！！！！！！！！要调试下
        
        # 计算当前时间与物料进入中心的时间差
    current_time = time.time()
    if current_time - global_vars.material_timer[material_color] >= 1.5:  # 停留超过0.5秒
        print(f"{material_color} 物料已经停留超过 2 秒")
        global_vars.material_timer={'red':0,'green':0,'blue':0}
        return True
    else:
        print(f"{material_color} 物料在中心区域时间不足 0.5 秒")
        return False
    '''
    else:
        print(f"{material_color} 物料未进入中心区域")
        # 如果物料没有进入中心区域，重置计时
        material_timer[material_color] = 0                  #使下次进入时可以正确初始化！
        return False
    '''

"""
##########发送x,y坐标给小车调整位置  !!现在没用上位置调整 2.20
"""
def Send_message_to_adjust(xcentre,ycentre):
    
    #img.draw_cross(int(obj.x + obj.w / 2), int(obj.y + obj.h / 2), color = image.COLOR_GREEN)

    #####计算误差并滤波
    err_x_pos =320/2 -  xcentre          #和中间值的误差
    err_x_pos =0.15*err_x_pos+0.75*(global_vars.last_err_x_pos)           #一阶滤波
    (global_vars.last_err_x_pos)=err_x_pos

    err_y_pos =240/2 - ycentre  #这里可能有问题，我觉得是减号
    err_y_pos =0.15*err_y_pos+0.75*(global_vars.last_err_x_pos)
    (global_vars.last_err_x_pos)=err_y_pos
            
    result1 = f"!,{err_x_pos:.2f}, {err_y_pos:.2f}, {global_vars.valid_data},#"        
    #serial.write(result1.encode('utf-8') )
    print(f"!!!!!!!!!!!!!!!!!!---{result1}----!!!!!!!!!!!!!!!!!!!!!!")


"""
##########判断物料状态
"""

def get_material_state(item, materials):
    """判断物料的状态"""
    color, bolbs_data,xcenter,ycenter= item[0], item[1],item[2],item[3]
    print(f'3.7test_color{color},test_conter{global_vars.material_timer}')
    # 判断物料是否在可抓取区域
    nearest_material = Get_ymin_material(materials)
    if not nearest_material or nearest_material[0] != color:
        return "not_in_range"
    
    # 判断物料是否在中心区域
    if not Check_material_in_center(material_center, xcenter, color):
        global_vars.valid_data=0
        #global_vars.material_timer[color] = 0
        #global_vars.material_timer={'red':0,'green':0,'blue':0}
        return "not_in_center"
    #global_vars.material_timer={'red':0,'green':0,'blue':0}#字典关键字指向三个计时器时间

    print(f'############################################################################')
    print(f'清零后的数据{global_vars.material_timer}')
    
    #调整位置
    #global_vars.valid_data=1
    #Send_message_to_adjust(xcenter,ycenter) !!现在没用上位置调整 2.20

    # 等待小车应答
   # if not wait_for_ack():
    #    return "not_ready_to_grab"

    # 判断物料是否已经被抓取
    #print(f"color:{color}  grabbed_materials:{grabbed_materials}")
    #if color in grabbed_materials:
       # return "already_grabbed"
        
    #color_priority = global_vars.QRmaterial_order[priority_index]
    #if color_priority != color:
      #  return "wrong_priority"
           
    return "ready_to_grab"

"""
##########根据物料状态执行相应操作
    根据物料状态执行相应操作。
    参数:
        item (tuple): 包含物料信息的元组，第一个元素为颜色。
        state (str): 物料的状态，可以是以下之一：
            - "already_grabbed": 物料已经被抓取。
            - "wrong_priority": 物料颜色与当前优先级颜色不一致。
            - "not_in_range": 物料未进入可抓取区域。
            - "not_in_center": 物料未在中心区域或停留时间不足。
            - "ready_to_grab": 物料可以被抓取。
            - 其他: 未知状态。
    返回:
        bool: 根据物料状态返回是否可以抓取物料。
    """
def handle_material(item, state,img):
   
    """根据物料状态执行相应操作"""
    global grad_count, priority_index
    color = item[0]
    err_x_blob1=item[4]
    err_y_blob1=item[5]

    match state:
        case "already_grabbed":
            return False
            
        case "wrong_priority":
            print(f"当前颜色{color}与优先级颜色{global_vars.QRmaterial_order[priority_index]}不一致")
            img.draw_string(0, 0 + 25, f"当前颜色{color}与优先级颜色{global_vars.QRmaterial_order[priority_index]}不一致", image.COLOR_BLUE)
            
            return False
            
        case "not_in_range":
            print("目标颜色未进入可抓取区域")
            img.draw_string(0, 0 + 25, "目标颜色未进入可抓取区域", image.COLOR_BLUE)
            return False
            
        case "not_in_center":
            print("不可抓取 - 未在中心区域或停留时间不足")
            img.draw_string(0, 0 + 25, "不可抓取 - 未在中心区域或停留时间不足", image.COLOR_BLUE)
            return False

        case "not_ready_to_grab":
            print("未收到应答")
            return False
          

        case "ready_to_grab":
            print(f"{color} 物料可以被抓取")
            Send_message_to_grab(color,err_x_blob1,err_y_blob1)  # 发送可以抓取信息
            #grabbed_materials.add(color)    # 记录已抓取的物料
            print(f"已发送抓取{color}物料信息")
            #grad_count += 1
            #priority_index += 1
            return True
            
        case _:
            print("未知状态")
            return False



"""
########## ##########    ##########找色块的主函数 ######################################################################
####################################################################################################
""" 
def Myfind_blobs(detect1,dis,cam):
    img=cam.read()
    img = img.lens_corr(strength=1.5)	# 调整strength的值直到画面不再畸变

    msg1=f'{global_vars.screen_count}'
    img.draw_string(30, 50, msg1, color = image.COLOR_RED)

    #if global_vars.switch_priority_flag==2  :  
    if wait_for_ack(timeout=5) :
        print("收到切换确认，切换到物体识别模式")
        global_vars.mode = "object_detect"
    else:
        print("未收到切换确认，保持当前模式")  

    global grad_count,priority_index
    
    if not wait_for_priority_order():      #等待接收优先级顺序
        print("等待接收优先级顺序")
        return
    #if global_vars.priority_order_flag==0:
       # return
    
    #change_priority_order()                        #改变抓取优先级

    if global_vars.Find_blobs_enable==0:        #完成一轮抓取失能找色块
        return


    #if not check_priority_index(): # 检查是否所有物料都已抓取完成
       ## print("开始新一轮物料抓取")  
        #return
        
    
  
    objs = detect1.detector.detect(img, conf_th = 0.5, iou_th = 0.3)
    color_map_1 = {'Red': 'red', 'Green': 'green', 'Blue': 'blue'}  # yolo模型识别的颜色映射
    
    materials = []  # 每次循环前清空物料列表

    if len(objs)==0:
        dis.show(img)
        global_vars.valid_data1=0
        result1 = f"!,0, 0, {global_vars.valid_data1},#"
        global_vars.material_timer={'red':0,'green':0,'blue':0}
        serial.write(result1.encode('utf-8') )
        print("没有检测到物体")
        return   
    
    else:

        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
            img.draw_cross(int(obj.x),int(obj.y),color = image.COLOR_RED)
            #img.draw_cross(int(obj.x + obj.w / 2), int(obj.y + obj.h / 2), color = image.COLOR_GREEN)
            msg = f'{detect1.detector.labels[obj.class_id]}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)
            
            blob_x=obj.x + obj.w / 2    # 计算色块中心坐标
            blob_y=obj.y + obj.h / 2    
            
            err_x_blob=detect1.image_width / 2 - blob_x     #计算色块中心与图像中心的误差
            err_x_blob=kf_x.update(err_x_blob)  #应用卡尔曼滤波

            err_y_blob=detect1.image_height / 2 - blob_y
            err_y_blob=kf_y.update(err_y_blob)  #应用卡尔曼滤波

            err_x_blob=round(err_x_blob,2)
            err_y_blob=round(err_y_blob,2)

            #msg_test=f'centre:{blob_x},c:{blob_x-160}' 
            msg320=f'{err_x_blob,err_y_blob}'
            img.draw_string(obj.x, obj.y+15, msg320, color = image.COLOR_BLUE)
        # 使用 color_map_1 进行颜色映射
            color = color_map_1.get(detect1.detector.labels[obj.class_id], 'unknown')
            print(f"color:{color}")
            materials.append((color, obj.y, blob_x, blob_y,err_x_blob,err_y_blob))  # 传入 y 值，还有色块中心值
            print(f"materials:{materials}") 
        

    
    #位置调整模式直接放在外面就可以了，感觉还是要设置一个应答机制，下次测试完应答功能了再写

    dis.show(img)
     
    #判断逻辑：是否被抓取过了->优先级颜色->y轴坐标最大的颜色->在中心区域停留够0.5s->抓取   
     ####### 判断物料状态 并执行相应操作
    if len(materials) == 0:
        return
    for item in materials:
        state = get_material_state(item, materials)
        if handle_material(item, state,img):
            break
                             
"""
##########检查优先级索引是否超出范围
#重置相关变量
"""

def check_priority_index():
    global priority_index, grad_count, grabbed_materials, material_timer
    #if grad_count >= 3:  ###     
    print("所有物料已抓取完成")        
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

        # 重置所有相关变量
    #priority_index = 0
    #grad_count = 0
    #grabbed_materials.clear()
    global_vars.material_timer = {'red': 0, 'green': 0, 'blue': 0}
    #global_vars.Find_blobs_enable = 0
    '''
        if global_vars.switch_priority_flag == 0:
            # 等待切换确认
            if wait_for_ack(timeout=5):  # 超时时间
                print("收到切换确认，切换到物体识别模式")
                global_vars.mode = "object_detect"
            else:
                print("未收到切换确认，保持当前模式")
                return True
            #return False  # 切换到物体识别模式
    '''   
    if global_vars.switch_priority_flag == 1:
        global_vars.Find_blobs_enable = 0
        global_vars.QRmaterial_order = 0
        global_vars.switch_priority_flag = 2  # 将标志位加1，表示第二轮抓取完成
        return True
            
    return True  # 当grad_count < 3时返回False，继续执行抓取流程
    
    '''
def check_priority_index():
    global priority_index, grad_count, grabbed_materials, material_timer
    if grad_count >= 3:  ###     
        print("所有物料已抓取完成")        
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

        # 重置所有相关变量
        priority_index = 0
        grad_count = 0
        grabbed_materials.clear()
        material_timer = {'red': 0, 'green': 0, 'blue': 0}
        global_vars.Find_blobs_enable = 0
        if global_vars.switch_priority_flag==0:#   ！！ 测试用：记得改回来，现在是只抓一轮就切换到物体识别了
            #global_vars.switch_priority_flag=1#完成第一次抓取 用于改变两次抓取的优先级  #切换模式在中断中完成
            global_vars.mode="object_detect"
        if global_vars.switch_priority_flag==1:
            global_vars.Find_blobs_enable = 0#防止重复抓取 似乎不起作用
            global_vars.QRmaterial_order=0

            return True
        
        #测试用 这里已经不会执行了
        #if global_vars.switch_priority_flag==1: 
            #global_vars.switch_priority_flag=2  
            #global_vars.mode="object_detect"
           

        return False
    return True
'''

"""
########### 发送可以抓取信息的函数 连发三次数据
"""

def Send_message_to_grab(color,err_x_blob1,err_y_blob1):
    if color =='red': global_vars.can_grab = 1  # 可以抓取
    if color =='green':global_vars.can_grab = 2
    if color =='blue':global_vars.can_grab = 3

    message = f"!,{err_x_blob1},{err_y_blob1},{global_vars.can_grab},#"
    #for _ in range(10):  # 连发3次数据
    serial.write(message.encode('utf-8'))
    time.sleep(0.1)  # 间隔 0.1 秒
    global_vars.screen_count+=1
    print(f"发送抓取信息{message}")
    global_vars.can_grab = 0

"""

###########  等待抓取优先级顺序

"""
def wait_for_priority_order():
    #start_time = time.time()   
    #while time.time() - start_time < 3:       
        #global_vars.Rdata = serial.read(64)  # 读取串口数据
       # print(f"%%%%%%%%%%%%%%%%%{global_vars.Rdata}")

    #ParseData(global_vars.Rdata,ReceivedData(0xAA,0XBB,8,serial))
    
    Priority_list=[
        global_vars.test_number0,
        global_vars.test_number1,
        global_vars.test_number2,
        global_vars.test_number3,
        global_vars.test_number4,
        global_vars.test_number5  
        ]
    '''
    Priority_list=[
    1,2,3,3,2,1
    ]
    '''
    print(Priority_list)
    if any(num == 0 for num in Priority_list):
        return False
    else:
        return True
    '''
    color_map = {1: 'red', 2: 'green', 3: 'blue'}
    
    global_vars.first_priority_order=[color_map[num] for num in Priority_list[:3]]
    global_vars.second_priority_order=[color_map[num] for num in Priority_list[3:]]
    #print("first_priority_order:",global_vars.first_priority_order)
    #print("second_priority_order:",global_vars.second_priority_order)

    if global_vars.first_priority_order and global_vars.second_priority_order:
        global_vars.priority_order_flag=1
        print(f"priority_order_flag{global_vars.priority_order_flag}")
        return True
    else:
        global_vars.priority_order_flag=0
        print(f"priority_order_flag{global_vars.priority_order_flag}")
        return False
    '''


"""

###########  改变抓取优先级

"""
def change_priority_order():   
    if global_vars.switch_priority_flag==0:     #第一次抓取的优先级
        if wait_for_ack(timeout=5):  # 超时时间
            print("收到切换确认，切换到物体识别模式")
            global_vars.mode = "object_detect"
        else:
            print("未收到切换确认，保持当前模式")
                   
        global_vars.QRmaterial_order=global_vars.first_priority_order
        print("first_priority_order:",global_vars.first_priority_order)
        
    if global_vars.switch_priority_flag==1:       #第二次抓取的优先级                  
        global_vars.QRmaterial_order=global_vars.second_priority_order
        global_vars.Find_blobs_enable=1            #重新使能找色块
        global_vars.switch_priority_flag=2  #防止重复抓取
        print("second_priority_order:",global_vars.second_priority_order)


