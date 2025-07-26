from maix import image, camera, display, app, touchscreen
import cv2
import json
import os

class LABThresholdAdjuster:
    """LAB颜色空间阈值调整器"""
    
    def __init__(self):
        # LAB阈值参数 (L_min, L_max, A_min, A_max, B_min, B_max)
        self.lab_thresholds = [0, 100, -128, 127, -128, 127]
        self.threshold_names = ["L_min", "L_max", "A_min", "A_max", "B_min", "B_max"]
        self.threshold_ranges = [(0, 100), (0, 100), (-128, 127), (-128, 127), (-128, 127), (-128, 127)]
        
        # 当前选中的通道
        self.selected_channel = 0
        
        # GUI参数
        self.slider_y = 200  # 滑块Y位置
        self.slider_width = 280
        self.slider_height = 20
        self.button_height = 30
        
        # 配置文件路径
        self.config_file = "laser_lab_config.json"
        
        # 加载保存的配置
        self.load_config()
    
    def load_config(self):
        """加载配置文件"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    self.lab_thresholds = config.get('lab_thresholds', self.lab_thresholds)
                    print(f"已加载配置: {self.lab_thresholds}")
        except Exception as e:
            print(f"加载配置失败: {e}")
    
    def save_config(self):
        """保存配置文件"""
        try:
            config = {'lab_thresholds': self.lab_thresholds}
            with open(self.config_file, 'w') as f:
                json.dump(config, f)
            print(f"已保存配置: {self.lab_thresholds}")
        except Exception as e:
            print(f"保存配置失败: {e}")
    
    def draw_channel_buttons(self, img):
        """绘制通道选择按钮"""
        button_width = 45
        start_x = 5
        y = 170
        
        for i, name in enumerate(self.threshold_names):
            x = start_x + i * (button_width + 2)
            color = image.COLOR_RED if i == self.selected_channel else image.COLOR_GRAY
            
            # 绘制按钮背景
            img.draw_rect(x, y, button_width, self.button_height, color, 2)
            
            # 绘制按钮文字
            img.draw_string(x + 5, y + 8, name, image.COLOR_WHITE, scale=0.5)
    
    def draw_slider(self, img):
        """绘制滑块"""
        # 获取当前通道的值和范围
        current_value = self.lab_thresholds[self.selected_channel]
        min_val, max_val = self.threshold_ranges[self.selected_channel]
        
        # 计算滑块位置
        slider_pos = int((current_value - min_val) / (max_val - min_val) * self.slider_width)
        
        # 绘制滑块轨道
        img.draw_rect(20, self.slider_y, self.slider_width, self.slider_height, image.COLOR_GRAY, 2)
        
        # 绘制滑块
        img.draw_rect(20 + slider_pos - 5, self.slider_y - 5, 10, self.slider_height + 10, image.COLOR_BLUE, -1)
        
        # 显示当前值
        value_text = f"{self.threshold_names[self.selected_channel]}: {current_value}"
        img.draw_string(20, self.slider_y + 25, value_text, image.COLOR_WHITE, scale=0.6)
        
        # 显示范围
        range_text = f"Range: {min_val} ~ {max_val}"
        img.draw_string(20, self.slider_y + 40, range_text, image.COLOR_WHITE, scale=0.5)
    
    def handle_touch(self, touch_x, touch_y):
        """处理触摸事件"""
        # 检查是否点击了通道按钮
        button_width = 45
        start_x = 5
        button_y = 170
        
        if button_y <= touch_y <= button_y + self.button_height:
            for i in range(len(self.threshold_names)):
                x = start_x + i * (button_width + 2)
                if x <= touch_x <= x + button_width:
                    self.selected_channel = i
                    return True
        
        # 检查是否点击了滑块区域
        if self.slider_y - 10 <= touch_y <= self.slider_y + self.slider_height + 10:
            if 20 <= touch_x <= 20 + self.slider_width:
                # 计算新的值
                min_val, max_val = self.threshold_ranges[self.selected_channel]
                ratio = (touch_x - 20) / self.slider_width
                new_value = int(min_val + ratio * (max_val - min_val))
                
                # 限制范围
                new_value = max(min_val, min(max_val, new_value))
                self.lab_thresholds[self.selected_channel] = new_value
                return True
        
        return False
    
    def get_lab_threshold_tuple(self):
        """获取LAB阈值元组"""
        return tuple(self.lab_thresholds)

def laser_detection_with_gui(disp, cam):
    """
    带GUI的激光笔识别模式
    集成LAB阈值调整功能
    """
    # 初始化触摸屏
    try:
        ts = touchscreen.TouchScreen()
        print("触摸屏初始化成功")
        # 测试触摸屏数据格式
        test_touches = ts.read()
        print(f"触摸屏测试数据格式: {type(test_touches)}, 内容: {test_touches}")
    except Exception as e:
        print(f"警告: 触摸屏初始化失败: {e}")
        print("将使用键盘模拟触摸（仅用于调试）")
        ts = None
    
    # 初始化阈值调整器
    adjuster = LABThresholdAdjuster()
    
    # 帧差法相关参数
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    point_x, point_y = 0, 0
    last_img_cv_gray = None
    frame_diff_threshold = 25  # 帧差阈值
    min_contour_area = 200     # 最小轮廓面积
    
    # 显示模式: 0=原图+检测结果, 1=LAB阈值调试
    display_mode = 0
    mode_switch_time = 0
    
    print("激光笔识别模式已启动")
    print("触摸屏幕切换显示模式")
    print("调试模式下可调整LAB阈值参数")
    
    while not app.need_exit():
        img = cam.read()
        
        # 处理触摸事件 - 使用更安全的方式
        if ts:
            try:
                touches = ts.read()
                touch_x, touch_y = None, None

                # 尝试不同的触摸数据格式解析
                if touches:
                    print(f"触摸数据: {touches}, 类型: {type(touches)}")

                    # 格式1: 直接是坐标元组 (x, y)
                    if isinstance(touches, (list, tuple)) and len(touches) >= 2:
                        if isinstance(touches[0], (int, float)) and isinstance(touches[1], (int, float)):
                            touch_x, touch_y = int(touches[0]), int(touches[1])

                    # 格式2: 嵌套列表 [[x, y, ...], ...]
                    elif isinstance(touches, (list, tuple)) and len(touches) > 0:
                        touch_point = touches[0]
                        if isinstance(touch_point, (list, tuple)) and len(touch_point) >= 2:
                            touch_x, touch_y = int(touch_point[0]), int(touch_point[1])

                    # 如果成功获取到坐标
                    if touch_x is not None and touch_y is not None:
                        print(f"触摸坐标: ({touch_x}, {touch_y})")

                        if display_mode == 1:  # 调试模式
                            if adjuster.handle_touch(touch_x, touch_y):
                                pass  # 参数已更新
                            elif touch_y < 50:  # 点击顶部切换模式
                                display_mode = 0
                                adjuster.save_config()  # 保存配置
                                print("切换到检测模式")
                        else:  # 检测模式
                            if touch_y < 50:  # 点击顶部切换模式
                                display_mode = 1
                                print("切换到调试模式")

            except Exception as e:
                print(f"触摸事件处理错误: {e}")
                # 继续运行，不因触摸错误而中断
        
        if display_mode == 0:
            # 激光检测模式
            img_cv = image.image2cv(img, False, False)
            img_cv_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
            
            if last_img_cv_gray is not None:
                # 帧差法检测
                img_diff = cv2.absdiff(img_cv_gray, last_img_cv_gray)
                _, img_binary = cv2.threshold(img_diff, frame_diff_threshold, 255, cv2.THRESH_BINARY)
                img_binary = cv2.dilate(img_binary, kernel, iterations=2)
                
                # 查找轮廓
                contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    contour_area = cv2.contourArea(contour)
                    if contour_area < min_contour_area:
                        continue
                    
                    # 计算轮廓中心
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        point_x = int(M["m10"] / M["m00"])
                        point_y = int(M["m01"] / M["m00"])
                        
                        # 获取激光点区域
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # 使用调整后的LAB阈值进行颜色验证
                        lab_threshold = adjuster.get_lab_threshold_tuple()
                        hist = img.get_histogram(thresholds=[lab_threshold], roi=(x, y, w, h))
                        value = hist.get_statistics().a_median()
                        
                        print(f'LAB统计值: {value}, 轮廓面积: {contour_area}')
                        print(f'当前LAB阈值: {lab_threshold}')
            
            last_img_cv_gray = img_cv_gray.copy()
            
            # 绘制检测结果
            img.draw_cross(point_x, point_y, image.COLOR_BLUE, 5, 2)
            
            # 显示当前LAB阈值
            threshold_text = f"LAB: {adjuster.get_lab_threshold_tuple()}"
            img.draw_string(5, 5, threshold_text, image.COLOR_WHITE, scale=0.4)
            img.draw_string(5, 20, "Touch top to adjust", image.COLOR_YELLOW, scale=0.5)
            
        else:
            # LAB阈值调试模式
            # 应用LAB阈值显示过滤结果
            lab_threshold = adjuster.get_lab_threshold_tuple()
            
            try:
                # 创建LAB阈值掩码
                hist = img.get_histogram(thresholds=[lab_threshold])
                
                # 在图像上叠加调试信息
                img.draw_string(5, 5, "LAB Threshold Debug Mode", image.COLOR_WHITE, scale=0.6)
                img.draw_string(5, 25, "Touch top to return", image.COLOR_YELLOW, scale=0.5)
                
                # 绘制GUI控件
                adjuster.draw_channel_buttons(img)
                adjuster.draw_slider(img)
                
                # 显示完整的LAB阈值
                full_threshold = f"LAB: {lab_threshold}"
                img.draw_string(5, 140, full_threshold, image.COLOR_WHITE, scale=0.5)
                
            except Exception as e:
                img.draw_string(5, 50, f"Error: {str(e)}", image.COLOR_RED, scale=0.5)
        
        disp.show(img)

# 为了与main.py系统兼容，提供统一的接口函数
def laser_detection_mode_with_gui(disp, cam):
    """
    带GUI的激光笔识别模式 - 供main.py调用的接口
    """
    return laser_detection_with_gui(disp, cam)

# 如果直接运行此文件（用于测试）
if __name__ == "__main__":
    cam = camera.Camera(320, 240)
    disp = display.Display()

    try:
        laser_detection_with_gui(disp, cam)
    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"发生错误: {e}")
