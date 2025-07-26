from maix import image, camera, display, app
import cv2
import json
import os

def load_lab_config():
    """加载LAB阈值配置"""
    config_file = "laser_lab_config.json"
    default_thresholds = (0, 100, -128, 127, -128, 127)

    try:
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = json.load(f)
                thresholds = config.get('lab_thresholds', list(default_thresholds))
                return tuple(thresholds)
    except Exception as e:
        print(f"加载LAB配置失败: {e}")

    return default_thresholds

def laser_detection_mode(disp, cam):
    """
    激光笔识别模式
    通过帧差法检测激光点位置并在屏幕上显示
    使用可配置的LAB阈值进行颜色验证
    """
    # 加载LAB阈值配置
    lab_thresholds = load_lab_config()
    print(f"使用LAB阈值: {lab_thresholds}")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))  # 创建一个 5x5 的矩形膨胀核

    # 初始化激光点坐标位置变量
    point_x = 0
    point_y = 0

    last_img_cv_gray = None  # 用于保存前一帧图像的灰度值，用于帧差计算( 前后两帧是否相同 ，来判断是否有运动的物体)
    
    
    while not app.need_exit():
        img = cam.read()
        img_cv = image.image2cv(img, False, False)  # 将 MaixPy 图像对象转换为 OpenCV 图像
        img_cv_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)  # 转换为灰度图，以便进行帧差计算
        
        if last_img_cv_gray is None:  # 首帧处理（初始化历史帧）,如果是第一帧，直接保存当前帧为"上一帧"；
            last_img_cv_gray = img_cv_gray.copy()
            continue
        
        # 计算差值
        img_diff = cv2.absdiff(img_cv_gray, last_img_cv_gray)
        # 二值化处理(灰度图画面特别暗淡不清晰，需要二值化)，返回的是元组，第一个是返回阈值，第二个是处理后的图，直接用图二覆盖图一
        _, img_binary = cv2.threshold(img_diff, 25, 255, cv2.THRESH_BINARY)
        # 阈值25，高于此值为运动
        # 膨胀处理强化激光点
        img_binary = cv2.dilate(img_binary, kernel, iterations=2)  # 使用开始定义的膨胀核做两次膨胀
        
        # check point 放开注释，应能黑色背景下激光点二值化后的白点
        # img_show = image.cv2image(img_binary, False, False)
        # disp.show(img_show)
        
        # 查找轮廓
        contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # 计算轮廓面积
            contour_area = cv2.contourArea(contour)
            # 选点太小不过，过滤掉太小范围的轮廓
            if contour_area < 200:  # 小于200当作噪点
                continue
            # 计算轮廓中心作为激光点坐标坐标
            M = cv2.moments(contour)
            if M["m00"] != 0:  # 避免除零错误
                point_x = int(M["m10"] / M["m00"])
                point_y = int(M["m01"] / M["m00"])
                # 获取激光点轮廓的外接矩形
                x, y, w, h = cv2.boundingRect(contour)
                # 使用配置的LAB阈值进行颜色验证
                hist = img.get_histogram(thresholds=[lab_thresholds], roi=(x, y, w, h))
                value = hist.get_statistics().a_median()

                print('LAB统计值: {} 轮廓面积: {} LAB阈值: {}'.format(value, contour_area, lab_thresholds))
        
        last_img_cv_gray = img_cv_gray.copy()
        img.draw_cross(point_x, point_y, image.COLOR_BLUE, 5, 2)
        disp.show(img)


