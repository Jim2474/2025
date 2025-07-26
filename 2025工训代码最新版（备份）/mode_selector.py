"""
模式选择器 - 用于切换不同的识别模式
"""
import global_vars
from maix import app
import time

def show_menu():
    """显示模式选择菜单"""
    print("\n" + "="*50)
    print("           模式选择菜单")
    print("="*50)
    print("1. 二维码识别模式 (find_qr)")
    print("2. 目标检测模式 (object_detect)")
    print("3. 颜色块检测模式 (Find_blobs)")
    print("4. 激光笔识别模式 (laser_detect)")
    print("5. 激光笔+触摸GUI调试 (laser_gui)")
    print("6. 激光笔+简化GUI调试 (laser_simple)")
    print("7. 退出程序")
    print("="*50)

def select_mode():
    """选择运行模式"""
    while True:
        show_menu()
        try:
            choice = input("请输入选择的模式编号 (1-7): ").strip()

            if choice == "1":
                global_vars.mode = "find_qr"
                print("已切换到二维码识别模式")
                break
            elif choice == "2":
                global_vars.mode = "object_detect"
                print("已切换到目标检测模式")
                break
            elif choice == "3":
                global_vars.mode = "Find_blobs"
                print("已切换到颜色块检测模式")
                break
            elif choice == "4":
                global_vars.mode = "laser_detect"
                print("已切换到激光笔识别模式")
                break
            elif choice == "5":
                global_vars.mode = "laser_gui"
                print("已切换到激光笔+触摸GUI调试模式")
                print("提示: 触摸屏幕顶部可切换调试/检测模式")
                break
            elif choice == "6":
                global_vars.mode = "laser_simple"
                print("已切换到激光笔+简化GUI调试模式")
                print("提示: 自动切换模式，无需触摸操作")
                break
            elif choice == "7":
                print("退出程序...")
                app.set_exit_flag(True)
                return False
            else:
                print("无效选择，请输入1-7之间的数字")
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            app.set_exit_flag(True)
            return False
        except Exception as e:
            print(f"输入错误: {e}")
            time.sleep(1)
    
    return True

def main():
    """主函数"""
    print("欢迎使用多模式识别系统!")
    
    # 显示当前模式
    print(f"当前模式: {global_vars.mode}")
    
    # 选择新模式
    if select_mode():
        print(f"模式已设置为: {global_vars.mode}")
        print("请运行 main.py 开始识别")
    else:
        print("程序已退出")

if __name__ == "__main__":
    main()
