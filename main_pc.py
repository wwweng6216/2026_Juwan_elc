import cv2
import src.model.cam as camera
import src.model.detector as Detector
import src.model.tracker as Tracker
import src.model.stepper as Stepper
# from model.status import GPIN
import queue
import threading
import time
from collections import deque
# import Hobot.GPIO as GPIO

cam = camera.Camera(index = 0, width = 640, height = 480)
detector = Detector.Detector(min_area = 5000, max_area = 500000)
tracker = Tracker.Tracker(img_width = 640, img_height = 480, vfov= 48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5)
stepper_yaw = Stepper.MotorController(port = 'COM8' , baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.MotorController(port = 'COM7' , baudrate = 115200, timeout = 1, motor_id = 1)
# heart_beat = GPIN(pin =1 , mode = 1)
# task_info = GPIN(pin =2 , mode = 1)
# lazer = GPIN(pin = 3, mode = 1)
# task_switch = GPIN(pin = 4, mode = 0)


def nothing(x):
    pass

# 初始化 OpenCV 调试界面
def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)               #创建参数控制面板窗口
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)              #创建名为 Result 的显示窗口，允许自由拉伸     
    cv2.namedWindow('Mask', cv2.WINDOW_FREERATIO)                #显示二值化
    cv2.moveWindow('Mask', 360, 180)                             #将 Mask 窗口移至屏幕坐标 (x=360, y=180)
    cv2.moveWindow('Result', 540, 180)
    cv2.moveWindow('Controls', 0, 0)                           
    cv2.resizeWindow('Mask', 170, 150)                           #设置 Mask 窗口固定尺寸为 170×150 像素
    cv2.resizeWindow('Result', 170, 150)                   
    cv2.resizeWindow('Controls', 320, 500)
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)
    #创建滑动条
    #cv2.createTrackbar('滑动条名称', '绑定的窗口名', 初始值, 最大值, 回调函数)
    cv2.createTrackbar('board_min_area', 'Controls', 5000, 307200, nothing) #矩形最小面积
    cv2.createTrackbar('board_max_area', 'Controls', 54000, 307200, nothing) #最大面积
    #纯P控制
    cv2.createTrackbar('yaw_kp', 'Controls', 5, 100, nothing)  # 偏航角倍率: 0-10
    cv2.createTrackbar('pitch_kp', 'Controls', 6, 100, nothing)  # 俯仰角倍率: 0-10
    cv2.createTrackbar('vel_rpm', 'Controls', 5000, 5000, nothing)  # 速度: 0-5000 RPM
    cv2.createTrackbar('acc', 'Controls', 255, 255, nothing)  # 加速度: 0-255
    # cv2.createTrackbar('cx_offset', 'Controls', 30, 60, nothing)#xy偏移补偿
    # cv2.createTrackbar('cy_offset', 'Controls', 30, 60, nothing)
    # cv2.createTrackbar('show', 'Controls', 0, 1, nothing)#调试图像显示开关
    # cv2.createTrackbar('shoot_tol', 'Controls', 14, 200, nothing)#开火的阈值（小于就开火）
    # cv2.createTrackbar('offset_pitch', 'Controls', 11, 20, nothing)#俯仰轴机械安装偏移校准？

# 更新
def update_hsv():
    # # 读取 HSV
    # h_min = cv2.getTrackbarPos('H Min', 'Controls')#cv2.getTrackbarPos()：实时读取指定滑动条的当前值
    # h_max = cv2.getTrackbarPos('H Max', 'Controls')
    # s_min = cv2.getTrackbarPos('S Min', 'Controls')
    # s_max = cv2.getTrackbarPos('S Max', 'Controls')
    # v_min = cv2.getTrackbarPos('V Min', 'Controls')
    # v_max = cv2.getTrackbarPos('V Max', 'Controls')
    #读取面积限制
    board_min_area = cv2.getTrackbarPos('board_min_area', 'Controls')
    board_max_area = cv2.getTrackbarPos('board_max_area', 'Controls')
    # # 读取轮廓长宽比限制
    # diameter_ratio = cv2.getTrackbarPos('diameter_ratio', 'Controls')
    #读取P
    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')
    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls')
    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')
    # cx_offset = cv2.getTrackbarPos('cx_offset', 'Controls')
    # cy_offset = cv2.getTrackbarPos('cy_offset', 'Controls')
    #开关
    # show = cv2.getTrackbarPos('show', 'Controls')
    # #读取开火限制
    # shoot_tol = cv2.getTrackbarPos('shoot_tol', 'Controls')
    # #读取俯仰轴机械安装偏移校准
    # offset_pitch = cv2.getTrackbarPos('offset_pitch', 'Controls')

    #打包到对应对象
    detector.board_min_area = board_min_area
    detector.board_max_area = board_max_area
    # detector.cx_offset = cx_offset - 30      #需要现场修改偏移量     
    # detector.cy_offset = cy_offset - 30      #需要现场修改偏移量
    # detector.show_img = show
    # tracker.offset_pitch = offset_pitch - 10
    # tracker.shoot_tol = shoot_tol

    return yaw_kp / 100.0, pitch_kp / 100.0, vel_rpm, acc

'''
while running.is_set():
    try:
        # 需要循环执行的业务代码
        ...
    except Exception as e:
        # 异常处理逻辑
        ...
'''
def tracking_and_steering_thread(tracker, target_yaw, target_pitch, position_queue, dt_queue, running):
    """跟踪和转向线程，使用 kp 倍率控制"""
    while running.is_set():
        try:
            #获取最新目标
            target_yaw, target_pitch = 0, 0

            # 2. 获取新帧并解算
            if not position_queue.empty():
                board = position_queue.get()
                result = tracker.solve(board)

                if result is not None:
                    target_yaw, target_pitch, dist = result
                    tracker.if_lost = False
                else:
                    tracker.if_lost = True

            # 3. 统一执行控制
            if tracker.if_lost == True:
                try:
                    stepper_yaw.emm_v5_move_to_angle(angle_deg=3, vel_rpm=2000, acc=0, abs_mode=False)
                except Exception as e:
                    print(f"Yaw 电机错误: {str(e)}")
            else:
                yaw_kp, pitch_kp, vel_rpm, acc = update_hsv()
                if abs(target_yaw) > detector.cx_offset:
                    try:
                        stepper_yaw.emm_v5_move_to_angle(angle_deg=target_yaw * yaw_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                    except Exception as e:
                        print(f"Yaw 电机错误: {str(e)}")
                if abs(target_pitch) > detector.cy_offset:
                    try:
                        stepper_pitch.emm_v5_move_to_angle(angle_deg=target_pitch * pitch_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                    except Exception as e:
                        print(f"Pitch 电机错误: {str(e)}")

                        

        except Exception as e:
            print(f"跟踪线程错误: {str(e)}")

        # 移除 time.sleep 以提高频率
        time.sleep(0.000001)  # 可选：如果 CPU 使用率过高，可尝试 0.1ms

def decision(running, detector, tracker):
    """决策线程，用于处理心跳、任务信息显示和任务切换"""
    try:
        while running.is_set():
            # heart_beat.flash()
            # if detector.task in [0, 1]:
            #     task_info.set_value(1 if detector.task > 0 else 0)
            #     lazer.set_value(0 if detector.task > 0 else 0)
            # new_task = task_switch.button_callback(detector.task)
            # detector.task = new_task
            # if not tracker.shoot:
            #     lazer.set_value(0)
            # else:
            #     lazer.set_value(1)
            time.sleep(0.01)
            
    except Exception as e:
        print(f"决策线程错误: {str(e)}")
    finally:
        # heart_beat.set_value(0)
        # task_info.set_value(0)
        # lazer.set_value(0)
        pass

def main():
    init_board()
    if not cam.cam.isOpened():
        print("Camera open failed")
        return

    position_queue = queue.Queue(maxsize=1)
    dt_queue = queue.Queue(maxsize=1)
    running = threading.Event()
    running.set()

    tracking_thread = threading.Thread(target=tracking_and_steering_thread, 
                                     args=(tracker, stepper_yaw, stepper_pitch, position_queue, dt_queue, running))
    tracking_thread.daemon = True
    tracking_thread.start()

    decision_thread = threading.Thread(target=decision,
                                     args=(running, detector, tracker))
    decision_thread.daemon = True
    decision_thread.start()

    dt = 1/30
    last_time = time.time()
    last_frame_time = time.time()
    beat = 0
    frame_count = 0
    fps = 0

    try:
        while True:
            try:
                ret, frame = cam.cam.read()
                if not ret:
                    print("Failed to read frame")
                    break

                update_hsv()  # 同步滑动条参数

                # 统一调用新接口，接收 (标注图, Board对象)
                result, board = detector.process_image(frame)
                
                # 队列放入 board 对象（含 center/is_valid/points）
                if position_queue.full():
                    position_queue.get()  # 丢弃旧帧
                position_queue.put(board)  # 传入完整 Board 实例
                
                # 显示逻辑适配新返回值
                # if detector.show_img == 1:
                #     # Mask 窗口：显示 process_image 内部存储的二值图
                #     if detector.last_binary is not None:
                #         # 转3通道以便彩色显示（可选）
                #         mask_3ch = cv2.cvtColor(detector.last_binary, cv2.COLOR_GRAY2BGR)
                #         cv2.imshow('Mask', mask_3ch)
                #     # Result 窗口：直接显示带标注的结果图
                #     cv2.imshow('Result', result)
                # Mask 窗口：显示 process_image 内部存储的二值图
                if detector.last_binary is not None:
                    # 转3通道以便彩色显示（可选）
                    mask_3ch = cv2.cvtColor(detector.last_binary, cv2.COLOR_GRAY2BGR)
                    cv2.imshow('Mask', mask_3ch)
                # Result 窗口：直接显示带标注的结果图
                cv2.imshow('Result', result)

                # 打印是否检测到有效靶板
                print(f"Board valid: {board.is_valid}, shoot: {tracker.shoot}")
                
                # 计时与 FPS 计算（保持不变）
                current_time = time.time()
                frame_count += 1
                dt = current_time - last_frame_time
                if dt_queue.full():
                    dt_queue.get()
                dt_queue.put(dt)
                last_frame_time = current_time

                elapsed_time = current_time - last_time
                if elapsed_time >= 1.0:
                    fps = frame_count / elapsed_time
                    frame_count = 0
                    last_time = current_time
                    print(f"FPS: {fps:.2f}")
                beat += 1
                beat %= 255

                if cv2.waitKey(1) == ord('q'):
                    break

            except Exception as e:
                print(f"主循环错误: {str(e)}")
        
    finally:
        running.clear()
        tracking_thread.join()
        decision_thread.join()
        cam.cam.release()
        try:
            stepper_yaw.close()
        except Exception as e:
            print(f"关闭 yaw 步进电机错误: {str(e)}")
        try:
            stepper_pitch.close()
        except Exception as e:
            print(f"关闭 pitch 步进电机错误: {str(e)}")
        # GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()