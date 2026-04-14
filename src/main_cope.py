import cv2
import time
import model.cam as camera
import model.detector as Detector
import model.tracker as Tracker
import model.stepper as Stepper

def nothing(x):
    pass

def init_board():
    """初始化 OpenCV 窗口与滑动条"""
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('Controls', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Controls', 0, 0)
    cv2.resizeWindow('Controls', 320, 500)
    cv2.resizeWindow('Result', 320, 240)

    # HSV 与识别参数
    cv2.createTrackbar('H Min', 'Controls', 0, 179, nothing)
    cv2.createTrackbar('H Max', 'Controls', 179, 179, nothing)
    cv2.createTrackbar('S Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('S Max', 'Controls', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('V Max', 'Controls', 55, 255, nothing)
    cv2.createTrackbar('board_min_area', 'Controls', 5000, 307200, nothing)
    cv2.createTrackbar('board_max_area', 'Controls', 54000, 307200, nothing)
    cv2.createTrackbar('diameter_ratio', 'Controls', 40, 70, nothing)
    
    # 控制参数
    cv2.createTrackbar('yaw_kp', 'Controls', 5, 100, nothing)
    cv2.createTrackbar('pitch_kp', 'Controls', 6, 100, nothing)
    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)  # 建议初始值调低防抖动
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)

def update_hsv():
    """读取滑动条值并更新到模块中"""
    h_min = cv2.getTrackbarPos('H Min', 'Controls')
    h_max = cv2.getTrackbarPos('H Max', 'Controls')
    s_min = cv2.getTrackbarPos('S Min', 'Controls')
    s_max = cv2.getTrackbarPos('S Max', 'Controls')
    v_min = cv2.getTrackbarPos('V Min', 'Controls')
    v_max = cv2.getTrackbarPos('V Max', 'Controls')
    
    detector.board_lower = (h_min, s_min, v_min)
    detector.board_upper = (h_max, s_max, v_max)
    detector.diameter_ratio = cv2.getTrackbarPos('diameter_ratio', 'Controls') / 100.0
    detector.board_min_area = cv2.getTrackbarPos('board_min_area', 'Controls')
    detector.board_max_area = cv2.getTrackbarPos('board_max_area', 'Controls')
    detector.show_img = cv2.getTrackbarPos('show', 'Controls')

    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls') / 100.0
    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls') / 100.0
    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')

    return yaw_kp, pitch_kp, vel_rpm, acc

def main():
    init_board()
    
    # 基础跟踪默认使用 task1，可根据实际改为动态切换
    detector.task = 0  

    if not cam.cam.isOpened():
        print("❌ 摄像头打开失败")
        return

    last_frame_time = time.time()
    print("✅ 系统启动，按 'q' 退出...")

    try:
        while True:
            # 1. 读取图像
            ret, frame = cam.cam.read()
            if not ret:
                print("⚠️ 读取帧失败，可能摄像头断开")
                break

            # 2. 更新调参
            yaw_kp, pitch_kp, vel_rpm, acc = update_hsv()

            # 3. 目标识别
            position = detector.task1(frame)

            # 4. 追踪计算 & 电机控制（单线程顺序执行）
            if position is not None:
                # 计算帧间隔时间 dt
                current_time = time.time()
                dt = current_time - last_frame_time
                last_frame_time = current_time

                # 计算目标角度
                target_yaw, target_pitch = tracker.track(position, dt)

                # 死区过滤 + KP倍率 + 发送指令
                # 注意：确保 emm_v5_move_to_angle 是非阻塞的，或 timeout 极短
                if abs(target_yaw) > 0.5:
                    try:
                        stepper_yaw.emm_v5_move_to_angle(
                            angle_deg=target_yaw * yaw_kp, 
                            vel_rpm=vel_rpm, acc=acc, abs_mode=False
                        )
                    except Exception as e:
                        print(f"⚠️ Yaw 电机指令异常: {e}")
                        
                if abs(target_pitch) > 0.5:
                    try:
                        stepper_pitch.emm_v5_move_to_angle(
                            angle_deg=target_pitch * pitch_kp, 
                            vel_rpm=vel_rpm, acc=acc, abs_mode=False
                        )
                    except Exception as e:
                        print(f"⚠️ Pitch 电机指令异常: {e}")

            # 5. 画面显示
            if detector.show_img == 1:
                result = detector.display(frame)
                cv2.imshow('Result', result)

            # 6. 退出检测
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n⏹️ 收到中断信号...")
    except Exception as e:
        print(f"❌ 主循环异常: {str(e)}")
    finally:
        print("🧹 正在释放资源...")
        cam.cam.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except Exception as e:
            print(f"⚠️ 电机关闭异常: {e}")
        cv2.destroyAllWindows()
        print("✅ 系统已安全关闭")

# ================= 硬件实例化 =================
cam = camera.Camera(index=0, format='MJPG', width=640, height=480, fps=240)
detector = Detector.Detector(
    board_color=[(13, 255, 152), (0, 51, 110)], 
    board_min_area=18310, board_max_area=50000, diameter_ratio=0.5
)
tracker = Tracker.Tracker(
    img_width=640, img_height=480, vfov=100, use_kf=False, 
    frame_add=50, shoot_tol=5, ref_point=(-0.03, 0, 0), 
    offset_pitch=-10, offset_yaw=0.0, is_mirrored=False
)
stepper_yaw = Stepper.MotorController(port='/dev/ttyS1', baudrate=115200, timeout=0.001, motor_id=1)
stepper_pitch = Stepper.MotorController(port='/dev/ttyS3', baudrate=115200, timeout=0.001, motor_id=2)
# =================================================

if __name__ == "__main__":
    main()