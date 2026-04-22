import model.cam as Camera
import model.detector as Detector
import model.stepper as Stepper
import model.tracker as Tracker
from model.tracker import Status 
import time
import cv2
import model.pid as pid

# --- 硬件全局实例化 ---
# 1. 摄像头
cam = Camera.Camera(index=4, width=640, height=480)

# 2. 检测器
detector = Detector.Detector(min_area=5000, max_area=500000)

# 3. 跟踪器 (开启卡尔曼滤波)
tracker = Tracker.Tracker(
    img_width=640, 
    img_height=480, 
    vfov=48.0, 
    hfov=80.0, 
    f_pixel_h=725.6, 
    real_height=17.5, 
    use_kf=True  
)

# 4. 电机
stepper_yaw = Stepper.EmmMotor(port='COM20', baudrate=115200, timeout=1, motor_id=1)
stepper_pitch = Stepper.EmmMotor(port='COM7', baudrate=115200, timeout=1, motor_id=2)

# 5. PID 控制器
pid_yaw = pid.PIDController(Kp=5, Ki=5, Kd=5, dt=1/30)
pid_pitch = pid.PIDController(Kp=5, Ki=5, Kd=5, dt=1/30)

def nothing(x):
    pass

# --- UI 初始化 ---
def init_board():
    # 主结果窗口
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Result', 540, 180)
    
    # 控制面板窗口
    cv2.namedWindow('Controls', cv2.WINDOW_FREERATIO)
    # 【修复点1】去掉了多余的 'each' 字符
    cv2.moveWindow('Controls', 0, 0) 
    cv2.resizeWindow('Controls', 320, 500)
    
    # 调试专用窗口
    cv2.namedWindow('DETECTOR', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('BIN', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('Tracker_View', cv2.WINDOW_FREERATIO) 
    
    # 创建滑动条
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)
    cv2.createTrackbar('yaw_kp', 'Controls', 2, 100, nothing)   
    cv2.createTrackbar('yaw_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('yaw_kd', 'Controls', 1, 100, nothing)  

    cv2.createTrackbar('pitch_kp', 'Controls', 2, 100, nothing)   
    cv2.createTrackbar('pitch_ki', 'Controls', 0, 100, nothing)   
    cv2.createTrackbar('pitch_kd', 'Controls', 1, 100, nothing)  

    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('Show_Debug_Windows', 'Controls', 1, 1, nothing)

# --- 参数更新 ---
last_thresh = -1

def update_hsv():
    global last_thresh
    
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')
    
    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls') / 100.0
    yaw_ki = cv2.getTrackbarPos('yaw_ki', 'Controls') / 10000.0
    yaw_kd = cv2.getTrackbarPos('yaw_kd', 'Controls') / 10000.0

    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls') / 100.0
    pitch_ki = cv2.getTrackbarPos('pitch_ki', 'Controls') / 10000.0
    pitch_kd = cv2.getTrackbarPos('pitch_kd', 'Controls') / 10000.0
    
    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')
    show_debug = cv2.getTrackbarPos('Show_Debug_Windows', 'Controls')

    # 赋值给检测器
    detector.threshold_value = current_thresh
    
    # 去重打印
    if current_thresh != last_thresh:
        print(f"[Update] New Threshold: {current_thresh}")
        last_thresh = current_thresh

    # 赋值给 PID
    pid_yaw.set_Kp(yaw_kp)
    pid_yaw.set_Ki(yaw_ki)
    pid_yaw.set_Kd(yaw_kd)
    
    pid_pitch.set_Kp(pitch_kp)
    pid_pitch.set_Ki(pitch_ki)
    pid_pitch.set_Kd(pitch_kd)
    
    return vel_rpm, acc, show_debug

def main():
    init_board()
    
    if not cam.cam.isOpened():
        print("摄像头打不开")
        return
        
    print("系统启动，按 'q' 退出...")
    
    prev_time = time.time()

    try:
        while True:
            # 1. 读帧
            ret, frame = cam.cam.read()
            if not ret: 
                print("无法读帧")
                break

            # 2. 更新参数
            vel_rpm, acc, show_debug = update_hsv()

            # 3. 识别目标
            # 假设 process_image 返回: (带标注的帧, 目标信息board)
            annotated_frame, board = detector.process_image(frame)
            
            # 【修复点2】数据同步：确保 detector 和 tracker 能访问到当前帧用于 display
            # 很多 display 函数需要原始图像来绘制背景。
            # 如果你们的 detector/tracker 类内部没有自动保存 frame，这里必须手动赋值。
            if hasattr(detector, 'raw'):
                detector.raw = frame
            if hasattr(tracker, 'raw'):
                tracker.raw = frame
            
            # 如果 detector.process_image 内部生成了二值图并保存在了 detector.last_binary 或类似变量
            # 请确保该变量已更新。如果没有，可能需要手动获取：
            # if hasattr(detector, 'last_binary'):
            #     bin_img = detector.last_binary 

            # 4. 跟踪与解算
            res = tracker.track(board)
            yaw, pitch, dist, status, laser_pos = res
            
            # 5. 计算 FPS
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-6)
            prev_time = curr_time

            # 6. 格式化状态文本
            if status == Status.TRACK:
                info_status = f"[TRACK] Y:{yaw:.2f} P:{pitch:.2f} D:{dist:.1f}"
                color = (0, 255, 0) 
            elif status == Status.TMP_LOST:
                info_status = f"[PREDICT] Y:{yaw:.2f} P:{pitch:.2f} D:{dist:.1f}"
                color = (0, 255, 255) 
            else:
                info_status = "[LOST] Searching..."
                color = (0, 0, 255) 

            # 终端打印
            print(f"FPS: {fps:.1f} | {info_status}")

            # 在主画面绘制信息
            cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated_frame, info_status, (10, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
            # 7. 控制电机
            if status in (Status.TRACK, Status.TMP_LOST):
                try:
                    correction_yaw = pid_yaw.compute(yaw)
                    stepper_yaw.emm_v5_move_to_angle(
                        angle_deg=-correction_yaw, 
                        vel_rpm=vel_rpm, 
                        acc=acc, 
                        abs_mode=False
                    )
                except Exception as e:
                    print(f" Yaw 电机指令异常: {e}")
                            
                try:
                    correction_pitch = pid_pitch.compute(pitch)
                    stepper_pitch.emm_v5_move_to_angle(
                        angle_deg=-correction_pitch, 
                        vel_rpm=vel_rpm, 
                        acc=acc, 
                        abs_mode=False
                    )
                except Exception as e:
                    print(f" Pitch 电机指令异常: {e}")

            elif status == Status.LOST:
                pid_yaw.reset()
                pid_pitch.reset()

            # 8. 显示与调试
            
            # 8.1 显示主结果
            cv2.imshow('Result', annotated_frame)
            
            # 8.2 显示调试窗口
            if show_debug:
                try:
                    # 尝试调用 detector 的 display 方法
                    # 注意：请确认你的 detector 类里真的有 display 方法，且返回两个值
                    vis_det, bin_img = detector.display(dis=1)
                    if vis_det is not None:
                        cv2.imshow("DETECTOR", vis_det)
                    if bin_img is not None:
                        cv2.imshow("BIN", bin_img)
                except AttributeError:
                    # 如果没有 display 方法，可以尝试直接显示内部变量（如果存在）
                    if hasattr(detector, 'last_binary') and detector.last_binary is not None:
                         cv2.imshow("BIN", detector.last_binary)
                except Exception as e:
                    # print(f"Detector Display Error: {e}") # 调试时可打开
                    pass

                try:
                    # 尝试调用 tracker 的 display 方法
                    vis_trk = tracker.display(dis=1, laser_pos=laser_pos)
                    if vis_trk is not None:
                        cv2.imshow("Tracker_View", vis_trk)
                except Exception as e:
                    # print(f"Tracker Display Error: {e}") # 调试时可打开
                    pass

            # 9. 退出控制
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n 收到中断信号...")
    except Exception as e:
        print(f" 主循环异常: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        print(" 正在释放资源...")
        cam.cam.release()
        try:
            stepper_yaw.close()
            stepper_pitch.close()
        except Exception as e:
            print(f" 电机关闭异常: {e}")
        
        cv2.destroyAllWindows()
        print(" 系统已安全关闭")

if __name__ == "__main__":
    main()