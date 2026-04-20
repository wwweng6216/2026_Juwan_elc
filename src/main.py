import model.cam as Camera
import model.detector as Detector
import model.stepper as Stepper
import model.tracker as Tracker
import time
import cv2

#硬件全局实例化
'''
要实例的有
cam ,detector, tracker, stepper_yaw, stepper_pitch
顺序逻辑：数据源(摄像头) → 处理源(识别) → 决策源(跟踪) → 执行器(电机控制)
'''
cam = Camera.Camera(index = 4, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5)
stepper_yaw = Stepper.EmmMotor(port ='COM20', baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port ='COM7', baudrate = 115200, timeout = 1, motor_id = 2)

def nothing(x):
    pass
#UI
def init_board():
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Result', 540, 180)
    cv2.namedWindow('Controls', cv2.WINDOW_FREERATIO)
    cv2.moveWindow('Controls', 0, 0)
    cv2.resizeWindow('Controls', 320, 500)
    '''
    创建滑动条
    '''
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)

    cv2.createTrackbar('yaw_kp', 'Controls', 5, 100, nothing)
    cv2.createTrackbar('pitch_kp', 'Controls', 6, 100, nothing)
    cv2.createTrackbar('vel_rpm', 'Controls', 3000, 5000, nothing)
    cv2.createTrackbar('acc', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)
#读原始值——数据转换——赋值给模块——返回控制参
def update_hsv():
    #读原始值
    current_thresh = cv2.getTrackbarPos('Threshold', 'Controls')
    yaw_kp = cv2.getTrackbarPos('yaw_kp', 'Controls')/ 100.0     #"/ 100.0"是数据转换
    pitch_kp = cv2.getTrackbarPos('pitch_kp', 'Controls') / 100.0   #"/ 100.0"是数据转换
    vel_rpm = cv2.getTrackbarPos('vel_rpm', 'Controls')
    acc = cv2.getTrackbarPos('acc', 'Controls')
    #数据处理还要写一个零点偏移
    
    #赋值给模块
    detector.threshold_value = current_thresh

    #返回控制参
    return yaw_kp, pitch_kp, vel_rpm, acc

def main ():
    init_board()
    if not cam.cam.isOpened():
        print("摄像头打不开")
        return
    ast_frame_time = time.time()
    print(" 系统启动，按 'q' 退出...")

    try:
        while True:
            '''
            读帧-更新参数-识别目标-计算角度-控制电机-显示画面-延时与退出
            '''
            #读帧
            ret, frame = cam.cam.read()
            if not ret: 
                print("无法读帧")
                break

            #更新参数
            yaw_kp, pitch_kp, vel_rpm, acc = update_hsv()

            #识别目标
            annotated_frame, board = detector.process_image(frame)

            #计算角度 & 控制电机
            if board.is_valid:
                res = tracker.solve(board)
                if res:  # 确保 solve 返回了有效结果（如元组）
                    yaw, pitch, dist = res
                    info = f"Yaw:{yaw:.2f} Pitch:{pitch:.2f} Dis:{dist:.1f}cm"
                    cv2.putText(annotated_frame, info, (10, 70), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # 运行电机
                    if abs(yaw) > 0:
                        try:
                            print(f"yaw: {yaw}")
                            stepper_yaw.emm_v5_move_to_angle(
                                angle_deg=yaw * yaw_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                        except Exception as e:
                            print(f" Yaw 电机指令异常: {e}")
                            
                    if abs(pitch) > 0:
                        try:
                            print(f"pitch: {pitch}")
                            stepper_pitch.emm_v5_move_to_angle(
                                angle_deg=pitch * pitch_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                        except Exception as e:
                            print(f" pitch 电机指令异常: {e}")
                    
            # 显示与退出
            cv2.imshow('bin', detector.last_binary)
            cv2.imshow('Result', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    except Exception as e:
        print(f" 主循环异常: {str(e)}")
    except KeyboardInterrupt:
        print(" 收到中断信号...")
    
    #资源清理
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