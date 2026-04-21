import model.cam as Camera
import model.detector as Detector
import model.stepper as Stepper
import model.tracker as Tracker
import time
import cv2
import Hobot.GPIO as GPIO
import model.status as GPIN

#硬件全局实例化
'''
要实例的有
cam ,detector, tracker, stepper_yaw, stepper_pitch, 需要的GPIO
顺序逻辑：数据源(摄像头) → 处理源(识别) → 决策源(跟踪) → 执行器(电机控制)
'''
cam = Camera.Camera(index = 4, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(img_width=640, img_height=480, vfov=48.0, hfov =80.0, f_pixel_h=725.6, real_height=17.5, use_kf = False)
stepper_yaw = Stepper.EmmMotor(port ='COM20', baudrate = 115200, timeout = 1, motor_id = 1)
stepper_pitch = Stepper.EmmMotor(port ='COM7', baudrate = 115200, timeout = 1, motor_id = 2)
heart_beat = GPIN.GPIN(pin=13, mode=1) #呼吸灯，用于表示主程序还在跑

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
            #呼吸灯，证明主程序在运行(单线程中闪烁频率完全受制于主循环的运行速度)
            heart_beat.flash()
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

            # 控制电机
            res = tracker.track(board)
            yaw, pitch, dist, status, laser_pos = res
            info =  f"Status:{status.name} Yaw:{yaw:.2f} P:{pitch:.2f} D:{dist:.1f}"
            color = (0, 255, 0) if status == Tracker.Status.TRACK else (0, 255, 255) # 绿色正常，黄色预测
            cv2.putText(annotated_frame, info, (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
            # 运行电机
            if status in (Tracker.Status.TRACK, Tracker.Status.TMP_LOST):
                try:
                    print(f"yaw: {yaw}")
                    stepper_yaw.emm_v5_move_to_angle(
                        angle_deg=yaw * yaw_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                except Exception as e:
                    print(f" Yaw 电机指令异常: {e}")
                            
                try:
                    print(f"pitch: {pitch}")
                    stepper_pitch.emm_v5_move_to_angle(
                        angle_deg=pitch * pitch_kp, vel_rpm=vel_rpm, acc=acc, abs_mode=False)
                except Exception as e:
                    print(f" pitch 电机指令异常: {e}")
            elif status == Tracker.Status.LOST:
                pass

                    
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