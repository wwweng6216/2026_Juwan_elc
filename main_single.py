# main_single.py - 纯上位机视觉调试模块（无电机/无GPIO）
import cv2
import model.cam as camera
import model.detector as Detector
import model.tracker as Tracker
import queue
import threading
import time

# ========== 硬件实例化（仅保留相机） ==========
cam = camera.Camera(index=4, width=640, height=480)
detector = Detector.Detector(min_area=5000, max_area=500000)
tracker = Tracker.Tracker(
    img_width=640, img_height=480, 
    vfov=48.0, hfov=80.0, 
    f_pixel_h=725.6, real_height=17.5
)

# ========== 全局状态（替代GPIO任务切换） ==========
class SimpleState:
    def __init__(self):
        self.task = 0  # 0: task1, 1: task2（用键盘1/2切换）
        self.show_img = 1

state = SimpleState()


def nothing(x):
    pass


def init_board():
    """初始化调试窗口和滑动条"""
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Result', cv2.WINDOW_FREERATIO)
    cv2.namedWindow('Mask', cv2.WINDOW_FREERATIO)
    
    cv2.moveWindow('Mask', 360, 180)
    cv2.moveWindow('Result', 540, 180)
    cv2.moveWindow('Controls', 0, 0)
    
    cv2.resizeWindow('Mask', 170, 150)
    cv2.resizeWindow('Result', 170, 150)
    cv2.resizeWindow('Controls', 320, 500)
    
    # 基础参数
    cv2.createTrackbar('Threshold', 'Controls', 127, 255, nothing)
    cv2.createTrackbar('H Min', 'Controls', 0, 179, nothing)
    cv2.createTrackbar('H Max', 'Controls', 179, 179, nothing)
    cv2.createTrackbar('S Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('S Max', 'Controls', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('V Max', 'Controls', 55, 255, nothing)
    
    # 过滤参数
    cv2.createTrackbar('board_min_area', 'Controls', 5000, 307200, nothing)
    cv2.createTrackbar('board_max_area', 'Controls', 54000, 307200, nothing)
    cv2.createTrackbar('diameter_ratio', 'Controls', 40, 70, nothing)
    
    # 控制参数（仅用于显示/调试，不驱动电机）
    cv2.createTrackbar('yaw_kp', 'Controls', 5, 100, nothing)
    cv2.createTrackbar('pitch_kp', 'Controls', 6, 100, nothing)
    cv2.createTrackbar('cx_offset', 'Controls', 30, 60, nothing)
    cv2.createTrackbar('cy_offset', 'Controls', 30, 60, nothing)
    cv2.createTrackbar('shoot_tol', 'Controls', 14, 200, nothing)
    cv2.createTrackbar('offset_pitch', 'Controls', 11, 20, nothing)
    
    # 开关
    cv2.createTrackbar('show', 'Controls', 1, 1, nothing)  # 默认开启显示


def update_params():
    """读取滑动条并更新算法参数"""
    # HSV
    h_min = cv2.getTrackbarPos('H Min', 'Controls')
    h_max = cv2.getTrackbarPos('H Max', 'Controls')
    s_min = cv2.getTrackbarPos('S Min', 'Controls')
    s_max = cv2.getTrackbarPos('S Max', 'Controls')
    v_min = cv2.getTrackbarPos('V Min', 'Controls')
    v_max = cv2.getTrackbarPos('V Max', 'Controls')
    
    # 过滤
    detector.board_min_area = cv2.getTrackbarPos('board_min_area', 'Controls')
    detector.board_max_area = cv2.getTrackbarPos('board_max_area', 'Controls')
    detector.diameter_ratio = cv2.getTrackbarPos('diameter_ratio', 'Controls') / 100.0
    
    # 控制参数（注入tracker/detector供solve使用）
    detector.board_lower = (h_min, s_min, v_min)
    detector.board_upper = (h_max, s_max, v_max)
    detector.cx_offset = cv2.getTrackbarPos('cx_offset', 'Controls') - 30
    detector.cy_offset = cv2.getTrackbarPos('cy_offset', 'Controls') - 30
    tracker.offset_pitch = cv2.getTrackbarPos('offset_pitch', 'Controls') - 10
    tracker.shoot_tol = cv2.getTrackbarPos('shoot_tol', 'Controls')
    
    # 显示开关
    state.show_img = cv2.getTrackbarPos('show', 'Controls')
    
    # 返回控制参数（本模块暂不使用，但保留接口方便后续扩展）
    return (
        cv2.getTrackbarPos('yaw_kp', 'Controls') / 100.0,
        cv2.getTrackbarPos('pitch_kp', 'Controls') / 100.0
    )


def main():
    init_board()
    
    if not cam.cam.isOpened():
        print("❌ Camera open failed")
        return
    
    print("✅ 上位机调试模式启动 | 按 [1]/[2] 切换任务 | [Q] 退出")
    
    # 可选：保留队列用于后续扩展（当前不使用）
    position_queue = queue.Queue(maxsize=1)
    dt_queue = queue.Queue(maxsize=1)
    
    # 计时初始化
    last_frame_time = time.time()
    last_fps_time = time.time()
    frame_count = 0
    
    try:
        while True:
            # 1. 读帧
            ret, frame = cam.cam.read()
            if not ret:
                print("⚠️ Failed to read frame")
                break
            
            # 2. 更新参数 + 键盘任务切换
            yaw_kp, pitch_kp = update_params()
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('1'):
                state.task = 0
                print("🔹 切换任务: task1")
            elif key == ord('2'):
                state.task = 1
                print("🔹 切换任务: task2")
            
            # 3. 视觉检测（根据任务调用不同逻辑，此处统一用 process_image）
            result, board = detector.process_image(frame)
            
            # 4. 角度解算调试（核心：验证 tracker.solve）
            if board.is_valid:
                solve_result = tracker.solve(board)
                if solve_result:
                    yaw, pitch, dist = solve_result
                    # 控制台实时打印调试信息
                    print(f"\r🎯 Yaw:{yaw:6.2f}° Pitch:{pitch:6.2f}° Dist:{dist:6.1f}cm | "
                          f"Valid:✅ Shoot:{'🔥' if abs(yaw)<tracker.shoot_tol and abs(pitch)<tracker.shoot_tol else '○'}", 
                          end='', flush=True)
                    
                    # 可选：放入队列（供后续扩展）
                    if position_queue.full():
                        position_queue.get()
                    position_queue.put(board)
            
            # 5. 图像显示
            if state.show_img:
                if detector.last_binary is not None:
                    mask_3ch = cv2.cvtColor(detector.last_binary, cv2.COLOR_GRAY2BGR)
                    cv2.imshow('Mask', mask_3ch)
                cv2.imshow('Result', result)
            
            # 6. FPS 计算
            current_time = time.time()
            frame_count += 1
            dt = current_time - last_frame_time
            last_frame_time = current_time
            
            if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                print(f" | FPS: {fps:.1f}", end='')
                frame_count = 0
                last_fps_time = current_time
                
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except Exception as e:
        print(f"❌ 主循环错误: {e}")
    finally:
        # 资源清理
        cam.cam.release()
        cv2.destroyAllWindows()
        print("\n✅ 资源已释放，程序退出")


if __name__ == '__main__':
    main()