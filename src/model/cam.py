import cv2

class Camera:
    def __init__(self, index = 4):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 获取实际生效的画面宽度与高度
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def read(self):
        return self.cap.read()
    
if __name__ == '__main__':
    cam = Camera()
    print(f"当前分辨率: {cam.width} × {cam.height}")

    while True:
        ret, frame = cam.read()  # 在循环内不断读取新帧
        if ret:
            cv2.imshow('Camera View', frame)
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("获取画面失败，请检查摄像头连接。")
            break
    
    # 释放资源
    cam.cap.release()
    cv2.destroyAllWindows()