import Hobot.GPIO as GPIO
import time


class GPIN :

    def __init__(self, pin = 1, mode = 1):
        # 保存参数
        self.pin = pin
        self.mode = mode

        # 初始化
        self.times = 0
        self.status = GPIO.LOW # 当前电平

        # 配置硬件
        GPIO.setmode(GPIO.BOard)# 设置引脚编号模式为物理引脚号
        GPIO.setwarnings(False)#关闭 GPIO 库的警告信息

        #模式判断
        if self.mode == 1:
            GPIO.setup(self.pin, GPIO.OUT)#规定输出还是输入：GPIO.OUT(输出),GPIO.IN(输入)
            GPIO.output(self.pin, GPIO.LOW)#仅对输出模式有效:GPIO.HIGH (高电平),GPIO.LOW (低电平)

        elif self.mode == 0:
            GPIO.setup(self.pin, GPIO.IN)

    def 