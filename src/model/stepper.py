import serial
import enum
import time
import struct

# 定义系统参数枚举，对应树莓派例程中的 key
'''
枚举是为了让代码可读性更强
S_VER 对应 0x1F 是因为协议规定读取版本号的命令字是 0x1F
'''
class SysParams(enum.Enum):
    S_VER = 'S_VER'   # 0x1F
    S_RL = 'S_RL'     # 0x20
    S_PID = 'S_PID'   # 0x21
    S_VBUS = 'S_VBUS' # 0x24
    S_CPHA = 'S_CPHA' # 0x27
    S_ENCL = 'S_ENCL' # 0x31
    S_TPOS = 'S_TPOS' # 0x33
    S_VEL = 'S_VEL'   # 0x35
    S_CPOS = 'S_CPOS' # 0x36
    S_PERR = 'S_PERR' # 0x37
    S_FLAG = 'S_FLAG' # 0x3A
    S_ORG = 'S_ORG'   # 0x3B
    S_Conf = 'S_Conf' # 0x42 + 0x6C
    S_State = 'S_State' # 0x43 + 0x7A

class EmmMotor:
            '''参数:
            port : 串口端口号
            baudrate : 波特率
            timeout : 读取超时时间(秒)
            motor_id : 电机地址ID
            '''
    def __init__(self, port ='COM', baudrate = 115200, timeout = 1, motor_id = 1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_id = motor_id