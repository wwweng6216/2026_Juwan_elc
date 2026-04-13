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
        #保留参数
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_id = motor_id
        #初始化
        self.serial_port = None
        #打开串口
        self._init_serial()

    def _init_serial(self):
        '''打开串口的操作'''
        #用try有利于检查错误
        try:
            '''创建串口对象'''
            self.serial_port = serial.Serial(
                port = self.port,
                baudrate = self.baudrate,
                timeout = self.timeout,
                bytesize=serial.EIGHTBITS,#八位数据
                parity=serial.PARITY_NONE,#无校验位
                stopbits=serial.STOPBITS_ONE # 1位停止位
            )
            # 清空缓冲区，防止残留数据干扰
            if self.serial_port.is_open:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()

        except AttributeError:
            raise Exception("串口初始化失败：pyserial 模块可能未正确安装或被覆盖")
        except serial.SerialException as e:
            raise Exception(f"打开串口 {self.port} 失败: {e}")
        
    #把字节流扔进串口
    def _send_cmd(self, cmd_bytes):
        #检查串口有没有打开
        if not self.serial_port or not self.serial_port.is_open:
            raise Exception("串口未打开")
        #发送数据
        self.serial_port.write(cmd_bytes)

    '''实现核心通信协议''' 
    def emm_v5_read_sys_params(self, addr=None, s: SysParams = None):
        """
        读取系统参数
        参考树莓派例程 Emm_V5_Read_Sys_Params
        """
        addr = self.motor_id if addr is None else addr
        cmd = bytearray()
        cmd.append(addr)
        
        # 功能码映射
        func_codes = {
            SysParams.S_VER: 0x1F,
            SysParams.S_RL: 0x20,
            SysParams.S_PID: 0x21,
            SysParams.S_VBUS: 0x24,
            SysParams.S_CPHA: 0x27,
            SysParams.S_ENCL: 0x31,
            SysParams.S_TPOS: 0x33,
            SysParams.S_VEL: 0x35,
            SysParams.S_CPOS: 0x36,
            SysParams.S_PERR: 0x37,
            SysParams.S_FLAG: 0x3A,
            SysParams.S_ORG: 0x3B,
            SysParams.S_Conf: 0x42,
            SysParams.S_State: 0x43
        }
        
        if s not in func_codes:
            raise ValueError(f"未知的参数类型: {s}")
            
        cmd.append(func_codes[s])
        
        # 特殊参数需要辅助码
        if s == SysParams.S_Conf:
            cmd.append(0x6C)
        elif s == SysParams.S_State:
            cmd.append(0x7A)
            
        cmd.append(0x6B) # 校验字节
        
        self._send_cmd(bytes(cmd))
        
        # 读取响应 (根据参数不同，返回长度可能不同，这里尝试读取一定长度)
        # 注意：实际项目中需要根据具体协议文档确定返回字节数
        time.sleep(0.01) # 等待驱动器处理
        return self.serial_port.read(32)
    
    def emm_v5_reset_curpos_to_zero(self, addr = None)
        '''位置清零'''
        addr = self.motor_id if addr is None else addr
        cmd = bytes([
            addr,
            0x0A, 
            0x6D,
            0x6B
        ])
        self._send_cmd(cmd)
    
    def emm_v5_reset_clog_pro(self, addr=None):
        '''解除堵转保护'''
        addr = self.motor_id if addr is None else addr

        #组装命令
        cmd = bytes([
            addr,   # 地址
            0x0E,   # 命令字
            0x52,   # 数据位
            0x6B    # 校验/结束符
        ])
        self._send_cmd(cmd)

    def emm_v5_stop_now(self, addr=None, snF=False):
        '''立刻停止'''
        addr = self.motor_id if addr is None else addr

        cmd = bytes([
            addr,
            0xFE,
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_en_control(self, addr=None):
        '''使能/去使能'''
        addr = self.motor_id if addr is None else addr

