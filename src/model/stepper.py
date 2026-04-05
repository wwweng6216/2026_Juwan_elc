import serial
import enum
import time
import struct

# 定义系统参数枚举，对应树莓派例程中的 key
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

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1, motor_id=1):
        """初始化电机控制器并打开串口"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_id = motor_id
        self.serial_port = None
        self._init_serial()

    def _init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            # 清空缓冲区，防止残留数据干扰
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
        except AttributeError:
            raise Exception("串口初始化失败：pyserial 模块可能未正确安装或被覆盖")
        except serial.SerialException as e:
            raise Exception(f"打开串口 {self.port} 失败: {e}")

    def _send_cmd(self, cmd_bytes):
        """发送命令并可选地读取响应"""
        if not self.serial_port or not self.serial_port.is_open:
            raise Exception("串口未打开")
        self.serial_port.write(cmd_bytes)
        # 注意：大多数控制命令不需要立即读取，除非是查询类命令
        # 如果需要调试，可以在这里添加 read

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

    def emm_v5_reset_curpos_to_zero(self, addr=None):
        """将当前位置清零"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x0A, 0x6D, 0x6B]
        cmd = bytes([addr, 0x0A, 0x6D, 0x6B])
        self._send_cmd(cmd)

    def emm_v5_reset_clog_pro(self, addr=None):
        """解除堵转保护"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x0E, 0x52, 0x6B]
        cmd = bytes([addr, 0x0E, 0x52, 0x6B])
        self._send_cmd(cmd)

    def emm_v5_modify_ctrl_mode(self, addr=None, svF=False, ctrl_mode=0):
        """修改开环/闭环控制模式"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x46, 0x69, svF, ctrl_mode, 0x6B]
        cmd = bytes([
            addr, 
            0x46, 
            0x69, 
            0x01 if svF else 0x00, 
            ctrl_mode, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_en_control(self, addr=None, state=False, snF=False):
        """使能信号控制"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0xF3, 0xAB, state, snF, 0x6B]
        cmd = bytes([
            addr, 
            0xF3, 
            0xAB, 
            0x01 if state else 0x00, 
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_vel_control(self, addr=None, dir=0, vel=0, acc=0, snF=False):
        """速度模式控制"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0xF6, dir, vel_h, vel_l, acc, snF, 0x6B]
        cmd = bytes([
            addr, 
            0xF6, 
            dir & 0xFF,
            (vel >> 8) & 0xFF, 
            vel & 0xFF,
            acc & 0xFF,
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_pos_control(self, addr=None, dir=0, vel=0, acc=0, clk=0, raF=False, snF=False):
        """
        位置模式控制
        clk: 脉冲数/位置值 (int32)
        raF: True为绝对位置, False为相对位置 (注意：例程中 raF 对应 bit10，1为绝对/相位标志?)
             根据例程: cmd[10] = 0x01 if raF else 0x00
        """
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0xFD, dir, vel_h, vel_l, acc, clk_24-31, clk_16-23, clk_8-15, clk_0-7, raF, snF, 0x6B]
        cmd = bytes([
            addr, 
            0xFD, 
            dir & 0xFF,
            (vel >> 8) & 0xFF, 
            vel & 0xFF,
            acc & 0xFF,
            (clk >> 24) & 0xFF, 
            (clk >> 16) & 0xFF, 
            (clk >> 8) & 0xFF, 
            clk & 0xFF,
            0x01 if raF else 0x00,
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_stop_now(self, addr=None, snF=False):
        """立即停止"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0xFE, 0x98, snF, 0x6B]
        cmd = bytes([
            addr, 
            0xFE, 
            0x98, 
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_synchronous_motion(self, addr=None):
        """多机同步运动触发"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0xFF, 0x66, 0x6B]
        cmd = bytes([addr, 0xFF, 0x66, 0x6B])
        self._send_cmd(cmd)

    def emm_v5_origin_set_o(self, addr=None, svF=False):
        """设置单圈回零的零点位置"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x93, 0x88, svF, 0x6B]
        cmd = bytes([
            addr, 
            0x93, 
            0x88, 
            0x01 if svF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_origin_modify_params(self, addr=None, svF=False, o_mode=0, o_dir=0, o_vel=0, o_tm=0, sl_vel=0, sl_ma=0, sl_ms=0, potF=False):
        """修改回零参数"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x4C, 0xAE, svF, o_mode, o_dir, o_vel_h, o_vel_l, o_tm_24..0, sl_vel_h, sl_vel_l, sl_ma_h, sl_ma_l, sl_ms_h, sl_ms_l, potF, 0x6B]
        cmd = bytes([
            addr, 
            0x4C, 
            0xAE, 
            0x01 if svF else 0x00,
            o_mode & 0xFF,
            o_dir & 0xFF,
            (o_vel >> 8) & 0xFF, o_vel & 0xFF,
            (o_tm >> 24) & 0xFF, (o_tm >> 16) & 0xFF, (o_tm >> 8) & 0xFF, o_tm & 0xFF,
            (sl_vel >> 8) & 0xFF, sl_vel & 0xFF,
            (sl_ma >> 8) & 0xFF, sl_ma & 0xFF,
            (sl_ms >> 8) & 0xFF, sl_ms & 0xFF,
            0x01 if potF else 0x00,
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_origin_trigger_return(self, addr=None, o_mode=0, snF=False):
        """触发回零"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x9A, o_mode, snF, 0x6B]
        cmd = bytes([
            addr, 
            0x9A, 
            o_mode & 0xFF,
            0x01 if snF else 0x00, 
            0x6B
        ])
        self._send_cmd(cmd)

    def emm_v5_origin_interrupt(self, addr=None):
        """强制中断并退出回零"""
        addr = self.motor_id if addr is None else addr
        # Cmd: [Addr, 0x9C, 0x48, 0x6B]
        cmd = bytes([addr, 0x9C, 0x48, 0x6B])
        self._send_cmd(cmd)

    # 比赛就用这个控制云台
    def emm_v5_move_to_angle(self, addr=None, angle_deg=0.0, vel_rpm=0, acc=0, abs_mode=False):
        """
        按角度移动电机
        注意：这里的角度转换公式需要根据你的电机细分和编码器分辨率确认。
        树莓派例程中：Motor_Cur_Pos = float(pos) * 360.0 / 65536.0
        这意味着 65536 个单位 = 360 度。
        所以 1 度 = 65536 / 360 ≈ 182.044 单位。
        
        但在 emm_v5_pos_control 中，clk 是 int32。
        如果驱动器内部逻辑是：clk 单位直接对应编码器计数，那么：
        clk = angle_deg * (65536 / 360.0)
        """
        addr = self.motor_id if addr is None else addr
        
        # 确定方向
        dir = 0
        if angle_deg < 0:
            dir = 1
            angle_deg = -angle_deg
            
        # 计算脉冲数/位置值
        # 假设分辨率是 65536 per circle (根据树莓派解码逻辑推断)
        # 如果之前的旧代码用的是 51200，请确认硬件手册。这里暂按树莓派例程的 65536 计算
        resolution = 65536.0
        clk = int(angle_deg * resolution / 360.0)
        
        # raF: True for Absolute, False for Relative
        self.emm_v5_pos_control(addr, dir, vel_rpm, acc, clk, raF=abs_mode, snF=False)

    def get_current_position_angle(self, addr=None):
        """
        读取当前电机位置并转换为角度
        返回: 角度 (float)
        """
        addr = self.motor_id if addr is None else addr
        data = self.emm_v5_read_sys_params(addr, SysParams.S_CPOS)
        
        if len(data) < 7:
            return None
            
        # 解析数据
        # 假设返回格式: [Addr, FuncCode(0x36), Direction, Pos_24-31, Pos_16-23, Pos_8-15, Pos_0-7, ...]
        # 树莓派例程中:
        # pos1 = ustruct.unpack('>I', bytes.fromhex(''.join(data1_hex[3:7])))[0]
        # data1_hex[0] is Addr, [1] is Func, [2] is Dir?, [3:7] is Pos
        
        if data[0] != addr or data[1] != 0x36:
            return None
            
        direction_byte = data[2]
        # 提取位置整数 (Big Endian)
        pos_int = struct.unpack('>I', data[3:7])[0]
        
        # 转换角度
        angle = float(pos_int) * 360.0 / 65536.0
        
        # 处理方向 (如果方向位非0，则为负? 需根据实际硬件确认，通常方向由速度/dir决定，位置可能是有符号或无符号)
        # 树莓派例程: if int(data1_hex[2], 16): Motor_Cur_Pos1 = -Motor_Cur_Pos1
        if direction_byte != 0:
            angle = -angle
            
        return angle

    def close(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

def test_motor_control(motor: MotorController, addr=None):
    """测试电机控制功能"""
    addr = motor.motor_id if addr is None else addr
    try:
        print("1. Reset Current Position to Zero...")
        motor.emm_v5_reset_curpos_to_zero(addr)
        time.sleep(0.1)
        
        print("2. Set Control Mode (Closed Loop, Save)...")
        # ctrl_mode: 0=Open Loop, 1=Current, 2=Speed, 3=Position? 
        # 需查阅具体手册，这里假设 2 或 3 是位置/速度模式。
        # 树莓派例程没显式改模式，默认可能是上次保存的。
        # 这里我们尝试设置为位置模式并存储 (svF=True)
        motor.emm_v5_modify_ctrl_mode(addr, svF=True, ctrl_mode=3) 
        time.sleep(0.1)
        
        print("3. Enable Motor...")
        motor.emm_v5_en_control(addr, state=True, snF=False)
        time.sleep(0.5) # 等待使能稳定
        
        print("4. Move to 90 degrees (Relative)...")
        motor.emm_v5_move_to_angle(addr, angle_deg=90.0, vel_rpm=500, acc=100, abs_mode=False)
        time.sleep(2) # 等待运动完成
        
        print("5. Read Current Position...")
        pos = motor.get_current_position_angle(addr)
        print(f"   Current Angle: {pos:.2f} deg")
        
        print("6. Stop Now...")
        motor.emm_v5_stop_now(addr, snF=False)
        time.sleep(0.1)
        
        print("7. Disable Motor...")
        motor.emm_v5_en_control(addr, state=False, snF=False)
        
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        motor.close()

if __name__ == "__main__":
    # 请根据你的实际端口修改，Windows 下可能是 'COM3', Linux 下 '/dev/ttyUSB0'
    PORT = 'COM11'  # <--- 修改这里
    motor = MotorController(port=PORT, motor_id=1)
    try:
        test_motor_control(motor)
    except Exception as e:
        print(f"Init Error: {e}")
        motor.close()