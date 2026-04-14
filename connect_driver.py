from pymodbus.client import ModbusSerialClient as ModbusClient    # Thư viện Modbus để giao tiếp với thiết bị qua giao thức Modbus
import numpy as np
import time
import threading
import controller_motor
from config import AGVConfig
from config_2 import AGVConfig_2
import math


# driver_motor = {"ket_noi": False, "message": "Disconnect", "COM": data_setting["cong_driver"][0], "baudrate": data_setting["cong_driver"][1]}
com = AGVConfig.driver_motor["COM"]
bause = AGVConfig.driver_motor["baudrate"]


print(com, bause, AGVConfig_2.distance_max)
van_toc_text = ""



class sent_data_driver:
    def __init__(self):
        self.com = com
        self.bause = bause
            
        self.da_ngat_dong_co = False
        self.thiet_lap_ket_noi = False
        self.chay_thread_gui_tin_hieu_driver = False

        self.van_toc_gui_driver_trai = 0
        self.van_toc_gui_driver_phai = 0
        
        self.quay_trai = 0
        self.quay_phai = 0

        self.v_tien_max = 0
        self.v_re_max = 0
        self.toa_do_diem_dau = []
        self.toa_do_diem_dich = []
        self.toa_do_hien_tai = []
        self.toa_do_diem_huong = []
        self.angle = 0 
        self.distance = 0
        self.check_angle_distance = 0
        self.stop = 0
        self.di_cham = 0
        self.a_v = 800
        self.dang_re = 0
        self.tien_rl = 0
        self.di_thuan_nguoc = 0
        self.xac_dinh_vi_tri_xe = 0
        self.di_chuyen_luon = {"update": 0, 
                                "co_huong": 0, 
                                "khong_huong": 0, 
                                "van_toc_di_chuyen_luon": None, 
                                "van_toc_min": None, 
                                "khoang_nhin_phia_truoc": None, 
                                "delta": None, 
                                "khong_xoay": None}
        self.distance_dich_ly_tuong = None
        self.toa_do_dich_ly_tuong = None

        self.check_time = time.time()
        self.time_stop_dk_tay = 0
        self.da_reset_5s = False

        self.unit_id = 1  # Đặt ID thiết bị Modbus
        self.ID = 1  # Đặt ID của thiết bị Modbus
        ######################
        ## Register Address ##
        ######################
        ## Common
        self.CONTROL_REG = 0x200E  # Thanh ghi điều khiển
        self.OPR_MODE = 0x200D  # Thanh ghi chế độ hoạt động
        self.L_ACL_TIME = 0x2080  # Thời gian tăng tốc bánh trái
        self.R_ACL_TIME = 0x2081  # Thời gian tăng tốc bánh phải
        self.L_DCL_TIME = 0x2082  # Thời gian giảm tốc bánh trái
        self.R_DCL_TIME = 0x2083  # Thời gian giảm tốc bánh phải

        ## Velocity control
        self.L_CMD_RPM = 0x2088  # Tốc độ đặt cho bánh trái (RPM)
        self.R_CMD_RPM = 0x2089  # Tốc độ đặt cho bánh phải (RPM)
        self.L_FB_RPM = 0x20AB  # Tốc độ phản hồi bánh trái (RPM)
        self.R_FB_RPM = 0x20AC  # Tốc độ phản hồi bánh phải (RPM)

        ## Position control
        self.POS_CONTROL_TYPE = 0x200F  # Loại điều khiển vị trí

        self.L_MAX_RPM_POS = 0x208E  # Tốc độ tối đa khi điều khiển vị trí bánh trái
        self.R_MAX_RPM_POS = 0x208F  # Tốc độ tối đa khi điều khiển vị trí bánh phải

        self.L_CMD_REL_POS_HI = 0x208A  # Thanh ghi vị trí tương đối cao của bánh trái
        self.L_CMD_REL_POS_LO = 0x208B  # Thanh ghi vị trí tương đối thấp của bánh trái
        self.R_CMD_REL_POS_HI = 0x208C  # Thanh ghi vị trí tương đối cao của bánh phải
        self.R_CMD_REL_POS_LO = 0x208D  # Thanh ghi vị trí tương đối thấp của bánh phải

        self.L_FB_POS_HI = 0x20A7  # Thanh ghi vị trí phản hồi cao của bánh trái
        self.L_FB_POS_LO = 0x20A8  # Thanh ghi vị trí phản hồi thấp của bánh trái
        self.R_FB_POS_HI = 0x20A9  # Thanh ghi vị trí phản hồi cao của bánh phải
        self.R_FB_POS_LO = 0x20AA  # Thanh ghi vị trí phản hồi thấp của bánh phải

        ## Troubleshooting
        self.L_FAULT = 0x20A5  # Mã lỗi bánh trái
        self.R_FAULT = 0x20A6  # Mã lỗi bánh phải

        ########################
        ## Control CMDs (REG) ##
        ########################
        self.EMER_STOP = 0x05  # Lệnh dừng khẩn cấp
        self.ALRM_CLR = 0x06  # Lệnh xóa báo động
        self.DOWN_TIME = 0x07  # Lệnh tắt động cơ
        self.ENABLE = 0x08  # Lệnh bật động cơ
        self.POS_SYNC = 0x10  # Lệnh đồng bộ vị trí
        self.POS_L_START = 0x11  # Lệnh khởi động bánh trái
        self.POS_R_START = 0x12  # Lệnh khởi động bánh phải

        ####################
        ## Operation Mode ##
        ####################
        self.POS_REL_CONTROL = 1  # Chế độ điều khiển vị trí tương đối
        self.POS_ABS_CONTROL = 2  # Chế độ điều khiển vị trí tuyệt đối
        self.VEL_CONTROL = 3  # Chế độ điều khiển vận tốc

        self.ASYNC = 0  # Chế độ không đồng bộ
        self.SYNC = 1  # Chế độ đồng bộ

        #################
        ## Fault codes ##
        #################
        self.NO_FAULT = 0x0000  # Không có lỗi
        self.OVER_VOLT = 0x0001  # Lỗi quá áp
        self.UNDER_VOLT = 0x0002  # Lỗi dưới áp
        self.OVER_CURR = 0x0004  # Lỗi quá dòng
        self.OVER_LOAD = 0x0008  # Lỗi quá tải
        self.CURR_OUT_TOL = 0x0010  # Lỗi dòng điện ngoài ngưỡng
        self.ENCOD_OUT_TOL = 0x0020  # Lỗi encoder ngoài ngưỡng
        self.MOTOR_BAD = 0x0040  # Lỗi động cơ
        self.REF_VOLT_ERROR = 0x0080  # Lỗi điện áp tham chiếu
        self.EEPROM_ERROR = 0x0100  # Lỗi EEPROM
        self.WALL_ERROR = 0x0200  # Lỗi tường
        self.HIGH_TEMP = 0x0400  # Lỗi nhiệt độ cao
        self.FAULT_LIST = [self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR, self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL, \
                    self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR, self.WALL_ERROR, self.HIGH_TEMP]  # Danh sách lỗi

        ##############
        ## Odometry ##
        ##############
        ## 8 inches wheel
        self.travel_in_one_rev = 0.655  # Quãng đường đi được trong một vòng quay (mét)
        self.cpr = 16385  # Số xung trên mỗi vòng quay
        self.R_Wheel = 0.105  # Bán kính bánh xe (mét)


        # Khởi tạo đối tượng điều khiển
        if AGVConfig.thiet_lap_ket_noi["driver_motor"] == "on":
            self.client = ModbusClient(port=self.com, baudrate=self.bause, timeout=2, parity='N', stopbits=1, bytesize=8)
            self.disable_motor()

        

    ## Một số trường hợp đọc ngay sau khi ghi có thể gây lỗi ModbusIOException khi lấy dữ liệu từ thanh ghi
    def modbus_fail_read_handler(self, ADDR, WORD):
        # Hàm xử lý lỗi khi đọc thanh ghi Modbus
        read_success = False
        reg = [None]*WORD  # Tạo danh sách rỗng để lưu giá trị thanh ghi
        while not read_success:
            result = self.client.read_holding_registers(address=ADDR, count=WORD, slave=self.ID)  # Đọc thanh ghi
            try:
                for i in range(WORD):
                    reg[i] = result.registers[i]  # Lưu giá trị thanh ghi vào danh sách
                read_success = True  # Đọc thành công
            except AttributeError as e:
                print(e)  # In lỗi nếu xảy ra
                pass

        return reg  # Trả về danh sách giá trị thanh ghi

    def rpm_to_radPerSec(self, rpm):
        # Chuyển đổi tốc độ từ RPM sang radian/giây
        return rpm*2*np.pi/60.0

    def rpm_to_linear(self, rpm):
        # Chuyển đổi tốc độ từ RPM sang vận tốc tuyến tính
        W_Wheel = self.rpm_to_radPerSec(rpm)  # Tốc độ góc bánh xe
        V = W_Wheel*self.R_Wheel  # Vận tốc tuyến tính
        return V

    def set_mode(self, MODE):
        # Thiết lập chế độ hoạt động
        if MODE == 1:
            print("Set relative position control")  # Chế độ điều khiển vị trí tương đối
        elif MODE == 2:
            print("Set absolute position control")  # Chế độ điều khiển vị trí tuyệt đối
        elif MODE == 3:
            print("Set speed rpm control")  # Chế độ điều khiển vận tốc
        else:
            print("set_mode ERROR: set only 1, 2, or 3")  # Báo lỗi nếu chế độ không hợp lệ
            return 0

        try:
            result = self.client.write_register(self.OPR_MODE, MODE, slave=self.ID)  # Ghi chế độ vào thanh ghi
            return result
        except Exception as e:
            print(f"Error setting mode: {e}")
            return None

    def get_mode(self):
        # Lấy chế độ hoạt động hiện tại
        registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)  # Đọc thanh ghi chế độ
        mode = registers[0]  # Lấy giá trị chế độ
        return mode

    def enable_motor(self):
        """
        Bật động cơ.
        """
        try:
            result = self.client.write_register(self.CONTROL_REG, self.ENABLE, slave=self.ID)
            if result.isError():
                print("Failed to enable motor.")
                return None
            else:
                print("Motor enabled successfully.")
                return result
        except Exception as e:
            print(f"Error enabling motor: {e}")
            return None
    def disable_motor(self):
        # Tắt động cơ
        try:
            self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, slave=self.ID)
        except Exception as e:
            print(f"Error disabling motor: {e}")
        # pass

    def get_fault_code(self):
        """
        Lấy mã lỗi của động cơ.
        """
        try:
            # Đọc mã lỗi từ thanh ghi
            fault_codes = self.client.read_holding_registers(address=self.L_FAULT, count=2, slave=self.unit_id)
            L_fault_code = fault_codes.registers[0]  # Mã lỗi bánh trái
            R_fault_code = fault_codes.registers[1]  # Mã lỗi bánh phải

            # Kiểm tra lỗi
            L_fault_flag = L_fault_code in self.FAULT_LIST
            R_fault_flag = R_fault_code in self.FAULT_LIST

            return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)
        except Exception as e:
            print(f"Error getting fault code: {e}")
            return (False, 0), (False, 0)

    def clear_alarm(self):
        # Xóa báo động
        try:
            result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, slave=self.ID)
        except Exception as e:
            print(f"Error clearing alarm: {e}")

    def set_accel_time(self, L_ms, R_ms):
        # Thiết lập thời gian tăng tốc cho bánh trái và bánh phải
        if L_ms > 32767:
            L_ms = 32767  # Giới hạn giá trị tối đa
        elif L_ms < 0:
            L_ms = 0  # Giới hạn giá trị tối thiểu

        if R_ms > 32767:
            R_ms = 32767
        elif R_ms < 0:
            R_ms = 0

        try:
            result = self.client.write_registers(self.L_ACL_TIME, [int(L_ms), int(R_ms)], slave=self.ID)  # Ghi giá trị vào thanh ghi
        except Exception as e:
            print(f"Error setting accel time: {e}")

    def set_decel_time(self, L_ms, R_ms):

        if L_ms > 32767:
            L_ms = 32767
        elif L_ms < 0:
            L_ms = 0

        if R_ms > 32767:
            R_ms = 32767
        elif R_ms < 0:
            R_ms = 0

        try:
            result = self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)], slave=self.ID)
        except Exception as e:
            print(f"Error setting decel time: {e}")

    def int16Dec_to_int16Hex(self,int16):

        lo_byte = (int16 & 0x00FF)
        hi_byte = (int16 & 0xFF00) >> 8

        all_bytes = (hi_byte << 8) | lo_byte

        return all_bytes


    def set_rpm(self, L_rpm, R_rpm):
        L_rpm = int(L_rpm/100)
        R_rpm = -int(R_rpm/100)
        # print("L_rpm",L_rpm)
        # print("R_rpm",R_rpm)

        if L_rpm > 3000:
            L_rpm = 3000
        elif L_rpm < -3000:
            L_rpm = -3000

        if R_rpm > 3000:
            R_rpm = 3000
        elif R_rpm < -3000:
            R_rpm = -3000

        left_bytes = self.int16Dec_to_int16Hex(L_rpm)
        right_bytes = self.int16Dec_to_int16Hex(R_rpm)

        try:
            result = self.client.write_registers(self.L_CMD_RPM, [right_bytes, left_bytes], slave=self.unit_id)
        except Exception as e:
            print(f"Error setting rpm: {e}")

    def get_rpm(self):
        registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
        return registers

    def get_linear_velocities(self):
        rpmL, rpmR = self.get_rpm()
        VL = self.rpm_to_linear(rpmL)
        VR = self.rpm_to_linear(-rpmR)
        return VL, VR

    def map(self, val, in_min, in_max, out_min, out_max):
            return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def set_maxRPM_pos(self, max_L_rpm, max_R_rpm):

        if max_L_rpm > 1000:
            max_L_rpm = 1000
        elif max_L_rpm < 1:
            max_L_rpm = 1

        if max_R_rpm > 1000:
            max_R_rpm = 1000
        elif max_R_rpm < 1:
            max_R_rpm = 1

        try:
            result = self.client.write_registers(self.L_MAX_RPM_POS, [int(max_L_rpm), int(max_R_rpm)], slave=self.ID)
        except Exception as e:
            print(f"Error setting max RPM pos: {e}")

    def set_position_async_control(self):
        try:
            result = self.client.write_register(self.POS_CONTROL_TYPE, self.ASYNC, slave=self.ID)
        except Exception as e:
            print(f"Error setting position async control: {e}")

    def move_left_wheel(self):
        try:
            result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, slave=self.ID)
        except Exception as e:
            print(f"Error moving left wheel: {e}")

    def move_right_wheel(self):
        try:
            result = self.client.write_register(self.CONTROL_REG, self.POS_R_START, slave=self.ID)
        except Exception as e:
            print(f"Error moving right wheel: {e}")

    def deg_to_32bitArray(self, deg):
        dec = int(self.map(deg, -1440, 1440, -65536, 65536))
        HI_WORD = (dec & 0xFFFF0000) >> 16
        LO_WORD = dec & 0x0000FFFF
        return [HI_WORD, LO_WORD]

    def set_relative_angle(self, ang_L, ang_R):
        L_array = self.deg_to_32bitArray(ang_L)
        R_array = self.deg_to_32bitArray(ang_R)
        all_cmds_array = L_array + R_array

        try:
            result = self.client.write_registers(self.L_CMD_REL_POS_HI, all_cmds_array, slave=self.ID)
        except Exception as e:
            print(f"Error setting relative angle: {e}")

    def get_wheels_travelled(self):
        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]

        l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
        l_travelled = (float(l_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter
        r_travelled = (float(r_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter

        return l_travelled, r_travelled

    def get_wheels_tick(self):

        registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]

        l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

        return l_tick, r_tick

################################################################################

    def disconnect(self):
        self.da_ngat_dong_co = True # có thể ngắt cả vòng lặp ở thread
        self.thiet_lap_ket_noi = False
        self.clear_alarm()
        self.disable_motor()
        self.chay_thread_gui_tin_hieu_driver = False
        print("--------- tắt kết nối driver motor ---------")

    def load_data_sent_drive(self, data):
        # v_tien_max,  v_re_max, 
        #                      toa_do_diem_dau, toa_do_diem_dich, toa_do_hien_tai, diem_huong,
        #                      angle, distance, check_angle_distance, 
        #                      stop = 0, di_cham = 0, a_v = 800, dang_re = 0, tien_rl = 200, di_thuan_nguoc = 0, xac_dinh_vi_tri_xe = 0, di_chuyen_luon = None,
        #                      distance_dich_ly_tuong= None, toa_do_dich_ly_tuong = None
        if data is not None:
            self.v_tien_max = data["v_tien_max"]
            self.v_re_max = data["v_re_max"]

            if AGVConfig_2.loi_an_toan == "vung_3":
                self.v_tien_max = min(self.v_tien_max, AGVConfig_2.van_toc_an_toan["vung_3"]["van_toc_tien"])
                self.v_re_max = min(self.v_re_max, AGVConfig_2.van_toc_an_toan["vung_3"]["van_toc_re"])
            if AGVConfig_2.loi_an_toan == "vung_2":
                self.v_tien_max = min(self.v_tien_max, AGVConfig_2.van_toc_an_toan["vung_2"]["van_toc_tien"])
                self.v_re_max = min(self.v_re_max, AGVConfig_2.van_toc_an_toan["vung_2"]["van_toc_re"])
            if AGVConfig_2.loi_an_toan == "vung_1":
                self.v_tien_max = AGVConfig_2.van_toc_an_toan["vung_1"]["van_toc_tien"]
                self.v_re_max = AGVConfig_2.van_toc_an_toan["vung_1"]["van_toc_re"]

            self.toa_do_diem_dau = data["toa_do_diem_dau"]
            self.toa_do_diem_dich = data["toa_do_diem_dich"]
            self.toa_do_diem_huong = data["toa_do_diem_huong"]
            self.distance = data["distance"]
            self.angle = data["angle"]
            self.check_angle_distance = data["check_angle_distance"]
            self.stop = data["stop"]
            self.di_cham = data["di_cham"]
            self.a_v = data["a_v"]
            self.dang_re = data["dang_re"]
            self.tien_rl = data["tien_rl"]
            self.di_thuan_nguoc = data["di_thuan_nguoc"]
            self.xac_dinh_vi_tri_xe = data["xac_dinh_vi_tri_xe"]
            if data["di_chuyen_luon"] != None:
                self.di_chuyen_luon = data["di_chuyen_luon"]
            self.distance_dich_ly_tuong = data["distance_dich_ly_tuong"] # kiểm tra lại sau
            self.toa_do_dich_ly_tuong = data["toa_do_dich_ly_tuong"]
        else:
            self.stop = 1

        
        if self.thiet_lap_ket_noi == True:
            data_fb = self.get_rpm()
            if data_fb[0] > 10000:
                AGVConfig_2.van_toc_phan_hoi_trai = - int(data_fb[0] - 65536) * 10
            else:
                AGVConfig_2.van_toc_phan_hoi_trai = - int(data_fb[0]) * 10

            if data_fb[1] > 10000:
                AGVConfig_2.van_toc_phan_hoi_phai = int(data_fb[1] - 65536) * 10
            else:
                AGVConfig_2.van_toc_phan_hoi_phai = int(data_fb[1]) * 10


    # dùng cho điều khiển bàn phím
    def sent_data_controller(self, vt_trai = 500,vt_phai = 500):
        if self.thiet_lap_ket_noi == True:
            self.van_toc_gui_driver_phai = vt_phai
            self.van_toc_gui_driver_trai = vt_trai

            data_fb = self.get_rpm()
            if data_fb[0] > 10000:
                AGVConfig_2.van_toc_phan_hoi_trai = - int(data_fb[0] - 65536) * 10
            else:
                AGVConfig_2.van_toc_phan_hoi_trai = - int(data_fb[0]) * 10
            if data_fb[1] > 10000:
                AGVConfig_2.van_toc_phan_hoi_phai = int(data_fb[1] - 65536) * 10
            else:
                AGVConfig_2.van_toc_phan_hoi_phai = int(data_fb[1]) * 10
        
    def setup_driver_motor(self):
        if self.da_ngat_dong_co == False:
            self.stop = 1
            self.clear_alarm()
            self.disable_motor()

            self.set_accel_time(300,300)
            self.set_decel_time(200,200)

            self.set_mode(3)
            self.enable_motor()
            lf, rf = self.get_fault_code()
            print("Left fault:", lf)
            print("Right fault:", rf)
            self.move_left_wheel()
            self.move_right_wheel()

    # def sent_data_controller(self, vt_trai = 500,vt_phai = 500):
        
    #     if self.on_setup == 1:
    #         self.vt_trai_sent = vt_trai
    #         self.vt_phai_sent = vt_phai

    #         data_fb = self.get_rpm()
    #         # print("ggggggggg", data_fb)
    #         if data_fb[0] > 10000:
    #             self.vt_trai = - int(data_fb[0] - 65536)
    #         else:
    #             self.vt_trai = - int(data_fb[0])
    #         if data_fb[1] > 10000:
    #             self.vt_phai = int(data_fb[1] - 65536)
    #         else:
    #             self.vt_phai = int(data_fb[1])
    def ket_noi_lai(self):
        # "thiet_lap_ket_noi": {"lidar": "off", "driver_motor": "off", "esp32": "off", "pin": "off", "process_lidar": "on"},
        # print("connect_driver",AGVConfig.thiet_lap_ket_noi["driver_motor"] == "on", AGVConfig.thiet_lap_ket_noi["driver_motor"])
        if AGVConfig.thiet_lap_ket_noi["driver_motor"] == "on":
            if time.time() - self.check_time > 2 or AGVConfig.motor_state == False or AGVConfig.tat_phan_mem == True:
                if self.da_ngat_dong_co == False:
                    print("------------------ disconnect driver motor -------------------") 
                    self.disconnect() # đã có cả tắt vòng lặp thread_sent_data_driver, nếu chương trình lỗi động cơ sẽ dừng ở vận tốc 0
            else:
                if AGVConfig.motor_state == True:
                    self.da_ngat_dong_co = False
                    if self.thiet_lap_ket_noi == False:
                        self.setup_driver_motor()
                        self.thiet_lap_ket_noi = True
                    if self.chay_thread_gui_tin_hieu_driver == False:
                        self.chay_thread_gui_tin_hieu_driver = True
                        threading.Thread(target=self.thread_sent_data_driver).start()
                        print("bật giao tiếp")
                    
                    if self.chay_thread_gui_tin_hieu_driver == True:
                        delta_time = time.time() - AGVConfig_2.time_thread_driver_feedback
                        if delta_time > 2:
                            self.set_rpm(0, 0)
                            print("lỗi luồng thread driver motor")

                    


    


    def thread_sent_data_driver(self):
        while self.da_ngat_dong_co == False:
            AGVConfig_2.time_thread_driver_feedback = time.time()

            # nhấn reset 5s để loại bỏ alarm của driver
            if AGVConfig_2.reset_5s == True and self.da_reset_5s == False:
                self.da_reset_5s = True
                self.clear_alarm()
            if AGVConfig_2.reset_5s == False:
                self.da_reset_5s = False

            # kiểm tra kết nối với chương trình chính nếu chương trình chính lỗi vận tốc = 0    
            if time.time() - self.check_time > 2 or AGVConfig.motor_state == False or AGVConfig.tat_phan_mem == 1:
                self.set_rpm(0, 0)

            else:
                # dieu_khien_agv = {"dieu_khien_thu_cong": False, "tien": 0, "lui": 0, "trai": 0, "phai": 0}
                if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False:
                    v_tien_max0,  v_re_max = [self.v_tien_max, self.v_re_max]
                    # print(f"v_tien_max0: {v_tien_max0}, v_re_max: {v_re_max}")
                    toa_do_diem_dau, toa_do_diem_dich, toa_do_diem_huong = [self.toa_do_diem_dau, self.toa_do_diem_dich, self.toa_do_diem_huong]
                    # print(f"toa_do_diem_dau: {toa_do_diem_dau}, toa_do_diem_dich: {toa_do_diem_dich}, toa_do_diem_huong: {toa_do_diem_huong}")
                    angle, distance, check_angle_distance = [self.angle, self.distance, self.check_angle_distance]
                    # print(f"angle: {angle}, distance: {distance}, check_angle_distance: {check_angle_distance}")
                    stop, di_cham, a_v, dang_re, tien_rl = [self.stop, self.di_cham, self.a_v, self.dang_re, self.tien_rl]
                    # print(f"stop: {stop}, di_cham: {di_cham}, a_v: {a_v}, dang_re: {dang_re}, tien_rl: {tien_rl}")
                    di_thuan_nguoc, xac_dinh_vi_tri_xe, di_chuyen_luon = [self.di_thuan_nguoc, self.xac_dinh_vi_tri_xe, self.di_chuyen_luon]
                    # print(f"di_thuan_nguoc: {di_thuan_nguoc}, xac_dinh_vi_tri_xe: {xac_dinh_vi_tri_xe}, di_chuyen_luon: {di_chuyen_luon}")
                    distance_dich_ly_tuong, toa_do_dich_ly_tuong = [self.distance_dich_ly_tuong, self.toa_do_dich_ly_tuong]
                    # print(f"distance_dich_ly_tuong: {distance_dich_ly_tuong}, toa_do_dich_ly_tuong: {toa_do_dich_ly_tuong}")
                    toa_do_hien_tai = AGVConfig.toa_do_agv_mm
                    # print(f"toa_do_hien_tai: {toa_do_hien_tai}")


                    # set vận tốc gửi ban đầu = 0
                    van_toc_gui_driver_phai = 0
                    van_toc_gui_driver_trai = 0
                    
                    # nếu là các điểm đích ở lấy trả hàng, xoay vuông góc, song song
                    if xac_dinh_vi_tri_xe == 1:
                        v_tien_max0 = min(v_tien_max0, 1000)
                    if di_cham != 0:
                        v_tien_max0 = 3000
                    if v_tien_max0 == 0:
                        stop = 1
                    v_tien_max = v_tien_max0
                    
                
                    v_re = v_re_max 
                    min_angle = 500
                    # giảm khi góc bé
                    if abs(angle) <= 10:
                        min_angle = 300
                    if abs(angle) <= 20:
                        v_re = v_re * (abs(angle) / 20)
                    if v_re < min(min_angle, v_re_max):
                        v_re = min(min_angle, v_re_max) 
                    v_re_max = min(v_re_max, v_re)
                    if v_re_max < 300:
                        v_re_max = 300
                    
                    
                    if stop == 0:
                        if check_angle_distance == "distance" and dang_re == 0 and self.quay_trai == 0 and self.quay_phai == 0:
                            ty_le_distance = 1
                            van_toc_min = min(1800, v_tien_max)
                            # giảm vận tốc max khi gần đến đích
                            if distance_dich_ly_tuong is not None:
                                if di_chuyen_luon["update"] == 1 and di_chuyen_luon["van_toc_min"] is not None and distance_dich_ly_tuong < 100:
                                    v_tien_max = min(v_tien_max, di_chuyen_luon["van_toc_min"])
                                elif distance_dich_ly_tuong <= AGVConfig_2.distance_max and distance_dich_ly_tuong != 0:
                                    ty_le_distance = AGVConfig_2.distance_max/distance_dich_ly_tuong
                                    v_tien = int(v_tien_max / ty_le_distance)
                                    v_tien_max = int(v_tien)

                                    if v_tien_max < van_toc_min:
                                        v_tien_max = van_toc_min
                                else:
                                    v_max_trai_phai = max(abs(AGVConfig_2.van_toc_phan_hoi_trai), abs(AGVConfig_2.van_toc_phan_hoi_phai))
                                    if v_tien_max > v_max_trai_phai + 1000:
                                        v_tien_max = v_max_trai_phai + 1000
                                    if v_tien_max < van_toc_min:
                                        v_tien_max = van_toc_min
                            else:
                                v_max_trai_phai = max(abs(AGVConfig_2.van_toc_phan_hoi_trai), abs(AGVConfig_2.van_toc_phan_hoi_phai))
                                if v_tien_max > v_max_trai_phai + 1000:
                                    v_tien_max = v_max_trai_phai + 1000
                                if v_tien_max < van_toc_min:
                                    v_tien_max = van_toc_min

                            khoang_cach_hai_banh = AGVConfig_2.wheel_base_mm
                            v_min = min(1800, max((v_tien_max-500), 500))
                            # di chuyển hình cung
                            if di_chuyen_luon is not None:
                                if di_chuyen_luon["update"] == 1 and di_chuyen_luon["van_toc_di_chuyen_luon"] is not None:
                                    v_tien_max = di_chuyen_luon["van_toc_di_chuyen_luon"]
                                    v_min = 0

                            if di_thuan_nguoc == 0:
                                van_toc_gui_driver_phai, van_toc_gui_driver_trai = controller_motor.agv_bam_duong(x=toa_do_hien_tai[0], y=toa_do_hien_tai[1], 
                                                                                    toa_do_diem_huong = toa_do_diem_huong,
                                                                                    x0 = toa_do_diem_dau[0], y0 = toa_do_diem_dau[1],
                                                                                    xg = toa_do_dich_ly_tuong[0], yg = toa_do_dich_ly_tuong[1],
                                                                                    wheel_base = khoang_cach_hai_banh,
                                                                                    v_max = v_tien_max, v_min = v_min,
                                                                                    di_chuyen_luon = di_chuyen_luon
                                                                                    )
                            else:
                                van_toc_gui_driver_trai, van_toc_gui_driver_phai = controller_motor.agv_bam_duong(x=toa_do_hien_tai[0], y=toa_do_hien_tai[1], 
                                                                                    toa_do_diem_huong = toa_do_diem_huong,
                                                                                    x0 = toa_do_diem_dau[0], y0 = toa_do_diem_dau[1],
                                                                                    xg = toa_do_dich_ly_tuong[0], yg = toa_do_dich_ly_tuong[1],
                                                                                    wheel_base = khoang_cach_hai_banh,
                                                                                    v_max = v_tien_max, v_min = v_min,
                                                                                    di_chuyen_luon = di_chuyen_luon
                                                                                    )
                            # print("---- van_toc_gui_driver_trai  @@@@@----", van_toc_gui_driver_trai, van_toc_gui_driver_phai)
                        else:
                            if abs(angle) > 2:
                                if (angle > 0 or self.quay_phai == 1) and self.quay_trai == 0:
                                    van_toc_gui_driver_trai = -v_re_max
                                    van_toc_gui_driver_phai = v_re_max
                                    self.quay_phai = 1
                                else:
                                    van_toc_gui_driver_phai = -v_re_max
                                    van_toc_gui_driver_trai = v_re_max
                                    self.quay_trai = 1
                            else:
                                van_toc_gui_driver_trai = 0
                                van_toc_gui_driver_phai = 0
                                self.quay_trai = 0
                                self.quay_phai = 0

                        # reset rẽ với trường hợp đặc biệt
                        if di_chuyen_luon["update"] == 1 and di_chuyen_luon["van_toc_di_chuyen_luon"] is not None:
                            self.quay_trai = 0
                            self.quay_phai = 0
                        else:
                            # thực hiện rẽ
                            if abs(angle) > 60:
                                if self.quay_phai == 1:
                                    van_toc_gui_driver_trai = -v_re_max
                                    van_toc_gui_driver_phai = v_re_max
                                elif self.quay_trai == 1:
                                    van_toc_gui_driver_phai = -v_re_max
                                    van_toc_gui_driver_trai = v_re_max  

                        # kiểm tra đầu ra của vận tốc trước khi gửi để tránh tốc độ quá lớn
                        max_v = max(v_re_max, v_tien_max)
                        if abs(van_toc_gui_driver_phai) > max_v or abs(van_toc_gui_driver_trai) > max_v:
                            # print("-------------------- warning 0001 ---------------------", van_toc_gui_driver_phai, van_toc_gui_driver_trai, max_v)
                            van_toc_gui_driver_phai = 0
                            van_toc_gui_driver_trai = 0

                        # đảo dấu với trường hợp đi ngược
                        if di_thuan_nguoc == 1 and self.quay_phai == 0 and self.quay_trai == 0:
                            van_toc_gui_driver_phai = - van_toc_gui_driver_phai
                            van_toc_gui_driver_trai = - van_toc_gui_driver_trai 

                        # thực hiện quay chậm, tránh thay đổi tốc độ đột ngột
                        if van_toc_gui_driver_trai == - van_toc_gui_driver_phai:
                            delta_re = 200
                            if abs(AGVConfig_2.van_toc_phan_hoi_trai) > delta_re or abs(AGVConfig_2.van_toc_phan_hoi_phai) > delta_re:
                                if (abs(AGVConfig_2.van_toc_phan_hoi_trai + van_toc_gui_driver_trai) > (abs(van_toc_gui_driver_trai) + delta_re) or 
                                                            abs(AGVConfig_2.van_toc_phan_hoi_phai + van_toc_gui_driver_phai) > abs(van_toc_gui_driver_phai + delta_re)):
                                    van_toc_gui_driver_trai = 0
                                    van_toc_gui_driver_phai = 0
                        else: # thực hiện so sánh đầu vào và đầu ra của vận tốc tránh thay đổi đột ngột với trường hợp tiến lùi
                            van_toc_gui_driver_trai, van_toc_gui_driver_phai = self.tinh_van_toc_dong_bo(van_toc_gui_driver_trai, 
                                                                                                         van_toc_gui_driver_phai, 
                                                                                                         AGVConfig_2.van_toc_phan_hoi_trai, 
                                                                                                         AGVConfig_2.van_toc_phan_hoi_phai, 
                                                                                                         gia_toc_max=1000)
                            
                        # self.set_rpm(int(van_toc_gui_driver_trai), int(van_toc_gui_driver_phai))
                        # reset rẽ
                        if abs(angle) <= 30:
                            self.quay_phai = 0
                            self.quay_trai = 0     
                    else:
                        if AGVConfig_2.van_toc_phan_hoi_trai != 0 or AGVConfig_2.van_toc_phan_hoi_phai != 0:
                            self.set_rpm(0, 0)
                            print("stop_agv -- agv")
                        
                # điều khiển bằng tay
                else:  
                    van_toc_gui_driver_trai = 0
                    van_toc_gui_driver_phai = 0

                    if self.van_toc_gui_driver_trai == 0 and self.van_toc_gui_driver_phai == 0:
                        if self.time_stop_dk_tay == 0:
                            self.time_stop_dk_tay = time.time()
                            van_toc_gui_driver_trai = int(AGVConfig_2.van_toc_phan_hoi_trai)
                            van_toc_gui_driver_phai = int(AGVConfig_2.van_toc_phan_hoi_phai)
                        else:
                            ty_le = (time.time() - self.time_stop_dk_tay)/0.7
                            if ty_le >= 1:
                                van_toc_gui_driver_trai = 0
                                van_toc_gui_driver_phai = 0
                            else:
                                van_toc_gui_driver_trai = int(AGVConfig_2.van_toc_phan_hoi_trai * (1 - ty_le))
                                van_toc_gui_driver_phai = int(AGVConfig_2.van_toc_phan_hoi_phai * (1 - ty_le)) 
                    else:
                        self.time_stop_dk_tay = 0

                        delta_v_trai = abs(self.van_toc_gui_driver_trai - AGVConfig_2.van_toc_phan_hoi_trai)
                        delta_v_phai = abs(self.van_toc_gui_driver_phai - AGVConfig_2.van_toc_phan_hoi_phai)

                        k = self.van_toc_gui_driver_trai * self.van_toc_gui_driver_phai

                        if k <=0:
                            van_toc_gui_driver_trai = self.van_toc_gui_driver_trai
                            van_toc_gui_driver_phai = self.van_toc_gui_driver_phai
                        else:
                            if self.van_toc_gui_driver_trai != 0 and self.van_toc_gui_driver_phai != 0:
                                ty_le_van_toc = self.van_toc_gui_driver_trai/self.van_toc_gui_driver_phai

                                if delta_v_trai > 1000:
                                    if AGVConfig_2.van_toc_phan_hoi_trai < self.van_toc_gui_driver_trai:
                                        if AGVConfig_2.van_toc_phan_hoi_trai + 1000 < self.van_toc_gui_driver_trai: # vận tốc hiện tại + 1000 vẫn nhỏ hơn vận tốc gửi đến
                                            van_toc_gui_driver_trai = AGVConfig_2.van_toc_phan_hoi_trai + 1000
                                        else:
                                            van_toc_gui_driver_trai = self.van_toc_gui_driver_trai
                                    else:
                                        if AGVConfig_2.van_toc_phan_hoi_trai - 1000 > self.van_toc_gui_driver_trai: # vận tốc hiện tại - 1000 vẫn lớn hơn vận tốc gửi đến
                                            van_toc_gui_driver_trai = AGVConfig_2.van_toc_phan_hoi_trai - 1000
                                        else:
                                            van_toc_gui_driver_trai = self.van_toc_gui_driver_trai

                                else:
                                    van_toc_gui_driver_trai = self.van_toc_gui_driver_trai

                                if delta_v_phai > 1000:
                                    if AGVConfig_2.van_toc_phan_hoi_phai < self.van_toc_gui_driver_phai:
                                        if AGVConfig_2.van_toc_phan_hoi_phai + 1000 < self.van_toc_gui_driver_phai: # vận tốc hiện tại + 1000 vẫn nhỏ hơn vận tốc gửi đến
                                            van_toc_gui_driver_phai = AGVConfig_2.van_toc_phan_hoi_phai + 1000
                                        else:
                                            van_toc_gui_driver_phai = self.van_toc_gui_driver_phai
                                    else:
                                        if AGVConfig_2.van_toc_phan_hoi_phai - 1000 > self.van_toc_gui_driver_phai: # vận tốc hiện tại - 1000 vẫn lớn hơn vận tốc gửi đến
                                            van_toc_gui_driver_phai = AGVConfig_2.van_toc_phan_hoi_phai - 1000
                                        else:
                                            van_toc_gui_driver_phai = self.van_toc_gui_driver_phai
                                else:
                                    van_toc_gui_driver_phai = self.van_toc_gui_driver_phai

                                # kiểm tra lại tỉ lệ vận tốc 
                                if ty_le_van_toc > 1: # v_trai > v_phai
                                    van_toc_gui_driver_phai = int(van_toc_gui_driver_trai / ty_le_van_toc)
                                else:
                                    van_toc_gui_driver_trai = int(van_toc_gui_driver_phai * ty_le_van_toc)

                print("-----------",self.van_toc_gui_driver_trai, self.van_toc_gui_driver_phai, int(van_toc_gui_driver_trai), int(van_toc_gui_driver_phai))
                self.set_rpm(int(van_toc_gui_driver_trai), int(van_toc_gui_driver_phai))
                # print(" -------------- van_toc_gui_driver_trai ------------", van_toc_gui_driver_trai, van_toc_gui_driver_phai)
            time.sleep(0.1)
    def check_connect(self):
        self.check_time = time.time()

    def tinh_van_toc_dong_bo(self, yc_trai, yc_phai, ht_trai, ht_phai, gia_toc_max=1000):
        """
        Hàm tính toán vận tốc setpoint tiếp theo để đảm bảo 2 bánh tăng/giảm tốc đồng bộ.
        gia_toc_max: Bước thay đổi vận tốc tối đa cho phép trong 1 chu kỳ.
        """
        delta_trai = yc_trai - ht_trai
        delta_phai = yc_phai - ht_phai

        # Tìm độ lệch lớn nhất (bánh xe cần thay đổi nhiều nhất)
        max_delta = max(abs(delta_trai), abs(delta_phai))

        # Nếu độ lệch nhỏ hơn gia tốc tối đa, trả về luôn đích đến (đã đến nơi)
        if max_delta <= gia_toc_max:
            return yc_trai, yc_phai

        # Tính tỷ lệ scale dựa trên bánh xe cần thay đổi nhiều nhất
        scale = gia_toc_max / max_delta

        # Tính vận tốc tiếp theo: Bánh cần thay đổi ít sẽ thay đổi chậm lại theo tỷ lệ
        next_trai = int(ht_trai + (delta_trai * scale))
        next_phai = int(ht_phai + (delta_phai * scale))

        return next_trai, next_phai
        



if __name__ == "__main__":
    import cv2
    driver_motor = sent_data_driver()
    img = np.zeros((480, 640, 3), dtype= np.uint8)
    driver_motor.clear_alarm()
    vt_trai = 0
    vt_phai = 0
    
    while True:
        driver_motor.check_connect()
        # driver_motor.sent_data_controller(200, 200)
        vt_trai = 0
        vt_phai = 0
        cv2.imshow("map", img),
        key = cv2.waitKey(1) & 0xFF

        lf, rf = driver_motor.get_fault_code()
        print("Left fault:", lf)
        print("Right fault:", rf)

        if key == ord('l'):
            vt_phai = 10000
            vt_trai = 10000
        driver_motor.sent_data_controller(vt_trai=vt_trai, vt_phai=vt_phai)

        if key == ord('q') or cv2.getWindowProperty('map', cv2.WND_PROP_VISIBLE) < 1:
            driver_motor.disconnect()
            break
        time.sleep(1)