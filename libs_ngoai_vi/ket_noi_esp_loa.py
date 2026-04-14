import serial
import time
import shutil,os
from libs_file import remove
from libs_ngoai_vi import music
import config_2 as cfg
from config import AGVConfig



path_data_input_output = cfg.PATH_PHAN_MEM + "/data_input_output"
path_esp_sent_py = remove.tao_folder(path_data_input_output + "/esp_sent_py")
path_py_sent_esp = remove.tao_folder(path_data_input_output + "/py_sent_esp")

sent_data = ""
sent_data_new = ""
connect_esp = 1
check_connect_esp = False
input_esp =  {"IN1":0,"IN2":0,"IN3":0,"IN4":0,"IN5":0,"IN6":0,"IN7":0,"IN8":0,"IN9":0,"IN10":0,"IN11":0,"IN12":0}
da_nang_xong = 0
da_ha_xong = 0
remove.remove_all_folder_in_folder(path_esp_sent_py)
gui_off_24v_thanh_cong = 0
gui_bat_loa_thanh_cong = 0
gui_nang_ha_thanh_cong = 0
gui_nha_phanh_thanh_cong = 0
dung_hoat_dong = 0
loi_motor_nang_ha = 0

check_time = 0
reset_5s = False


com = AGVConfig.esp32["COM"]
hz = AGVConfig.esp32["baudrate"]
print("esp",com,hz)

def thap_phan_sang_nhi_phan(n): 
    return list(bin(n).replace("0b", ""))[::-1]

class Python_Esp:
    def __init__(self):
        self.connected = True
        # try:
        self.serial = serial.Serial()
        # except OSError as e:
        #     self.connected = False
        
        self.data_esp_sent = []

        self.time_check_connect = 0
        self.data = ""
        self.data_sent = ""
        self.list_ok = ["0","1","2","3","4","5","6","7","8","9"]   
        self.data_move = ""
        self.data_read_ouput = ""
        self.load_angle = 0
        self.angle = 0
        self.input_8 = 0

        self.load_data_esp = 0
        self.close_all = 0
    def khai_bao_serial(self):
        # if self.connected == True:
        self.serial.port = str(com)
        self.serial.baudrate = int(hz)
        self.serial.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.serial.timeout = 3            #non-block read
        self.serial.writeTimeout = 2     #timeout for write
        self.serial.open()
        self.connected = True
        self.out = ""
        self.lay_du_lieu = 0
        self.data_old = ""
        self.time_sent = 0

        self.alpha_servo = 90
        self.name_data = ""

        self.input_esp = {"IN1":0,"IN2":0,"IN3":0,"IN4":0,"IN5":0,"IN6":0,"IN7":0,"IN8":0,"IN9":0,"IN10":0,"IN11":0,"IN12":0}
        self.gui_off_24v_thanh_cong = 0
        self.gui_bat_loa_thanh_cong = 0
        self.gui_nang_ha_thanh_cong = 0
        self.gui_nha_phanh_thanh_cong = 0
        self.dung_hoat_dong = 0
        self.nang_ha = ""
        self.loi_motor_nang_ha = 0

        self.reset_5s = False
        self._reset_press_time = 0
        self._reset_triggered = False
        self.RESET_IN = "IN12"   # đổi nếu bạn dùng IN khác
            
        # except OSError as e:
        #     self.connected = False
        #     print(self.connected,"esp32")
    def check_connect(self):
        try:
            self.serial.inWaiting()
            self.load_data_esp = 0
        except:
            self.connected = False
    def thread_load_data(self):
        if self.load_data_esp == 0 and self.close_all == 0:
            self.load_data_esp = 1
            self.load_data()
    
    def check_reset_5s(self):
        # nút nhấn mức 0 (active LOW)
        if self.input_esp[self.RESET_IN] == 0:
            if self._reset_press_time == 0:
                self._reset_press_time = time.time()
                self._reset_triggered = False

            if not self._reset_triggered and time.time() - self._reset_press_time >= 5:
                self.reset_5s = True
                self._reset_triggered = True
                print("⚠ RESET giữ > 5s")

        else:
            self._reset_press_time = 0
            self._reset_triggered = False
            self.reset_5s = False
        return self.reset_5s



    def check_data_angle(self,data):
        list_ok = ["0","1","2","3","4","5","6","7","8","9","-","."]
        list_data = list(data)
        output = True
        for i in range(0,len(list_data)):
            output = False
            for i2 in range(0,len(list_ok)):
                if list_data[i] == list_ok[i2]:
                    output = True
                    break
            if output == False:
                break
        # print(output)
        return output

    def _parse_and_handle_message(self, line):
        """Phân tích một tin nhắn từ ESP32 và cập nhật trạng thái."""
        print("output:", line)
        self.out = line

        # Gửi ACK ngay lập tức để ESP32 biết đã nhận được
        ack_message = f"reply#{self.out}\r\n"
        try:
            self.serial.write(ack_message.encode())
            print("sent_data---------------rrrr", ack_message.strip())
        except Exception as e:
            print(f"Lỗi khi gửi ACK: {e}")
            self.connected = False
            return

        parts = line.split('#')

        # Cập nhật trạng thái dừng khẩn cấp
        if "dung_hoat_dong_1" in parts:
            self.dung_hoat_dong = 1
        elif "dung_hoat_dong_0" in parts:
            self.dung_hoat_dong = 0

        # Phân tích thông tin lệnh và trạng thái input
        if len(parts) >= 2:
            self._update_command_ack(parts[0])
            self._update_input_status(parts[1])

        # Xử lý yêu cầu gửi lại
        if "check_sent" in parts and self.data_sent:
            self._resend_last_command()

    def _update_command_ack(self, command_info):
        """Cập nhật cờ xác nhận lệnh thành công dựa trên command_info từ ESP."""
        if command_info.startswith("off_24v"):
            self.gui_off_24v_thanh_cong = 1
            print("gửi tắt thành công")
        elif command_info.startswith("nang_ha") and ":" in command_info:
            try:
                self.gui_nang_ha_thanh_cong = command_info.split(':', 1)[1]
            except IndexError:
                print(f"Cảnh báo: Lệnh 'nang_ha' không đúng định dạng: {command_info}")
        elif command_info.startswith("loai_bo_phanh_xe") and ":" in command_info:
            self.gui_nha_phanh_thanh_cong = command_info.split(':', 1)[1]

    def _update_input_status(self, input_status_str):
        """Cập nhật trạng thái các chân IN1-IN12 từ chuỗi trạng thái."""
        try:
            input_val = int(input_status_str)
            bin_str = bin(input_val)[2:]

            for i in range(1, 13):
                self.input_esp[f"IN{i}"] = 0

            for i, bit in enumerate(reversed(bin_str)):
                if i < 12:
                    self.input_esp[f"IN{i+1}"] = int(bit)

            self.loi_motor_nang_ha = 1 if self.input_esp["IN8"] == 0 else 0
        except (ValueError, IndexError) as e:
            print(f"Lỗi phân tích trạng thái input '{input_status_str}': {e}")

    def _internal_send(self, command_to_send, log_prefix="sent_data"):
        """Hàm nội bộ để định dạng và gửi lệnh qua serial, tránh lặp code."""
        try:
            dung_hoat_dong_str = "dung_hoat_dong_1" if self.dung_hoat_dong == 1 else "dung_hoat_dong_0"
            data_to_send = f"{command_to_send}#{dung_hoat_dong_str}#0\r\n"
            self.serial.write(data_to_send.encode())
            print(f"{log_prefix}---------------->", data_to_send.strip())
        except Exception as e:
            print(f"Lỗi khi gửi dữ liệu ({log_prefix}): {e}")
            self.connected = False

    def _resend_last_command(self):
        """Gửi lại lệnh cuối cùng cho ESP32, giới hạn 1 lần/giây."""
        # Chỉ gửi lại lệnh nếu đã hơn 1 giây kể từ lần gửi cuối cùng
        if time.time() - self.time_sent > 1:
            print("Phát hiện không khớp (check_sent). Gửi lại lệnh cuối.")
            # Cập nhật thời gian gửi ngay trước khi gửi
            self.time_sent = time.time()
            # Sử dụng hàm nội bộ để gửi lại lệnh cuối cùng đã lưu
            self._internal_send(self.data_sent, log_prefix="resend_data")

    def load_data(self):
        if self.connected and not self.close_all:
            # Vòng lặp để xử lý tất cả các tin nhắn đang chờ trong bộ đệm
            while self.serial.inWaiting() > 0:
                if self.close_all == 1:
                    break
                try:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._parse_and_handle_message(line)
                except UnicodeDecodeError:
                    print("Cảnh báo: Nhận được dữ liệu không phải UTF-8, bỏ qua dòng.")
                    continue
            
            if self.close_all == 1:
                self.serial.close()
            self.load_data_esp = 0

    def read_data(self):
        # Hàm này không còn cần thiết, logic đã được chuyển vào load_data
        # Để lại để tránh lỗi nếu có chỗ khác gọi, nhưng nên được xóa đi
        return ""
    def check_data(self,data):
        ds = list(data)
        ok = "0"
        for i in range(0,len(ds)):
            for i2 in range(0,len(self.list_ok)):
                if self.list_ok[i2] == ds[i]:
                    ok = "1"
                    break
            if ok == "0":
                break
        return ok

    def sent_data(self,data):
        self.data_sent = data
        self.time_sent = time.time()
        # Sử dụng hàm nội bộ để gửi lệnh mới
        self._internal_send(self.data_sent, log_prefix="sent_data")

    def close_serial(self):
        self.close_all = 1
        self.serial.close()
        print("close_serial 2")
def py_sent_esp(data = "",reset = 0):
    global sent_data,sent_data_new
    sent_data = data
    if reset == 1:
        sent_data_new = ""
        sent_data = ""

def esp_sent_py(input_esp):
    global da_nang_xong, da_ha_xong
    if input_esp["IN7"] == 0:
        da_ha_xong = 0
        da_nang_xong = 1
    if input_esp["IN6"] == 0:
        da_ha_xong = 1
        da_nang_xong = 0

def check_connect():
    global check_time
    check_time = time.time()
def python_esp32():
    global sent_data,sent_data_new, connected, input_esp, check_connect_esp, gui_off_24v_thanh_cong, gui_bat_loa_thanh_cong, connect_esp
    global gui_nha_phanh_thanh_cong, gui_nang_ha_thanh_cong, dung_hoat_dong, loi_motor_nang_ha, reset_5s
    
    py_esp = Python_Esp()
    py_esp.khai_bao_serial()

    while connect_esp:
        gui_off_24v_thanh_cong = py_esp.gui_off_24v_thanh_cong
        gui_bat_loa_thanh_cong = py_esp.gui_bat_loa_thanh_cong
        gui_nha_phanh_thanh_cong = py_esp.gui_nha_phanh_thanh_cong
        gui_nang_ha_thanh_cong = py_esp.gui_nang_ha_thanh_cong
        dung_hoat_dong = py_esp.dung_hoat_dong
        loi_motor_nang_ha = py_esp.loi_motor_nang_ha    

        AGVConfig.reset_5s = py_esp.check_reset_5s()

        # print("gui_off_24v_thanh_cong", gui_off_24v_thanh_cong)
        # print("gui_bat_loa_thanh_cong", gui_bat_loa_thanh_cong)
        # print("gui_nha_phanh_thanh_cong", gui_nha_phanh_thanh_cong) 
        # print("gui_nang_ha_thanh_cong", gui_nang_ha_thanh_cong)
        # print("dung_hoat_dong", dung_hoat_dong)
        # print("loi_motor_nang_ha", loi_motor_nang_ha)



        input_esp = py_esp.input_esp
        # print("input_esp", input_esp)
        esp_sent_py(input_esp)
        

        check_connect_esp = py_esp.connected
        if check_time != 0 and time.time() - check_time > 2:
            py_esp.close_serial()
            print("------- tắt kết nối esp32 --------")
            connect_esp = 0
        else:
            py_esp.check_connect()
            py_esp.load_data()
            if py_esp.connected == True:
                connected = True
            if py_esp.connected == False:
                py_esp.khai_bao_serial()
            if sent_data != "":
                if sent_data != sent_data_new:
                    py_esp.sent_data(sent_data)
                    sent_data_new = sent_data
        time.sleep(0.1)
# py_esp = Python_Esp()
# py_esp.khai_bao_serial()

# def while_esp():
#     global sent_data,sent_data_new, connected, input_esp, py_esp
#     input_esp = py_esp.input_esp
#     if connect_esp == 0:
#         py_esp.close_serial()
#         # break
#     py_esp.check_connect()
#     py_esp.load_data()
#     if py_esp.connected == True:
#         connected = True
#     if py_esp.connected == False:
#         py_esp.khai_bao_serial()
#     if sent_data != "":
#         py_esp.sent_data(sent_data)
# python_esp32()
