import serial
import time
import shutil,os
from libs_file import remove
from config import AGVConfig
from config_2 import AGVConfig_2


sent_data = ""
sent_data_new = ""
connect_serial = 1
check_connect_serial = False
input_esp =  {"IN1":0,"IN2":0,"IN3":0,"IN4":0,"IN5":0,"IN6":0,"IN7":0,"IN8":0,"IN9":0,"IN10":0,"IN11":0,"IN12":0}
da_nang_xong = 0
da_ha_xong = 0
gui_off_24v_thanh_cong = 0
gui_bat_loa_thanh_cong = 0
check_time = 0

phan_tram_pin = 0 
check_pin = False
time_check_pin = 0


com = AGVConfig.pin["COM"]
hz = AGVConfig.pin["baudrate"]
vol_max = AGVConfig_2.vol_max
vol_min = AGVConfig_2.vol_min
print("pin",com,hz)

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
        self.gui_nang_ha = ""

        self.phan_tram_pin = 0 
        self.check_pin = False
        self.vol_max = vol_max
        self.vol_min = vol_min
            
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
    
    def parse_voltage_from_frame(self, hex_frame):
        # Chuyển chuỗi hex thành danh sách các byte
        bytes_data = [hex_frame[i:i+2] for i in range(0, len(hex_frame), 2)]
        # Kiểm tra độ dài khung dữ liệu
        if len(bytes_data) < 12:
            return False, 0, 0
        # Byte1~2: Tổng điện áp tích lũy (0.1V)
        cumulative_voltage_raw = int(bytes_data[4] + bytes_data[5], 16)
        cumulative_voltage = cumulative_voltage_raw * 0.1  # đơn vị V
        # Byte3~4: Tổng điện áp thu thập (1V)
        gathered_voltage_raw = int(bytes_data[6] + bytes_data[7], 16)
        gathered_voltage = gathered_voltage_raw * 1.0  # đơn vị V
        return True, cumulative_voltage, gathered_voltage
    
    def load_data(self):
        # print(self.connected == True and self.close_all == 0)
        if self.connected == True and self.close_all == 0:
            while self.serial.inWaiting() > 0:
                self.data_sent = ""
                if self.close_all == 1:
                    break
                self.out = ""
                self.out = self.read_data()
                if self.out: # Chỉ xử lý nếu self.out không rỗng
                    # print("Received from ESP (HEX):", self.out) # Dữ liệu nhận được đã là chuỗi HEX
                    try:
                        self.check_pin, cumulative_voltage, gathered_voltage = self.parse_voltage_from_frame(self.out)
                        print(f"Voltage: Cumulative={cumulative_voltage}V, Gathered={gathered_voltage}V") # max 28.8 min 23v
                        self.phan_tram_pin = (cumulative_voltage - self.vol_min) / (self.vol_max - self.vol_min) * 100

                    except ValueError as e:
                        # Bỏ qua các khung dữ liệu không hợp lệ hoặc quá ngắn
                        # print(f"Could not parse frame: {e}")
                        pass

                        
                                
                        # print("self.input_esp", self.input_esp)
            # print(self.data_sent, self.data_sent, self.data_move,time.time() - self.time_sent)
            # print(self.data_sent != "" , self.data_sent != self.data_move , time.time() - self.time_sent > 2)
            if self.data_sent != "" and time.time() - self.time_sent > 5:
                self.time_sent = time.time()
                # Mặc định gửi chuỗi HEX này nếu không có dữ liệu nào khác
                data_to_send_hex = self.data_sent
                try:
                    
                    # Chuyển chuỗi HEX thành bytes và gửi đi
                    byte_data = bytes.fromhex(data_to_send_hex)
                    # print(f"Sending data to ESP (HEX): {data_to_send_hex}")
                    self.serial.write(byte_data)
                    self.data_move = self.data_sent
                except (serial.SerialException, ValueError) as e:
                    print(f"Error writing to ESP: {e}")
                    self.connected = False

            if self.close_all == 1:
                self.serial.close()
            self.load_data_esp = 0

    def read_data(self):
        load = 0
        try:
            self.data = self.serial.readline()
        except serial.SerialException as e:
            print(f"Error reading from ESP: {e}")
            self.connected = False
            return None

        if self.data:
            # Chuyển đổi chuỗi byte nhận được thành chuỗi HEX
            return self.data.hex().upper()
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
        # self.check_pin = False
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
def python_serial():
    global sent_data,sent_data_new, connected, check_connect_serial, connect_serial, phan_tram_pin, check_pin, time_check_pin
    
    py_esp = Python_Esp()
    py_esp.khai_bao_serial()
    sent_data = "A540900800000000000000007D"

    while connect_serial:
        phan_tram_pin = int(py_esp.phan_tram_pin)
        check_pin = py_esp.check_pin
        # if time.time() - time_check_pin > 10:
        time_check_pin = time.time()
        # print("phan_tram_pin",phan_tram_pin)
        # print("check_pin",check_pin)
        check_connect_serial = py_esp.connected
        py_esp.check_connect()
        py_esp.load_data()
        if py_esp.connected == True:
            connected = True
        if py_esp.connected == False:
            py_esp.khai_bao_serial()
        if sent_data != "":
            py_esp.sent_data(sent_data)
        if check_time != 0 and time.time() - check_time > 2:
            py_esp.close_serial()
            print("------- tắt kết nối Pin --------")
            connect_serial = 0
        time.sleep(1)
# py_esp = Python_Esp()
# py_esp.khai_bao_serial()
# sent_data = "on"

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
# python_serial()

