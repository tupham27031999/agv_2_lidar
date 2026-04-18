import socket
import numpy as np
import struct
from PyQt6 import QtCore, QtGui, QtWidgets
from config import AGVConfig


UDP_IP = AGVConfig.lidar2["ip"]
UDP_PORT = AGVConfig.lidar2["port"]

print("host_lidar_2", UDP_IP)
print("port_lidar_2", UDP_PORT)


class LidarP:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.timer = QtCore.QTimer()
        self.final_data = np.array([[0, 0, 0]])
        self.final_data_old = np.array([[0, 0, 0]])
        self.final_data_new = []
        self.connect = True
        self.data_ok = 0
        self.setup()
        self.start = False


    def setup(self):
        print("setup", UDP_IP, UDP_PORT)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.timer.start(40)

    def decode_data(self, data):
        """Giải mã dữ liệu từ LR-1BS3/5."""
        if len(data) != 8:
            raise ValueError("Dữ liệu đầu vào phải là 8 byte.")
        
        angle_raw, distance_raw, signal_raw, _ = struct.unpack("<HHHH", data)
        angle = angle_raw * 0.01
        distance = distance_raw  # Giả sử tỷ lệ khoảng cách là 1mm
        signal = signal_raw
        return angle, distance, signal, _
    def get_data(self):
        output = self.final_data_old
        check = False 
        if self.data_ok == 1:
            output = self.final_data
            self.final_data_old = self.final_data
            self.data_ok = 0
            check = True
        return output, check
    # def process_data(self):
    #     while self.connect:
    #         # print(self.connect, "hhhhhhhhhhhh")
    #         try: 
    #             data, addr = self.sock.recvfrom(1240)
    #         except socket.timeout:
    #             self.connect = False
    #             print("connect False")
    #             break
    #         body = data[40:]
    #         data_array = []

    #         # Chuyển toàn bộ body (1200 bytes) thành mảng NumPy
    #         data0 = np.frombuffer(body, dtype=np.uint16).reshape(-1, 4)

    #         data = data0[data0[:, 1] != 0]

    #         # Giải mã dữ liệu
    #         angles0 = data0[:, 0] * 0.01  # Góc quét (angle)
    #         angles = data[:, 0] * 0.01  # Góc quét (angle)
    #         # print(angles)
    #         distances = data[:, 1]      # Khoảng cách đo được (distance)
    #         signals = data[:, 2]        # Cường độ tín hiệu (signal)

    #         data_array = np.column_stack((signals, angles, distances))

    #         if int(angles0[0]) == 0:
    #             if self.data_ok == 0:
    #                 self.final_data = self.final_data_new  # Ensure it's a NumPy array
    #                 self.data_ok = 1
    #             self.final_data_new = []
    #         else:
    #             self.final_data_new = [*self.final_data_new, *data_array]
    #     print("lidar 2 đóng")
    #     self.sock.close()
    def process_data(self):
        # Thiết lập ngưỡng: None để lấy tất cả, hoặc số cụ thể (ví dụ: 800) để lọc phản quang
        # INTENSITY_THRESHOLD = 4000 
        INTENSITY_THRESHOLD = None

        while self.connect:
            try: 
                data, addr = self.sock.recvfrom(1240)
            except socket.timeout:
                self.connect = False
                break
            
            body = data[40:]
            data0 = np.frombuffer(body, dtype=np.uint16).reshape(-1, 4)
            
            # Lọc bỏ các điểm có distance = 0
            valid_mask = data0[:, 1] != 0
            data = data0[valid_mask]

            # Tách các kênh dữ liệu
            angles0 = data0[:, 0] * 0.01
            angles = data[:, 0] * 0.01
            distances = data[:, 1]
            signals = data[:, 2]

            # --- LOGIC LỌC THEO CƯỜNG ĐỘ ---
            if INTENSITY_THRESHOLD is not None:
                # Chỉ giữ lại các điểm có cường độ cao hơn ngưỡng (Reflectors)
                reflector_mask = signals > INTENSITY_THRESHOLD
                # Cập nhật lại các mảng dữ liệu chỉ chứa điểm phản quang
                angles = angles[reflector_mask]
                distances = distances[reflector_mask]
                signals = signals[reflector_mask]

            # Gộp dữ liệu thành mảng (signals, angles, distances)
            data_array = np.column_stack((signals, angles, distances))

            if int(angles0[0]) == 0:
                if self.data_ok == 0:
                    self.final_data = np.array(self.final_data_new)
                    self.data_ok = 1
                self.final_data_new = []
            else:
                # Dùng list extend hoặc vstack tùy vào cách bạn quản lý bộ nhớ
                if len(data_array) > 0:
                    self.final_data_new.extend(data_array.tolist())

        print("lidar 1 đóng")
        self.sock.close()

            
