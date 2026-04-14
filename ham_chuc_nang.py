import numpy as np
import config_2 as cfg
from config import AGVConfig
from config_2 import AGVConfig_2
import cv2
import os
import time
from libs_lidar import convert_2_lidar, connect_lidar_sick_1, connect_lidar_sick_2, connect_lidar
import threading
from libs_ngoai_vi import phan_tram_pin, music, ket_noi_esp_loa
import socket









class du_lieu_lidar_scan:
    def __init__(self):
        self.stt_scan = 0
        

        
        if AGVConfig.thiet_lap_ket_noi["lidar"] == "on":
            if AGVConfig.loai_lidar != "usb":
                self.lidar_1 = connect_lidar_sick_1.LidarP()
                threading.Thread(target=self.lidar_1.process_data).start()
                
                self.lidar_2 = connect_lidar_sick_2.LidarP()
                threading.Thread(target=self.lidar_2.process_data).start()
            else:
                self.lidar = connect_lidar.main_lidar()
                self.lidar.connect()

    def load_data_lidar(self):
        if AGVConfig.thiet_lap_ket_noi["lidar"] == "on":
            if AGVConfig.loai_lidar != "usb":
                scan_1, check_1 = self.lidar_1.get_data()
                scan_2, check_2 = self.lidar_2.get_data()
            else:
                scan, check = self.lidar.return_data()
                self.lidar.time_close = time.time()
                scan_1 = scan
                scan_2 = scan
                check_1 = check
                check_2 = check
        else:
            scan_1 = np.array([[0, 0, 0]])
            scan_2 = np.array([[0, 0, 0]])
            check_1 = False
            check_2 = False
        scan_1 = np.array(scan_1)
        scan_2 = np.array(scan_2)
        return scan_1, scan_2, check_1, check_2
    def mix_data_lidar(self, ten_lidar, huong_agv_khong_icp, lidar1_orient_deg, lidar2_orient_deg, chieu_ngang_xe, chieu_doc_xe, scaling_factor = 1):
        if AGVConfig.thiet_lap_ket_noi["lidar"] == "on":
            scan_alpha_1, scan_alpha_2, _, _ = self.load_data_lidar()
        else:
            if self.stt_scan < len(AGVConfig_2.list_data0) - 1:
                self.stt_scan = self.stt_scan + 1
                self.stt_scan = 10

                scan_alpha_1 = np.load(cfg.PATH_FOLDER_SCAN_DATA_0 + "/scan_"+ str(self.stt_scan) +".npy")
                scan_alpha_2 = np.load(cfg.PATH_FOLDER_SCAN_DATA_1 + "/scan_"+ str(self.stt_scan) +".npy")
                # scan_alpha_2 = np.array([[0, 0, 0]])
            else:
                self.stt_scan = self.stt_scan - 1
                scan_alpha_1 = np.load(cfg.PATH_FOLDER_SCAN_DATA_0 + "/scan_"+ str(self.stt_scan) +".npy")
                scan_alpha_2 = np.load(cfg.PATH_FOLDER_SCAN_DATA_1 + "/scan_"+ str(self.stt_scan) +".npy")
        
        
        # scan_alpha_2 = np.array([])
        scan_xy, scan1, scan2, huong_agv_toa_do_xyz = convert_2_lidar.convert_scan_lidar(scan1_data_example=scan_alpha_1, 
                                                                    scan2_data_example=scan_alpha_2, 
                                                                    scan_an_toan = huong_agv_khong_icp,
                                                                    ten_lidar = ten_lidar,
                                                                    scaling_factor = scaling_factor,
                                                                    lidar1_orient_deg = lidar1_orient_deg,
                                                                    lidar2_orient_deg = lidar2_orient_deg,
                                                                    agv_w=chieu_ngang_xe,
                                                                    agv_l=chieu_doc_xe)
        return scan_xy, scan1, scan2, huong_agv_toa_do_xyz
    

class Loa_pin_music_esp32:
    def __init__(self):
        self.time_loa = 0
        # "thiet_lap_ket_noi": {"lidar": "off", "driver_motor": "off", "esp32": "off", "music": "off", "pin": "off", "process_lidar": "on"},
        if AGVConfig.thiet_lap_ket_noi["esp32"] == "on":
            try:
                threading.Thread(target=ket_noi_esp_loa.python_esp32).start()
            except OSError as e:
                print("error 44")
                pass

        if AGVConfig.thiet_lap_ket_noi["pin"] == "on":
            try:
                threading.Thread(target=phan_tram_pin.python_serial).start()
            except OSError as e:
                print("error 45")
                pass

    def loop(self):
        # kiểm tra an toàn
        music.data["vung_3"] = 0
        music.data["vung_2"] = 0
        music.data["vung_1"] = 0
        if AGVConfig_2.loi_an_toan == "vung_3":
            music.data["vung_3"] = 1 
        if AGVConfig_2.loi_an_toan == "vung_2":
            music.data["vung_2"] = 1 
        if AGVConfig_2.loi_an_toan == "vung_1":
            music.data["vung_1"] = 1 

        if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False:
            music.void_loop_sound_speak()
        music.check_connect()
        music.xu_ly_du_lieu()
        ket_noi_esp_loa.check_connect()
        phan_tram_pin.check_connect()
        AGVConfig.phan_tram_pin = phan_tram_pin.phan_tram_pin

        # Xử lý yêu cầu bật loa từ giao diện web
        if AGVConfig.loa_state == 1:
            AGVConfig.loa_state = 0  # Reset lại cờ ngay sau khi xử lý
            # ket_noi_esp_loa.py_sent_esp("data#" + str(int("100001000", 2)) + "#" + str(self.on_off_24v) + "#" + str(self.phanh_agv) + "#")
            ket_noi_esp_loa.py_sent_esp("data#" + str(int("100010000", 2)) + "#0")
            print("bat_loa")
            self.time_loa = time.time()
        
        if self.time_loa != 0:
            if time.time() - self.time_loa > 7:
                # ket_noi_esp_loa.py_sent_esp("data#" + str(int("100000000", 2)) + "#" + str(self.on_off_24v) + "#" + str(self.phanh_agv) + "#")
                ket_noi_esp_loa.py_sent_esp("data#" + str(int("100000000", 2)) + "#0")
                self.time_loa = 0
        # gửi tín hiệu ngắt nguồn 
        if AGVConfig.tat_phan_mem == True:
            ket_noi_esp_loa.py_sent_esp("off_24v#0")
            if ket_noi_esp_loa.gui_off_24v_thanh_cong == 1:
                # self.tat_nguon()
                AGVConfig_2.gui_tat_nguon_thanh_cong = True
        self.music_an_toan_pin()

    def music_an_toan_pin(self):
        phan_tram_pin_min = AGVConfig_2.phan_tram_pin_can_di_sac
        if AGVConfig.phan_tram_pin < phan_tram_pin_min and phan_tram_pin.check_pin == True:
            music.data["het_pin"] = 1
        else:
            music.data["het_pin"] = 0
            if AGVConfig_2.loi_an_toan == "vung_1" and music.data["da_den_dich"] == 0:
                music.data["vung_1"] = 1
            else:
                music.data["vung_1"] = 0
            if AGVConfig_2.loi_an_toan == "vung_2" and music.data["da_den_dich"] == 0:
                music.data["vung_2"] = 1
            else:
                music.data["vung_2"] = 0
            if AGVConfig_2.loi_an_toan == "vung_3" and music.data["da_den_dich"] == 0:
                music.data["vung_3"] = 1    
            else:
                music.data["vung_3"] = 0
    # def tat_nguon(self):
    #     if self.kiem_tra_connect["lidar"] == "on":
    #         if self.loai_lidar != "usb":
    #             # self.lidar_sick.disconnect()
    #             self.lidar_1.connect = False
    #             self.lidar_2.connect = False
    #         else:
    #             self.lidar.disconnect()
    #     self.connect_while = False
    #     music.stop() 
    #     # if os.name == "posix":
    #     #     ham_chuc_nang.shutdown() # Killed


class def_scan:
    def __init__(self):
        pass

    def filter_exclusion_zones(self, points, exclusion_zones):
        """
        Lọc bỏ các điểm nằm trong các vùng loại trừ.
        
        Args:
            points (np.ndarray): Mảng các điểm lidar (N, 2) hoặc (N, 3).
            exclusion_zones (list): Danh sách các vùng loại trừ, mỗi vùng là [xmin, xmax, ymin, ymax].

        Returns:
            np.ndarray: Mảng các điểm đã được lọc.
        """
        if not exclusion_zones or points.size == 0:
            return points
        
        mask = np.ones(len(points), dtype=bool)
        # vung_loai_bo_x1y1x2y2
        for (xmin, ymin, xmax, ymax) in exclusion_zones:
            mask &= ~((points[:, 0] >= xmin) & (points[:, 0] <= xmax) & (points[:, 1] >= ymin) & (points[:, 1] <= ymax))
        return points[mask]
    
    def filter_inclusion_zones(self, points, inclusion_zones):
        """
        Lọc để chỉ giữ lại các điểm nằm trong một hoặc nhiều vùng chỉ định.

        Args:
            points (np.ndarray): Mảng các điểm lidar (N, 2) hoặc (N, 3).
            inclusion_zones (list): Danh sách các vùng cần giữ lại, mỗi vùng là [xmin, xmax, ymin, ymax].

        Returns:
            np.ndarray: Mảng các điểm đã được lọc (chỉ chứa các điểm trong các vùng).
        """
        if not inclusion_zones or points.size == 0:
            return np.array([]) # Trả về mảng rỗng nếu không có vùng hoặc không có điểm

        # Bắt đầu với một mặt nạ toàn False
        final_mask = np.zeros(len(points), dtype=bool)
        
        # Duyệt qua từng vùng và "OR" các mặt nạ lại với nhau
        for (xmin, ymin, xmax, ymax) in inclusion_zones:
            zone_mask = (points[:, 0] >= xmin) & (points[:, 0] <= xmax) & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
            final_mask |= zone_mask
            
        return points[final_mask]


    


