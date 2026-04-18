import numpy as np
import config_2 as cfg
from config import AGVConfig
from config_2 import AGVConfig_2
import os
import time
import ham_chuc_nang
from libs_lidar import scan_an_toan, driver_control_input, detect_gicp
from ham_logic import angle_and_distance as ad 
import connect_driver






class xu_ly_du_lieu_lidar:
    def __init__(self):
        self.ten_lidar = "duoi"
        # Hướng của agv, dùng để chuyển hệ tọa độ Lidar về hệ tọa độ chung, dùng để xác định vùng an toàn [sig, ang, dis]
        self.huong_agv_khong_icp = np.array([[1, 29.30319303, 892.55780885]])
        # góc xoay lidar 1 và lidar 2 so với phương thẳng đứng của AGV, tính theo chiều kim đồng hồ
        self.lidar1_orient_deg = 45
        self.lidar2_orient_deg = -136
        #{"lidar": "off", "driver_motor": "off", "esp32": "off", "music": "off", "pin": "off", "process_lidar": "on"}
        # thiet_lap_ket_noi = data_setting["thiet_lap_ket_noi"]
        if AGVConfig.thiet_lap_ket_noi["esp32"] == "on":
            load_data_esp = 1
        else:
            load_data_esp = 0
        self.xac_dinh_vi_tri_xe = 0
        self.time_start_process_lidar = 0

        self.sent_data_driver_motor = {"stop": 0, "scan": np.array([]), "resolution_mm": 0}

        # {"lidar": "off", "driver_motor": "off", "esp32": "off", "music": "off", "pin": "off", "process_lidar": "on"}
        

        self.du_lieu_lidar = ham_chuc_nang.du_lieu_lidar_scan()
        self.scan_vat_can = scan_an_toan.kiem_tra_vat_can()
        self.detect_data_driver = driver_control_input.detect_data_sent_driver()
        self.def_scan = ham_chuc_nang.def_scan()
        self.driver_motor = connect_driver.sent_data_driver()
        self.thiet_bi_ngoai_vi = ham_chuc_nang.Loa_pin_music_esp32()

        load_path = None
        map_to_load = AGVConfig.ten_ban_do
        if map_to_load:
            # Bỏ phần mở rộng file nếu có
            map_name_no_ext, _ = os.path.splitext(map_to_load)
            load_path = os.path.join(cfg.path_map_folder, map_name_no_ext)
            print(f"Yêu cầu tải bản đồ: {load_path}")

        self.detect_gicp_lidar = detect_gicp.FastLidarMapper(load_from_path=load_path)
        
    

    def loop(self):
        # điều khiển bằng bàn phím 
        self.dk_ban_phim()
        self.thiet_bi_ngoai_vi.loop()
        
        # lấy dữ liệu Lidar để xử lý
        # t1 = time.time()
        scan_xy, scan1, scan2, huong_agv_toa_do_xyz = self.du_lieu_lidar.mix_data_lidar(self.ten_lidar, self.huong_agv_khong_icp, 
                                                                                        self.lidar1_orient_deg, self.lidar2_orient_deg, 
                                                                                        AGVConfig.chieu_ngang_xe, AGVConfig.chieu_doc_xe)

        if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False:
            AGVConfig_2.loi_an_toan, scan_an_toan = self.kiem_tra_an_toan_lidar(scan_xy, huong_agv_toa_do_xyz)
        else:
            AGVConfig_2.loi_an_toan = ""
            scan_an_toan = np.array([])
        # print(scan_an_toan)
        # AGVConfig.danh_sach_diem_vat_can = scan_an_toan
        

        # detect_gicp - tải lại bản đồ nếu cần
        if AGVConfig.cap_nhat_ban_do_moi == True:
            AGVConfig.cap_nhat_ban_do_moi = False
            path_nang_xe_hang = os.path.join(cfg.path_map_folder, AGVConfig.ten_ban_do)
            self.detect_gicp_lidar.load_state(path_nang_xe_hang)
        # lưu bản đồ
        if AGVConfig.lenh_luu_ban_do == 1 and AGVConfig.ten_ban_do_moi != "":  
            self.detect_gicp_lidar.save_state(cfg.path_map_folder + "/" + AGVConfig.ten_ban_do_moi)
            print("Lưu bản đồ thành công!", cfg.path_map_folder + "/" + AGVConfig.ten_ban_do_moi)
            AGVConfig.lenh_luu_ban_do = 0


        # # xử lý dữ liệu lidar kết hợp với detect_gicp
        if AGVConfig.thiet_lap_ket_noi["process_lidar"] == "on" and scan_xy.shape[0] != 0:
            stop_agv, scan_all, pts_world, rmse, center_px, resolution_mm, \
                            goc_agv_driver_control = self.xu_ly_du_lieu_lidar(scan_xy, AGVConfig.vung_loai_bo_x1y1x2y2, AGVConfig_2.vung_xe, scan_an_toan)
            # gọi hàm tìm đường đi driver_control_input
            driver_motor_quay_trai, driver_motor_quay_phai, data_return = self.detect_data_driver.void_loop(stop_agv, center_px, resolution_mm, goc_agv_driver_control)
            # cập nhật để điều khiển motor
            if driver_motor_quay_trai is not None:
                self.driver_motor.quay_trai = driver_motor_quay_trai
            if driver_motor_quay_phai is not None:
                self.driver_motor.quay_phai = driver_motor_quay_phai
            # print(f"data_return: {data_return}")
            self.driver_motor.load_data_sent_drive(data_return)

            if AGVConfig.motor_state == True:
                self.driver_motor.check_connect()
            self.driver_motor.ket_noi_lai()

            

        
    def xu_ly_du_lieu_lidar(self, scan_xy, vung_loai_bo, vung_xe, scan_an_toan):
        if AGVConfig_2.distan_scan_all[0] == 1: # giới hạn khoảng cách quét của lidar 
            x_coords = scan_xy[:, 0]
            y_coords = scan_xy[:, 1]
            distances = np.sqrt(x_coords**2 + y_coords**2)
            scan_all = scan_xy[distances < AGVConfig_2.distan_scan_all[1]]
        else:
            scan_all = scan_xy
        
        # lọc lần 2 (lấy các điểm trong vùng loại bỏ hoặc ngoài vùng loại bỏ)
        if AGVConfig.kiem_tra_vi_tri_xe == False:
            # Lọc các điểm trong vùng loại bỏ
            scan_all = self.def_scan.filter_exclusion_zones(scan_all, vung_loai_bo)
        else:
            # chỉ dùng để hiển thị các điểm trong vùng xe để theo dõi các chân xe linh kiện (là nút kiểm tra vị trí xe)
            scan_all = self.def_scan.filter_inclusion_zones(scan_all, vung_xe)

        if scan_all.shape[0] == 0:
            return
        
        px = scan_xy[:,0]
        py = scan_xy[:,1]
        arr_test = np.vstack((px, py, np.zeros_like(px))).T  # Thêm chiều z = 0 để tạo PointCloud 3D

        if self.time_start_process_lidar == 0:
            self.time_start_process_lidar = time.time()
        stop_agv = 0
        # cap_nhat_vi_tri_agv = {"toa_do": [0, 0], "huong": 0, "update_tam_thoi": False, "update": False}
        tam_x_pixel = AGVConfig.cap_nhat_vi_tri_agv["toa_do"][0]
        tam_y_pixel = AGVConfig.cap_nhat_vi_tri_agv["toa_do"][1]
        goc_agv = AGVConfig.cap_nhat_vi_tri_agv["huong"]
        resolution_mm = 20
        center_px = [0, 0]
        rmse = 1000
        goc_agv_driver_control = 0
        pts_world = None
        if time.time() - self.time_start_process_lidar > 1:
            if AGVConfig.che_do_tao_ban_do and not AGVConfig.trang_thai_tam_dung_tao_ban_do:
                cap_nhat_ban_do = 1 # giống nút w lúc trước nhấn w thì cập nhật bản đồ, là add_all_point lúc trước
            else:
                cap_nhat_ban_do = 0
            if AGVConfig.cap_nhat_vi_tri_agv["update_tam_thoi"] == True and AGVConfig.cap_nhat_vi_tri_agv["toa_do"] != [0, 0]:
                cap_nhat_vi_tri = 1
                stop_agv = 1
            else:
                cap_nhat_vi_tri = 0
            # print(f"cap_nhat_vi_tri: {cap_nhat_vi_tri}, cap_nhat_ban_do: {cap_nhat_ban_do}, cap_nhat_vi_tri_agv: {AGVConfig.cap_nhat_vi_tri_agv}")
            # print(f"scan_all shape: {scan_all.shape}, arr_test shape: {arr_test.shape}")
            # print(f"tam_x_pixel: {tam_x_pixel}, tam_y_pixel: {tam_y_pixel}, goc_agv: {goc_agv}")
            # print(f"cap nhat vi tri: {cap_nhat_vi_tri}, cap nhat ban do: {cap_nhat_ban_do}, update_all_point_in_map: {AGVConfig.update_all_point_in_map}")
            if arr_test.shape[0] > 1:
                center_px, pts_world, rmse, tam_x_mm, tam_y_mm, tam_x_pixel, \
                    tam_y_pixel, goc_agv, resolution_mm, AGVConfig.danh_sach_diem_vat_can = self.detect_gicp_lidar.process_scan(arr_test, integrate=True, 
                                                                                x_agv_pixel = tam_x_pixel, y_agv_pixel = tam_y_pixel, 
                                                                                goc_agv_do = goc_agv, cap_nhat_vi_tri = cap_nhat_vi_tri,
                                                                                cap_nhat_ban_do = cap_nhat_ban_do, 
                                                                                update_all_point_in_map = AGVConfig.update_all_point_in_map,
                                                                                scan_an_toan = scan_an_toan)
                goc_agv_rad = ad.normalize_rad(goc_agv) # Chuẩn hóa một góc về khoảng [-180, 180] độ. goc_agv đầu ra là rad rồi
                AGVConfig.huong_agv_do_thuc_rad = goc_agv_rad
                AGVConfig.toa_do_agv_mm = [tam_x_mm, tam_y_mm]
                AGVConfig.toa_do_agv_pixel = [tam_x_pixel, tam_y_pixel]
                AGVConfig.huong_agv_do_img = (np.pi/2 - goc_agv_rad) * 180 / np.pi # dùng để hiển thị góc agv trên web
                goc_agv_driver_control = goc_agv_rad - np.pi/2 # chuyển góc về hệ tọa độ của driver_control_input
                goc_agv_driver_control = ad.normalize_rad(goc_agv_driver_control) # Chuẩn hóa một góc về khoảng [-180, 180] độ.

                # print(f"AGVConfig.huong_agv_do_thuc_rad: {AGVConfig.huong_agv_do_thuc_rad}, \
                #       AGVConfig.toa_do_agv_mm: {AGVConfig.toa_do_agv_mm}, \
                #       AGVConfig.toa_do_agv_pixel: {AGVConfig.toa_do_agv_pixel}, AGVConfig.huong_agv_do_img: {AGVConfig.huong_agv_do_img}")
            else:
                stop_agv = 1
        return stop_agv, scan_all, pts_world, rmse, center_px, resolution_mm, goc_agv_driver_control


                


    def kiem_tra_an_toan_lidar(self, scan_xy, huong_agv_toa_do_xyz):
        di_thuan_nguoc = self.detect_data_driver.di_thuan_nguoc
        dang_re = self.detect_data_driver.dang_re
        # loai_bo_coc_xe = {"che_do_lay_mau": 0,"ten_vung_loai_bo": ten_vung_loai_bo_last, "luu_vung_loai_bo": False, "update": 0}
        che_do_lay_mau = AGVConfig.loai_bo_coc_xe["che_do_lay_mau"]
        di_chuyen_luon = self.detect_data_driver.di_chuyen_luon
        xac_dinh_vi_tri_xe = self.xac_dinh_vi_tri_xe
        loi_an_toan, scan_an_toan = self.scan_vat_can.detect(scan_xy, huong_agv_toa_do_xyz, di_thuan_nguoc, dang_re, che_do_lay_mau, xac_dinh_vi_tri_xe, di_chuyen_luon)

        return loi_an_toan, scan_an_toan
    

    def dk_ban_phim(self):
        van_toc_max_tien = AGVConfig.van_toc_tien_max
        if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == True:
            # dieu_khien_agv = {"dieu_khien_thu_cong": False, "tien": 0, "lui": 0, "trai": 0, "phai": 0}
            # stop nếu tất cả == 0
            # print(AGVConfig.dieu_khien_agv)
            if AGVConfig.dieu_khien_agv["tien"] == 1 and AGVConfig.dieu_khien_agv["trai"] == 0 and AGVConfig.dieu_khien_agv["phai"] == 0 and AGVConfig.dieu_khien_agv["lui"] == 0: # tiến
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = van_toc_max_tien)
                pass
            elif AGVConfig.dieu_khien_agv["trai"] == 1 and AGVConfig.dieu_khien_agv["tien"] == 0 and AGVConfig.dieu_khien_agv["lui"] == 0: # rẽ trái
                self.driver_motor.sent_data_controller(vt_trai = -int(van_toc_max_tien/6), vt_phai = int(van_toc_max_tien/6))
                # print(-int(van_toc_max_tien/6), int(van_toc_max_tien/6))
            elif AGVConfig.dieu_khien_agv["phai"] == 1 and AGVConfig.dieu_khien_agv["tien"] == 0 and AGVConfig.dieu_khien_agv["lui"] == 0: # rẽ phải
                self.driver_motor.sent_data_controller(vt_trai = int(van_toc_max_tien/6), vt_phai = -int(van_toc_max_tien/6))
            elif AGVConfig.dieu_khien_agv["trai"] == 1 and AGVConfig.dieu_khien_agv["tien"] == 1 and AGVConfig.dieu_khien_agv["lui"] == 0: # dịch trái
                self.driver_motor.sent_data_controller(vt_trai = int((van_toc_max_tien/4)*3), vt_phai = van_toc_max_tien)
                # print("dich_trai")
            elif AGVConfig.dieu_khien_agv["phai"] == 1 and AGVConfig.dieu_khien_agv["tien"] == 1 and AGVConfig.dieu_khien_agv["lui"] == 0: # dịch phải
                self.driver_motor.sent_data_controller(vt_trai = van_toc_max_tien, vt_phai = int((van_toc_max_tien/4)*3))
                # print("dich_phai")

            elif AGVConfig.dieu_khien_agv["lui"] == 1 and AGVConfig.dieu_khien_agv["trai"] == 0 and AGVConfig.dieu_khien_agv["phai"] == 0:
                self.driver_motor.sent_data_controller(vt_trai = -van_toc_max_tien, vt_phai = -van_toc_max_tien)
            elif AGVConfig.dieu_khien_agv["trai"] == 1 and AGVConfig.dieu_khien_agv["lui"] == 1: # lùi dịch trái
                self.driver_motor.sent_data_controller(vt_trai = -van_toc_max_tien, vt_phai =-int((van_toc_max_tien/4)*3))
            elif AGVConfig.dieu_khien_agv["phai"] == 1 and AGVConfig.dieu_khien_agv["lui"] == 1: # lùi dịch phải
                self.driver_motor.sent_data_controller(vt_trai =  -int((van_toc_max_tien/4)*3), vt_phai = -int(van_toc_max_tien))
            else:
                self.driver_motor.sent_data_controller(vt_trai = 0, vt_phai = 0)