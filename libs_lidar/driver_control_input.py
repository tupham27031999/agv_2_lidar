
from libs_ngoai_vi import music
from ham_logic import tim_duong_di, tinh_luong_giac, angle_and_distance, angle_and_distance as ad
import numpy as np
from libs_lidar import scan_an_toan
import math
import time
from libs_ngoai_vi import ket_noi_esp_loa
import config_2 as cfg
from config import AGVConfig
from config_2 import AGVConfig_2

class detect_data_sent_driver:
    def __init__(self):
        music.name_music = "none"
        self.convert_data_run_agv0 = {"run_diem": "NG", "run_huong": "NG", "run_huong_2": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        ########################################################################################
        self.angle = 0                      # xu_ly_tin_hieu - ok
        self.angle_min = 3
        self.distance = 0                   # xu_ly_tin_hieu - ok
        self.check_angle_distance = 0       # xu_ly_tin_hieu - ok
        self.di_cham = 0                    # load_data_process - ok
        self.a_v = 800
        self.dang_re = 0                    # xu_ly_tin_hieu - ok
        self.tien_rl = 0
        self.v_tien_max = 0                 # load_data_web - ok
        self.v_re_max = 0                   # load_data_web - ok

        
        self.toa_do_diem_dau = [] # dùng để tạo 
        self.toa_do_cu_1 = []
        self.toa_do_cu_2 = []
        self.toa_do_diem_dich = [] # dùng để tạo self.point_old, kiểm tra vị trí điểm 1 so với point_old
        self.toa_do_diem_kiem_soat = []
        self.toa_do_diem_cong_1 = []
        self.toa_do_diem_cong_2 = []
        self.toa_do_diem_huong_agv = []
        self.done_tin_hieu_tam_thoi = 0
        self.robot_direction = []
        self.robot_direction_nguoc = []
        self.stop = 0
        self.convert_data_run_agv = {"run_diem": "NG", "run_huong": "NG", "run_huong_2": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        self.distance_old = 1000

        self.ten_diem_bat_dau = ""
        self.di_thuan_nguoc = 0

        self.nang_ha = ""
        

        self.xoay_goc = 1
        self.xac_dinh_vi_tri_xe = 0
        self.so_lan_check_vi_tri_xe = 0
        self.check_nang_ha = 0

        self.dung_hoat_dong = 0  # nhấn rest --> start
        self.time_reset_dung_hoat_dong = 0
        
        self.thong_tin_nang_ha_0 = {"che_do_nang_ha": None, "diem_nang_ha": None, "diem_huong": None, 
                                    "trang_thai": None, "0_song_song_1_vuong_goc": 0, "da_nang_xong": 0, "da_ha_xong": 0, 
                                    "xac_dinh_huong": False, "xac_dinh_huong_xong": False,
                                    "kiem_tra_xong_vi_tri_xe": False, "da_hoan_thanh": False,
                                    "toa_do_tam_thoi": []}
        self.thong_tin_nang_ha = self.thong_tin_nang_ha_0.copy()


        self.hoan_thanh_den_vi_tri_dich = False
        self.dich_den_tin_hieu_nhan = ""
        self.trang_thai_tin_hieu_nhan = ""
        self.vi_tri_nhan_tin_hieu_nhan = ""
        self.list_data = None
        self.stt_list_data = 1
        self.data_thong_tin_nang_ha = None
        self.tin_hieu_nhan_step = {}

        self.vung_loai_bo_co_xe = 0
        self.setup_vung_loai_bo = 0

        self.di_chuyen_luon0 = {"update": 0, 
                                "co_huong": 0, 
                                "khong_huong": 0, 
                                "van_toc_di_chuyen_luon": None, 
                                "van_toc_min": None, 
                                "khoang_nhin_phia_truoc": None, 
                                "delta": None, 
                                "khong_xoay": None}
        self.di_chuyen_luon = self.di_chuyen_luon0.copy()

        self.duong_di_ly_tuong = []
        self.distance_dich_ly_tuong = None
        self.toa_do_dich_ly_tuong = []
        self.den_diem_gan_nhat = False


        self.test_tranh_nhau = 1
        self.debug_stop = 0
        self.loi_paths_web = 0
        self.time_vung_loai_bo = time.time()

        self.stt_test = 0


        # tab code
        self.name_music_old_code = None
        self.thoi_gian_bat_dau_tam_dung = None
        
    # stop_agv == 1 do cập nhật vị trí hoặc số điểm quét quá ít hoặc nhấn stop hoặc cảm biến va chạm tác động
    def void_loop(self, stop_agv, center_px, resolution_mm, goc_agv_driver_control):
        # lấy tín hiệu từ esp32
        da_nang_xong = ket_noi_esp_loa.da_nang_xong
        da_ha_xong = ket_noi_esp_loa.da_ha_xong
        # cập nhật trạng thái nâng hạ cho tin hieu gủi điều khiển trung tâm
        if da_nang_xong == 1:
            AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai_nang_ha"] = "nang"
        if da_ha_xong == 1:
            AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai_nang_ha"] = "ha"
        AGVConfig.xy_lanh_code = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai_nang_ha"]
        
        # --------------------------------------- lỗi: cảm biến an toàn - dung_hoat_dong ---------------------------------------
        if ket_noi_esp_loa.dung_hoat_dong == 1:
            music.data["dung_hoat_dong"] = 1
            self.dung_hoat_dong = 1
        else:
            music.data["dung_hoat_dong"] = 0
            if self.dung_hoat_dong == 1:
                self.time_reset_dung_hoat_dong = time.time()
                self.dung_hoat_dong = 0
                music.data["bat_dau_di_chuyen"] = 1
            if self.time_reset_dung_hoat_dong != 0 and time.time() - self.time_reset_dung_hoat_dong > AGVConfig_2.time_reset_dung_hoat_dong:
                self.time_reset_dung_hoat_dong = 0
                music.data["bat_dau_di_chuyen"] = 0
        # dữ liệu chính
        driver_motor_quay_trai, driver_motor_quay_phai = None, None
        trang_thai = ""
        data_return = None
        # print(f"AGVConfig.dieu_khien_agv = {AGVConfig.dieu_khien_agv}")
        # print(AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False)
        # print(f"toa_do_hien_tai", AGVConfig.toa_do_agv_mm) # toa_do_hien_tai [np.float64(-16068.8015275983), np.float64(-3829.197552147558)]

        if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False:
            # load các giá trị tọa độ góc của agv, check an toàn
            self.load_data_process()

            if AGVConfig.run_state == 0 or self.dung_hoat_dong == 1 or self.time_reset_dung_hoat_dong != 0:
                stop_agv = 1
            # load các giá trị điểm đầu điểm đích, ... từ web, sau đó tính các giá trị góc, khoảng cách yêu cầu
            if stop_agv == 1:
                self.stop = 1
            else:
                driver_motor_quay_trai, driver_motor_quay_phai, trang_thai = self.xu_ly_tin_hieu(center_px, resolution_mm, goc_agv_driver_control, da_nang_xong, da_ha_xong) 
            
            
            self.v_tien_max = AGVConfig.van_toc_tien_max
            print("AGVConfig.van_toc_tien_max_code", AGVConfig.van_toc_tien_max_code)
            if AGVConfig.van_toc_tien_max_code is not None:
                self.v_tien_max = AGVConfig.van_toc_tien_max_code

            self.v_re_max = AGVConfig.van_toc_re_max
            if AGVConfig.van_toc_re_max_code is not None:
                self.v_re_max = AGVConfig.van_toc_re_max_code

            if self.dang_re == 1:
                AGVConfig.dang_re_code = True
            else:
                AGVConfig.dang_re_code = False

            if self.di_thuan_nguoc == 0:
                AGVConfig.di_thuan_nguoc_code = "thuan"
            else:
                AGVConfig.di_thuan_nguoc_code = "nguoc"


            # xử lý dữ liệu khi chạy
            self.luu_toa_do_cu() 
            data_return = {"v_tien_max": self.v_tien_max,
                        "v_re_max": self.v_re_max,
                        "toa_do_diem_dau": self.toa_do_diem_dau,
                        "toa_do_diem_dich": self.toa_do_diem_dich,
                        "toa_do_diem_huong_agv": self.toa_do_diem_huong_agv,
                        "toa_do_diem_kiem_soat": self.toa_do_diem_kiem_soat,
                        "angle": self.angle,
                        "distance": self.distance,
                        "check_angle_distance": self.check_angle_distance,
                        "stop": self.stop,
                        "dang_re": self.dang_re,
                        "di_thuan_nguoc": self.di_thuan_nguoc,
                        }
        else:
            if AGVConfig.dieu_khien_agv["ha_xe"] == 1 and da_ha_xong != 1: # 5
                ket_noi_esp_loa.py_sent_esp("nang_ha#ha#0")
                music.data["dang_ha_hang"] = 1
                if self.debug_stop == 1:
                    print("---- stop do webserver gửi tín hiệu nâng hạ xe ----")
            else:
                music.data["dang_ha_hang"] = 0
                
            if AGVConfig.dieu_khien_agv["nang_xe"] == 1 and da_nang_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#nang#0")
                music.data["dang_nang_hang"] = 1
            else:
                music.data["dang_nang_hang"] = 0

        
        # nang, ha, lay_hang, tra_hang, xoay_agv_song_song, xoay_agv_vuong_goc, nghi hoặc ""
        danh_sach_vi_tri_xe_lk = []
        for key, value in AGVConfig.thong_tin_lay_tra_hang.items():
            danh_sach_vi_tri_xe_lk.append(value[0])
        trang_thai_agv_gui_dktt = ""
        # IDLE,       // Nghỉ / Chờ lệnh
        # PICKING,    // Đang lấy hàng
        # DROPPING,   // Đang trả hàng
        # CHARGING,   // Đang sạc
        # BLOCKED,    // Bị vật cản - tạm thời chưa dùng
        # ERROR,      // Lỗi kỹ thuật
        # OFFLINE     // Mất kết nối
        
        if trang_thai != "" or AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] == "":
            if self.hoan_thanh_den_vi_tri_dich == True:
                trang_thai_agv_gui_dktt = "IDLE" # đợi lệnh có thể là đã ở đích chờ lệnh tiếp theo
            elif AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] == "":
                if da_nang_xong == 1:
                    trang_thai_agv_gui_dktt = "ERROR" # lỗi do đang nâng mà lại không có đích đến vì mới mở phần mềm auto hạ 
                    # print("lỗi do đang nâng mà lại không có đích đến vì mới mở phần mềm auto hạ ")
                else:
                    trang_thai_agv_gui_dktt = "IDLE" # đang không làm gì nên chờ lệnh
            else:
                if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] in danh_sach_vi_tri_xe_lk:
                    trang_thai_agv_gui_dktt = "DROPPING" # đang trả hàng
                else:
                    trang_thai_agv_gui_dktt = "PICKING" # đang lấy hàng
        else:
            trang_thai_agv_gui_dktt = "ERROR" # có thể đang lỗi hoặc đang điều khiển thủ công tạm hiểu là lỗi hoặc chưa thiết lập đường đi
            # print("có thể đang lỗi hoặc đang điều khiển thủ công tạm hiểu là lỗi hoặc chưa thiết lập đường đi")
        if AGVConfig.sac_pin == True:
            trang_thai_agv_gui_dktt = "CHARGING" # đang sạc

        AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai_agv_gui"] = trang_thai_agv_gui_dktt

        return driver_motor_quay_trai, driver_motor_quay_phai, data_return


    def xu_ly_tin_hieu(self, center_px, resolution_mm, goc_agv_driver_control, da_nang_xong, da_ha_xong):
        # phát nhạc theo yêu cầu tab code
            # dừng tên nhạc cũ
        if self.name_music_old_code is not None:
            if AGVConfig.music_name_code != self.name_music_old_code:
                music.data[self.name_music_old_code] = 0
                self.name_music_old_code = None
            # phát nhạc mới
        if AGVConfig.music_name_code is not None:
            self.name_music_old_code = AGVConfig.music_name_code
            if AGVConfig.music_name_code in music.data:
                if music.data[AGVConfig.music_name_code] == 0:
                    music.data[AGVConfig.music_name_code] = 1
        
        # cài thời gian bắt đầu tính tạm dừng
        if AGVConfig.dung_trong_giay_code is not None:
            if self.thoi_gian_bat_dau_tam_dung is None:
                self.thoi_gian_bat_dau_tam_dung = time.time()
        else:
            self.thoi_gian_bat_dau_tam_dung = None


        # cập nhật biến quay trái quay phải của driver_motor
        driver_motor_quay_trai = None
        driver_motor_quay_phai = None
        trang_thai = ""
        
        stop = 0

        # ----------------------------------------------- stop ------------------------------------------------
        # điều khiển trung tâm gửi đến lệnh stop
        if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["stop"] == True:
            stop = 1
            if self.debug_stop == 1:
                print("---- stop do webserver gửi tín hiệu stop ----")
        # stop do tab code
        if AGVConfig.dung_trong_giay_code is not None:
            if self.thoi_gian_bat_dau_tam_dung is not None:
                stop = 1
                if self.debug_stop == 1:
                    print("---- stop do tab code ----")
                if time.time() - self.thoi_gian_bat_dau_tam_dung > AGVConfig.dung_trong_giay_code:
                    self.thoi_gian_bat_dau_tam_dung = None
                    AGVConfig.dung_trong_giay_code = None

        # dừng đợi lệnh resume AGVConfig.stop_code_resume
        if AGVConfig.stop_code_resume == True:
            stop = 1
            if self.debug_stop == 1:
                print("---- stop do stop_code_resume ----")
        # dừng do đang nâng hạ
        # kiểm tra nâng hạ của tab code
        nang_ha = None
        if AGVConfig.nang_ha_xe_code is not None:
            if AGVConfig.nang_ha_xe_code == "nang":
                nang_ha = "nang"
            if AGVConfig.nang_ha_xe_code == "ha":
                nang_ha = "ha"
        if nang_ha is not None:
            stop = 1
            if self.debug_stop == 1:
                print("---- stop do nang_ha_xe_code ----")
            if nang_ha == "nang" and da_nang_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#nang#0")
                music.data["dang_nang_hang"] = 1
            else:
                music.data["dang_nang_hang"] = 0

            if nang_ha == "ha" and da_ha_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#ha#0")
                music.data["dang_ha_hang"] = 1
            else:
                music.data["dang_ha_hang"] = 0    
        # --------------------------------------------
        # điều kiện để vào if này là đã nhận được tín hiệu điểm đích mới từ webserver, có điểm đích đến, đang ở trạng thái run, và điểm đích đến không phải là "None"
        if AGVConfig.tin_hieu_nhan != {} and AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "" and AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "None":
            # nếu chưa có vị trí hiện tại của agv
            diem_dau, khoang_cach = self.tim_diem_gan_nhat(AGVConfig.danh_sach_diem, center_px, resolution_mm)
            if khoang_cach <= 600:
                if self.vi_tri_nhan_tin_hieu_nhan == "":
                    self.vi_tri_nhan_tin_hieu_nhan = diem_dau
                    AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["vi_tri_hien_tai"] = diem_dau
            
            # nếu có vị trí hiện tại và vị trí đích thì tìm đường đi
            if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["vi_tri_hien_tai"] != "":
                
                # print("graph = ", graph)
                # tìm danh sách đường đi
                p_actual = []
                if len(AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]) == 0:
                    graph = tim_duong_di.tao_graph()
                    if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["diem_tiep_theo"] == "":
                        start_node = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["vi_tri_hien_tai"]
                    else:
                        start_node = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["diem_tiep_theo"]
                    goal_node = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"]

                    obstacles_points = []
                    obstacles_paths = []

                    # print(f"Tìm đường từ {start_node} -> {goal_node} với vật cản điểm {obstacles_points} và đường {obstacles_paths}")
                    p_ideal, c_ideal, p_actual, c_actual, c_diff = tim_duong_di.a_star(start_node, goal_node, vat_can_diem=obstacles_points, vat_can_duong=obstacles_paths)
                    p_actual, c_ideal = tim_duong_di.toi_uu_hoa_duong_di(p_actual, 5)
                    # thêm vị trí hiện tại vào đầu danh sách p_actual
                    # print(f"\nĐường đi lý tưởng: {p_ideal} (Chi phí: {c_ideal:.2f})")
                    # print(f"Đường đi thực tế: {p_actual} (Chi phí: {c_actual:.2f})")

                else:
                    # chỉ vào khi self.list_data là None, tức là chưa có dữ liệu đường đi nào được tạo ra trước đó, thì mới lấy đường đi từ webserver để tối ưu hóa lại
                    p_actual = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]
                    # print("++++++++++++++++", p_actual)




                # print("AGVConfig.danh_sach_duong_di", AGVConfig.danh_sach_duong_di)
                # ----------------------------------------------------- xử lý -------------------------------------------------------------------------
                # cập nhật danh sách đường đi nếu đến đích tiếp theo
                if (self.convert_data_run_agv["run_diem"] == "OK" and self.convert_data_run_agv["run_huong"] == "OK") or len(AGVConfig.danh_sach_duong_di) == 0:
                    if len(AGVConfig.danh_sach_duong_di) == 0:
                        AGVConfig.da_den_diem_tiep_theo_code = False
                    else:
                        AGVConfig.da_den_diem_tiep_theo_code = True

                    
                    van_toc_phan_hoi_max = max(AGVConfig.van_toc_phan_hoi_trai, AGVConfig.van_toc_phan_hoi_phai)
                    if abs(van_toc_phan_hoi_max) < 200:
                        if AGVConfig.xoay_goc_code is None and AGVConfig.xoay_goc_code is None and AGVConfig.nang_ha_xe_code is None and AGVConfig.dung_trong_giay_code is None:
                            AGVConfig.danh_sach_duong_di = p_actual
                            AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["danh_sach_duong_di"] = p_actual
                            # reset để di chuyển tiếp trên đường mới
                            driver_motor_quay_trai = 0
                            driver_motor_quay_phai = 0
                            self.distance_old = 1000
                            self.vi_tri_nhan_tin_hieu_nhan = ""
                            self.convert_data_run_agv = self.convert_data_run_agv0.copy()
                            self.dang_re = 0
                    else:
                        stop = 1
                        if self.debug_stop == 1:
                            print("stop do đến đích tiếp theo")
                else:
                    AGVConfig.da_den_diem_tiep_theo_code = False
                

                ten_diem_bat_dau = ""
                ten_diem_tiep_theo = ""
                self.hoan_thanh_den_vi_tri_dich == False
                if len(AGVConfig.danh_sach_duong_di) >= 2:
                    ten_diem_bat_dau = AGVConfig.danh_sach_duong_di[0]
                    ten_diem_tiep_theo = AGVConfig.danh_sach_duong_di[1]
                elif len(AGVConfig.danh_sach_duong_di) == 1:
                    self.hoan_thanh_den_vi_tri_dich == True

                AGVConfig.vi_tri_hien_tai_code = ten_diem_bat_dau
                AGVConfig.vi_tri_tiep_theo_code = ten_diem_tiep_theo
                AGVConfig.vi_tri_diem_cuoi_code = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"]
                AGVConfig.danh_sach_duong_di_code = AGVConfig.danh_sach_duong_di

                if self.hoan_thanh_den_vi_tri_dich == True:
                    music.data["den_dich"] = 1
                    stop = 1
                    if self.debug_stop == 1:
                        print("---- stop do đã hoàn thành đến điểm đích ----")
                else:
                    music.data["den_dich"] = 0

                
                # lấy dữ liệu điểm đầu và điểm tiếp theo để di chuyển
                if ten_diem_bat_dau != "" and ten_diem_tiep_theo != "":
                    check_angle_distance = "distance"
                    angle_deg = 0
                    self.dang_re = 0

                    AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["diem_tiep_theo"] = ten_diem_tiep_theo # bên điều khiển trung tâm sẽ lấy điểm tiếp theo này và đích để tìm đường
                    goc_dich = [AGVConfig.danh_sach_diem[ten_diem_tiep_theo][2], AGVConfig.danh_sach_diem[ten_diem_tiep_theo][3]]

                    # tọa độ điểm đầu và điểm tiếp theo
                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_bat_dau][0], AGVConfig.danh_sach_diem[ten_diem_bat_dau][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    toa_do_diem_bat_dau = [int(world_x_mm), int(world_y_mm)]

                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_tiep_theo][0], AGVConfig.danh_sach_diem[ten_diem_tiep_theo][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    toa_do_diem_tiep_theo = [int(world_x_mm), int(world_y_mm)]

                    distance = tinh_luong_giac.calculate_distance(AGVConfig.toa_do_agv_mm, toa_do_diem_tiep_theo)
                    if ten_diem_tiep_theo == AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"]:
                        AGVConfig.khoang_cach_den_dich_code = distance
                     

                    # xử lý đường cong
                    ten_duong_1 = f"{ten_diem_bat_dau}_{ten_diem_tiep_theo}"
                    ten_duong_2 = f"{ten_diem_tiep_theo}_{ten_diem_bat_dau}"
                    ten_duong_thuc_te = ten_duong_1 if ten_duong_1 in AGVConfig.danh_sach_duong else (ten_duong_2 if ten_duong_2 in AGVConfig.danh_sach_duong else None)
                    toa_do_diem_kiem_soat = []
                    if not ten_duong_thuc_te:
                        print("Đường ten_duong_1 hoặc ten_duong_2 không tồn tại trong danh sách đường.", ten_duong_1, ten_duong_2)
                        # X1_H04_C_X1-H04 H04_X1_C_H04-X1
                    else:
                        # "C11_C12": [["C11","C12"],"curve","C11-C12"]
                        if AGVConfig.danh_sach_duong[ten_duong_thuc_te][1] == "curve":
                            # 1. Lấy thông tin đường và các điểm tọa độ
                            path_info = AGVConfig.danh_sach_duong[ten_duong_thuc_te]  # [["C11","C12"],"curve","C11-C12"]
                            diem_kiem_soat = AGVConfig.danh_sach_diem[path_info[2]][:2]   # Điểm kiểm soát (lấy từ index 3 của path_info) "C11-C12"]
                            world_x_mm = (diem_kiem_soat[0] - center_px[0]) * resolution_mm
                            world_y_mm = (diem_kiem_soat[1] - center_px[1]) * resolution_mm
                            toa_do_diem_kiem_soat = [int(world_x_mm), int(world_y_mm)]

                            # Giả sử hướng hiện tại của AGV lấy từ hệ thống (đổi ra radian)
                            huong_hien_tai_rad = AGVConfig.huong_agv_do_thuc_rad
                            tam_nhin = AGVConfig_2.tam_nhin_duong_cong
                            sai_so_dich = 1
                            buoc_nhin_xa_huong = 300 # Khoảng cách cộng thêm để tìm điểm định hướng
                            toa_do_agv = AGVConfig.toa_do_agv_mm

                            # 2. GỌI HÀM TÍNH TOÁN ĐIỀU HƯỚNG
                            reached, diem_A, diem_B, diem_Sau_A, goc_target, sai_so = tim_duong_di.calculate_agv_guidance(
                                toa_do_agv, huong_hien_tai_rad, toa_do_diem_bat_dau, toa_do_diem_kiem_soat, toa_do_diem_tiep_theo, tam_nhin, sai_so_dich, buoc_nhin_xa_huong
                            )
                            
                            if len(diem_A) != 0:
                                toa_do_diem_bat_dau = [int(diem_A[0]), int(diem_A[1])]
                            else:
                                toa_do_diem_bat_dau = []
                                print("error 1")
                            if len(diem_B) != 0:
                                toa_do_diem_tiep_theo = [int(diem_B[0]), int(diem_B[1])]
                            else:
                                toa_do_diem_tiep_theo = []
                                print("error 2")


                    self.toa_do_diem_kiem_soat = toa_do_diem_kiem_soat
                    self.toa_do_diem_dau = toa_do_diem_bat_dau
                    self.toa_do_diem_dich = toa_do_diem_tiep_theo
                    self.distance = distance

                    # xác định điểm hướng cần đi đến
                    angle_rad_thuan = tinh_luong_giac.angle_with_ox(AGVConfig.toa_do_agv_mm, self.robot_direction)
                    angle_rad_nguoc = tinh_luong_giac.angle_with_ox(AGVConfig.toa_do_agv_mm, self.robot_direction_nguoc)
                    angle_rad_dich = tinh_luong_giac.angle_with_ox(self.toa_do_diem_dau, self.toa_do_diem_dich)

                    delta_deg_thuan = ad.normalize_angle((angle_rad_thuan - angle_rad_dich) * 180 / np.pi)
                    delta_deg_nguoc = ad.normalize_angle((angle_rad_nguoc - angle_rad_dich) * 180 / np.pi)

                    # Logic quyết định đi tiến hay lùi cho xe 2 đầu
                    # Nếu góc đi tiến quá lớn ( > 90 độ), thì chọn đi lùi
                    if abs(delta_deg_thuan) > 90:
                        self.di_thuan_nguoc = 1 # Đi lùi
                        angle_deg = delta_deg_nguoc
                        self.toa_do_diem_huong_agv = self.robot_direction_nguoc
                    else:
                        self.di_thuan_nguoc = 0 # Đi tiến
                        angle_deg = delta_deg_thuan
                        self.toa_do_diem_huong_agv = self.robot_direction

                    # bật nhạc sắp đến đích
                    
                    if distance <= 2000 and ten_diem_tiep_theo == AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"]:
                        khoang_cach_den_dich_code = distance

                        if distance > 100:
                            music.data["sap_den_dich"] = 1
                            music.data["da_den_dich"] = 0
                        else:
                            music.data["da_den_dich"] = 1
                            music.data["sap_den_dich"] = 0
                    else:
                        music.data["sap_den_dich"] = 0
                        music.data["da_den_dich"] = 0

                    # agv rẽ trái hoặc phải do góc quá lớn và không phải là đang đi trên đường cong
                    angle_check = 10
                    if len(self.toa_do_diem_kiem_soat) != 0:
                        angle_check = 50
                    if abs(angle_deg) >= angle_check:
                        self.dang_re = 1
                        print("rẽ do lệch")
                    else:
                        self.dang_re = 0

                    # music
                    if self.dang_re == 1:
                        if angle_deg < -angle_check:
                            music.data["re_phai"] = 1
                        else:
                            music.data["re_phai"] = 0

                        if angle_deg > angle_check:
                            music.data["re_trai"] = 1
                        else:
                            music.data["re_trai"] = 0
                    else:
                        music.data["re_phai"] = 0
                        music.data["re_trai"] = 0

                    if goc_dich[0] == "có hướng":
                        pass
                    else:
                        self.convert_data_run_agv["run_huong"] = "OK"

                    if self.convert_data_run_agv["run_diem"] != "OK":
                        # lựa chọn khoảng cách đến đích, các điểm không cần độ chính xác cao
                        add_kc_di_chuyen_luon = 0
                        del_kc_di_chuyen_luon = 0
                        
                        khoang_cach_dich = AGVConfig_2.khoang_cach_dich_min
                        delta_kc_dich = 20

                        # kiểm tra điều kiện đã đến đích
                        if distance <= 700:
                            delta_distan = abs(distance - self.distance_old)
                            if distance <= khoang_cach_dich or (distance <= 400 and distance > self.distance_old and delta_distan > delta_kc_dich):
                                self.convert_data_run_agv["run_diem"] = "OK"

                        if distance < self.distance_old:
                            self.distance_old = distance
                    else:
                        if self.convert_data_run_agv["run_huong"] != "OK":
                            check_angle_distance = "angle"
                            self.dang_re = 1
                            print("rẽ trực tiếp")

                            if self.convert_data_run_agv["run_huong"] != "OK":
                                angle_deg = int(goc_agv_driver_control * 180 / np.pi - float(goc_dich[1]))  
                            else:
                                if self.di_thuan_nguoc == 1: # Đi lùi
                                    A1 = np.array(self.toa_do_diem_dau)
                                    B1 = np.array(self.toa_do_diem_dich)
                                    A2 = np.array(AGVConfig.toa_do_agv_mm)
                                    B2 = np.array(self.robot_direction_nguoc)
                                    goc_1 = angle_and_distance.angle_with_ox(A1, B1)
                                    goc_2 = angle_and_distance.angle_with_ox(A2, B2)
                                    goc_1 = angle_and_distance.normalize_angle(goc_1)
                                    goc_2 = angle_and_distance.normalize_angle(goc_2)
                                    angle_deg = goc_1 - goc_2
                                else:
                                    A1 = np.array(self.toa_do_diem_dau)
                                    B1 = np.array(self.toa_do_diem_dich)
                                    A2 = np.array(AGVConfig.toa_do_agv_mm)
                                    B2 = np.array(self.robot_direction)
                                    goc_1 = angle_and_distance.angle_with_ox(A1, B1)
                                    goc_2 = angle_and_distance.angle_with_ox(A2, B2)
                                    goc_1 = angle_and_distance.normalize_angle(goc_1)
                                    goc_2 = angle_and_distance.normalize_angle(goc_2)
                                    angle_deg = goc_1 - goc_2
                                    
                            angle_deg = angle_and_distance.normalize_angle_90(angle_deg)
                            delta_ang = self.angle_min
                            # điểu kiện để tun_huong OK
                            van_toc_phan_hoi_max = max(AGVConfig.van_toc_phan_hoi_trai, AGVConfig.van_toc_phan_hoi_phai)
                            if abs(van_toc_phan_hoi_max) < 200:
                                if abs(angle_deg) <= delta_ang:
                                    self.convert_data_run_agv["run_huong"] = "OK"
                                    self.convert_data_run_agv["run_huong_2"] = "OK"
                            # music
                            if angle_deg < 0:
                                music.data["re_phai"] = 1
                            else:
                                music.data["re_phai"] = 0

                            if angle_deg > 0:
                                music.data["re_trai"] = 1
                            else:
                                music.data["re_trai"] = 0

                    self.check_angle_distance = check_angle_distance
                    self.angle =  angle_and_distance.normalize_angle_90(angle_deg)
                    self.distance = distance
                else:
                    driver_motor_quay_trai = 0
                    driver_motor_quay_phai = 0
                    self.distance_old = 1000
                    self.vi_tri_nhan_tin_hieu_nhan = ""
                    self.convert_data_run_agv = self.convert_data_run_agv0.copy()
                    self.dang_re = 0
                    stop = 1
                    if self.debug_stop == 1:
                        print("đã đến đích")
            else:
                stop = 1
                if self.debug_stop == 1:
                    print("stop do không có vị trí hiện tại")
        else:
            stop = 1
            if self.debug_stop == 1:
                print("stop do không có điểm cuối")
        self.stop = stop
        if self.stop == 1:
            music.data["re_trai"] = 0
            music.data["re_phai"] = 0
        return driver_motor_quay_trai, driver_motor_quay_phai, trang_thai


    def load_data_process(self):
        huong_x = int(AGVConfig.toa_do_agv_mm[0] + 2000 * math.cos(AGVConfig.huong_agv_do_thuc_rad - np.pi/2)) # thuan am, nghich dương
        huong_y = int(AGVConfig.toa_do_agv_mm[1] + 2000 * math.sin(AGVConfig.huong_agv_do_thuc_rad - np.pi/2))
        huong_x_nguoc = int(AGVConfig.toa_do_agv_mm[0] + 2000 * math.cos(AGVConfig.huong_agv_do_thuc_rad + np.pi/2))
        huong_y_nguoc = int(AGVConfig.toa_do_agv_mm[1] + 2000 * math.sin(AGVConfig.huong_agv_do_thuc_rad + np.pi/2))

        self.robot_direction = [huong_x, huong_y]
        self.robot_direction_nguoc = [huong_x_nguoc, huong_y_nguoc]

    def luu_toa_do_cu(self):
        # self.driver_motor.load_data_sent_drive(self.v_tien_max_new, self.v_re_max_new, 
        #                                        self.toa_do_diem_dau, self.toa_do_diem_dich, AGVConfig.toa_do_agv_mm, self.toa_do_diem_huong,
        #                                        self.angle, self.distance, self.check_angle_distance,
        #                                        self.stop, self.di_cham, self.a_v, self.dang_re, self.tien_rl, self.di_thuan_nguoc, self.xac_dinh_vi_tri_xe, self.di_chuyen_luon,
        #                                        self.distance_dich_ly_tuong, self.toa_do_dich_ly_tuong)

        if len(self.toa_do_cu_1) == 0:
            self.toa_do_cu_1 = AGVConfig.toa_do_agv_mm.copy()
        if len(self.toa_do_cu_2) == 0:
            self.toa_do_cu_2 = AGVConfig.toa_do_agv_mm.copy()
        if self.toa_do_cu_2[0] != AGVConfig.toa_do_agv_mm[0] or self.toa_do_cu_2[1] != AGVConfig.toa_do_agv_mm[1]:
            self.toa_do_cu_1 = self.toa_do_cu_2.copy()
            self.toa_do_cu_2 = AGVConfig.toa_do_agv_mm.copy()
    
    def tim_diem_gan_nhat(self, danh_sach_diem, center_px, resolution_mm):
        """
        Tìm điểm gần nhất trong danh_sach_diem so với một tọa độ cho trước.

        Args:
            current_position (list or tuple): Tọa độ hiện tại [x, y].
            danh_sach_diem (dict): Dictionary chứa thông tin các điểm.
                                    Ví dụ: {"P1": [100, 200, "không hướng", 0], ...}

        Returns:
            tuple: Một tuple chứa (tên điểm gần nhất, khoảng cách nhỏ nhất).
                Trả về (None, float('inf')) nếu danh_sach_diem rỗng.
        """
        if not danh_sach_diem:
            return None, float('inf')

        nearest_point_name = None
        min_distance = float('inf')
        
        current_x = AGVConfig.toa_do_agv_mm[0]
        current_y = AGVConfig.toa_do_agv_mm[1]
        # print("danh_sach_diem = ", danh_sach_diem)
        for point_name, point_data in danh_sach_diem.items():
            point_x, point_y = point_data[0], point_data[1]

            world_x_mm = (point_x - center_px[0]) * resolution_mm
            world_y_mm = (point_y - center_px[1]) * resolution_mm
            
            # Tính khoảng cách Euclidean
            distance = math.sqrt((world_x_mm - current_x)**2 + (world_y_mm - current_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_point_name = point_name
                
        return nearest_point_name, min_distance
    
    # # lấy danh sách tọa độ từ danh sách điểm Adanh_sach_diem = {'P1': [1690, 2309, 'không hướng', 0.0], 'P2': [1645, 2309, 'không hướng', 0.0], 'P3': [1675, 2240, 'không hướng', 0.0], ...}
    # # dánh sách lấy tọa độ p_actual =  ['X1', 'W1', 'G3', 'P19']
    # def convert_danh_sach_duong_di(self, p_actual):
    #     # AGVConfig.danh_sach_diem
    #     danh_sach_duong_di = []
    #     for i in range(len(p_actual)):
    #         toa_do_x = AGVConfig.danh_sach_diem[p_actual[i]][0]
    #         toa_do_y = AGVConfig.danh_sach_diem[p_actual[i]][1]
    #         danh_sach_duong_di.append([toa_do_x, toa_do_y])
    #     return danh_sach_duong_di
    
    # def kiem_tra_tin_hieu_esp32(self, data):
    #     self.input_esp = ket_noi_esp_loa.input_esp
    #     self.connect_esp32 = ket_noi_esp_loa.check_connect_esp

    #     """
    #     Kiểm tra xem tất cả các điều kiện trong check_data có khớp với input_esp không.

    #     Args:
    #         check_data (list): Danh sách các điều kiện cần kiểm tra.
    #                         Mỗi điều kiện là một list con dạng [key, value_mong_muon].
    #                         Ví dụ: [['IN4', 0], ['IN3', 1]]
    #         input_esp (dict): Dictionary chứa trạng thái của các input.
    #                         Ví dụ: {"IN1":0, "IN2":0, ..., "IN12":0}

    #     Returns:
    #         bool: True nếu tất cả các điều kiện trong check_data khớp với input_esp,
    #             ngược lại là False.
    #     """
    #     if not data:
    #         # Nếu không có điều kiện nào để kiểm tra, mặc định là True.
    #         # Bạn có thể thay đổi hành vi này nếu cần (ví dụ: trả về False).
    #         return True
    #     if self.connect_esp32 == False:
    #         # Nếu không kết nối được với esp32, coi như không khớp.
    #         return False

    #     for dieu_kien in data:
    #         # Đảm bảo mỗi điều kiện là một cặp [key, value]
    #         if not isinstance(dieu_kien, list) or len(dieu_kien) != 2:
    #             # Điều kiện không hợp lệ, coi như không khớp.
    #             # Hoặc bạn có thể raise một Exception ở đây nếu muốn.
    #             return False
            
    #         key_can_kiem_tra = dieu_kien[0]
    #         gia_tri_mong_muon = dieu_kien[1]

    #         # 1. Kiểm tra xem key có tồn tại trong input_esp không
    #         if key_can_kiem_tra not in self.input_esp:
    #             return False  # Key không tồn tại, điều kiện không khớp

    #         # 2. Kiểm tra xem giá trị có khớp không
    #         # Giả định rằng kiểu dữ liệu của giá trị mong muốn và giá trị trong input_esp là tương thích để so sánh.
    #         if self.input_esp[key_can_kiem_tra] != gia_tri_mong_muon:
    #             return False  # Giá trị không khớp

    #     # Nếu tất cả các điều kiện trong check_data đều được kiểm tra và khớp
    #     return True

    #         # kiem tra tin hieu
    #     if len(tin_hieu) != 0:
    #         if self.kiem_tra_tin_hieu_esp32(tin_hieu) == True:
    #             tin_hieu = []
    #     if len(tin_hieu_tam_thoi) != 0:
    #         if self.kiem_tra_tin_hieu_esp32(tin_hieu_tam_thoi) == True:
    #             self.done_tin_hieu_tam_thoi = 1

    #     if len(tin_hieu) == 0:
    #         self.convert_data_run_agv["run_tin_hieu"] = "OK"
    #     else:
    #         self.convert_data_run_agv["run_tin_hieu"] = "NG"
    #     if self.done_tin_hieu_tam_thoi == 1:
    #         self.convert_data_run_agv["run_tin_hieu_tam_thoi"] = "OK"
# if __name__ == '__main__':
    # Ví dụ sử dụng hàm find_nearest_point
    # danh_sach_diem_test = {
    #     "P1": [100, 200, "không hướng", 0],
    #     "P2": [550, 480, "không hướng", 0],
    #     "P3": [120, 210, "có hướng", 90]
    # }
    # agv_position = [125, 215]

    # ten_diem_gan_nhat, khoang_cach = find_nearest_point(agv_position, danh_sach_diem_test)

    # print(f"Tọa độ AGV: {agv_position}")
    # print(f"Điểm gần nhất là: {ten_diem_gan_nhat}")
    # print(f"Khoảng cách tới điểm đó là: {khoang_cach}")
    
