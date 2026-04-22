
from libs_ngoai_vi import music
from ham_logic import tim_duong_di, tinh_luong_giac, angle_and_distance, angle_and_distance as ad
import numpy as np
from libs_lidar import scan_an_toan
import math
import time
from libs_ngoai_vi import ket_noi_esp_loa
import config_2 as cfg
# from config import AGVConfig
# from config_2 import AGVConfig_2

class detect_data_sent_driver:
    def __init__(self):
        music.name_music = "none"
        self.convert_data_run_agv0 = {"run_diem_2": "NG", "run_huong": "NG", "run_huong_2": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
        ########################################################################################
        self.angle = 0                      # xu_ly_tin_hieu - ok
        self.angle_min = 3
        self.distance = 0                   # xu_ly_tin_hieu - ok
        self.check_angle_distance = 0       # xu_ly_tin_hieu - ok
        self.di_cham = 0                    # load_data_process - ok
        self.a_v = 800
        self.dang_re = 0                    # xu_ly_tin_hieu - ok
        self.tien_rl = 0
        self.v_tien_max_new = 0                 # load_data_web - ok
        self.v_re_max_new = 0                   # load_data_web - ok

        
        self.toa_do_diem_dau = [] # dùng để tạo 
        self.toa_do_cu_1 = []
        self.toa_do_cu_2 = []
        self.toa_do_diem_dich = [] # dùng để tạo self.point_old, kiểm tra vị trí điểm 1 so với point_old
        self.toa_do_diem_kiem_soat = []
        self.toa_do_diem_cong_1 = []
        self.toa_do_diem_cong_2 = []
        self.toa_do_diem_huong = []
        self.done_tin_hieu_tam_thoi = 0
        self.robot_direction = []
        self.robot_direction_nguoc = []
        self.stop = 0
        self.convert_data_run_agv = {"run_diem_2": "NG", "run_huong": "NG", "run_huong_2": "NG", "run_tin_hieu": "NG", "run_tin_hieu_tam_thoi": "NG"}
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
        if AGVConfig.dieu_khien_agv["dieu_khien_thu_cong"] == False:
            # kiểm tra nâng hạ của tab code
            da_nang_xong = ket_noi_esp_loa.da_nang_xong
            da_ha_xong = ket_noi_esp_loa.da_ha_xong
            nang_ha = None
            if AGVConfig.nang_ha_xe_code is not None:
                if AGVConfig.nang_ha_xe_code == "nang":
                    nang_ha = "nang"
                if AGVConfig.nang_ha_xe_code == "ha":
                    nang_ha = "ha"
            if nang_ha is not None:
                if nang_ha == "nang" and da_nang_xong != 1:
                    ket_noi_esp_loa.py_sent_esp("nang_ha#nang#0")
                    music.data["dang_nang_hang"] = 1

                if nang_ha == "ha" and da_ha_xong != 1:
                    ket_noi_esp_loa.py_sent_esp("nang_ha#ha#0")
                    music.data["dang_ha_hang"] = 1

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

            # vùng loại bỏ thực hiện hiện hoàn toàn qua tab code
            # if AGVConfig.tam_thoi_reset_vung_loai_bo:
            #     AGVConfig.vung_loai_bo_x1y1x2y2 = []
            #     AGVConfig.vung_loai_bo_x1y1x2y2_pixel = []
            # else:
            #     if len(AGVConfig.vung_loai_bo_x1y1x2y2) == 0 and len(AGVConfig.vung_loai_bo_x1y1x2y2_pixel) == 0:
            #         # Load lại từ file đang chọn trong cấu hình
            #         AGVConfig.load_loai_bo(AGVConfig.loai_bo_coc_xe.get("ten_vung_loai_bo"))
            
        
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
            # print(f"run_state: {AGVConfig.run_state}")
            # print(f"dung_hoat_dong: {self.dung_hoat_dong}")

            if AGVConfig.run_state == 0 or self.dung_hoat_dong == 1 or self.time_reset_dung_hoat_dong != 0:
                stop_agv = 1
            # load các giá trị điểm đầu điểm đích, ... từ web, sau đó tính các giá trị góc, khoảng cách yêu cầu
            if stop_agv == 1:
                self.stop = 1
            else:
                driver_motor_quay_trai, driver_motor_quay_phai, trang_thai = self.xu_ly_tin_hieu(center_px, resolution_mm, goc_agv_driver_control) 
            
                
            # xử lý dữ liệu khi chạy
            self.luu_toa_do_cu() 
            data_return = {"v_tien_max": self.v_tien_max_new,
                        "v_re_max": self.v_re_max_new,
                        "toa_do_diem_dau": self.toa_do_diem_dau,
                        "toa_do_diem_dich": self.toa_do_diem_dich,
                        "toa_do_diem_huong": self.toa_do_diem_huong,
                        "toa_do_diem_cong_1": self.toa_do_diem_cong_1,
                        "toa_do_diem_cong_2": self.toa_do_diem_cong_2,
                        "toa_do_diem_kiem_soat": self.toa_do_diem_kiem_soat,
                        "angle": self.angle,
                        "distance": self.distance,
                        "check_angle_distance": self.check_angle_distance,
                        "stop": self.stop,
                        "di_cham": self.di_cham,
                        "a_v": self.a_v,
                        "dang_re": self.dang_re,
                        "tien_rl": self.tien_rl,
                        "di_thuan_nguoc": self.di_thuan_nguoc,
                        "xac_dinh_vi_tri_xe": self.xac_dinh_vi_tri_xe,
                        "di_chuyen_luon": self.di_chuyen_luon,
                        "distance_dich_ly_tuong": self.distance_dich_ly_tuong,
                        "toa_do_dich_ly_tuong": self.toa_do_dich_ly_tuong,
                        }
        else:
            if AGVConfig.dieu_khien_agv["ha_xe"] == 1 and da_ha_xong != 1: # 5
                ket_noi_esp_loa.py_sent_esp("nang_ha#ha#0")
                music.data["dang_ha_hang"] = 1
                if self.debug_stop == 1:
                    print("---- stop do webserver gửi tín hiệu nâng hạ xe ----")
                
            if AGVConfig.dieu_khien_agv["nang_xe"] == 1 and da_nang_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#nang#0")
                music.data["dang_nang_hang"] = 1

        
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
            


            

    def convert_tin_hieu(self, tin_hieu_agv, name_agv, center_px, resolution_mm, da_nang_xong, da_ha_xong):
        """
        Kiểm tra và tách chuỗi tín hiệu AGV.

        Args:
            tin_hieu: json tín hiệu đầu vào.
                                Ví dụ:  {'name_agv': 'agv1', 'dich_den': "P1", 'trang_thai': 'run'}
            danh_sach_diem (dict): Dictionary chứa thông tin các điểm.
                                Ví dụ: {"P1": [100, 200, "không hướng", 0], ...}
            danh_sach_duong (dict): Dictionary chứa thông tin các đường đi.
                                    Ví dụ: {"P1_P2": ["P1", "P2"], ...}

        Returns:
            dict: Dictionary chứa thông tin đã được xử lý.
        """
        tin_hieu = tin_hieu_agv[name_agv]
        dich_cuoi = tin_hieu["dich_den"]
        trang_thai_cuoi = tin_hieu["trang_thai_gui_agv"]

        diem_dau, khoang_cach = self.tim_diem_gan_nhat(AGVConfig.danh_sach_diem, center_px, resolution_mm)
        # print("diem_dau, khoang_cach", diem_dau, khoang_cach)
        if khoang_cach <= 600:
            if self.ten_diem_bat_dau == "":
                self.ten_diem_bat_dau = diem_dau
            if self.vi_tri_nhan_tin_hieu_nhan == "":
                self.vi_tri_nhan_tin_hieu_nhan = diem_dau
                AGVConfig.tin_hieu_nhan[name_agv]["vi_tri_hien_tai"] = diem_dau


        # tạo list_data và data_thong_tin_nang_ha để tạo danh sách đường đi đến đích 
        # print("self.list_data = ", self.list_data)
        # print("\n")
        # print("tin_hieu", tin_hieu)
        # self.list_data =  {'data1': [['X1', 'W1'], 'ha', ['W1', 'G3'], 'ha'], 'data2': [['W1', 'G3'], 'ha', ['G3', 'P19'], 'ha'], 'data3': [['G3', 'P19'], 'ha', ['G3', 'P19'], 'ha']}


        # tin_hieu {'vi_tri_hien_tai': 'X1', 'dich_den': 'P19', 'trang_thai': 'nang', 'message': 'none', 'danh_sach_duong_di': [], 'paths': [], 
        # 'di_chuyen_khong_hang': True, 'che_do_ha_xe': False, 'ngat_dong_co': False, 'dieu_khien_tay': False, 'dung_hoat_dong': False} che_do_ha_xe
        if self.list_data is None:
            self.dich_den_tin_hieu_nhan = dich_cuoi
            self.trang_thai_tin_hieu_nhan = trang_thai_cuoi

            trang_thai_hien_tai = ""
            if da_nang_xong == 1:
                trang_thai_hien_tai = "nang"
            if da_ha_xong == 1:
                trang_thai_hien_tai = "ha"
            graph = tim_duong_di.tao_graph()
            print("lllll", len(tin_hieu["paths"]), tin_hieu["paths"])
            if len(tin_hieu["paths"]) == 0:
                start_node = self.vi_tri_nhan_tin_hieu_nhan
                goal_node = self.dich_den_tin_hieu_nhan

                obstacles_points = []
                obstacles_paths = []

                print(f"Tìm đường từ {start_node} -> {goal_node} với vật cản điểm {obstacles_points} và đường {obstacles_paths}")
                p_ideal, c_ideal, p_actual, c_actual, c_diff = tim_duong_di.a_star(start_node, goal_node, vat_can_diem=obstacles_points, vat_can_duong=obstacles_paths)
                self.duong_di_ly_tuong, c_ideal = tim_duong_di.toi_uu_hoa_duong_di(p_actual, 5)
                print("jjjj", tim_duong_di.toi_uu_hoa_duong_di(p_actual, 5))
                print(f"\nĐường đi lý tưởng: {p_ideal} (Chi phí: {c_ideal:.2f})")
                print(f"Đường đi thực tế: {p_actual} (Chi phí: {c_actual:.2f})")

                print("graph = ", graph)
                print("p_actual = ", p_actual) #p_actual =  ['X1', 'W1', 'G3', 'P19']
                AGVConfig.danh_sach_duong_di = p_actual
                AGVConfig.tin_hieu_nhan[name_agv]["danh_sach_duong_di"] = p_actual


                print("trang_thai_hien_tai = ", trang_thai_hien_tai)
                print("trang_thai_cuoi = ", trang_thai_cuoi)
            else:
                # chỉ vào khi self.list_data là None, tức là chưa có dữ liệu đường đi nào được tạo ra trước đó, thì mới lấy đường đi từ webserver để tối ưu hóa lại
                p_actual = tin_hieu["paths"]
                self.duong_di_ly_tuong, c_ideal = tim_duong_di.toi_uu_hoa_duong_di(p_actual, 5)
                print("++++++++++++++++", p_actual, self.duong_di_ly_tuong)
                AGVConfig.danh_sach_duong_di = p_actual
                AGVConfig.tin_hieu_nhan[name_agv]["danh_sach_duong_di"] = p_actual


            err, data = tim_duong_di.tao_lenh_di_chuyen(graph,
                                                        p_actual,
                                                        trang_thai_hien_tai=trang_thai_hien_tai,
                                                        trang_thai_cuoi=trang_thai_cuoi,
                                                        force_ha=tin_hieu["di_chuyen_khong_hang"]) # 4


            if err == "" and len(data) != 0:
                self.list_data = data
                self.data_thong_tin_nang_ha = tim_duong_di.tao_thong_tin_nang_ha(graph, p_actual)
            else:
                print("err = ", err)
                print("self.list_data = ", self.list_data, err)
                print("self.data_thong_tin_nang_ha = ", self.data_thong_tin_nang_ha)


        tien_max = tin_hieu.get('van_toc_tien_max', None)
        re_max = tin_hieu.get('van_toc_re_max', None)
        tin_hieu_input = tin_hieu.get('tin_hieu_input', [])
        tin_hieu_tam_thoi = tin_hieu.get('tin_hieu_tam_thoi', [])

        return tien_max, re_max, tin_hieu_input, tin_hieu_tam_thoi





    def xu_ly_tin_hieu(self, center_px, resolution_mm, goc_agv_driver_control):
        # cập nhật biến quay trái quay phải của driver_motor
        driver_motor_quay_trai = None
        driver_motor_quay_phai = None

        trang_thai = ""
        
        # lấy tín hiệu từ esp32
        da_nang_xong = ket_noi_esp_loa.da_nang_xong
        da_ha_xong = ket_noi_esp_loa.da_ha_xong

        # if da_nang_xong == 1:
        #     trang_thai = "nang"
        # if da_ha_xong == 1:
        #     trang_thai = "ha"


        # tắt âm nâng ha
        if da_nang_xong == 1:
            music.data["dang_nang_hang"] = 0
        if da_ha_xong == 1:
            music.data["dang_ha_hang"] = 0

        stop = 0

        # điều khiển trung tâm gửi đến lệnh stop
        if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["stop"] == True:
            stop = 1
            if self.debug_stop == 1:
                print("---- stop do webserver gửi tín hiệu stop ----")
        # điều khuyển trung tâm gửi đến lệnh hạ xe
        # dieu_khien_agv = {"dieu_khien_thu_cong": False, "tien": 0, "lui": 0, "trai": 0, "phai": 0, "ha_xe": 0, "nang_xe": 0}
        nang_ha_thu_cong = 0
        

        # điều khiển nâng xe
        if self.check_nang_ha == 1 and nang_ha_thu_cong == 0:
            if self.nang_ha == "nang" and da_nang_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#nang#0")
                music.data["dang_nang_hang"] = 1

            if self.nang_ha == "ha" and da_ha_xong != 1:
                ket_noi_esp_loa.py_sent_esp("nang_ha#ha#0")
                music.data["dang_ha_hang"] = 1

            if ((self.nang_ha == "nang" and da_nang_xong == 1) or (self.nang_ha == "ha" and da_ha_xong == 1)):
                self.check_nang_ha = 0
        # print( "and",  self.hoan_thanh_den_vi_tri_dich)
        # print("bbbbbbbbbbbbbb", AGVConfig.tin_hieu_nhan != {} , AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "" ,
        #                                      AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "None")
        # --------------------------------------------
        # điều kiện để vào if này là đã nhận được tín hiệu điểm đích mới từ webserver, có điểm đích đến, đang ở trạng thái run, và điểm đích đến không phải là "None"
        if AGVConfig.tin_hieu_nhan != {} and AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "" and \
                                             AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != "None":
            # tìm trang_thai_check để so sách điều kiện vào các hàm sau
            trang_thai_check = ""
            if self.list_data is not None:
                if len(self.list_data) != 0:
                    ten_data = "data" + str(self.stt_list_data)
                    trang_thai_check = self.list_data[ten_data][1] # lay_hang, tra_hang, nang, ha danh_sach_di_chuyen_luon ...

            # --------------------------------------- xử lý tín hiệu điểm đích mới---------------------------------------
            # print("trang_thai_check", trang_thai_check, self.dich_den_tin_hieu_nhan)
            # self.dich_den_tin_hieu_nhan chính là điểm cuối của lần tín hiệu dktt gửi xuống lần trước
            # điều kiện khi kích thước paths != 0: điểm đầu của paths trùng với điểm đích hiện tại và đã đến đích
            cap_nhat_duong_moi_do_paths = 0
            if len(AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]) != 0 and self.list_data is not None:
                # ten_data = "data" + str(self.stt_list_data)
                # ten_diem_dich = self.list_data[ten_data][0][1]
                # if ten_diem_dich == AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"][-1]:
                cap_nhat_duong_moi_do_paths = 1 # có paths nên cập nhật đường đi

            # không có đích đến sẽ stop và cập nhật lại đích đến
            if self.dich_den_tin_hieu_nhan  == "":
                stop = 1
                if self.debug_stop == 1:
                    print("---- stop do chưa có tín hiệu điểm đích nào được nhận từ webserver ----")

            # reset toàn bộ biến
            # print("cap_nhat_duong_moi_do_paths", cap_nhat_duong_moi_do_paths)
            if ((AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != self.dich_den_tin_hieu_nhan or 
                            AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai_gui_agv"] != self.trang_thai_tin_hieu_nhan) or 
                            cap_nhat_duong_moi_do_paths == 1) and \
                            self.dich_den_tin_hieu_nhan != "" and trang_thai_check != "xoay_agv_vuong_goc" and trang_thai_check != "xoay_agv_song_song":
                # print("AGVConfig.tin_hieu_nhan[AGVConfig.name_agv", AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"], 
                #                                                     self.vi_tri_nhan_tin_hieu_nhan, self.dich_den_tin_hieu_nhan, 
                #                                                     AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["vi_tri_hien_tai"], 
                #                                                     self.hoan_thanh_den_vi_tri_dich )
                # self.den_diem_gan_nhat = True
                if (self.convert_data_run_agv["run_diem_2"] == "OK" 
                                                and self.convert_data_run_agv["run_huong"] == "OK" 
                                                and self.convert_data_run_agv["run_huong_2"] == "OK"):
                    # self.den_diem_gan_nhat = False
                    if AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] != self.vi_tri_nhan_tin_hieu_nhan:
                        self.list_data = None
                        self.data_thong_tin_nang_ha = None
                        self.vi_tri_nhan_tin_hieu_nhan = ""
                        self.dich_den_tin_hieu_nhan = ""
                        self.stt_list_data = 1

                        self.thong_tin_nang_ha = self.thong_tin_nang_ha_0.copy()
                        self.convert_data_run_agv = self.convert_data_run_agv0.copy()
                        self.ten_diem_bat_dau = ""
                        
                        self.distance_old = 1000
                        self.xoay_goc = 1
                        self.so_lan_check_vi_tri_xe = 0
                        self.xac_dinh_vi_tri_xe = 0
                        AGVConfig_2.toa_do_tam_xe_mm = None
                        AGVConfig_2.goc_hcn = None
                        AGVConfig_2.diem_gan_nhat = None
                        AGVConfig_2.xac_dinh_vi_tri_xe = 0
                        self.hoan_thanh_den_vi_tri_dich = False

            # cập nhật cả self.list_data và self.data_thong_tin_nang_ha khi có tín hiệu điểm đích mới, để đảm bảo rằng khi có tín hiệu điểm đích mới thì sẽ tạo lại đường đi mới và thông tin nâng hạ mới
            tien_max, re_max, tin_hieu_input, tin_hieu_tam_thoi =  self.convert_tin_hieu(AGVConfig.tin_hieu_nhan, AGVConfig.name_agv, 
                                                                                        center_px, resolution_mm, da_nang_xong, da_ha_xong)
            # sẽ dùng sau, hiện tại agv chưa cần đến
            tin_hieu = []
            
            danh_sach_dich = ["P40", "P14"]
            if danh_sach_dich[self.stt_test] == AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] and self.hoan_thanh_den_vi_tri_dich == True:
                self.stt_test = self.stt_test + 1
                if self.stt_test == len(danh_sach_dich):
                    self.stt_test = 0
                AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] = danh_sach_dich[self.stt_test]
            # else:
            # print(danh_sach_dich[self.stt_test], AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"], self.stt_test)


            # bắt đầu tìm kiếm đường đi khi self.list_data khác rỗng
            if self.list_data is not None:
                if len(self.list_data) != 0:
                    ten_data = "data" + str(self.stt_list_data)
                    trang_thai = self.list_data[ten_data][1] # nang, ha, lay_hang, tra_hang, xoay_agv_song_song, xoay_agv_vuong_goc, nghi - do trang thái cuối là trả hàng và chưa có lệnh
                    ten_diem_dau = self.list_data[ten_data][0][0]
                    ten_diem_dich = self.list_data[ten_data][0][1]
                    ten_diem_dau_tiep_theo = self.list_data[ten_data][2][0]
                    ten_diem_dich_tiep_theo = self.list_data[ten_data][2][1]
                    goc_dich = [AGVConfig.danh_sach_diem[ten_diem_dich][2], AGVConfig.danh_sach_diem[ten_diem_dich][3]]

                    # góc giữa điểm đầu và điểm đích so vơi ox
                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_dau][0], AGVConfig.danh_sach_diem[ten_diem_dau][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    toa_do_diem_dau = [int(world_x_mm), int(world_y_mm)]

                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_dich][0], AGVConfig.danh_sach_diem[ten_diem_dich][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    toa_do_diem_dich = [int(world_x_mm), int(world_y_mm)]
                    angle_deg_1 = tinh_luong_giac.angle_with_ox(toa_do_diem_dau, toa_do_diem_dich) * 180 / math.pi

                    # góc giữa điểm đầu và điểm đích tiếp theo so vơi ox    
                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_dich_tiep_theo][0], AGVConfig.danh_sach_diem[ten_diem_dich_tiep_theo][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    toa_do_diem_dich_tiep_theo = [int(world_x_mm), int(world_y_mm)]
                    angle_deg_2 = tinh_luong_giac.angle_with_ox(toa_do_diem_dich, toa_do_diem_dich_tiep_theo) * 180 / math.pi

                    
                    
                    # print("ten_diem_dau", ten_diem_dau)
                    # print("ten_diem_dich", ten_diem_dich)

                    # delta angle
                    # di_chuyen_luon = 0
                    delta_angle = angle_and_distance.normalize_angle_90(angle_deg_2 - angle_deg_1)
                    # if abs(delta_angle) < 10:
                    #     di_chuyen_luon = 1
                    
                    
                    #  --------------------------------------- lỗi motor nâng hạ ----------------------------------------------------------
                    # if ket_noi_esp_loa.loi_motor_nang_ha == 1:
                    #     music.data["loi_motor_nang_ha"] = 1
                    #     stop = 1
                    #     # print("stop_9")
                    # else:
                    #     music.data["loi_motor_nang_ha"] = 0

                    # --------------------------------------- lỗi cảm biến an toàn - dung_hoat_dong ---------------------------------------
                    # if ket_noi_esp_loa.dung_hoat_dong == 1:
                    #     music.data["dung_hoat_dong"] = 1
                    #     self.dung_hoat_dong = 1
                    #     stop = 1
                    #     if self.debug_stop == 1:
                    #         print("---- stop do cảm biến an toàn ----")
                    # else:
                    #     music.data["dung_hoat_dong"] = 0
                    #     if self.dung_hoat_dong == 1:
                    #         self.time_reset_dung_hoat_dong = time.time()
                    #         self.dung_hoat_dong = 0
                    #         music.data["bat_dau_di_chuyen"] = 1
                    #     if self.time_reset_dung_hoat_dong != 0:
                    #         stop = 1
                    #         if self.debug_stop == 1:
                    #             print("---- stop do đang trong thời gian reset sau khi cảm biến an toàn kích hoạt ----")
                    #     if self.time_reset_dung_hoat_dong != 0 and time.time() - self.time_reset_dung_hoat_dong > time_reset_dung_hoat_dong:
                    #         self.time_reset_dung_hoat_dong = 0
                    #         music.data["bat_dau_di_chuyen"] = 0
                        
                    # --------------------------------------- tín hiệu nhận từ đktt ---------------------------------------
                    if AGVConfig_2.stop_rmse == 1 or AGVConfig_2.stop_vat_can == 1:
                        stop = 1
                        if self.debug_stop == 1: # hiển thị để biết stop ở đâu
                            if AGVConfig_2.stop_rmse == 1:
                                print("---- stop do rmse lớn hơn ngưỡng ----")
                            if AGVConfig_2.stop_vat_can == 1:
                                print("---- stop do phát hiện vật cản ----")
                    

                    # tìm kiếm ten_diem_dich trong AGVConfig_2.data_di_chuyen_luon để cập nhật self.di_chuyen_luon (để điều chỉnh vận tốc, hướng, ...)
                    reset_di_chuyen_luon = 1
                    for i in range(len(AGVConfig_2.data_di_chuyen_luon)):
                        name = "loai_" + str(i + 1)
                        # nếu là trạng thái không dùng điểm đầu
                        if AGVConfig_2.data_di_chuyen_luon[name]["danh_sach_diem_dau"] is None:
                            if list(ten_diem_dich)[0] in AGVConfig_2.data_di_chuyen_luon[name]["danh_sach_diem_dich"]:
                                self.di_chuyen_luon = AGVConfig_2.data_di_chuyen_luon[name]["data"]
                                reset_di_chuyen_luon = 0
                                break
                        else:
                            if ten_diem_dau is not None:
                                if list(ten_diem_dich)[0] in AGVConfig_2.data_di_chuyen_luon[name]["danh_sach_diem_dich"] and list(ten_diem_dau)[0] in AGVConfig_2.data_di_chuyen_luon[name]["danh_sach_diem_dau"]:
                                    self.di_chuyen_luon = AGVConfig_2.data_di_chuyen_luon[name]["data"]
                                    reset_di_chuyen_luon = 0
                                    break
                    # nếu không trùng thì di chuyển bình thường
                    if reset_di_chuyen_luon == 1:
                        self.di_chuyen_luon = self.di_chuyen_luon0.copy()


                    # đã đến điểm cập nhật trạng thái tiếp theo 
                    if (self.convert_data_run_agv["run_diem_2"] == "OK" 
                                    and self.convert_data_run_agv["run_huong"] == "OK" 
                                    and self.convert_data_run_agv["run_huong_2"] == "OK") \
                                    or ten_diem_dau == ten_diem_dich:
                        # loại bỏ luôn quay trái, phải của driver
                        driver_motor_quay_trai = 0
                        driver_motor_quay_phai = 0

                        if self.check_nang_ha == 0:
                            # đã đạt trạng thái đến điểm đích thì mới vào đây
                            # xử lý thông tin sau khi đã có danh sách cập nhật điểm đầu và đích tiếp theo hoặc lấy trả xe
                            for key, value in self.list_data.items():
                                if ten_diem_dau == value[0][0] and ten_diem_dich == value[0][1] and trang_thai == value[1]:
                                    if trang_thai in ["nang"]:
                                        self.nang_ha = "nang"
                                        if da_nang_xong != 1:
                                            self.check_nang_ha = 1
                                    elif trang_thai in ["ha", "tra_hang"]:
                                        self.nang_ha = "ha"
                                        if da_ha_xong != 1:
                                            self.check_nang_ha = 1
                                    else: # khi nào trạng thái là lấy hàng, xoay_agv_song_song, xoay_agv_vuong_goc, nghi không chuyển sang điểm đích mới ngay mà cần thực hiện thêm thao tác
                                        for key2, value2 in self.data_thong_tin_nang_ha.items():
                                            if ten_diem_dich == value2["ten_diem"] and trang_thai == value2["trang_thai"]: # có thể có trạng thái nghỉ
                                                kiem_tra_xong_vi_tri_xe = self.thong_tin_nang_ha["kiem_tra_xong_vi_tri_xe"]
                                                da_hoan_thanh = self.thong_tin_nang_ha["da_hoan_thanh"]
                                                xac_dinh_huong = self.thong_tin_nang_ha["xac_dinh_huong"]
                                                xac_dinh_huong_xong = self.thong_tin_nang_ha["xac_dinh_huong_xong"]
                                                toa_do_tam_thoi = self.thong_tin_nang_ha["toa_do_tam_thoi"]
                                                self.thong_tin_nang_ha = {"che_do_nang_ha": "nang", "diem_nang_ha": ten_diem_dich, "diem_huong": value2["diem_huong"], 
                                                                        "trang_thai": value2["trang_thai"], "0_song_song_1_vuong_goc": value2["0_song_song_1_vuong_goc"], 
                                                                        "da_nang_xong": da_nang_xong, "da_ha_xong": da_ha_xong, 
                                                                        "xac_dinh_huong": xac_dinh_huong, "xac_dinh_huong_xong": xac_dinh_huong_xong,
                                                                        "kiem_tra_xong_vi_tri_xe": kiem_tra_xong_vi_tri_xe, "da_hoan_thanh": da_hoan_thanh,
                                                                        "toa_do_tam_thoi": toa_do_tam_thoi}
                                                break
                                        
                                    # cập nhật điểm tiếp theo nếu đã hoàn thành
                                    if self.thong_tin_nang_ha["che_do_nang_ha"] is not None and self.thong_tin_nang_ha["da_hoan_thanh"] == False:
                                        break
                                    elif self.check_nang_ha == 0: # chuyển qua đích mới luôn
                                        # stop = 1
                                        # print("stop_6")
                                        if self.stt_list_data < len(self.list_data):
                                            # cập nhật điểm bình thường
                                            self.thong_tin_nang_ha = self.thong_tin_nang_ha_0.copy()
                                            
                                            if self.test_tranh_nhau == 1 and len(AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]) != 0:
                                                self.stt_list_data = 1
                                                # test khi không có dktt
                                                # paths = AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]
                                                # del paths[0]
                                                # AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"] = paths
                                                # if len(AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"]) >= 2:
                                                #     AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] = paths[1]
                                            else:
                                                self.stt_list_data += 1
                                        
                                            ten_diem_dau = value[2][0]
                                            ten_diem_dich = value[2][1]
                                            trang_thai = value[3]

                                            # if abs(self.driver_motor.vt_trai * 10) <= 5 and abs(self.driver_motor.vt_phai * 10) <= 5:
                                            self.convert_data_run_agv = self.convert_data_run_agv0.copy()
                                            self.ten_diem_bat_dau = ""
                                            self.distance_old = 1000
                                            self.xoay_goc = 1
                                            self.so_lan_check_vi_tri_xe = 0
                                            self.xac_dinh_vi_tri_xe = 0
                                            AGVConfig_2.toa_do_tam_xe_mm = None
                                            AGVConfig_2.goc_hcn = None
                                            AGVConfig_2.diem_gan_nhat = None
                                            AGVConfig_2.xac_dinh_vi_tri_xe = 0
                                            print("cập nhật điểm bình thường")
                                            break
                                        else:
                                            self.hoan_thanh_den_vi_tri_dich = True
                                            AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["vi_tri_hien_tai"] = ten_diem_dich
                # print("+++++++++++", AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"], AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["paths"], self.list_data)
                # đã đến đích, phát loa + stop 
                if self.hoan_thanh_den_vi_tri_dich == True:
                    music.data["den_dich"] = 1
                    stop = 1
                    if self.debug_stop == 1:
                        print("---- stop do đã hoàn thành đến điểm đích ----")

                    
                else:
                    music.data["den_dich"] = 0

                    # xử lý thông tin của 1 quãng đường 
                    tinh_chinh_vi_tri_xe = 0
                    if self.thong_tin_nang_ha["che_do_nang_ha"] is not None:
                        AGVConfig_2.nang_vuong_goc = self.thong_tin_nang_ha["0_song_song_1_vuong_goc"]
                        ten_diem_dau = self.thong_tin_nang_ha["diem_huong"]
                        ten_diem_dich = self.thong_tin_nang_ha["diem_nang_ha"]

                        self.thong_tin_nang_ha["da_nang_xong"] = da_nang_xong
                        self.thong_tin_nang_ha["da_ha_xong"] = da_ha_xong

                        # self.thong_tin_nang_ha = {"che_do_nang_ha": "nang", "diem_nang_ha": ten_diem_dich, "diem_huong": value2["diem_huong"], 
                        #                         "trang_thai": value2["trang_thai"], "0_song_song_1_vuong_goc": value2["0_song_song_1_vuong_goc"], 
                        #                         "da_nang_xong": da_nang_xong, "da_ha_xong": da_ha_xong, 
                        #                         "xac_dinh_huong": xac_dinh_huong, "xac_dinh_huong_xong": xac_dinh_huong_xong,
                        #                         "kiem_tra_xong_vi_tri_xe": kiem_tra_xong_vi_tri_xe, "da_hoan_thanh": da_hoan_thanh}
                        
                        if self.thong_tin_nang_ha["che_do_nang_ha"] == "nang":
                            if self.thong_tin_nang_ha["kiem_tra_xong_vi_tri_xe"] == False:
                                # bước 1: xác nhận xe đã hạ nếu như kiem_tra_xong_vi_tri_xe == False
                                if self.thong_tin_nang_ha["da_ha_xong"] == 0:
                                    self.nang_ha = "ha"
                                    self.check_nang_ha = 1
                                elif self.check_nang_ha == 0:
                                    # bước 2: xác nhận vị trí xe nếu như đã hạ xe
                                    if self.thong_tin_nang_ha["xac_dinh_huong"] == True and \
                                                                    self.convert_data_run_agv["run_diem_2"] == "OK" and \
                                                                        self.convert_data_run_agv["run_huong"] == "OK" and \
                                                                            self.convert_data_run_agv["run_huong_2"] == "OK":
                                        self.thong_tin_nang_ha["xac_dinh_huong_xong"] = True
                                    if self.thong_tin_nang_ha["xac_dinh_huong_xong"] == False:
                                        if self.thong_tin_nang_ha["xac_dinh_huong"] == False:
                                            self.thong_tin_nang_ha["xac_dinh_huong"] = True
                                            self.convert_data_run_agv["run_diem_2"] = "NG"
                                            self.convert_data_run_agv["run_huong"] = "NG"
                                            self.convert_data_run_agv["run_huong_2"] = "NG"
                                        tinh_chinh_vi_tri_xe = 1
                                    else:
                                        # print("self.so_lan_check_vi_tri_xe", self.so_lan_check_vi_tri_xe, self.convert_data_run_agv["run_diem_2"], 
                                            #   self.convert_data_run_agv["run_huong"], self.convert_data_run_agv["run_huong_2"])
                                        if self.so_lan_check_vi_tri_xe >= 2:
                                            self.thong_tin_nang_ha["kiem_tra_xong_vi_tri_xe"] = True
                                            self.convert_data_run_agv["run_diem_2"] = "OK"
                                            self.convert_data_run_agv["run_huong"] = "OK"
                                            self.convert_data_run_agv["run_huong_2"] = "OK"
                                        else:
                                            if self.convert_data_run_agv["run_diem_2"] == "OK" and self.convert_data_run_agv["run_huong"] == "OK" and self.convert_data_run_agv["run_huong_2"] == "OK":
                                                self.so_lan_check_vi_tri_xe = self.so_lan_check_vi_tri_xe + 1

                                                self.convert_data_run_agv["run_diem_2"] = "NG"
                                                self.convert_data_run_agv["run_huong"] = "NG"
                                                self.convert_data_run_agv["run_huong_2"] = "NG"
                                            
                                            self.xac_dinh_vi_tri_xe = 1
                                            driver_motor_quay_trai = 0
                                            driver_motor_quay_phai = 0
                                            self.distance_old = 1000
                                        
                            else:
                                # bước 3: nâng xe
                                if self.thong_tin_nang_ha["da_nang_xong"] == 0:
                                    self.nang_ha = "nang"
                                    self.check_nang_ha = 1
                                    # print("buoc 3", self.nang_ha)
                                else:
                                    # bước 4: đến vị trí tiếp theo
                                    self.thong_tin_nang_ha["da_hoan_thanh"] = True

                    
                    if len(self.thong_tin_nang_ha["toa_do_tam_thoi"]) == 0:
                        if self.nang_ha == "ha" and self.check_nang_ha == 1 and self.thong_tin_nang_ha["che_do_nang_ha"] is not None:
                            if trang_thai == "xoay_agv_song_song" or trang_thai == "xoay_agv_vuong_goc":
                                self.thong_tin_nang_ha["toa_do_tam_thoi"] = AGVConfig.toa_do_agv_mm
                    else:
                        if (self.thong_tin_nang_ha["trang_thai"] != "xoay_agv_song_song" and self.thong_tin_nang_ha["trang_thai"] != "xoay_agv_vuong_goc"):
                            self.thong_tin_nang_ha["toa_do_tam_thoi"] = []

                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_dau][0], AGVConfig.danh_sach_diem[ten_diem_dau][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    self.toa_do_diem_dau = [int(world_x_mm), int(world_y_mm)]

                    toa_do_pixel = [AGVConfig.danh_sach_diem[ten_diem_dich][0], AGVConfig.danh_sach_diem[ten_diem_dich][1]]
                    world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                    world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                    self.toa_do_diem_dich = [int(world_x_mm), int(world_y_mm)]

                    AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["diem_tiep_theo"] = ten_diem_dich

                    # xử lý đường cong
                    ten_duong_1 = f"{ten_diem_dau}_{ten_diem_dich}"
                    ten_duong_2 = f"{ten_diem_dich}_{ten_diem_dau}"
                    
                    ten_duong_thuc_te = ten_duong_1 if ten_duong_1 in AGVConfig.danh_sach_duong else (ten_duong_2 if ten_duong_2 in AGVConfig.danh_sach_duong else None)

                    toa_do_diem_kiem_soat = []
                    diem_A = []
                    diem_B = []
                    if not ten_duong_thuc_te:
                        print("Đường ten_duong_1 hoặc ten_duong_2 không tồn tại trong danh sách đường.", ten_duong_1, ten_duong_2)
                        # X1_H04_C_X1-H04 H04_X1_C_H04-X1
                    else:
                        # "C11_C12": [["C11","C12"],"curve","C11-C12"]
                        if AGVConfig.danh_sach_duong[ten_duong_thuc_te][1] == "curve":
                            # 1. Lấy thông tin đường và các điểm tọa độ
                            path_info = AGVConfig.danh_sach_duong[ten_duong_thuc_te]
                            # diem_dau = AGVConfig.danh_sach_diem[ten_diem_dau][:2]   # Điểm bắt đầu
                            # diem_cuoi = AGVConfig.danh_sach_diem[ten_diem_dich][:2]  # Điểm kết thúc
                            diem_kiem_soat = AGVConfig.danh_sach_diem[path_info[2]][:2]   # Điểm kiểm soát (lấy từ index 3 của path_info)
                            world_x_mm = (diem_kiem_soat[0] - center_px[0]) * resolution_mm
                            world_y_mm = (diem_kiem_soat[1] - center_px[1]) * resolution_mm
                            toa_do_diem_kiem_soat = [int(world_x_mm), int(world_y_mm)]

                            # print("--------------", [AGVConfig.danh_sach_diem[ten_diem_dau][0], AGVConfig.danh_sach_diem[ten_diem_dau][1]], diem_dau)
                            # print("--------------", diem_cuoi, diem_kiem_soat, path_info)

                            # Giả sử hướng hiện tại của AGV lấy từ hệ thống (đổi ra radian)
                            huong_hien_tai_rad = AGVConfig.huong_agv_do_thuc_rad
                            tam_nhin = AGVConfig_2.tam_nhin_duong_cong
                            sai_so_dich = 5
                            buoc_nhin_xa_huong = 500 # Khoảng cách cộng thêm để tìm điểm định hướng
                            toa_do_agv = AGVConfig.toa_do_agv_mm

                            # 2. GỌI HÀM TÍNH TOÁN ĐIỀU HƯỚNG
                            reached, diem_A, diem_B, diem_Sau_A, goc_target, sai_so = tim_duong_di.calculate_agv_guidance(
                                toa_do_agv, huong_hien_tai_rad, self.toa_do_diem_dau, toa_do_diem_kiem_soat, self.toa_do_diem_dich, tam_nhin, sai_so_dich, buoc_nhin_xa_huong
                            )
                            
                            # print("diem_A", diem_A)
                            # print("diem_B", diem_B, toa_do_agv)
                    if len(diem_A) != 0:
                        self.toa_do_diem_cong_1 = [int(diem_A[0]), int(diem_A[1])]
                    else:
                        self.toa_do_diem_cong_1 = []
                    if len(diem_B) != 0:
                        self.toa_do_diem_cong_2 = [int(diem_B[0]), int(diem_B[1])]
                    else:
                        self.toa_do_diem_cong_2 = []
                    if len(toa_do_diem_kiem_soat) != 0:
                        self.toa_do_diem_kiem_soat = [int(toa_do_diem_kiem_soat[0]), int(toa_do_diem_kiem_soat[1])]
                    else:
                        self.toa_do_diem_kiem_soat = []

                    # self.toa_do_diem_kiem_soat = toa_do_diem_kiem_soat

                    # if da_nang_xong == 1:
                    #     webserver.tin_hieu_nhan[name_agv]["trang_thai"] = "nang"
                    #     webserver.tin_hieu_nhan[name_agv]["dich_den"] = "P48"
                    # if ten_diem_dich == "X2" and da_nang_xong == 1:
                    #     webserver.tin_hieu_nhan[name_agv]["trang_thai"] = "nang"
                    #     webserver.tin_hieu_nhan[name_agv]["dich_den"] = "P15"
        
                    index = None
                    # print("self.den_diem_gan_nhat", self.den_diem_gan_nhat)
                    if self.den_diem_gan_nhat == False:
                        print("00000000000000", ten_diem_dau, self.duong_di_ly_tuong)
                        if ten_diem_dau in self.duong_di_ly_tuong or ten_diem_dich in self.duong_di_ly_tuong:
                            if ten_diem_dau in self.duong_di_ly_tuong:
                                index = self.duong_di_ly_tuong.index(ten_diem_dau)
                            else:
                                index = self.duong_di_ly_tuong.index(ten_diem_dich) - 1
                            if index + 1 < len(self.duong_di_ly_tuong):
                                toa_do_pixel = [AGVConfig.danh_sach_diem[self.duong_di_ly_tuong[index + 1]][0], AGVConfig.danh_sach_diem[self.duong_di_ly_tuong[index + 1]][1]]
                                world_x_mm = (toa_do_pixel[0] - center_px[0]) * resolution_mm
                                world_y_mm = (toa_do_pixel[1] - center_px[1]) * resolution_mm
                                self.toa_do_dich_ly_tuong = [int(world_x_mm), int(world_y_mm)]
                            else:
                                self.toa_do_dich_ly_tuong = self.toa_do_diem_dich
                        else:
                            self.toa_do_dich_ly_tuong = self.toa_do_diem_dich
                    else:
                        self.toa_do_dich_ly_tuong = self.toa_do_diem_dich
                        
                    if len(self.toa_do_dich_ly_tuong) != 0:
                        self.distance_dich_ly_tuong = tinh_luong_giac.calculate_distance(AGVConfig.toa_do_agv_mm, self.toa_do_dich_ly_tuong)
                    # else:
                    #     self.distance_dich_ly_tuong = None
                    
                    distance = tinh_luong_giac.calculate_distance(AGVConfig.toa_do_agv_mm, self.toa_do_diem_dich)
                    if distance < 700 and self.di_chuyen_luon["update"] == 1:
                        pass
                    else:
                        self.di_chuyen_luon["delta"] = None
                            
                    if len(self.thong_tin_nang_ha["toa_do_tam_thoi"]) != 0:
                        self.toa_do_diem_dich = self.thong_tin_nang_ha["toa_do_tam_thoi"]

                    # chọn vùng an toàn cho phù hợp với đang kéo xe hay không
                    if self.vung_loai_bo_co_xe == 0 and trang_thai == "xoay_agv_song_song":
                        self.vung_loai_bo_co_xe = 1
                    if self.vung_loai_bo_co_xe == 1 and da_ha_xong == 1:
                        self.vung_loai_bo_co_xe = 2
                    if self.vung_loai_bo_co_xe == 2 and da_nang_xong == 1 and trang_thai != "xoay_agv_song_song":
                        self.setup_vung_loai_bo = 1
                        self.vung_loai_bo_co_xe = 0

                    if da_ha_xong == 1:
                        self.setup_vung_loai_bo = 0

                    # test với self.setup_vung_loai_bo == 1
                    # self.setup_vung_loai_bo = 1


                    if self.setup_vung_loai_bo == 1:
                        if len(AGVConfig.vung_loai_bo_x1y1x2y2) == 0:
                            AGVConfig.tam_thoi_reset_vung_loai_bo = False
                    else:
                        if len(AGVConfig.vung_loai_bo_x1y1x2y2) != 0:
                            AGVConfig.tam_thoi_reset_vung_loai_bo = True

                    
                    # trường hợp xác  định vị trí xe nhưng góc lệch lớn thì không cập nhật vị trí xe nữa
                    goc_check_xe = tinh_luong_giac.angle_with_ox(self.toa_do_diem_dau, self.toa_do_diem_dich) * 180 / np.pi
                    angle_deg_vi_tri_xe = int(goc_agv_driver_control * 180 / np.pi - float(goc_check_xe))
                    angle_deg_vi_tri_xe = angle_and_distance.normalize_angle_90(angle_deg_vi_tri_xe)
                    if self.xac_dinh_vi_tri_xe == 1:
                        
                        if abs(ad.normalize_angle_90(abs(angle_deg_vi_tri_xe))) > 7:
                            AGVConfig_2.xac_dinh_vi_tri_xe = 0
                        else:
                            AGVConfig_2.xac_dinh_vi_tri_xe = 1

                    # thay đổi tọa độ điểm đầu, đích và góc đích
                    if tinh_chinh_vi_tri_xe == 1:
                        if self.convert_data_run_agv["run_diem_2"] != "OK":
                            if len(self.toa_do_cu_1) != 0:
                                self.toa_do_diem_dau = self.toa_do_cu_1.copy()
                    elif self.xac_dinh_vi_tri_xe == 1:
                        if AGVConfig_2.toa_do_tam_xe_mm is not None:
                            self.toa_do_diem_dau = AGVConfig_2.diem_gan_nhat # điểm gần nhất là tâm 1 trong 4 cạnh của hcn
                            if self.convert_data_run_agv["run_diem_2"] != "OK":
                                if len(self.toa_do_cu_1) != 0:
                                    self.toa_do_diem_dau = self.toa_do_cu_1.copy()
                            self.toa_do_diem_dich = AGVConfig_2.toa_do_tam_xe_mm.copy()

            else:
                stop = 1
                if self.debug_stop == 1:
                    print("---- stop do list_data trống ----")

            self.stop = stop
            # print("stop", self.stop)
            if self.stop == 0:
                if tien_max is not None:
                    self.v_tien_max_new = int(float(tien_max)) # trên dktt gửi xuống được ưu tiên
                else:
                    self.v_tien_max_new = AGVConfig.van_toc_tien_max # v max mặc định

                if re_max is not None:
                    self.v_re_max_new = int(float(re_max))
                else:
                    self.v_re_max_new = AGVConfig.van_toc_re_max

                if goc_dich[0] == "có hướng" and self.xac_dinh_vi_tri_xe == 0:
                    self.convert_data_run_agv["run_huong_2"] = "OK"
                else:
                    self.convert_data_run_agv["run_huong"] = "OK"

                


                # xử lý tín hiệu web truyền tới
                if len(self.toa_do_diem_dau) != 0 and len(self.toa_do_diem_dich) != 0:
                    distance = 0
                    angle_deg = 0
                    check_angle_distance = "distance"
                    
                    # print("-----hh-----", self.toa_do_hien_tai, self.convert_data_run_agv["run_diem_2"], self.convert_data_run_agv["run_huong"], self.convert_data_run_agv["run_huong_2"])
                    if self.toa_do_diem_dau[0] == self.toa_do_diem_dich[0] and self.toa_do_diem_dau[1] == self.toa_do_diem_dich[1]:
                        self.convert_data_run_agv["run_diem_2"] = "OK"
                        print("ok")

                    if self.convert_data_run_agv["run_diem_2"] != "OK":
                        self.dang_re = 0

                        # load điểm gần agv để đi tới
                        angle_rad_thuan = tinh_luong_giac.angle_with_ox(AGVConfig.toa_do_agv_mm, self.robot_direction)
                        angle_rad_nguoc = tinh_luong_giac.angle_with_ox(AGVConfig.toa_do_agv_mm, self.robot_direction_nguoc)
                        angle_rad_dich = tinh_luong_giac.angle_with_ox(self.toa_do_diem_dau, self.toa_do_diem_dich)
                        distance = tinh_luong_giac.calculate_distance(AGVConfig.toa_do_agv_mm, self.toa_do_diem_dich)
                        distance_hien_tai_dau = tinh_luong_giac.calculate_distance(AGVConfig.toa_do_agv_mm, self.toa_do_diem_dau)
                        distance_diem_dau_diem_dich = tinh_luong_giac.calculate_distance(self.toa_do_diem_dau, self.toa_do_diem_dich)

                        delta_deg_thuan = ad.normalize_angle((angle_rad_thuan - angle_rad_dich) * 180 / np.pi)
                        delta_deg_nguoc = ad.normalize_angle((angle_rad_nguoc - angle_rad_dich) * 180 / np.pi)

                        # Logic quyết định đi tiến hay lùi cho xe 2 đầu
                        # Nếu góc đi tiến quá lớn ( > 90 độ), thì chọn đi lùi
                        if abs(delta_deg_thuan) > 90:
                            self.di_thuan_nguoc = 1 # Đi lùi
                            angle_deg = delta_deg_nguoc
                            self.toa_do_diem_huong = self.robot_direction_nguoc
                        else:
                            self.di_thuan_nguoc = 0 # Đi tiến
                            angle_deg = delta_deg_thuan
                            self.toa_do_diem_huong = self.robot_direction

                        if distance <= 2000 and ten_diem_dich == AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"]:
                            if distance > 100:
                                music.data["sap_den_dich"] = 1
                                music.data["da_den_dich"] = 0
                            else:
                                music.data["da_den_dich"] = 1
                                music.data["sap_den_dich"] = 0
                            
  
                        else:
                            music.data["sap_den_dich"] = 0
                            music.data["da_den_dich"] = 0
                        

                        # # agv rẽ trái hoặc phải
                        if (self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["van_toc_min"] is not None):
                            angle_check = 10
                            if abs(angle_deg) >= angle_check:
                                self.dang_re = 1
                        elif self.xoay_goc == 1: 
                            angle_check = 10
                            if abs(angle_deg) >= angle_check:
                                self.dang_re = 1
                            else:
                                self.dang_re = 0
                                self.xoay_goc = 0
                        else:
                            
                            angle_check = 20
                            if abs(angle_deg) >= angle_check:
                                self.dang_re = 1
                            else:
                                self.dang_re = 0
                                self.xoay_goc = 0

                        if self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["khong_xoay"] is not None:  ################################################################ new
                            self.dang_re = 0
                            self.xoay_goc = 0

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


                        # lựa chọn khoảng cách đến đích, các điểm không cần độ chính xác cao
                        add_kc_di_chuyen_luon = 0
                        del_kc_di_chuyen_luon = 0
                        if self.di_chuyen_luon["update"] == 1:
                            add_kc_di_chuyen_luon = 10
                            del_kc_di_chuyen_luon = 10
                        
                        if self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["van_toc_min"] is not None and self.di_chuyen_luon["delta"] is not None:
                            khoang_cach_dich = 25
                            delta_kc_dich = 20
                        elif self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["van_toc_di_chuyen_luon"] is not None:
                            khoang_cach_dich = 60
                            delta_kc_dich = 10
                        else:
                            khoang_cach_dich = AGVConfig_2.khoang_cach_dich_min + add_kc_di_chuyen_luon
                            delta_kc_dich = 20 - del_kc_di_chuyen_luon

                        # kiểm tra điều kiện đã đến đích
                        if distance <= 700:
                            delta_distan = abs(distance - self.distance_old)
                            delta_distan_2 = distance_hien_tai_dau - distance_diem_dau_diem_dich
                            if self.xac_dinh_vi_tri_xe == 0:
                                if (distance <= khoang_cach_dich  or 
                                                    (distance <= 400 and distance > self.distance_old and delta_distan > 10 and 
                                                    delta_distan_2 > delta_kc_dich)):
                                    self.convert_data_run_agv["run_diem_2"] = "OK"






















                                    # test 
                                    
                                    # danh_sach_diem_dich = ["P53", "P15"]
                                    # if ten_diem_dich in danh_sach_diem_dich:
                                    #     index = danh_sach_diem_dich.index(ten_diem_dich)
                                    #     if index < len(danh_sach_diem_dich) - 1:
                                    #         AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai"] = "nang"
                                    #         AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] = danh_sach_diem_dich[index + 1]
                                    #     else:
                                    #         AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["trang_thai"] = "nang"
                                    #         AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["dich_den"] = danh_sach_diem_dich[0]
                            else:
                                if (distance <= 25):
                                    self.convert_data_run_agv["run_diem_2"] = "OK"

                            if distance < self.distance_old:
                                self.distance_old = distance
                        
                            if self.distance_dich_ly_tuong > 700 and self.thong_tin_nang_ha["che_do_nang_ha"] is None:
                                # print("next_lien @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                                self.convert_data_run_agv["run_diem_2"] = "OK"
                                self.convert_data_run_agv["run_huong"] = "OK"
                                self.convert_data_run_agv["run_huong_2"] = "OK"

                    else:
                        if self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["khong_huong"]  == 1:
                            # print("bo huong@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                            self.convert_data_run_agv["run_huong"] = "OK"
                            self.convert_data_run_agv["run_huong_2"] = "OK"

                        if self.convert_data_run_agv["run_huong"] != "OK" or self.convert_data_run_agv["run_huong_2"] != "OK":
                            check_angle_distance = "angle"
                            self.dang_re = 1

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
                                    
                                if re_max is not None:
                                    self.v_re_max_new = int(float(re_max))
                                else:
                                    self.v_re_max_new = AGVConfig.van_toc_re_max

                            angle_deg = angle_and_distance.normalize_angle_90(angle_deg)
                            delta_ang = self.angle_min
                            # if self.di_chuyen_luon["update"] == 1 and self.di_chuyen_luon["van_toc_min"] is not None:
                            #     delta_ang = 2

                            van_toc_max = max(AGVConfig_2.van_toc_phan_hoi_trai, AGVConfig_2.van_toc_phan_hoi_phai)
                            if abs(van_toc_max) < 200:
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
                    self.stop = 1
                    print("khong co diem dau hoac diem dich")
        
        # nang, ha, lay_hang, tra_hang, xoay_agv_song_song, xoay_agv_vuong_goc, nghi
        # IDLE,       // Nghỉ / Chờ lệnh
        # PICKING,    // Đang lấy hàng
        # DROPPING,   // Đang trả hàng
        # CHARGING,   // Đang sạc
        # BLOCKED,    // Bị vật cản
        # ERROR,      // Lỗi kỹ thuật
        # OFFLINE     // Mất kết nối
        else:
            self.stop = 1
            print("khong co diem dau hoac diem dich")

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
    
