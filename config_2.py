import numpy as np
import cv2
import os
import time
import json
import math
from libs_file import remove

def edit_path(input):
    return input.replace("\\", "/")

def read_json_file(file_path):
    """
    Đọc dữ liệu từ một file JSON.

    Args:
        file_path (str): Đường dẫn đến file JSON.

    Returns:
        tuple: (data, message)
            - data (dict | list | None): Dữ liệu đã được đọc từ file JSON,
                                          hoặc None nếu có lỗi.
            - message (str): Thông báo thành công hoặc lỗi.
    """
    if not os.path.exists(file_path):
        return None, f"Lỗi: File không tồn tại tại đường dẫn: {os.path.abspath(file_path)}"

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data, None
    except json.JSONDecodeError as e:
        return None, f"Lỗi giải mã JSON: {e}"
    except Exception as e:
        return None, f"Lỗi không xác định khi đọc file: {e}"

PATH_PHAN_MEM = edit_path(os.path.dirname(os.path.realpath(__file__)))
    
# paths
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    PATH_SETTING = PATH_PHAN_MEM + "/setting/setting_window.json"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    PATH_SETTING = PATH_PHAN_MEM + "/setting/setting_ubuntu.json"

path_logo = os.path.join(PATH_PHAN_MEM, "static", "logo.png")
path_map_folder = os.path.join(PATH_PHAN_MEM, "data_input_output", "maps")
path_folder_danh_sach_diem = os.path.join(PATH_PHAN_MEM, "data_input_output", "point_lists") # File lưu danh sách điểm
path_folder_danh_sach_duong = os.path.join(PATH_PHAN_MEM, "data_input_output", "path_lists") # File lưu danh sách đường đi (đơn vị mm)
path_folder_loai_bo_chan_xe = os.path.join(PATH_PHAN_MEM, "data_input_output", "loai_bo_chan_xe")
path_folder_setting = os.path.join(PATH_PHAN_MEM, "setting")

PATH_FOLDER_SCAN_DATA_0 = remove.tao_folder(os.path.join(PATH_PHAN_MEM, "data_input_output", "scan_data_0"))
PATH_FOLDER_SCAN_DATA_1 = remove.tao_folder(os.path.join(PATH_PHAN_MEM, "data_input_output", "scan_data_1"))
PATH_FOLDER_SCAN_DATA_2 = remove.tao_folder(os.path.join(PATH_PHAN_MEM, "data_input_output", "scan_data_2"))
PATH_FOLDER_SCAN_DATA_3 = remove.tao_folder(os.path.join(PATH_PHAN_MEM, "data_input_output", "scan_data_3"))




data_setting, error = read_json_file(PATH_SETTING)
# print(data_setting, error)
if error is not None:
    print(f"CẢNH BÁO: Không thể đọc file cấu hình '{os.path.abspath(PATH_SETTING)}'. Lỗi: {error}")
    print("Chương trình có thể không hoạt động đúng hoặc sẽ sử dụng các giá trị mặc định.")



class AGVConfig_2:
    list_data0 = os.listdir(PATH_FOLDER_SCAN_DATA_0)
    list_data1 = os.listdir(PATH_FOLDER_SCAN_DATA_1)
    list_data2 = os.listdir(PATH_FOLDER_SCAN_DATA_2)
    list_data3 = os.listdir(PATH_FOLDER_SCAN_DATA_3)

    #  các biến cục bộ 
    rmse_max = data_setting["rmse_max"] # chưa dùng
    rmse_stop = data_setting["rmse_stop"] # ok
    rmse_warn = data_setting["rmse_warn"] # ok
    alpha_scan = data_setting["alpha_scan"]
    loai_lidar = data_setting["loai_lidar"] # chưa dùng
    khoang_cach_duong_di = data_setting["khoang_cach_duong_di"] # loại bỏ
    khoang_cach_dich = data_setting["khoang_cach_dich"] # loại bỏ
    duong_kinh_coc = data_setting["duong_kinh_coc"] # dùng cho auto tìm loại bỏ cọc, vùng loại bỏ || bán kính gom cụm
    min_samples = data_setting["min_samples"] # Số điểm tối thiểu để tạo cụm

    cong_lidar = data_setting["cong_lidar"] # có thể sau này sẽ dùng

    loa = data_setting["loa"] # có thể sau này sẽ dùng

    vol_min = data_setting["vol_min"] # ok
    vol_max = data_setting["vol_max"] # ok

    # ok
    chu_thich_khong_xe = data_setting["chu_thich_khong_xe"]
    khoang_cach_an_toan_tren_re = data_setting["khoang_cach_an_toan_tren_re"]
    khoang_cach_an_toan_ben_canh_re = data_setting["khoang_cach_an_toan_ben_canh_re"]
    khoang_cach_an_toan_duoi_re = data_setting["khoang_cach_an_toan_duoi_re"]

    khoang_cach_an_toan_tren = data_setting["khoang_cach_an_toan_tren"]
    khoang_cach_an_toan_ben_canh = data_setting["khoang_cach_an_toan_ben_canh"]
    khoang_cach_an_toan_duoi = data_setting["khoang_cach_an_toan_duoi"]

    chu_thich_nang = data_setting["chu_thich_nang"]
    khoang_cach_an_toan_tren_nang = data_setting["khoang_cach_an_toan_tren_nang"]
    khoang_cach_an_toan_ben_canh_nang = data_setting["khoang_cach_an_toan_ben_canh_nang"]
    khoang_cach_an_toan_duoi_nang = data_setting["khoang_cach_an_toan_duoi_nang"]

    chu_thich_co_xe = data_setting["chu_thich_co_xe"]
    khoang_cach_an_toan_tren_xe = data_setting["khoang_cach_an_toan_tren_xe"]
    khoang_cach_an_toan_ben_canh_xe = data_setting["khoang_cach_an_toan_ben_canh_xe"]
    khoang_cach_an_toan_duoi_xe = data_setting["khoang_cach_an_toan_duoi_xe"]

    khoang_cach_an_toan_tren_xe_re = data_setting["khoang_cach_an_toan_tren_xe_re"]
    khoang_cach_an_toan_ben_canh_xe_re = data_setting["khoang_cach_an_toan_ben_canh_xe_re"]
    khoang_cach_an_toan_duoi_xe_re = data_setting["khoang_cach_an_toan_duoi_xe_re"]

    chu_thich_tools_xe = data_setting["chu_thich_tools_xe"]
    khoang_cach_an_toan_tren_tools = data_setting["khoang_cach_an_toan_tren_tools"]
    khoang_cach_an_toan_ben_canh_tools = data_setting["khoang_cach_an_toan_ben_canh_tools"]
    khoang_cach_an_toan_duoi_tools = data_setting["khoang_cach_an_toan_duoi_tools"]
    # ok


    distance_max = data_setting["distance_max"] # khoảng cách bắt đầu giảm tốc độ khi gần đến đích
    distan_scan_all = data_setting["distan_scan_all"] # khoảng cách lidar scan

    khoang_cach_dich_min = data_setting["khoang_cach_dich_min"] # khoảng cách được cho là đã đến đích

    time_reset_dung_hoat_dong = data_setting["time_reset_dung_hoat_dong"] # bị tác động sau số giây agv mới bắt đầu di chuyển
    vung_xe = data_setting["vung_xe"] # lọc các điểm trong vùng này 

    phan_tram_pin_can_di_sac = data_setting["phan_tram_pin_can_di_sac"]
    van_toc_an_toan = data_setting["van_toc_an_toan"]
    # van_toc_an_toan = {"vung_1": {"van_toc_tien": 0, "van_toc_re": 0},
    #                    "vung_2": {"van_toc_tien": 3000, "van_toc_re": 500},
    #                    "vung_3": {"van_toc_tien": 5000, "van_toc_re": 600},}



    cap_nhat_ban_do_1_lan = False
    gui_tat_nguon_thanh_cong = False

    danh_sach_duong_di = []
    toa_do_tam_xe_mm = None # tọa độ tâm xe linh kiện
    goc_hcn = None # góc xe linh kiện
    diem_gan_nhat = None
    xac_dinh_vi_tri_xe = 0
    
    nang_vuong_goc = 0
    vi_tri_hien_tai = ""
    them_vi_tri_xe_ban_do = 0
    goc_xoay = 0


    time_thread_driver_feedback = 0
    reset_5s = False # nhấn reset 5s để tắt báo lỗi động cơ

    

    data_di_chuyen_luon = {"loai_1": {"danh_sach_diem_dau": None,
                                    "danh_sach_diem_dich": ["X"],                                   
                                    "data": {"update": 1, "co_huong": 1, "khong_huong": 0, "van_toc_di_chuyen_luon": None, 
                                                "van_toc_min": 500, "khoang_nhin_phia_truoc": None, "delta": [350, 100], "khong_xoay": None}}, 
                        "loai_2": {"danh_sach_diem_dau": None,
                                    "danh_sach_diem_dich": ["W"],                                    
                                    "data": {"update": 1, "co_huong": 1, "khong_huong": 0, "van_toc_di_chuyen_luon": None, 
                                            "van_toc_min": 400, "khoang_nhin_phia_truoc": None, "delta": None, "khong_xoay": None}}, 
                        "loai_3": {"danh_sach_diem_dau": None,
                                    "danh_sach_diem_dich": ["C"],   
                                    "data": {"update": 1, "co_huong": 0, "khong_huong": 1,  "van_toc_di_chuyen_luon": 2000, 
                                            "van_toc_min": None, "khoang_nhin_phia_truoc": 200,  "delta": [500, 50],  "khong_xoay": 1}},
                        "loai_4": {"danh_sach_diem_dau": ["C"],
                                    "danh_sach_diem_dich": ["E"],                   
                                    "data": {"update": 1,"co_huong": 0, "khong_huong": 1, "van_toc_di_chuyen_luon": None, 
                                            "van_toc_min": None, "khoang_nhin_phia_truoc": None, "delta": None, "khong_xoay": 1}},
                            "loai_5": {"danh_sach_diem_dau": None,
                                    "danh_sach_diem_dich": ["H"],                   
                                    "data": {"update": 1,"co_huong": 0, "khong_huong": 1, "van_toc_di_chuyen_luon": None, 
                                            "van_toc_min": None, "khoang_nhin_phia_truoc": None, "delta": None, "khong_xoay": None}},
                            "loai_6": {"danh_sach_diem_dau": ["P00000"],
                                    "danh_sach_diem_dich": ["E"],                   
                                    "data": {"update": 1,"co_huong": 0, "khong_huong": 1, "van_toc_di_chuyen_luon": None, 
                                            "van_toc_min": None, "khoang_nhin_phia_truoc": None, "delta": None, "khong_xoay": None}}}

    CONFIG_DIFF_DRIVE = {
        # RMSE ICP (mm) sai số icp
        "rmse_warn": rmse_warn,     # cảnh báo – đi chậm
        "rmse_stop": rmse_stop,     # dừng hẳn

        # Kiểm tra tịnh tiến của thực tế và icp
        "min_linear_motion_mm": 200,   # xe đi >3cm
        "min_icp_motion_mm": 5,       # ICP coi như đứng im

        # Kiểm tra quay dựa vào thực tế và icp. độ dịch chuyển min
        "min_rotation_deg": 10,      # ~6 độ
        "min_icp_rotation_deg": 2, # ~1 độ

        # ICP nhảy, vị trí của agv do icp out ra tạị vong lặp hiện tại và trước đó lệch nhau quá lớn hoặc xoay quá lớn sẽ dừng agv
        "max_pose_jump_mm": 400,      # xxx cm / frame
        "max_angle_jump_deg": 20,  

        # Timeout thời gian nhận quá lâu
        "icp_timeout": 0.5,           # giây
    }
    wheel_base_mm = 550
    luu_log_icp = 0

    # vùng thay đổi cho conect_driver.py
    last_icp_time = 0       # ⚠️ timestamp ICP update cuối
    now_time = 0            # timestamp hiện tại
    van_toc_phan_hoi_trai = 0
    van_toc_phan_hoi_phai = 0
    di_chuyen_cham = False
    loi_icp = False
    ten_loi = ""
    stop_rmse = False # dừng xe do lớn hơn ngưỡng yêu cầu
    stop_vat_can = False # dừng xe do có vật cản ở gần
    loi_an_toan = "" # tên lỗi an toàn do cảm biến lidar phát hiện

    