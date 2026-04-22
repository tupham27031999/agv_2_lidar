import math
from ham_logic import tinh_luong_giac, angle_and_distance as ad
import numpy as np
from config import AGVConfig


# ================== HÀM TIỆN ÍCH ==================
def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ================== HÀM ĐIỀU KHIỂN CHÍNH ==================
def agv_bam_duong(
    x, y,                     # vị trí AGV (mm)
    toa_do_diem_huong_agv,        # điểm hướng [xh, yh] (mm) hướng của agv
    x0, y0,                   # điểm đầu (mm) (điểm đích)
    xg, yg,                   # điểm đích (mm)      (điểm hướng của điểm đích)
    wheel_base,               # khoảng cách 2 bánh (mm)
    v_max=5000,               # mm/s
    v_min=300,                 # mm/s,
    di_chuyen_cung = False
):
    """
    AGV bám đường thẳng từ (x0,y0) -> (xg,yg)
    - Dùng điểm hướng thay cho theta
    - Ít rung khi chạy nhanh
    - Gần đích luôn song song với đường
    - An toàn khe hẹp
    """

    # ================== 1. Tính yaw AGV từ điểm hướng ==================
    # distance_hien_tai_dau = tinh_luong_giac.calculate_distance([x,y], [x0,y0])
    # if v_max > 4000:
    #     if distance_hien_tai_dau < 500:
    #         v_max = 3000
    #     else:
    #         if distance_hien_tai_dau < 500:
    #             v_max = 4000
    xh, yh = toa_do_diem_huong_agv

    # vi_tri_hien_tai = [x, y]
    # diem_dau = [x0, y0]
    # diem_cuoi = [xg, yg]
    # angle_rad_agv = tinh_luong_giac.angle_with_ox(vi_tri_hien_tai, diem_huong)
    # angle_rad_dich = tinh_luong_giac.angle_with_ox(diem_dau, diem_cuoi)
    # delta_deg_line = ad.normalize_angle((angle_rad_agv - angle_rad_dich) * 180 / np.pi)
    # delta_deg_line = ad.normalize_angle_90(delta_deg_line)

    theta = math.atan2(yh - y, xh - x)
    # print(delta_deg_line, (theta - delta_deg_line * math.pi / 180) * 180 / math.pi, theta * 180 / math.pi)


    # ================== 2. Vector đường ==================
    dx = xg - x0
    dy = yg - y0
    path_len = math.hypot(dx, dy)
    if path_len < 1e-6:
        return 0, 0

    path_yaw = math.atan2(dy, dx)

    # ================== 3. Sai lệch ==================
    # Sai lệch ngang
    e_ct = ((x - x0) * dy - (y - y0) * dx) / path_len

    # Sai lệch hướng
    e_yaw = wrap_angle(path_yaw - theta)

    # ================== 4. Khoảng cách tới đích ==================
    dist_to_goal = math.hypot(xg - x, yg - y)
    # print("dist_to_goal", dist_to_goal)

    # ================== 5. Speed profile ==================
    v = v_max
    # if dist_to_goal > 800:
    #     v = v_max
    # else:
    #     v = v_max * dist_to_goal / 800.0
    #     v = max(v, v_min)

    # ================== 6. Gain điều khiển ==================
    # if dist_to_goal > 400:
    k_yaw = 5.0
    k_ct = 20.0
    omega_max = 10.0
    if di_chuyen_cung == True:
        k_ct = max((int(abs(e_yaw *180 / math.pi)) * 4), 60) # bám ngang
        k_yaw = max(10.0, 80 - k_ct) # độ bám hướng
        # k_ct = 100
        # k_yaw = 5
        omega_max = 200.0 # AGV được phép quay nhanh tối đa bao nhiêu 2 độ/s
        edit_v = max((2000 + (30 - int(abs(e_yaw *180 / math.pi))) * 100), 2000)
        v_max = min(v_max, edit_v)
        v = v_max
    # else:
    #     if dist_to_goal <= 400:
    #         # gần đích → khóa hướng song song
    #         k_yaw = 4.0
    #         k_ct = 0.5
    #         omega_max = 0.4
    #     else:
    #         # gần đích → khóa hướng song song
    #         k_yaw = 4.0
    #         k_ct = 0.5
    #         omega_max = 0.4

    # ================== 7. Stanley cải tiến ==================
    if AGVConfig.run_state == 1:
        print(k_yaw , e_yaw ,k_ct , math.atan2(e_ct, max(abs(v), 50)), e_yaw * 180 / math.pi)
    omega = k_yaw * e_yaw + k_ct * math.atan2(e_ct, max(abs(v), 50))

    # ================== 8. Giới hạn quay ==================
    omega = max(min(omega, omega_max), -omega_max)
    if AGVConfig.run_state == 1:
        print(omega, "omega")
    # ================== 9. Điều kiện dừng chính xác ==================
    if dist_to_goal < 10:
        v = 0
        omega = 0

    # ================== 10. Tính vận tốc bánh ==================
    v_l = v - omega * wheel_base / 2.0
    v_r = v + omega * wheel_base / 2.0

    # ================== 11. GIỚI HẠN VẬN TỐC ==================

    # # (a) Ép vận tốc tối đa
    # max_abs = max(abs(v_l), abs(v_r))
    # if max_abs > v_max:
    #     scale = v_max / max_abs
    #     v_l *= scale
    #     v_r *= scale

    # # (b) Đảm bảo vận tốc tối thiểu
    # if 0 < abs(v_l) < v_min:
    #     v_l = math.copysign(v_min, v_l)
    # if 0 < abs(v_r) < v_min:
    #     v_r = math.copysign(v_min, v_r)

    # # (c) Ép lại tối đa
    # max_abs = max(abs(v_l), abs(v_r))
    # if max_abs > v_max:
    #     scale = v_max / max_abs
    #     v_l *= scale
    #     v_r *= scale

    v_trai_moi = v_l
    v_phai_moi = v_r
    # print(v_trai_moi, v_phai_moi)

    # Giới hạn tốc độ (giữ tỷ lệ)
    max_abs = max(abs(v_trai_moi), abs(v_phai_moi), v_max)
    if max_abs > v_max:
        scale = v_max / max_abs
        v_trai_moi *= scale
        v_phai_moi *= scale

    # Đảm bảo min speed theo trị tuyệt đối
    min_abs = min(abs(v_trai_moi), abs(v_phai_moi))
    if 0 < min_abs < v_min:
        scale = v_min / min_abs
        v_trai_moi *= scale
        v_phai_moi *= scale
    
    # [THÊM] Giới hạn lại vận tốc tối đa một lần nữa sau khi scale cho vận tốc tối thiểu.
    # Điều này đảm bảo vận tốc không bao giờ vượt ngưỡng van_toc_max.
    max_abs_final = max(abs(v_trai_moi), abs(v_phai_moi))
    if max_abs_final > v_max:
        scale_final = v_max / max_abs_final
        v_trai_moi *= scale_final
        v_phai_moi *= scale_final

    # Giới hạn độ lệch sai_so
    diff = abs(v_phai_moi - v_trai_moi)
    max_diff = 1500
    if diff > max_diff and v_phai_moi != v_trai_moi:
        scale = max_diff / diff
        v_trai_moi *= scale
        v_phai_moi *= scale

    return int(v_trai_moi), int(v_phai_moi)


if __name__ == "__main__":
    # toa_do_diem_dau, toa_do_diem_dich, toa_do_hien_tai, diem_huong
    # [-16533, -991] [-16517.65, -963.8930625] [-16531, -990] [-15405, 662]
    toa_do_diem_dau = [-16533, -991]
    toa_do_diem_dich = [-16517.65, -963.8930625]
    toa_do_hien_tai = [-16531, -990]
    toa_do_diem_huong_agv = [-15405, 662]

#     def agv_bam_duong(
#     x, y,                     # vị trí AGV (mm)
#     diem_huong,               # điểm hướng [xh, yh] (mm)
#     x0, y0,                   # điểm đầu đường (mm)
#     xg, yg,                   # điểm đích (mm)
#     wheel_base,               # khoảng cách 2 bánh (mm)
#     v_max=5000,               # mm/s
#     v_min=300,                 # mm/s,
#     di_chuyen_luon = None
# )

    print(agv_bam_duong(x = toa_do_hien_tai[0], y = toa_do_hien_tai[1],
                        toa_do_diem_huong_agv= toa_do_diem_huong_agv,
                        x0 = toa_do_diem_dau[0], y0 = toa_do_diem_dau[1],
                        xg = toa_do_diem_dich[0], yg = toa_do_diem_dich[1],
                        wheel_base = 500, v_max = 400, v_min = 300))
