import math
# ghi chú
# Các hãng AGV lớn thường không để tam_nhin là một con số cố định. Họ sử dụng Dynamic Look-ahead:

# Theo tốc độ: tam_nhin = v * k (với v là vận tốc hiện tại, k là hệ số thời gian, ví dụ 1.5 giây). Xe chạy càng nhanh, nhìn càng xa.
# Theo độ cong: Khi cảm biến hoặc bản đồ báo sắp vào cua gắt, hệ thống sẽ chủ động giảm tốc độ và giảm tam_nhin để xe bám cua chính xác hơn.
def get_dist(p1, p2):
    """Tính khoảng cách giữa 2 điểm"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_quadratic_bezier_point(t, p0, p1, p2):
    """Công thức toán học Quadratic Bézier: B(t) = (1-t)^2*P0 + 2(1-t)*t*P1 + t^2*P2"""
    x = (1 - t)**2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0]
    y = (1 - t)**2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1]
    return [x, y]

def get_quadratic_bezier_tangent(t, p0, p1, p2):
    """Tính đạo hàm bậc 1 để lấy vector tiếp tuyến: B'(t) = 2(1-t)(P1-P0) + 2t(P2-P1)"""
    vx = 2 * (1 - t) * (p1[0] - p0[0]) + 2 * t * (p2[0] - p1[0])
    vy = 2 * (1 - t) * (p1[1] - p0[1]) + 2 * t * (p2[1] - p1[1])
    angle = math.atan2(vy, vx)
    return angle

def calculate_agv_guidance(toa_do_agv, huong_agv_rad, p0, p1, p2, tam_nhin=1.0, sai_so_dich=0.1, buoc_nhin_xa_huong=0.5):
    """
    Hàm tính toán điều hướng AGV dựa trên đường cong Bézier.
    
    Inputs:
    - toa_do_agv: [x, y] hiện tại của xe.
    - huong_agv_rad: hướng hiện tại của xe (radian).
    - p0, p1, p2: [x, y] của điểm đầu, điểm kiểm soát, và điểm cuối.
    - tam_nhin: khoảng cách nhìn xa (Look-ahead distance).
    - sai_so_dich: khoảng cách chấp nhận đã đến đích.
    - buoc_nhin_xa_huong: khoảng cách từ A đến B để vẽ vector hướng.
    
    Outputs:
    - reached_goal: (bool) True nếu đã đến đích.
    - diem_den: [x, y] Điểm mục tiêu A trên đường cong.
    - diem_huong: [x, y] Điểm định hướng B (A + tangent).
    - diem_sau_muc_tieu: [x, y] Điểm trên đường cong nằm phía trước A (gần đích hơn).
    - goc_huong_target: (float) Góc tiếp tuyến tại A (radian).
    - sai_so_goc: (float) Sai số góc giữa hướng xe và hướng đường (radian, chuẩn hóa -pi to pi).
    """
    # 1. Kiểm tra xem đã đến đích chưa
    dist_to_goal = get_dist(toa_do_agv, p2)
    if dist_to_goal <= sai_so_dich:
        return True, p2, p2, p2, 0, 0

    # 2. Tạo tập hợp các điểm trên đường cong (Sampling)
    num_samples = 100
    points_on_curve = [get_quadratic_bezier_point(i/num_samples, p0, p1, p2) for i in range(num_samples + 1)]

    # 3. Tìm điểm trên đường cong gần AGV nhất
    min_dist = float('inf')
    closest_idx = 0
    for i, pt in enumerate(points_on_curve):
        d = get_dist(toa_do_agv, pt)
        if d < min_dist:
            min_dist = d
            closest_idx = i

    # 4. Tìm điểm mục tiêu (Point A) nằm trong tầm nhìn (Look-ahead)
    diem_den = points_on_curve[-1]
    idx_den = num_samples
    for i in range(closest_idx, len(points_on_curve)):
        d = get_dist(toa_do_agv, points_on_curve[i])
        if d >= tam_nhin:
            diem_den = points_on_curve[i]
            idx_den = i
            break

    # 5. Xác định điểm phía sau mục tiêu (điểm nằm sau A, tiến về phía đích)
    # Lấy tiến lên 2 index để có một đoạn thẳng hướng về phía trước
    idx_sau = min(len(points_on_curve) - 1, idx_den + 2)
    diem_sau_muc_tieu = points_on_curve[idx_sau]

    # 6. Lấy hướng mục tiêu (Target Heading) dựa trên tiếp tuyến tại t của Điểm A
    t_den = idx_den / num_samples
    goc_huong_target = get_quadratic_bezier_tangent(t_den, p0, p1, p2)

    # 7. Tính điểm định hướng B
    # Vector AB sẽ hợp với trục Ox một góc đúng bằng goc_huong_target
    diem_huong = [
        diem_den[0] + math.cos(goc_huong_target) * buoc_nhin_xa_huong,
        diem_den[1] + math.sin(goc_huong_target) * buoc_nhin_xa_huong
    ]

    # 8. Tính sai số góc và chuẩn hóa về khoảng [-pi, pi]
    sai_so_goc = goc_huong_target - huong_agv_rad
    sai_so_goc = (sai_so_goc + math.pi) % (2 * math.pi) - math.pi

    return False, diem_den, diem_huong, diem_sau_muc_tieu, goc_huong_target, sai_so_goc


danh_sach_diem = {'C10': [0, 0, 'không hướng', 0], 'C11': [4, 4, 'không hướng', 0], "C10-C11": [0, 4, 'không hướng', 0]}
danh_sach_duong = {"C10_C11_C_C10-C11": ['C10', 'C11', 'C', 'C10-C11'], "C11_C10_C_C11-C10": ['C11', 'C10', 'C', 'C10-C11']}

toa_do_agv = [0, 0]
tam_nhin = 1.0
buoc_nhin_xa_huong = 0.5 # Khoảng cách cộng thêm để tìm điểm định hướng
sai_so_dich = 0.05 # Khoảng cách để coi là đã đến đích (ví dụ 10cm)

ten_diem_dau = "C10"
ten_diem_cuoi = "C11"
ten_duong_1 = f"{ten_diem_dau}_{ten_diem_cuoi}_C_{ten_diem_dau}-{ten_diem_cuoi}"
ten_duong_2 = f"{ten_diem_cuoi}_{ten_diem_dau}_C_{ten_diem_cuoi}-{ten_diem_dau}"
# print("Danh sách điểm:", danh_sach_diem)
# print("Danh sách đường:", danh_sach_duong)
print("Tên đường 1:", ten_duong_1)
print("Tên đường 2:", ten_duong_2)

ton_tai = 0
ten_duong_thuc_te = ten_duong_1 if ten_duong_1 in danh_sach_duong else (ten_duong_2 if ten_duong_2 in danh_sach_duong else None)

if not ten_duong_thuc_te:
    print("Đường ten_duong_1 hoặc ten_duong_2 không tồn tại trong danh sách đường.")
else:
    # 1. Lấy thông tin đường và các điểm tọa độ
    path_info = danh_sach_duong[ten_duong_thuc_te]
    diem_dau = danh_sach_diem[ten_diem_dau][:2]   # Điểm bắt đầu
    diem_cuoi = danh_sach_diem[ten_diem_cuoi][:2]  # Điểm kết thúc
    diem_kiem_soat = danh_sach_diem[path_info[3]][:2]   # Điểm kiểm soát (lấy từ index 3 của path_info)

    # Giả sử hướng hiện tại của AGV lấy từ hệ thống (đổi ra radian)
    huong_hien_tai_rad = math.radians(30)

    # 2. GỌI HÀM TÍNH TOÁN ĐIỀU HƯỚNG
    reached, diem_A, diem_B, diem_Sau_A, goc_target, sai_so = calculate_agv_guidance(
        toa_do_agv, huong_hien_tai_rad, diem_dau, diem_kiem_soat, diem_cuoi, tam_nhin, sai_so_dich, buoc_nhin_xa_huong
    )

    if reached:
        print(f"AGV đã đến đích. Dừng xe.")
    else:
        print(f"--- KẾT QUẢ ĐIỀU KHIỂN ---")
        print(f"Tọa độ AGV: {toa_do_agv}")
        print(f"Điểm mục tiêu A: {diem_A}")
        print(f"Điểm hướng mục tiêu (phía trước A): {diem_Sau_A}")
        print(f"Góc mục tiêu: {math.degrees(goc_target):.2f}°")
        print(f"Giải thích: Đoạn thẳng nối từ A đến Điểm B hợp với Ox {math.degrees(goc_target):.2f}°")
        print(f"Sai số góc cần quay: {math.degrees(sai_so):.2f}°")