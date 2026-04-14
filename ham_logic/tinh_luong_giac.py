import numpy as np

def calculate_distance(A, B):
    """
    Tính khoảng cách Euclidean giữa hai điểm A và B.
    
    Args:
    A (tuple): Tọa độ điểm A (x1, y1).
    B (tuple): Tọa độ điểm B (x2, y2).

    Returns:
    float: Khoảng cách giữa hai điểm A và B.
    """
    distance = np.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)
    return distance

def calculate_angle_cosine(A, B, C):
    """
    Tính góc BAC (góc giữa đoạn thẳng AB và AC) bằng định lý cosin.
    A != B and A != C
    Args:
    A (tuple): Tọa độ điểm A (x1, y1).
    B (tuple): Tọa độ điểm B (x2, y2).
    C (tuple): Tọa độ điểm C (x3, y3).

    Returns:
    float: Góc BAC theo độ.
    """
    output = True
    angle_deg = 0
    if (A[0] != B[0] or A[1] != B[1]) and (A[0] != C[0] or A[1] != C[1]):
        # Tính độ dài các cạnh của tam giác
        AB = calculate_distance(A, B)
        AC = calculate_distance(A, C)
        BC = calculate_distance(B, C)
        
        # Sử dụng định lý cosin để tính góc BAC
        cos_theta = (AB**2 + AC**2 - BC**2) / (2 * AB * AC)
        angle_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # Đảm bảo giá trị nằm trong khoảng [-1, 1]
        
        # Chuyển đổi góc từ radian sang độ
        angle_deg = np.degrees(angle_rad)
        
        # Sử dụng tích chéo để xác định hướng của góc
        AB_vector = (B[0] - A[0], B[1] - A[1])
        AC_vector = (C[0] - A[0], C[1] - A[1])
        cross_product = AB_vector[0] * AC_vector[1] - AB_vector[1] * AC_vector[0]
        
        # Theo quy ước chuẩn, quay trái (CCW) là góc dương.
        if cross_product < 0:
            angle_deg = -angle_deg
    else:
        output = False
    
    return output, angle_deg
def calculate_distance_and_angle(start_point0, end_point0, ang_point0):
    start_point = start_point0.copy()
    end_point = end_point0.copy()
    ang_point = ang_point0.copy()
    output, angle_deg = calculate_angle_cosine(start_point, end_point, ang_point)
    distance = calculate_distance(start_point, end_point)
    return output, distance, angle_deg

def angle_with_ox(A, B):
    """
    Tính góc (độ) giữa đoạn thẳng AB và trục Ox.
    Args:
        A, B: tuple/list (x, y)
    Returns:
        angle_deg: Góc hợp bởi AB và Ox, giá trị trong [-180, 180] độ
    """
    dx = B[0] - A[0]
    dy = B[1] - A[1]
    angle_rad = np.arctan2(dy, dx)
    return angle_rad

def distance_point_to_segment(A, B, C):
    """
    Tính khoảng cách ngắn nhất từ điểm A đến đoạn thẳng BC.

    Args:
        A (tuple/list): Tọa độ điểm A (x, y).
        B (tuple/list): Tọa độ điểm B (x, y) - điểm đầu của đoạn thẳng.
        C (tuple/list): Tọa độ điểm C (x, y) - điểm cuối của đoạn thẳng.

    Returns:
        float: Khoảng cách ngắn nhất từ điểm A đến đoạn thẳng BC.
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    C = np.array(C, dtype=float)

    # Vector từ B đến A (BA) và từ B đến C (BC)
    vec_BA = A - B
    vec_BC = C - B

    # Tính bình phương độ dài của đoạn thẳng BC
    len_sq_BC = np.dot(vec_BC, vec_BC)

    # Nếu B và C trùng nhau, tính khoảng cách từ A đến B (hoặc C)
    if len_sq_BC == 0:
        return calculate_distance(A, B)

    # Tính tham số t (tỉ lệ chiếu)
    # t = 0: chiếu tại B; t = 1: chiếu tại C; 0 < t < 1: chiếu nằm trong đoạn BC
    t = np.dot(vec_BA, vec_BC) / len_sq_BC

    # Xác định điểm gần nhất trên đoạn thẳng và tính khoảng cách
    if t <= 0: # Điểm gần nhất là B
        return calculate_distance(A, B)
    elif t >= 1: # Điểm gần nhất là C
        return calculate_distance(A, C)
    else: # Điểm gần nhất nằm trong đoạn BC (hình chiếu vuông góc)
        point_P = B + t * vec_BC
        return calculate_distance(A, point_P)

# B = (6, 1)
# A = (1, 6)
# print(angle_with_ox(A, B))  # Kết quả: khoảng 53.13 độ
# # Ví dụ sử dụng
# [308.8550496887132, 305.8713588764305] [356, 313] [357, 312]
# A = [-16289.703, -3687.08825]
# B = [-16291, -3679]
# ---------- [-16285, -3681] [-16289.703, -3687.08825] [-16291, -3679] NG OK NG P1 P2 xoay_agv_song_song
# C = [357, 312]

# distance_AB = calculate_distance(A, B)
# angle_BAC = calculate_angle_cosine(A, B, C)

# print(f"Khoảng cách AB: {distance_AB}")
# print(f"Góc BAC: {angle_BAC} độ")
