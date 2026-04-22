import math
import heapq
import os
import json
import config as cfg
from config import AGVConfig
from config_2 import AGVConfig_2



# Directories for saving data and maps
SAVED_DATA_DIR = cfg.PATH_PHAN_MEM + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"
PATH_POINTS_DIR = SAVED_DATA_DIR + "/point_lists"
PATH_PATHS_DIR = SAVED_DATA_DIR + "/path_lists"



DUONG_TANG_CHI_PHI = 50  # Chi phí tăng thêm cho các đường khó đi
CHI_PHI_DIEM_DUNG = 20 # Chi phí cho mỗi điểm dừng/chặng trên đường đi. Có thể điều chỉnh giá trị này.
graph = {}
danh_sach_duong = {}
danh_sach_duong_tang_chi_phi = [] # Ví dụ: đường từ P1 đến P2






# tải danh sách điểm
def load_points_route(filename): # Change parameter name to match the variable rule
    filename_base = filename.replace(".json", "") # Use a new variable name for the base filename
    filepath = os.path.join(PATH_POINTS_DIR, f"{filename_base}.json") # Use filename_base here
    if not os.path.exists(filepath):
        print("e0")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            loaded_points = json.load(f)
        AGVConfig.danh_sach_diem = loaded_points
        # print("danh_sach_diem: ", AGVConfig.danh_sach_diem)
    except Exception as e:
        print("e1")

def load_paths_route(filename_with_ext):
    filename = filename_with_ext.replace(".json", "")
    filepath = os.path.join(PATH_PATHS_DIR, f"{filename}.json")
    if not os.path.exists(filepath):
        print("e0")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            loaded_paths = json.load(f)
        
        # Validate if points for these paths exist in current danh_sach_diem
        valid_paths = {}
        for path_name, path_data in loaded_paths.items():
            # Hỗ trợ cả cấu trúc mới và cũ
            points = path_data[0] if isinstance(path_data[0], list) else path_data
            if len(points) == 2:
                p1_name, p2_name = points
                if p1_name in AGVConfig.danh_sach_diem and p2_name in AGVConfig.danh_sach_diem:
                    valid_paths[path_name] = path_data
                else:
                    print(f"Warning: Path '{path_name}' skipped during load. Point(s) '{p1_name}' or '{p2_name}' not in current point list.")
        
        AGVConfig.danh_sach_duong = valid_paths
        # print("danh_sach_duong: ", AGVConfig.danh_sach_duong)
    except Exception as e:
        print("e1")


def tinh_goc_cuc_bo(A, B, C, danh_sach_diem):
    import math
    xA, yA = danh_sach_diem[A][:2]
    xB, yB = danh_sach_diem[B][:2]
    xC, yC = danh_sach_diem[C][:2]

    v1 = (xB - xA, yB - yA)
    v2 = (xC - xB, yC - yB)

    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.hypot(*v1)
    mag2 = math.hypot(*v2)

    if mag1 == 0 or mag2 == 0:
        return 180

    cos = max(-1, min(1, dot / (mag1 * mag2)))
    return math.degrees(math.acos(cos))

def key_duong_2_chieu(a, b):
    na = int(a.replace("P", ""))
    nb = int(b.replace("P", ""))
    if na < nb:
        return f"{a}_{b}"
    else:
        return f"{b}_{a}"

# danh_sach_diem = {'C1': [1690, 2309, 'không hướng', 0.0], 'P2': [1645, 2309, 'không hướng', 0.0], 'P3': [1675, 2240, 'không hướng', 0.0], 'P4': [1643, 2171, 'không hướng', 0.0], 'P5': [1690, 2174, 'không hướng', 0.0], 'P6': [2527, 2225, 'không hướng', 0.0], 'P7': [2523, 2085, 'không hướng', 0.0], 'P8': [1690, 2115, 'không hướng', 0.0], 'P9': [1660, 2130, 'không hướng', 0.0], 'P10': [1803, 2104, 'không hướng', 0.0], 'P11': [1956, 2099, 'không hướng', 0.0], 'P12': [2105, 2093, 'không hướng', 0.0], 'P13': [2242, 2092, 'không hướng', 0.0], 'P14': [2372, 2087, 'không hướng', 0.0], 'P15': [2385, 2225, 'không hướng', 0.0], 'P16': [2232, 2230, 'không hướng', 0.0], 'P17': [2101, 2229, 'không hướng', 0.0], 'P18': [1955, 2237, 'không hướng', 0.0], 'P19': [1791, 2240, 'không hướng', 0.0], 'P20': [1730, 2108, 'không hướng', 0.0], 'P21': [1652, 2141, 'không hướng', 0.0], 'P22': [1645, 2240, 'không hướng', 0.0]}
def loc_diem_theo_ky_tu_dau(ky_tu, danh_sach_diem):
    """
    Lọc ra các điểm có tên bắt đầu bằng ký tự cho trước.

    Args:
        ky_tu (str): Ký tự đầu cần lọc (ví dụ: 'P', 'C', 'X').
        danh_sach_diem (dict): Dictionary chứa dữ liệu các điểm.

    Returns:
        dict: Dictionary mới chỉ chứa các điểm thỏa mãn điều kiện.
    """
    ket_qua = []
    
    # Duyệt qua từng điểm trong danh sách
    for ten_diem, thong_tin in danh_sach_diem.items():
        # Kiểm tra xem tên điểm có bắt đầu bằng ký tự yêu cầu không
        if ten_diem.startswith(ky_tu):
            ket_qua.append(ten_diem)
            
    return ket_qua

# print(loc_diem_theo_ky_tu_dau("C", danh_sach_diem))


# webserver.danh_sach_diem = {'P1': [1690, 2309, 'không hướng', 0.0], 'P2': [1645, 2309, 'không hướng', 0.0], 'P3': [1675, 2240, 'không hướng', 0.0], 'P4': [1643, 2171, 'không hướng', 0.0], 'P5': [1690, 2174, 'không hướng', 0.0], 'P6': [2527, 2225, 'không hướng', 0.0], 'P7': [2523, 2085, 'không hướng', 0.0], 'P8': [1690, 2115, 'không hướng', 0.0], 'P9': [1660, 2130, 'không hướng', 0.0], 'P10': [1803, 2104, 'không hướng', 0.0], 'P11': [1956, 2099, 'không hướng', 0.0], 'P12': [2105, 2093, 'không hướng', 0.0], 'P13': [2242, 2092, 'không hướng', 0.0], 'P14': [2372, 2087, 'không hướng', 0.0], 'P15': [2385, 2225, 'không hướng', 0.0], 'P16': [2232, 2230, 'không hướng', 0.0], 'P17': [2101, 2229, 'không hướng', 0.0], 'P18': [1955, 2237, 'không hướng', 0.0], 'P19': [1791, 2240, 'không hướng', 0.0], 'P20': [1730, 2108, 'không hướng', 0.0], 'P21': [1652, 2141, 'không hướng', 0.0], 'P22': [1645, 2240, 'không hướng', 0.0]}
# webserver.danh_sach_duong = {'P1_P2': [['P1', 'P2'], 'none'], 'P4_P5': [['P4', 'P5'], 'none'], 'P7_P6': [['P7', 'P6'], 'none'], 'P10_P11': [['P10', 'P11'], 'none'], 'P11_P12': [['P11', 'P12'], 'none'], 'P12_P13': [['P12', 'P13'], 'none'], 'P13_P14': [['P13', 'P14'], 'none'], 'P14_P7': [['P14', 'P7'], 'none'], 'P6_P15': [['P6', 'P15'], 'none'], 'P15_P16': [['P15', 'P16'], 'none'], 'P16_P17': [['P16', 'P17'], 'none'], 'P17_P18': [['P17', 'P18'], 'none'], 'P18_P19': [['P18', 'P19'], 'none'], 'P19_P3': [['P19', 'P3'], 'none'], 'P4_P21': [['P4', 'P21'], 'none'], 'P21_P9': [['P21', 'P9'], 'none'], 'P9_P8': [['P9', 'P8'], 'none'], 'P8_P20': [['P8', 'P20'], 'none'], 'P20_P10': [['P20', 'P10'], 'none'], 'P4_P22': [['P4', 'P22'], 'none'], 'P22_P2': [['P22', 'P2'], 'none'], 'P4_P3': [['P4', 'P3'], 'none'], 'P3_P2': [['P3', 'P2'], 'none']}
# def tu_dong_toi_uu_duong_di(danh_sach_diem, danh_sach_duong, nguong_goc=5):
#     from collections import defaultdict
#     list_loai_3 = cfg.data_di_chuyen_luon["loai_3"]["danh_sach_diem_dich"] loc_diem_theo_ky_tu_dau("P", danh_sach_diem))
#     list_loai_6 = cfg.data_di_chuyen_luon["loai_6"]["danh_sach_diem_dich"]

#     def key_2_chieu(a, b):
#         na, nb = int(a[1:]), int(b[1:])
#         return f"P{min(na, nb)}_P{max(na, nb)}"

#     # tạo graph
#     graph = defaultdict(list)
#     for v in danh_sach_duong.values():
#         a, b = v[0]
#         graph[a].append(b)
#         graph[b].append(a)

#     danh_sach_duong_moi = dict(danh_sach_duong)

#     def dfs(A, prev, current, visited):
#         for nxt in graph[current]:
#             if nxt in visited:
#                 continue

#             goc = tinh_goc_cuc_bo(prev, current, nxt, danh_sach_diem)
#             if goc > nguong_goc:
#                 continue

#             key = key_2_chieu(A, nxt)
#             if key not in danh_sach_duong_moi and A not in list_loai_3 and nxt not in list_loai_3 and A not in list_loai_6 and nxt not in list_loai_6:
#                 danh_sach_duong_moi[key] = [[A, nxt], "none"]

#             visited.add(nxt)
#             dfs(A, current, nxt, visited)

#     for A in graph:
#         for B in graph[A]:
#             visited = {A, B}
#             dfs(A, A, B, visited)

#     return danh_sach_duong_moi

def toi_uu_hoa_duong_di(path, nguong_goc=5):
    """
    Tối ưu hóa đường đi bằng cách loại bỏ các điểm trung gian nằm trên đường thẳng.
    Giữ lại các điểm nằm trong danh sách điểm đặc biệt (từ config).
    Trả về đường đi đã tối ưu và tổng chi phí của nó.
    """
    # list_loai_2 = loc_diem_theo_ky_tu_dau(cfg.data_di_chuyen_luon["loai_2"]["danh_sach_diem_dich"][0], webserver.danh_sach_diem)
    list_loai_2 = []
    # list_loai_3 = loc_diem_theo_ky_tu_dau(AGVConfig_2.data_di_chuyen_luon["loai_3"]["danh_sach_diem_dich"][0], AGVConfig.danh_sach_diem)
    list_loai_3 = ["G12", "G13", "G14", "G15"]

    danh_sach_diem_dac_biet = set(list_loai_2 + list_loai_3)

    if not path:
        return [], 0.0

    # Nếu đường đi quá ngắn để tối ưu, chỉ tính chi phí
    if len(path) < 3:
        cost = 0.0
        if len(path) == 2:
            cost = get_edge_cost(path[0], path[1])
        return path, cost

    optimized_path = [path[0]]
    total_cost = 0.0
    current_idx = 0

    while current_idx < len(path) - 1:
        best_k = current_idx + 1
        
        p_start = path[current_idx]
        p_next = path[current_idx+1]
        
        if p_start not in AGVConfig.danh_sach_diem or p_next not in AGVConfig.danh_sach_diem:
             optimized_path.append(p_next)
             # Tính chi phí cho đoạn không tối ưu được
             total_cost += get_edge_cost(p_start, p_next)
             current_idx += 1
             continue

        x_start, y_start = AGVConfig.danh_sach_diem[p_start][:2]
        x_next, y_next = AGVConfig.danh_sach_diem[p_next][:2]
        v_base = (x_next - x_start, y_next - y_start)

        # Duyệt các điểm tiếp theo để xem có thể đi thẳng tới đâu
        for k in range(current_idx + 2, len(path)):
            p_middle = path[k-1]
            p_target = path[k]

            # Điều kiện 1: Điểm giữa không được là điểm đặc biệt
            if p_middle in danh_sach_diem_dac_biet:
                break
            
            if p_middle not in AGVConfig.danh_sach_diem or p_target not in AGVConfig.danh_sach_diem:
                break

            # Điều kiện 2: Góc cục bộ (P_prev -> P_middle -> P_target) nhỏ hơn ngưỡng
            p_prev = path[k-2]
            goc_cuc_bo = tinh_goc_cuc_bo(p_prev, p_middle, p_target, AGVConfig.danh_sach_diem)
            if goc_cuc_bo > nguong_goc:
                break

            # Điều kiện 3: Góc giữa vector gốc (P_start -> P_next) và vector tới đích (P_start -> P_target) nhỏ hơn ngưỡng
            x_target, y_target = AGVConfig.danh_sach_diem[p_target][:2]
            v_target = (x_target - x_start, y_target - y_start)
            
            dot = v_base[0]*v_target[0] + v_base[1]*v_target[1]
            mag_base = math.hypot(*v_base)
            mag_target = math.hypot(*v_target)
            
            angle_deviation = 0
            if mag_base > 0 and mag_target > 0:
                cos_val = max(-1, min(1, dot / (mag_base * mag_target)))
                angle_deviation = math.degrees(math.acos(cos_val))
            
            if angle_deviation > nguong_goc:
                break
            
            best_k = k
        
        segment_start = path[current_idx]
        segment_end = path[best_k]

        optimized_path.append(segment_end)
        total_cost += get_edge_cost(segment_start, segment_end)
        
        current_idx = best_k
        
    return optimized_path, total_cost



# --- Tạo graph có hướng ---
def tao_graph():
    global graph, danh_sach_duong
    danh_sach_duong = AGVConfig.danh_sach_duong.copy()
    graph = {}
    for key, value in danh_sach_duong.items():
        (a, b) = value[0]
        direction = value[1]

        if direction == "none" or direction == "curve":  # Đường 2 chiều
            graph.setdefault(a, []).append(b)
            graph.setdefault(b, []).append(a)
        else:
            # direction dạng "P1-P4" -> chỉ có hướng P1 → P4
            start, end = direction.split("-")
            graph.setdefault(start, []).append(end)
    # print("graph: ", graph)
    return graph

# graph:  {'C71': ['C21', 'G3'], 'C21': ['C71', 'C70'], 
#          'C70': ['C21', 'C9'], 'C68': ['C20', 'C73'], 
#          'C20': ['C68', 'C67'], 'C67': ['C20', 'E54'], 
#          'E54': ['C67', 'P10', 'P72'], 'P10': ['E54'], 
#          'P72': ['P11', 'E54'], 'P11': ['P72', 'P12'], 
#          'P12': ['P11', 'P13'], 'P13': ['P12', 'P14'], 'P14': ['P13', 'G7'], 'W3': ['X3', 'G34'], 'X3': ['W3'], 'P43': ['P42', 'G34'], 'P42': ['P43', 'P41'], 'P41': ['P42', 'P40'], 'P40': ['P41', 'P39'], 'P39': ['P40', 'E37'], 'E37': ['P39', 'C66'], 'C66': ['E37', 'C65'], 'C65': ['C66', 'C55'], 'C55': ['C65', 'C64'], 'C64': ['C55', 'C63'], 'C63': ['C64', 'C38'], 'C38': ['C63', 'C62'], 'C62': ['C38', 'C61'], 'C61': ['C62', 'C32'], 'C32': ['C61', 'C60'], 'C60': ['C32', 'C59'], 'C59': ['C60', 'C31'], 'C31': ['C59', 'C58'], 'C58': ['C31', 'C27'], 'C27': ['C58', 'G29'], 'P44': ['P45', 'G29'], 'P45': ['P44', 'P46'], 'P46': ['P45', 'P47'], 'P48': ['P47', 'G35'], 'P47': ['P48', 'P46'], 'P53': ['P52', 'G36'], 'P52': ['P53', 'P51'], 'P51': ['P52', 'P50'], 'P50': ['P51', 'P49'], 'P49': ['P50', 'G28'], 'P19': ['P18', 'G3'], 'P18': ['P19', 'P17'], 'P17': ['P18', 'P16'], 'P16': ['P17', 'P15'], 'P15': ['P16', 'G6'], 'W4': ['X2', 'G29', 'G28'], 'X2': ['W4'], 'W2': ['X1', 'G3', 'G28'], 'X1': ['W2'], 'G29': ['C27', 'P44', 'W4'], 'G34': ['W3', 'P43', 'G35'], 'G35': ['P48', 'G34', 'G36'], 'G36': ['P53', 'G35'], 'G6': ['P15'], 'G7': ['P14'], 'C9': ['C70', 'C75'], 'C75': ['C9', 'C69'], 'C69': ['C75', 'C74'], 'C74': ['C69', 'C8'], 'C8': ['C74', 'C73'], 'C73': ['C8', 'C68'], 'G3': ['W2', 'C71', 'P19'], 'G28': ['W4', 'W2', 'P49']}

# --- Hàm tính khoảng cách Euclid ---
def heuristic(p1, p2):
    # print(p1,p2)
    # print("AGVConfig.danh_sach_diem", AGVConfig.danh_sach_diem)
    x1, y1 = AGVConfig.danh_sach_diem[p1][:2]
    x2, y2 = AGVConfig.danh_sach_diem[p2][:2]
    return math.hypot(x2 - x1, y2 - y1)

def get_edge_cost(p1, p2):
    """
    Tính chi phí thực tế để đi từ p1 đến p2, bao gồm cả chi phí tăng thêm.
    """
    # Chi phí cơ bản = khoảng cách + chi phí cho một điểm dừng
    cost = heuristic(p1, p2) + CHI_PHI_DIEM_DUNG

    # # Tạo tên đường chuẩn hóa để kiểm tra
    # num1 = int(p1.replace('P', ''))
    # num2 = int(p2.replace('P', ''))
    # if num1 > num2:
    #     edge_name = f"P{num2}_P{num1}"
    # else:
    #     edge_name = f"P{num1}_P{num2}"

    # Nếu đường này nằm trong danh sách tăng chi phí, cộng thêm chi phí
    # if edge_name in danh_sach_duong_tang_chi_phi:
    #     cost += DUONG_TANG_CHI_PHI
    return cost
# --- Thuật toán A* ---
def _a_star_internal(start, goal, vat_can_diem=None, vat_can_duong=None):
    """
    Hàm nội bộ để tìm đường đi ngắn nhất với các vật cản cho trước.
    """
    global danh_sach_duong
    # print(len(save_open.danh_sach_duong))
    vat_can_diem = set(vat_can_diem or [])
    vat_can_duong = set(vat_can_duong or [])

    if start in vat_can_diem or goal in vat_can_diem:
        return None, float('inf')
    # Sao chép graph có hướng (và loại bỏ vật cản)
    graph_copy = {n: [x for x in v if x not in vat_can_diem]
                  for n, v in graph.items() if n not in vat_can_diem}

    # Loại bỏ các đường bị chặn
    for duong in vat_can_duong:
        if duong in danh_sach_duong:
            (a, b), direction = danh_sach_duong[duong]
            if direction == "none":  # 2 chiều
                if a in graph_copy and b in graph_copy[a]:
                    graph_copy[a].remove(b)
                if b in graph_copy and a in graph_copy[b]:
                    graph_copy[b].remove(a)
            else:
                start_dir, end_dir = direction.split("-")
                if start_dir in graph_copy and end_dir in graph_copy[start_dir]:
                    graph_copy[start_dir].remove(end_dir)

    # Khởi tạo A*
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {p: float('inf') for p in AGVConfig.danh_sach_diem}
    g_score[start] = 0
    f_score = {p: float('inf') for p in AGVConfig.danh_sach_diem}
    f_score[start] = heuristic(start, goal)

    # --- Vòng lặp A* ---
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            # Truy vết đường đi
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            total_cost = g_score[goal]
            return path, total_cost

        for neighbor in graph_copy.get(current, []):
            # Sử dụng hàm get_edge_cost để tính chi phí cạnh, bao gồm cả chi phí tăng thêm
            # print(current, neighbor)
            tentative_g = g_score[current] + get_edge_cost(current, neighbor)
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # Không tìm thấy đường
    return None, float('inf')


def a_star(start, goal, vat_can_diem=None, vat_can_duong=None):
    """
    Tìm đường đi ngắn nhất trong hai trường hợp: lý tưởng (không vật cản) và thực tế (có vật cản).

    Returns:
        tuple: (path_ideal, cost_ideal, path_actual, cost_actual, cost_diff)
        - path_ideal (list | None): Đường đi ngắn nhất không có vật cản.
        - cost_ideal (float): Chi phí của đường đi lý tưởng.
        - path_actual (list | None): Đường đi ngắn nhất có xét vật cản.
        - cost_actual (float): Chi phí của đường đi thực tế.
        - cost_diff (float): Chênh lệch chi phí (cost_actual - cost_ideal).
    """
    # 1. Tìm đường đi và chi phí lý tưởng (không có vật cản)
    path_ideal, cost_ideal = _a_star_internal(start, goal, [], [])

    # 2. Tìm đường đi và chi phí thực tế (với vật cản hiện tại)
    path_actual, cost_actual = _a_star_internal(start, goal, vat_can_diem, vat_can_duong)

    # 3. Tính toán chênh lệch chi phí
    cost_diff = float('inf')
    if cost_ideal != float('inf') and cost_actual != float('inf'):
        cost_diff = cost_actual - cost_ideal
    if path_actual == None:
        path_actual = []
    return path_ideal, cost_ideal, path_actual, cost_actual, cost_diff





def tao_lenh_di_chuyen(graph, paths,
                       trang_thai_hien_tai, trang_thai_cuoi,
                       force_ha=False):
    # các trạng thái có trong list_data: nang, ha, lay_hang, tra_hang, xoay_agv_song_song, xoay_agv_vuong_goc
    list_data = {}
    kiem_tra_loi = ""
    danh_sach_vi_tri_xe = [v[0] for v in AGVConfig.thong_tin_lay_tra_hang.values()]

    # =====================
    # FORCE MODE: luôn ha
    # =====================
    if force_ha:
        for i in range(len(paths) - 2):
            list_data[f"data{i+1}"] = [
                [paths[i], paths[i+1]],
                "ha",
                [paths[i+1], paths[i+2]],
                "ha"
            ]
        # thêm 1 phần tử nữa vào list_data có dạng [C D C D] trong đó C D là 2 giá trị cuối của phần tử cuối trong list_data
        if len(paths) >= 2:
            list_data[f"data{len(paths)-1}"] = [
                [paths[-2], paths[-1]],
                "ha",
                [paths[-2], paths[-1]],
                "ha"
            ]
        if len(paths) == 1:
            list_data[f"data1"] = [
                [paths[-1], paths[-1]],
                "ha",
                [paths[-1], paths[-1]],
                "ha"
            ]
        return "", list_data

    # =====================
    # KIỂM TRA ĐƯỜNG ĐI
    # =====================
    for i in range(len(paths) - 1):
        if paths[i+1] not in graph.get(paths[i], []):
            return f"Đường đi không hợp lệ: {paths[i]} -> {paths[i+1]}", {}

    # =====================
    # KIỂM TRA NGHIỆP VỤ
    # =====================
    if trang_thai_cuoi == "lay_hang" and trang_thai_hien_tai != "ha":
        return "xe đang có hàng không thể lấy hàng", {}

    if trang_thai_cuoi == "tra_hang" and trang_thai_hien_tai != "nang":
        return "xe không có hàng để trả", {}

    # nếu điểm đầu hoặc điểm cuối trong paths đều không nằm trong danh_sach_vi_tri_xe và trạng thái khác trang thái hiện tại
    if paths[0] not in danh_sach_vi_tri_xe and paths[-1] not in danh_sach_vi_tri_xe and trang_thai_cuoi != trang_thai_hien_tai:
        return "trạng thái không hợp lệ do điểm đầu và điểm cuối không phải vị trí lấy/ trả hàng hoặc trạng thái đồng nhất", {}

    # nếu trạng thái ==  nâng còn trạng thái hiện tại == hạ hoặc nguuwocj lại
    if (trang_thai_cuoi == "nang" and trang_thai_hien_tai == "ha") or (trang_thai_cuoi == "ha" and trang_thai_hien_tai == "nang"):
        return "trạng thái không hợp lệ do trạng thái hiện tại và trạng thái yêu cầu không thể chuyển đổi trực tiếp", {}
    
    # nếu điểm cuối là vị trí lấy hàng nhưng trạng thái là hạ hoặc nang
    if paths[-1] in danh_sach_vi_tri_xe and (trang_thai_cuoi == "ha" or trang_thai_cuoi == "nang"):
        return "trạng thái không hợp lệ do điểm cuối là vị trí lấy hàng nhưng trạng thái yêu cầu không phù hợp", {}
    
    # nếu điểm cuối không phải là vị trí lấy trả nhưng trạng thái lại là lấy hàng hoặc trả hàng hoặc hạ
    if paths[-1] not in danh_sach_vi_tri_xe and (trang_thai_cuoi == "lay_hang" or trang_thai_cuoi == "tra_hang" or trang_thai_cuoi == "ha"):
        return "trạng thái không hợp lệ do điểm cuối không phải vị trí lấy/trả hàng nhưng trạng thái yêu cầu là lấy/trả hàng/hạ", {}
        



    

    # =====================
    # TRƯỜNG HỢP 3 (GIỮ NGUYÊN NHƯ BAN ĐẦU)
    # =====================
    # --------------------------------- trường hợp paths không có điểm nào nằm trong danh_sach_vi_tri_xe ---------------------------------
    if not any(p in danh_sach_vi_tri_xe for p in paths):
        print("loai_1")
        for i in range(len(paths) - 2):
            list_data[f"data{i+1}"] = [
                [paths[i], paths[i+1]],
                trang_thai_hien_tai,
                [paths[i+1], paths[i+2]],
                trang_thai_hien_tai
            ]
        # thêm 1 phần tử nữa vào list_data có dạng [C D C D] trong đó C D là 2 giá trị cuối của phần tử cuối trong list_data
        if len(paths) >= 2:
            list_data[f"data{len(paths)-1}"] = [
                [paths[-2], paths[-1]],
                trang_thai_hien_tai,
                [paths[-2], paths[-1]],
                trang_thai_hien_tai
            ]
        if len(paths) == 1:
            list_data[f"data1"] = [
                [paths[-1], paths[-1]],
                trang_thai_hien_tai,
                [paths[-1], paths[-1]],
                trang_thai_hien_tai
            ]
        return "", list_data

    # --------------------------------- trường hợp điểm cuối của paths nằm trong danh_sach_vi_tri_xe và trạng thái là lấy hàng ---------------------------------
    if paths[-1] in danh_sach_vi_tri_xe and trang_thai_cuoi == "lay_hang":
        print("loai_2")
        for i in range(len(paths) - 2):
            if i == len(paths) - 3:
                list_data[f"data{i+1}"] = [
                    [paths[i], paths[i+1]],
                    trang_thai_hien_tai,
                    [paths[i+1], paths[i+2]],
                    "lay_hang"
                ]
            else:
                list_data[f"data{i+1}"] = [
                    [paths[i], paths[i+1]],
                    trang_thai_hien_tai,
                    [paths[i+1], paths[i+2]],
                    trang_thai_hien_tai
                ]
        # thêm 1 phần tử nữa vào list_data có dạng [C D C D] trong đó C D là 2 giá trị cuối của phần tử cuối trong list_data
        if len(paths) >= 2:
            list_data[f"data{len(paths)-1}"] = [
                [paths[-2], paths[-1]],
                "lay_hang",
                [paths[-2], paths[-1]],
                "lay_hang"
            ]
        if len(paths) == 1:
            list_data[f"data1"] = [
                [paths[-1], paths[-1]],
                "lay_hang",
                [paths[-1], paths[-1]],
                "lay_hang"
            ]
        return "", list_data
    
    # --------------------------------- trường hợp điểm đầu của paths nằm trong danh_sach_vi_tri_xe và trạng thái là nang ---------------------------------
    if paths[0] in danh_sach_vi_tri_xe and trang_thai_cuoi == "nang":
        print("loai_3")
        for i in range(len(paths) - 2):
            if i == 0:
                list_data[f"data{i+1}"] = [
                    [paths[i], paths[i+1]],
                    "xoay_agv_song_song",
                    [paths[i+1], paths[i+2]],
                    trang_thai_hien_tai
                ]
            else:
                list_data[f"data{i+1}"] = [
                    [paths[i], paths[i+1]],
                    trang_thai_hien_tai,
                    [paths[i+1], paths[i+2]],
                    trang_thai_hien_tai
                ]
        # thêm 1 phần tử nữa vào list_data có dạng [C D C D] trong đó C D là 2 giá trị cuối của phần tử cuối trong list_data
        if len(paths) >= 2:
            list_data[f"data{len(paths)-1}"] = [
                [paths[-2], paths[-1]],
                trang_thai_hien_tai,
                [paths[-2], paths[-1]],
                trang_thai_hien_tai
            ]
        if len(paths) == 1:
            list_data[f"data1"] = [
                [paths[-1], paths[-1]],
                trang_thai_hien_tai,
                [paths[-1], paths[-1]],
                trang_thai_hien_tai
            ]
        return "", list_data

    # --------------------------------- trường hợp điểm cuối của paths nằm trong danh_sach_vi_tri_xe và trạng thái là trả hàng ---------------------------------
    # tại điểm cuối "P18" trạng thái sẽ là hạ
    if paths[-1] in danh_sach_vi_tri_xe and trang_thai_cuoi == "tra_hang":
        print("loai_4")
        if len(paths) == 1:
            list_data[f"data1"] = [
                [paths[-1], paths[-1]],
                "tra_hang",
                [paths[-1], paths[-1]],
                "tra_hang"
            ]
        elif len(paths) == 2:
            p0, p1 = paths[0], paths[1]
            list_data['data1'] = [[p0, p0], 'xoay_agv_vuong_goc', [p0, p1], 'ha']
            list_data['data2'] = [[p0, p1], 'ha', [p0, p1], 'ha']
        elif len(paths) == 3:
            p0, p1, p2 = paths[0], paths[1], paths[2]
            list_data['data1'] = [[p0, p1], 'xoay_agv_vuong_goc', [p1, p2], 'ha']
            list_data['data2'] = [[p1, p2], 'ha', [p1, p2], 'ha']
        else: # len(paths) > 3
            if paths[0] in danh_sach_vi_tri_xe:
                trang_thai_truoc_do = "xoay_agv_song_song" # Bắt đầu từ vị trí đặc biệt
            else:
                trang_thai_truoc_do = trang_thai_hien_tai # Bắt đầu từ một điểm thường trên đường

            trang_thai_sau = ""
            for i in range(len(paths) - 2):
                trang_thai_giua = trang_thai_truoc_do
                trang_thai_sau = trang_thai_hien_tai # Mặc định là 'nang'

                if i == len(paths) - 3: # Chặng áp chót
                    trang_thai_sau = "ha"
                elif i == len(paths) - 4: # Chặng trước chặng áp chót
                    trang_thai_sau = "xoay_agv_vuong_goc"

                list_data[f"data{i+1}"] = [
                    [paths[i], paths[i+1]],
                    trang_thai_giua,
                    [paths[i+1], paths[i+2]],
                    trang_thai_sau
                ]
                trang_thai_truoc_do = trang_thai_sau # Cập nhật trạng thái cho chặng tiếp theo
            
            if len(paths) >= 2:
                list_data[f"data{len(paths)-1}"] = [
                    [paths[-2], paths[-1]],
                    trang_thai_sau,
                    [paths[-2], paths[-1]],
                    trang_thai_sau
                ]
        return "", list_data

    # =====================
    # CÁC TRƯỜNG HỢP CÒN LẠI
    # =====================
    kiem_tra_loi = "Lỗi chưa xác định trong tạo lệnh di chuyển"
    return kiem_tra_loi, {}


def tao_thong_tin_nang_ha(graph, p_actual=None):
    # các trạng thái có thể có: lay_hang, nghi, xoay_agv_vuong_goc, xoay_agv_song_song
    data = {}
    danh_sach_vi_tri_xe = [v[0] for v in AGVConfig.thong_tin_lay_tra_hang.values()]
    idx = 1

    # các điểm nối trực tiếp với điểm lấy/trả
    diem_ke = set()
    for p in danh_sach_vi_tri_xe:
        diem_ke.update(graph.get(p, []))
    
    points_in_path = set(p_actual) if p_actual else None

    # -------------------------
    # DỮ LIỆU TẠI ĐIỂM LẤY/TRẢ
    # -------------------------
    for ten_diem in danh_sach_vi_tri_xe:
        for diem_huong in graph.get(ten_diem, []):
            data[f"data{idx}"] = {
                "ten_diem": ten_diem,
                "trang_thai": "lay_hang",
                "diem_huong": diem_huong,
                "0_song_song_1_vuong_goc": 1
            }
            idx += 1

            data[f"data{idx}"] = {
                "ten_diem": ten_diem,
                "trang_thai": "nghi",
                "diem_huong": diem_huong,
                "0_song_song_1_vuong_goc": 1
            }
            idx += 1

    # -------------------------
    # DỮ LIỆU TẠI ĐIỂM KỀ
    # -------------------------
    for ten_diem in diem_ke:
        for diem_huong in graph.get(ten_diem, []):

            if points_in_path and diem_huong not in points_in_path:
                continue

            # quay về điểm lấy/trả
            if diem_huong in danh_sach_vi_tri_xe:
                data[f"data{idx}"] = {
                    "ten_diem": ten_diem,
                    "trang_thai": "xoay_agv_vuong_goc",
                    "diem_huong": diem_huong,
                    "0_song_song_1_vuong_goc": 1
                }
                idx += 1

            # đi ra ngoài
            elif diem_huong not in danh_sach_vi_tri_xe:
                data[f"data{idx}"] = {
                    "ten_diem": ten_diem,
                    "trang_thai": "xoay_agv_song_song",
                    "diem_huong": diem_huong,
                    "0_song_song_1_vuong_goc": 0
                }
                idx += 1
    return data

def ve_debug_duong_di(
    danh_sach_diem,
    danh_sach_duong_goc,
    danh_sach_duong_moi,
    scale=1,
    show_name=True
):
    import cv2
    import numpy as np

    # ===== tạo canvas =====
    xs = [v[0] for v in danh_sach_diem.values()]
    ys = [v[1] for v in danh_sach_diem.values()]

    w = int(max(xs) * scale + 100)
    h = int(max(ys) * scale + 100)

    img = np.zeros((h, w, 3), dtype=np.uint8)

    def pt(p):
        x, y = danh_sach_diem[p][:2]
        return int(x * scale), int(y * scale)

    # ===== vẽ đường gốc (màu xanh) =====
    for v in danh_sach_duong_goc.values():
        a, b = v[0]
        cv2.line(
            img,
            pt(a),
            pt(b),
            (255, 0, 0),   # xanh
            1
        )

    # ===== vẽ đường được thêm (màu đỏ) =====
    for k, v in danh_sach_duong_moi.items():
        if k not in danh_sach_duong_goc:
            a, b = v[0]
            cv2.line(
                img,
                pt(a),
                pt(b),
                (0, 0, 255),  # đỏ
                1
            )

    # ===== vẽ điểm =====
    for name in danh_sach_diem:
        x, y = pt(name)
        cv2.circle(img, (x, y), 6, (255, 255, 255), -1)
        if show_name:
            cv2.putText(
                img,
                name,
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1
            )

    cv2.imshow("DEBUG DUONG DI (Xanh: goc | Do: toi uu)", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




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


# --- Ví dụ sử dụng ---
# if __name__ == "__main__":
#     tin_hieu_nhan = {
#                     "agv1": {"vi_tri_hien_tai": "P23", "dich_den": "P5", "trang_thai": "tra_hang", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv2": {"vi_tri_hien_tai": "P2", "dich_den": "P2", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv3": {"vi_tri_hien_tai": "P3", "dich_den": "P3", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv4": {"vi_tri_hien_tai": "P4", "dich_den": "P4", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv5": {"vi_tri_hien_tai": "P5", "dich_den": "P5", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv6": {"vi_tri_hien_tai": "P6", "dich_den": "P6", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0},
#                     "agv7": {"vi_tri_hien_tai": "P7", "dich_den": "P7", "trang_thai": "run", "message": "None", "danh_sach_duong_di": [], "nang_ha": "none", "di_chuyen_khong": 0}
#                 } # test
#     name_agv = "agv1"
#     start_node = tin_hieu_nhan[name_agv]["vi_tri_hien_tai"] # 'P1'
#     goal_node = tin_hieu_nhan[name_agv]["dich_den"] # 'P18'
#     trang_thai = tin_hieu_nhan[name_agv]["trang_thai"] # 'lay_hang'



#     load_points_route("t1.json")
#     load_paths_route("t1.json")
#     # webserver.danh_sach_duong = tu_dong_toi_uu_duong_di(webserver.danh_sach_diem, webserver.danh_sach_duong, nguong_goc=10)
#     print("danh_sach_diem", webserver.danh_sach_diem)
#     print("danh_sach_duong", webserver.danh_sach_duong)
#     t = time.time()
#     tao_graph()
#     print("time", time.time() - t)
#     print("\n")
#     print("graph", graph)
    
#     obstacles_points = []
#     obstacles_paths = []

#     print(f"Tìm đường từ {start_node} -> {goal_node} với vật cản điểm {obstacles_points} và đường {obstacles_paths}")

#     p_ideal, c_ideal, p_actual, c_actual, c_diff = a_star(start_node, goal_node, vat_can_diem=obstacles_points, vat_can_duong=obstacles_paths)

#     print(f"\nĐường đi lý tưởng: {p_ideal} (Chi phí: {c_ideal:.2f})")
#     print(f"Đường đi thực tế: {p_actual} (Chi phí: {c_actual:.2f})")
#     print(f"Chi phí chênh lệch: {c_diff:.2f}")

#     err, data = tao_lenh_di_chuyen(
#         graph,
#         p_actual,
#         trang_thai_hien_tai="nang",
#         trang_thai=trang_thai,
#         force_ha=False
#     )

#     print("err", err)
#     print("data11111", data, len(data))

#     data = tao_thong_tin_nang_ha(graph, p_actual)
#     print("data", data)

    # {'data1': {'ten_diem': 'P1', 'trang_thai': 'lay_hang', 'diem_huong': 'P2', '0_song_song_1_vuong_goc': 1}, 
    # 'data2': {'ten_diem': 'P1', 'trang_thai': 'nghi', 'diem_huong': 'P2', '0_song_song_1_vuong_goc': 1}, 
    # 'data3': {'ten_diem': 'P18', 'trang_thai': 'lay_hang', 'diem_huong': 'P19', '0_song_song_1_vuong_goc': 1}, 
    # 'data4': {'ten_diem': 'P18', 'trang_thai': 'nghi', 'diem_huong': 'P19', '0_song_song_1_vuong_goc': 1}, 
    # 'data5': {'ten_diem': 'P19', 'trang_thai': 'xoay_agv_song_song', 'diem_huong': 'P3', '0_song_song_1_vuong_goc': 0}, 
    # 'data6': {'ten_diem': 'P19', 'trang_thai': 'xoay_agv_song_song', 'diem_huong': 'P20', '0_song_song_1_vuong_goc': 0}, 
    # 'data7': {'ten_diem': 'P19', 'trang_thai': 'xoay_agv_vuong_goc', 'diem_huong': 'P18', '0_song_song_1_vuong_goc': 1}, 
    # 'data8': {'ten_diem': 'P2', 'trang_thai': 'xoay_agv_vuong_goc', 'diem_huong': 'P1', '0_song_song_1_vuong_goc': 1}, 
    # 'data9': {'ten_diem': 'P2', 'trang_thai': 'xoay_agv_song_song', 'diem_huong': 'P3', '0_song_song_1_vuong_goc': 0}, 
    # 'data10': {'ten_diem': 'P2', 'trang_thai': 'xoay_agv_song_song', 'diem_huong': 'P17', '0_song_song_1_vuong_goc': 0}}

    # data {'data1': {'ten_diem': 'P1', 'trang_thai': 'lay_hang', 'diem_huong': 'P2', '0_song_song_1_vuong_goc': 1}, 
    # 'data2': {'ten_diem': 'P1', 'trang_thai': 'nghi', 'diem_huong': 'P2', '0_song_song_1_vuong_goc': 1}, 
    # 'data3': {'ten_diem': 'P5', 'trang_thai': 'lay_hang', 'diem_huong': 'P4', '0_song_song_1_vuong_goc': 1}, 
    # 'data4': {'ten_diem': 'P5', 'trang_thai': 'nghi', 'diem_huong': 'P4', '0_song_song_1_vuong_goc': 1}, 
    # 'data5': {'ten_diem': 'P56', 'trang_thai': 'lay_hang', 'diem_huong': 'P57', '0_song_song_1_vuong_goc': 1}, 
    # 'data6': {'ten_diem': 'P56', 'trang_thai': 'nghi', 'diem_huong': 'P57', '0_song_song_1_vuong_goc': 1}, 
    # 'data7': {'ten_diem': 'P57', 'trang_thai': 'xoay_agv_vuong_goc', 'diem_huong': 'P56', '0_song_song_1_vuong_goc': 1}, 
    # 'data8': {'ten_diem': 'P57', 'trang_thai': 'xoay_agv_song_song', 'diem_huong': 'P34', '0_song_song_1_vuong_goc': 0}}
