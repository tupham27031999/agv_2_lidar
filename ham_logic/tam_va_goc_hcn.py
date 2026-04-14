import numpy as np
import cv2
import math
from ham_logic import tinh_luong_giac

def get_trolley_pose(points, current_agv_pose, direction="tren_duoi"):
    """
    Tính tâm và góc của xe đẩy dựa trên tập hợp điểm LiDAR quét chân xe.

    Args:
        points: list hoặc ndarray Nx3, mỗi điểm [x, y, 1] (đơn vị mét)
        current_agv_pose: list hoặc tuple [x, y] vị trí hiện tại của AGV (mét)
        direction (str): "tren_duoi" hoặc "trai_phai" để chọn điểm trả về.

    Returns:
        center: [x, y] tâm hình chữ nhật (mét)
        angle_deg: góc (degree) giữa cạnh dài của hình chữ nhật và trục Ox (từ -90 đến 90)
        closest_midpoint: [x, y] tâm của cạnh gần AGV nhất theo `direction` (mét)
    """
    points = np.array(points)
    if points.shape[0] < 2:
        return None, None, None

    # chỉ lấy 2 cột đầu [x, y]
    xy = points[:, :2]

    # cv2.minAreaRect yêu cầu float32 hoặc int32
    pts_cv = (xy * 1000).astype(np.int32)  # scale lên mm để tránh float precision

    # minAreaRect trả về ((center_x, center_y), (width, height), angle)
    rect = cv2.minAreaRect(pts_cv)
    (cx_mm, cy_mm), (w_mm, h_mm), angle = rect
    
    # convert center về meters
    center = [cx_mm / 1000.0, cy_mm / 1000.0]

    # OpenCV minAreaRect angle semantics:
    # - nếu width < height, angle = góc giữa width (cạnh ngắn) và trục Ox
    # - nếu width >= height, angle = góc giữa dài và trục Ox - 90 độ
    # Chúng ta muốn góc dựa trên **cạnh dài**
    if w_mm < h_mm:
        angle_deg = angle  # cạnh dài ~ height
    else:
        angle_deg = angle + 90  # width là cạnh dài

    # chuẩn hóa góc về [-90, 90]
    if angle_deg > 90:
        angle_deg -= 180
    if angle_deg < -90:
        angle_deg += 180

    # Lấy 4 điểm góc của hình chữ nhật (đơn vị mm)
    box_mm = cv2.boxPoints(rect)

    # Sắp xếp các điểm góc theo thứ tự nhất định (ví dụ: theo chiều kim đồng hồ)
    # để dễ dàng xác định các cạnh đối diện.
    # Sắp xếp dựa trên góc so với tâm.
    angles_from_center = np.arctan2(box_mm[:, 1] - cy_mm, box_mm[:, 0] - cx_mm)
    sorted_indices = np.argsort(angles_from_center)
    box_sorted_mm = box_mm[sorted_indices]

    # Tính toán trung điểm của 4 cạnh (đơn vị mm)
    mid1 = (box_sorted_mm[0] + box_sorted_mm[1]) / 2.0
    mid2 = (box_sorted_mm[1] + box_sorted_mm[2]) / 2.0
    mid3 = (box_sorted_mm[2] + box_sorted_mm[3]) / 2.0
    mid4 = (box_sorted_mm[3] + box_sorted_mm[0]) / 2.0

    # Xác định đâu là cạnh dài, đâu là cạnh ngắn
    dist13 = np.linalg.norm(mid1 - mid3)
    dist24 = np.linalg.norm(mid2 - mid4)

    if dist13 > dist24: # Cạnh 1-3 là cạnh dài
        long_mids = [mid1, mid3]
        short_mids = [mid2, mid4]
    else: # Cạnh 2-4 là cạnh dài
        long_mids = [mid2, mid4]
        short_mids = [mid1, mid4]

    # Sắp xếp và gán nhãn cho các trung điểm, sau đó chuyển về mét
    side_midpoints = {
        "tren": sorted(long_mids, key=lambda p: p[1], reverse=True)[0] / 1000.0,
        "duoi": sorted(long_mids, key=lambda p: p[1])[0] / 1000.0,
        "trai": sorted(short_mids, key=lambda p: p[0], reverse=True)[0] / 1000.0,
        "phai": sorted(short_mids, key=lambda p: p[0])[0] / 1000.0,
    }

    # Dựa vào direction để chọn điểm gần nhất trả về
    closest_midpoint = None
    if direction == "tren_duoi":
        dist_tren = tinh_luong_giac.calculate_distance(current_agv_pose, side_midpoints["tren"])
        dist_duoi = tinh_luong_giac.calculate_distance(current_agv_pose, side_midpoints["duoi"])
        if dist_tren < dist_duoi:
            closest_midpoint = side_midpoints["tren"]
        else:
            closest_midpoint = side_midpoints["duoi"]
    elif direction == "trai_phai":
        dist_trai = tinh_luong_giac.calculate_distance(current_agv_pose, side_midpoints["trai"])
        dist_phai = tinh_luong_giac.calculate_distance(current_agv_pose, side_midpoints["phai"])
        if dist_trai < dist_phai:
            closest_midpoint = side_midpoints["trai"]
        else:
            closest_midpoint = side_midpoints["phai"]


    return center, angle_deg, closest_midpoint

# --- Test ---
if __name__ == "__main__":
    # ví dụ 4 chân xe đẩy
    points = [
        [0.05, 0.05, 1],
        [0.95, 0.05, 1],
        [0.05, 0.95, 1],
        [0.95, 0.95, 1],
    ]

    agv_pos = [0.5, -0.2] # Vị trí giả định của AGV, gần cạnh dưới hơn

    # Test trường hợp "tren_duoi"
    center, angle, closest_midpoint_td = get_trolley_pose(points, agv_pos, "tren_duoi")
    print("Tâm hình chữ nhật:", center)
    print("Góc cạnh dài so với Ox (deg):", angle)
    if closest_midpoint_td is not None:
        print(f"Điểm gần nhất theo 'tren_duoi': [{closest_midpoint_td[0]:.3f}, {closest_midpoint_td[1]:.3f}]")

    # Test trường hợp "trai_phai"
    _, _, closest_midpoint_tp = get_trolley_pose(points, agv_pos, "trai_phai")
    if closest_midpoint_tp is not None:
        print(f"Điểm gần nhất theo 'trai_phai': [{closest_midpoint_tp[0]:.3f}, {closest_midpoint_tp[1]:.3f}]")
