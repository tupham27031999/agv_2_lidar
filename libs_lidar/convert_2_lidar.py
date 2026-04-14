import numpy as np
import cv2
import math
from config_2 import AGVConfig_2


alpha_scan = AGVConfig_2.alpha_scan
def _normalize_angle_deg(a):
    """Chuẩn hóa angle về [0,360). Hỗ trợ cả scalar và mảng."""
    if np.isscalar(a):
        a = float(a) % 360.0
        if a < 0:
            a += 360.0
        return a
    else:
        a = np.asarray(a, dtype=float)
        a = np.mod(a, 360.0)
        a[a < 0] += 360.0
        return a

def _polar_to_cartesian(angles_deg, distances):
    angles = np.deg2rad(angles_deg)
    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    return np.vstack((x, y)).T  # (N,2)

def transform_lidar_points_optimized(scan_data_lidar,
                                     lidar_pos_on_agv,
                                     lidar_orientation_on_agv_deg,
                                     angular_range_deg,
                                     scaling_factor=1.0,
                                     min_range=0.0,
                                     max_range=1e9):
    """
    Chuyển scan (N,3) [signal, angle_deg, distance] -> điểm (x,y,signal) trong hệ AGV.
    Lọc theo angular_range_deg = (a1, a2) (deg), và theo min/max range.
    """
    if scan_data_lidar is None or len(scan_data_lidar) == 0:
        return np.empty((0, 3), dtype=float)

    # Tương thích với cấu trúc cũ: col0 signal, col1 angle, col2 distance
    scan = np.asarray(scan_data_lidar)
    signals = scan[:, 0].astype(float)
    angles = scan[:, 1].astype(float)
    distances = scan[:, 2].astype(float) * scaling_factor

    # Lọc theo range
    mask_range = (distances > min_range) & (distances <= max_range)
    if not np.any(mask_range):
        return np.empty((0,3), dtype=float)
    signals = signals[mask_range]
    angles = angles[mask_range]
    distances = distances[mask_range]

    # Lọc theo dải góc angular_range_deg (hỗ trợ alpha1 <= alpha2 và alpha1 > alpha2)
    a1, a2 = angular_range_deg
    a1 = float(a1)
    a2 = float(a2)
    angles_norm = _normalize_angle_deg(angles)
    a1n = _normalize_angle_deg(a1)
    a2n = _normalize_angle_deg(a2)

    if a1n <= a2n:
        mask_angle = (angles_norm >= a1n) & (angles_norm <= a2n)
    else:
        # e.g. a1=300, a2=60 (wrap-around)
        mask_angle = (angles_norm >= a1n) | (angles_norm <= a2n)

    if not np.any(mask_angle):
        return np.empty((0,3), dtype=float)

    signals = signals[mask_angle]
    angles = angles[mask_angle]
    distances = distances[mask_angle]

    # Polar -> Cartesian (local lidar frame)
    pts_local = _polar_to_cartesian(angles, distances)  # (N,2)
    # Build rotation (lidar orientation wrt AGV)
    theta = math.radians(lidar_orientation_on_agv_deg)
    R = np.array([[math.cos(theta), -math.sin(theta)],
                  [math.sin(theta),  math.cos(theta)]])
    pts_rot = (R @ pts_local.T).T  # (N,2)
    # Tịnh tiến theo vị trí lắp đặt (lidar_pos_on_agv: (x,y))
    x_agv = pts_rot[:, 0] + lidar_pos_on_agv[0]
    y_agv = - pts_rot[:, 1] + lidar_pos_on_agv[1]  # giữ cùng convention với code cũ
    out = np.vstack((x_agv, y_agv, signals)).T
    return out

def voxel_downsample_keep_best(points, voxel_size, score_col=None):
    """
    Downsample 2D points bằng voxel grid:
    - points: (N, >=2) first two cols x,y. Can have score_col index to pick best within voxel.
    - Returns array of chosen points (same row dimension).
    """
    if points is None or len(points) == 0:
        return np.empty((0, points.shape[1]), dtype=float)

    pts = np.asarray(points)
    coords = np.floor(pts[:, :2] / float(voxel_size)).astype(np.int64)
    # lexsort to group identical voxels
    order = np.lexsort((coords[:,1], coords[:,0]))
    coords_sorted = coords[order]
    pts_sorted = pts[order]

    # find unique voxels
    diff = np.any(np.diff(coords_sorted, axis=0), axis=1)
    unique_mask = np.concatenate(([True], diff))
    # for voxels with multiple points, we should pick best by score (if provided) or smallest range
    # We'll process groups
    chosen_idx = []
    start = 0
    n = len(coords_sorted)
    for i in range(n):
        last = (i == n-1)
        if last or unique_mask[i+1] if (i+1)<n else True:
            # group is from start..i inclusive
            group = pts_sorted[start:i+1]
            if score_col is not None and score_col < group.shape[1]:
                scores = group[:, score_col]
                best = np.argmax(scores)
            else:
                # floor by distance to robot (sqrt(x^2+y^2)), keep closest
                d2 = np.sum(group[:, :2]**2, axis=1)
                best = np.argmin(d2)
            chosen_idx.append(start + best)
            start = i+1
    chosen = pts_sorted[chosen_idx]
    return chosen

def remove_overlap_by_voxel(p1, p2, voxel_size, score_col=2):
    """
    Gộp p1 và p2 nhưng nếu 2 điểm nằm trong cùng voxel thì chọn 1 điểm tốt hơn:
    - ưu tiên điểm có score (col index score_col) lớn hơn,
    - nếu score không phân biệt được -> giữ điểm gần robot hơn.
    """
    if (p1 is None or len(p1) == 0) and (p2 is None or len(p2) == 0):
        return np.empty((0,3))
    if p1 is None or len(p1) == 0:
        return p2.copy()
    if p2 is None or len(p2) == 0:
        return p1.copy()

    # concat with source tag
    s1 = np.hstack((p1, np.zeros((len(p1),1), dtype=float)))  # tag 0
    s2 = np.hstack((p2, np.ones((len(p2),1), dtype=float)))   # tag 1
    allp = np.vstack((s1, s2))  # columns: x,y,signal,tag
    # For simplicity, treat signal as score
    chosen = voxel_downsample_keep_best(allp, voxel_size, score_col=2)
    # drop tag column
    return chosen[:, :3]


def transform_lidar_points_an_toan(scan_data_lidar, lidar_pos_on_agv, lidar_orientation_on_agv_deg, angular_range_deg, scaling_factor):
    if scan_data_lidar.shape[0] < 1:
        return np.array([[]])

    signals = scan_data_lidar[0, 0]
    angles_deg = scan_data_lidar[0, 1]
    distances = scan_data_lidar[0, 2] * scaling_factor

    # print(signals)
    # if len(signals) == 0:
    #     return np.array([[]])

    # 2. Chuyển từ cực sang Descartes (local Lidar frame)
    angles_rad_local = np.radians(angles_deg)
    x_local = distances * np.cos(angles_rad_local)
    y_local = distances * np.sin(angles_rad_local)

    # 3. Xây dựng ma trận xoay cho Lidar
    lidar_orientation_rad = np.radians(lidar_orientation_on_agv_deg)
    cos_orient = np.cos(lidar_orientation_rad)
    sin_orient = np.sin(lidar_orientation_rad)

    # Áp dụng phép xoay
    x_rotated = x_local * cos_orient - y_local * sin_orient
    y_rotated = x_local * sin_orient + y_local * cos_orient

    # 4. Tịnh tiến điểm theo vị trí lắp đặt của Lidar (vào hệ AGV)
    x_agv = x_rotated + lidar_pos_on_agv[0]
    y_agv = - y_rotated + lidar_pos_on_agv[1]

    # Ghép các cột x_agv, y_agv, signal lại
    # Lưu ý thứ tự: (x, y, signal)
    transformed_points_agv_frame = np.vstack(([x_agv], [y_agv], [signals])).T
    
    return transformed_points_agv_frame


def convert_scan_lidar(scan1_data_example=np.array([[0, 0, 0]]),
                       scan2_data_example=np.array([[0, 0, 0]]),
                       scan_an_toan=np.array([[]]),
                       ten_lidar="",
                       scaling_factor=1,
                       lidar1_orient_deg=45,
                       lidar2_orient_deg=0,
                       agv_w=-45,
                       agv_l=126,
                       voxel_size=50.0,
                       min_range=10.0,
                       max_range=25000.0):
    """
    Phiên bản tối ưu: trả về (merged_points, p1_agv, p2_agv, p3_agv)
    - voxel_size: mm unit (tương tự với hệ đo của bạn). Mặc định 50 (tùy chỉnh).
    - angular_filter: None hoặc (a1,a2) để override global anpha_scan.
    """

    # Nếu angular_filter không truyền -> giữ như cài đặt cũ dùng toàn bộ
    # if angular_filter is None:
    #     # giữ toàn bộ góc 0..360
    #     alpha_range = (0.0, 359.999)
    # else:
    #     alpha_range = angular_filter

    lidar1_alpha_range = (alpha_scan[0], alpha_scan[1])
    lidar2_alpha_range = (alpha_scan[0], alpha_scan[1])

    # Vị trí lidar trên AGV (giữ cùng convention file cũ)
    lidar1_pos = (-agv_w / 2, agv_l / 2)
    lidar2_pos = (agv_w / 2, -agv_l / 2)

    # Transform từng lidar riêng
    p1_agv = transform_lidar_points_optimized(np.asarray(scan1_data_example),
                                              lidar1_pos,
                                              lidar1_orient_deg,
                                              lidar1_alpha_range,
                                              scaling_factor=scaling_factor,
                                              min_range=min_range,
                                              max_range=max_range)
    p2_agv = transform_lidar_points_optimized(np.asarray(scan2_data_example),
                                              lidar2_pos,
                                              lidar2_orient_deg,
                                              lidar2_alpha_range,
                                              scaling_factor=scaling_factor,
                                              min_range=min_range,
                                              max_range=max_range)

    # Trường hợp hack / safety: nếu empty arrays -> đảm bảo shape
    if p1_agv is None:
        p1_agv = np.empty((0,3))
    if p2_agv is None:
        p2_agv = np.empty((0,3))

    # Chuẩn hoá signal -> score trong range [0,1] nếu có
    def _normalize_signal(arr):
        if arr is None or len(arr)==0:
            return arr
        arr = arr.copy()
        sig = arr[:,2].astype(float)
        if np.max(sig) - np.min(sig) > 0:
            s = (sig - np.min(sig)) / (np.max(sig) - np.min(sig))
        else:
            s = np.ones_like(sig)
        arr[:,2] = s
        return arr

    p1_agv = _normalize_signal(p1_agv)
    p2_agv = _normalize_signal(p2_agv)

    # Loại bỏ duplicate local bằng voxel nhỏ trước
    p1_ds = voxel_downsample_keep_best(p1_agv, voxel_size, score_col=2)
    p2_ds = voxel_downsample_keep_best(p2_agv, voxel_size, score_col=2)

    # Gộp & remove overlap bằng voxel (chọn điểm tốt hơn trong cùng voxel)
    merged = remove_overlap_by_voxel(p1_ds, p2_ds, voxel_size, score_col=2)

    # Nếu có scan_an_toan (vùng an toàn) -> transform về AGV frame bằng cùng transform như p1/p2
    if ten_lidar == "tren":
        scan_an_toan_transformed = transform_lidar_points_an_toan(scan_an_toan, lidar1_pos, lidar1_orient_deg, lidar1_alpha_range, scaling_factor)
    else:
        scan_an_toan_transformed = transform_lidar_points_an_toan(scan_an_toan, lidar2_pos, lidar2_orient_deg, lidar2_alpha_range, scaling_factor)
    # Đảm bảo output shapes consistent
    return merged, p1_agv, p2_agv, scan_an_toan_transformed


# Optional: debug visualization helper
def visualize_two_lidars(p_all_agv, p1_agv, p2_agv, img_size=800, scale=0.05, agv_w=400, agv_l=800):
    """
    Trả về ảnh BGR (np.uint8) để debug: p1 색 đỏ, p2 xanh, merged tím.
    scale: pixel per mm (tùy đơn vị)
    """
    img = np.full((img_size, img_size, 3), 200, dtype=np.uint8)
    cx, cy = img_size//2, img_size//2
    def wp(pt):
        x = int(cx + pt[0]*scale)
        y = int(cy - pt[1]*scale)
        return x,y
    # draw p1
    if p1_agv is not None and len(p1_agv)>0:
        for pt in p1_agv:
            cv2.circle(img, wp(pt), 1, (0,0,255), -1)
    if p2_agv is not None and len(p2_agv)>0:
        for pt in p2_agv:
            cv2.circle(img, wp(pt), 1, (0,255,0), -1)
    if p_all_agv is not None and len(p_all_agv)>0:
        for pt in p_all_agv:
            cv2.circle(img, wp(pt), 1, (255,0,255), -1)
    # draw AGV box
    half_w = int((agv_w/2)*scale)
    half_l = int((agv_l/2)*scale)
    cv2.rectangle(img, (cx-half_w, cy-half_l), (cx+half_w, cy+half_l), (50,50,50), 1)
    return img

# If run as script for quick unit test (no external I/O)
if __name__ == "__main__":
    # small synthetic test
    # lidar1: 0..180 deg points at 1000mm
    ang1 = np.linspace(0,180,181)
    dist1 = np.ones_like(ang1)*1000
    sig1 = np.ones_like(ang1)
    scan1 = np.vstack((sig1, ang1, dist1)).T
    # lidar2: 180..360 deg points at 1200mm
    ang2 = np.linspace(180,359,180)
    dist2 = np.ones_like(ang2)*1200
    sig2 = np.ones_like(ang2)*0.8
    scan2 = np.vstack((sig2, ang2, dist2)).T

    merged, p1, p2, sa = convert_scan_lidar(scan1, scan2, np.array([[]]), ten_lidar="duoi",
                                            scaling_factor=1, lidar1_orient_deg=45, lidar2_orient_deg=-136,
                                            agv_w=300, agv_l=500, voxel_size=50)
    print("merged", merged.shape, "p1", p1.shape, "p2", p2.shape)
