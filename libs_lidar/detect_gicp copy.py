# detect_gicp.py (modified)
"""
Fast Lidar Mapper (ROS-like, pure Python)
- Input: scan_pts Nx3 numpy array (x,y,z) or (x,y,1)
- This modified version adds:
  * dynamic point filtering (avoid adding transient points like people)
  * temporal voting / voxel-count before committing to global_map
  * keyframe anchoring for ICP (reduce drift in corridors)
  * protections in occupancy update so "confirmed" wall pixels are not erased
  * stronger ICP sanity checks
Requirements: open3d, numpy, opencv-python
"""
import time
import numpy as np
try:
    import open3d as o3d
except Exception as e:
    raise ImportError("Open3D is required. pip install open3d") from e
import cv2
import math
from collections import deque
from libs_lidar import convert_2_lidar
import os
from libs_ngoai_vi import music
from ham_logic import tinh_luong_giac, tam_va_goc_hcn, angle_and_distance as ad
import config as cfg
# from config import AGVConfig
# from config_2 import AGVConfig_2
from libs_file import edit_csv_phay, edit_csv_tab, remove



# ----------------- helpers -----------------
def ensure_xyz(scan):
    arr = np.asarray(scan, dtype=np.float64)
    if arr.ndim == 2 and arr.shape[1] >= 2:
        if arr.shape[1] == 2:
            z = np.zeros((arr.shape[0],1))
            arr = np.hstack([arr, z])
        return arr[:, :3]
    raise ValueError("scan must be Nx2 or Nx3 array")


def estimate_delta_icp(scan_a, scan_b, max_correspondence_dist=0.5):
    """
    Ước lượng dịch chuyển (dx, dy, yaw) giữa hai tập điểm scan bằng ICP.
    scan_a: numpy array (N, 2) - điểm mốc (map hoặc submap)
    scan_b: numpy array (M, 2) - điểm cần so khớp (scan hiện tại)
    """
    if len(scan_a) < 10 or len(scan_b) < 10:
        return None

    # Tạo point cloud open3d
    pcd_a = o3d.geometry.PointCloud()
    pcd_a.points = o3d.utility.Vector3dVector(np.column_stack((scan_a, np.zeros(len(scan_a)))))

    pcd_b = o3d.geometry.PointCloud()
    pcd_b.points = o3d.utility.Vector3dVector(np.column_stack((scan_b, np.zeros(len(scan_b)))))

    # Căn chỉnh ICP (bản 2D)
    icp_result = o3d.pipelines.registration.registration_icp(
        pcd_b, pcd_a,
        max_correspondence_dist,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    if icp_result.fitness < 0.3:  # ít điểm khớp, loại bỏ
        return None

    T = icp_result.transformation
    dx, dy = T[0, 3], T[1, 3]
    yaw = np.arctan2(T[1, 0], T[0, 0])

    return np.array([dx, dy, yaw])

def pose_to_xyyaw(pose):
    """
    Trả về (x, y, yaw) cho cả dạng vector [x, y, yaw] hoặc ma trận 4x4.
    """
    pose = np.asarray(pose)
    if pose.shape == (4, 4):  # ma trận đồng nhất
        x = pose[0, 3]
        y = pose[1, 3]
        yaw = math.atan2(pose[1, 0], pose[0, 0])
    else:  # vector hoặc list
        if len(pose) >= 3:
            x, y, yaw = pose[:3]
        else:
            x, y, yaw = pose[0], pose[1], 0.0
    return x, y, yaw



def lidar_to_point_cloud(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud


def downsample_point_cloud(point_cloud, voxel_size):
    return point_cloud.voxel_down_sample(voxel_size)


def gicp(points1, points2, threshold=500,max_iteration=300, voxel_size=0.1, trans_init=np.eye(4)):
    source_pcd = lidar_to_point_cloud(points1)
    target_pcd = lidar_to_point_cloud(points2)

    source_pcd = downsample_point_cloud(source_pcd, voxel_size)
    target_pcd = downsample_point_cloud(target_pcd, voxel_size)

    source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria)
    return reg_p2p.inlier_rmse, reg_p2p.transformation



# ----------------- Main mapper class -----------------
class FastLidarMapper:
    def __init__(self, load_from_path=None): # Thêm tham số để tải từ đường dẫn
        """
        All units are in millimeters (mm).
        Additional behavior in this modified mapper:
        - temporal voxel voting: a point must be observed N times before committing
        - dynamic filtering using KD tree distance threshold
        - keyframes saved periodically and when moving far; ICP will try nearest keyframe local map
        """
        map_size_mm=100000.0                             # Kích thước tổng của bản đồ (mm).
        resolution_mm=AGVConfig.ty_le_mm_pixel           # Độ phân giải của bản đồ (mm/pixel).
        voxel_size_mm=30.0                              # Kích thước voxel để giảm mẫu điểm (downsampling).
        local_map_range_mm=12000.0                       # Bán kính của bản đồ cục bộ dùng cho ICP. bán kính để lấy trong bản đồ tĩnh đi xác định icp
        max_map_points=100000                               # Số lượng điểm tối đa trong bản đồ toàn cục.
        icp_max_iter=20                                       # Số lần lặp tối đa cho thuật toán ICP.
        icp_dist_mm=600.0                                  # Ngưỡng khoảng cách cho các điểm tương ứng trong ICP. (rmse bé hơn ngưỡng khoảng cách * 0.8 -- > 200)
        # async_map_update=True                           # Cập nhật bản đồ occupancy bất đồng bộ để không chặn luồng chính.
        display_scale=0.5                                 # Tỷ lệ hiển thị cho cửa sổ debug. miss_dec_strong
        do_day_tuong = 200                              # tường dày đơn vị mm 1
        
        # store mm parameters
        self.map_size_mm = float(map_size_mm)
        self.resolution_mm = float(resolution_mm)
        self.voxel_size_mm = float(voxel_size_mm)
        self.local_map_range_mm = float(local_map_range_mm)
        self.icp_dist_mm = float(icp_dist_mm)
        self.max_map_points = int(max_map_points)
        self.icp_max_iter = int(icp_max_iter)
        # self.async_map_update = bool(async_map_update)
        self.display_scale = float(display_scale)
        self.do_day_tuong = int(do_day_tuong)
        # self.do_day_tuong = int(do_day_tuong) # Sẽ thay thế bằng logic xác suất

        # occupancy grid (pixels)
        self.pixels = int(np.ceil(self.map_size_mm / self.resolution_mm))
        self.center_px = (self.pixels // 2, self.pixels // 2)
        self.log_odds = np.zeros((self.pixels, self.pixels), dtype=np.float32)
        
        # Ảnh màu tổng, khởi tạo màu xám (128)
        self.map_image_color = None
        # print(self.map_image_color.shape)

        # pose (map <- base) in mm
        self.pose = np.eye(4, dtype=np.float64)
        self.trajectory = deque(maxlen=5000) # lưu lịch sử quãng đường di chuyển

        # global pointstore (world coordinates in mm) - committed (confirmed) points - bản đồ tĩnh - tất cả các điểm
        self.global_map = np.empty((0,3), dtype=np.float64)

        # temporal voting for voxels: map voxel_id -> count
        self.voxel_counts = {}  # key: (vx, vy) integer voxel coords -> count
        self.voxel_point = {}   # representative point for that voxel
        self.min_observe_count = 2  # require >=2 observations before committing

        # keyframes for anchoring
        self.keyframes = []  # list of dicts: {'pose':pose, 'pcd': o3d.geometry.PointCloud(), 'timestamp':t}
        self.keyframe_dist_mm = 1500.0  # create keyframe every 1.5 m
        self.keyframe_interval_frames = 20
        self._frames_since_keyframe = 0

        # cache target pointcloud (Open3D) for local map
        self._target_pcd_cache = None
        self._target_cache_pose = None

        # async executor for occupancy updates
        # self.executor = ThreadPoolExecutor(max_workers=1) if self.async_map_update else None

        # debug
        self.rmse = 0.0
        self.tam_x_mm = 0.0
        self.tam_y_mm = 0.0
        self.tam_x_pixel = 0
        self.tam_y_pixel = 0
        self.goc_agv = 0.0

        # Nếu có đường dẫn, tải trạng thái đã lưu
        if load_from_path:
            if not self.load_state(load_from_path):
                print(f"Cảnh báo: Không thể tải trạng thái từ '{load_from_path}'. Bắt đầu với bản đồ trống.")

        self._last_update_time = None

        self.tai_ban_do_tam_thoi_1 = 0
        self.ten_ban_do_tam_thoi_1 = ""

        self.tam_hcn = None 
        self.goc_hcn = None
        self.diem_gan_nhat = None
        self.diem_gan_agv_hon = None
        self.kc_tam_agv_hcn = 0

        self.dt0 = 0
        self.dt1 = 0

        log_dir = remove.tao_folder(cfg.PATH_PHAN_MEM + "/data_input_output/log_check_localization")
        self.log_file = log_dir + "/log_diff_drive.csv"
        edit_csv_phay.new_csv_replace(self.log_file, ["timestamp","x","y","theta","v_l","v_r","rmse","dt","d_icp","d_enc","rot_icp","rot_enc","is_error","error_msg"])

        # Biến này không còn cần thiết trong lớp này, sẽ đọc trực tiếp từ webserver
        # self.vung_xoa_ban_do = {"vung_xoa": [], "update": []}

    # ----------------- internal: PCD utils -----------------
    def _scan_to_o3d(self, scan_pts):
        pts = ensure_xyz(scan_pts)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd

    def _filter_statistical_outliers(self, pcd, nb_neighbors=20, std_ratio=2.0):
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                                 std_ratio=std_ratio)
        return pcd.select_by_index(ind)

    def _adaptive_downsample(self, pcd):
        n = np.asarray(pcd.points).shape[0]
        if n > 30000:
            v = self.voxel_size_mm * 2.0
        elif n > 10000:
            v = self.voxel_size_mm * 1.5
        else:
            v = self.voxel_size_mm
        return pcd.voxel_down_sample(v)

    def _prepare_source(self, scan_pts):
        src = self._scan_to_o3d(scan_pts)
        src = self._filter_statistical_outliers(src, nb_neighbors=20, std_ratio=2.0)
        src = self._adaptive_downsample(src)
        try:
            src.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
            src.orient_normals_towards_camera_location(np.array([0.,0.,0.]))
        except Exception:
            pass
        return src

    def _build_global_kdtree(self):
        if self.global_map.shape[0] == 0:
            return None
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.global_map)
        return pcd

    def _refresh_target_cache(self):
        # build local target from nearest keyframe if available, else nearest points
        if len(self.keyframes) > 0:
            # find nearest keyframe by pose distance
            cur_xy = self.pose[:2,3]
            dists = [np.linalg.norm(kf['pose'][:2,3] - cur_xy) for kf in self.keyframes]
            idx = int(np.argmin(dists))
            kf = self.keyframes[idx]
            # if keyframe near enough, use it
            if dists[idx] < (self.local_map_range_mm * 0.9):
                self._target_pcd_cache = kf['pcd']
                self._target_cache_pose = kf['pose'].copy()
                return
        # fallback: build from global_map nearby
        if self.global_map.shape[0] < 30:
            self._target_pcd_cache = None
            self._target_cache_pose = None
            return

        px = self.pose[0,3]; py = self.pose[1,3]
        dists = np.linalg.norm(self.global_map[:, :2] - np.array([px,py]), axis=1)
        mask = dists < self.local_map_range_mm
        local = self.global_map[mask]
        if local.shape[0] < 20:
            self._target_pcd_cache = None
            self._target_cache_pose = None
            return
        tgt = o3d.geometry.PointCloud()
        tgt.points = o3d.utility.Vector3dVector(local)
        tgt = tgt.voxel_down_sample(self.voxel_size_mm)
        self._target_pcd_cache = tgt
        self._target_cache_pose = self.pose.copy()

    # ----------------- dynamic filtering / temporal voting -----------------
    def _voxel_id(self, point):
        # 2D voxel id on XY plane
        vx = int(np.floor(point[0] / self.voxel_size_mm))
        vy = int(np.floor(point[1] / self.voxel_size_mm))
        return (vx, vy)

    def _filter_dynamic_points(self, pts_world):
        # Use KD search against committed global_map to reject points that are too close to global_map but
        # appear only once (these are likely dynamic). Instead of removing existing global_map points,
        # we only decide whether to count new observations into voxel_counts.
        if pts_world.shape[0] == 0:
            return pts_world
        if self.global_map.shape[0] < 50:
            return pts_world

        # Build temporary KDTree via Open3D for search
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(self.global_map)
        kdtree = o3d.geometry.KDTreeFlann(temp_pcd)

        keep = []
        for p in pts_world:
            # If close to existing committed point: keep (it's consistent)
            [k, idx, dist] = kdtree.search_knn_vector_3d(p, 1)
            if k > 0 and dist[0] < (self.voxel_size_mm * 1.5)**2:
                keep.append(p)
                continue
            # If far from any committed point, keep as candidate (but not yet committed)
            keep.append(p)
        return np.array(keep)

    def _accumulate_voxels_and_commit(self, pts_world):
        # For each point, increment voxel observation count and commit iff count >= min_observe_count.
        committed_points = []
        for p in pts_world:
            vid = self._voxel_id(p)
            cnt = self.voxel_counts.get(vid, 0) + 1
            self.voxel_counts[vid] = cnt
            # store representative point (centroid style)
            if vid in self.voxel_point:
                # running avg
                self.voxel_point[vid] = (self.voxel_point[vid] * (cnt-1) + p) / cnt
            else:
                self.voxel_point[vid] = p.copy()

            if cnt == self.min_observe_count:
                # commit representative point to global_map
                committed_points.append(self.voxel_point[vid].copy())
        if len(committed_points) > 0:
            committed = np.vstack(committed_points)
            if self.global_map.shape[0] == 0:
                self.global_map = committed
            else:
                self.global_map = np.vstack((self.global_map, committed))

        # keep bounded memory
        if self.global_map.shape[0] > self.max_map_points:
            self.global_map = self.global_map[-self.max_map_points:, :]

    # ----------------- occupancy update (protected) -----------------
    def _update_occupancy_map(self, points_global, robot_pos, update_all_point_in_map=0,
        p_occ=0.8, p_free=0.2, delta_xy_pixels=500, cap_nhat_vi_tri_xe = 0):

        if points_global is None or len(points_global) == 0:
            return

        # --- Hằng số log-odds (đã tính trước để tránh gọi hàm log/exp liên tục) ---
        # hit_inc = np.log(p_occ / (1 - p_occ))       # dương (~0.847)
        # miss_dec = np.log(p_free / (1 - p_free))    # âm (~-1.38)
        # miss_dec_strong = miss_dec / 4.0            # giảm yếu hơn cho vùng occupied cũ
        hit_inc = 1.386     # np.log(0.8 / 0.2)
        miss_dec = -1.386   # np.log(0.2 / 0.8)
        miss_dec_strong = miss_dec / 4.0

        h, w = self.log_odds.shape
        map_center_xy = self.center_px

        robot_x_map = int(map_center_xy[0] + robot_pos[0] / self.resolution_mm)
        robot_y_map = int(map_center_xy[1] - robot_pos[1] / self.resolution_mm)

        x1 = max(0, robot_x_map - delta_xy_pixels)
        y1 = max(0, robot_y_map - delta_xy_pixels)
        x2 = min(w, robot_x_map + delta_xy_pixels)
        y2 = min(h, robot_y_map + delta_xy_pixels)

        log_odds_roi = self.log_odds[y1:y2, x1:x2]
        roi_h, roi_w = log_odds_roi.shape
        robot_x_roi = robot_x_map - x1
        robot_y_roi = robot_y_map - y1

        # --- Tạo mask free-space ---
        px_map = (map_center_xy[0] + points_global[:, 0] / self.resolution_mm).astype(np.int32)
        py_map = (map_center_xy[1] - points_global[:, 1] / self.resolution_mm).astype(np.int32)
        valid_mask = (px_map >= x1) & (px_map < x2) & (py_map >= y1) & (py_map < y2) # noqa: E712
        if not np.any(valid_mask):
            return

        px_roi = px_map[valid_mask] - x1
        py_roi = py_map[valid_mask] - y1

        mask_free_roi = np.zeros((roi_h, roi_w), dtype=np.uint8)
        # Tối ưu: Chỉ vẽ tia laser cho các điểm pixel duy nhất để giảm tải cv2.line
        unique_indices = np.unique(np.stack((px_roi, py_roi), axis=1), axis=0)
        # print("unique_indices", unique_indices.shape)
        for ux, uy in unique_indices:
            dx = int(ux) - robot_x_roi
            dy = int(uy) - robot_y_roi
            length = max(abs(dx), abs(dy))
            if length > 1:
                end_x = robot_x_roi + int(dx * (length - 1) / length)
                end_y = robot_y_roi + int(dy * (length - 1) / length)
                cv2.line(mask_free_roi, (robot_x_roi, robot_y_roi), (end_x, end_y), 1, 1)

        # --- Xử lý xuyên tường ---
        if update_all_point_in_map == False:
            # p > 0.7 tương đương log_odds > 0.847
            mask_occ_near = (log_odds_roi > 0.847).astype(np.uint8)
            dist_occ = cv2.distanceTransform(255 - (mask_occ_near * 255), cv2.DIST_L2, 3)
            dist_occ_mm = dist_occ * self.resolution_mm
            mask_blocked = (dist_occ_mm <= self.do_day_tuong) & (mask_free_roi == 1)
            mask_free_roi[mask_blocked] = 0
        else:
            # Khi force update, cho phép tia đi xuyên tường để xóa vùng đen
            pass

        # --- Tính xác suất và các mask ---
        # p_roi = 1.0 / (1.0 + np.exp(-log_odds_roi))
        # mask_conf_free = p_roi < 0.2
        # mask_conf_occ = p_roi > 0.8
        mask_conf_free = log_odds_roi < -1.386
        mask_conf_occ = log_odds_roi > 1.386

        # --- Cập nhật free-space bình thường ---
        dec_mask_normal = (mask_free_roi == 1) & (~mask_conf_occ)
        log_odds_roi[dec_mask_normal] += miss_dec
        dec_mask_strong = (mask_free_roi == 1) & (mask_conf_occ)
        log_odds_roi[dec_mask_strong] += miss_dec_strong


        # --- Cập nhật occupied (tường) ---
        if update_all_point_in_map == False and cap_nhat_vi_tri_xe == 0:
            # Tối ưu hóa: Vector hóa hoàn toàn việc cập nhật tường trong luồng bình thường
            hit_mask = np.zeros((roi_h, roi_w), dtype=bool)
            hit_mask[py_roi, px_roi] = True
            
            # Lọc nhiễu sau tường: hit nằm trong vùng free-ray nhưng lại chạm vào tường đã xác nhận
            is_noise = (mask_free_roi == 1) & mask_conf_occ
            valid_hits = hit_mask & (~is_noise) & (~mask_conf_free)
            log_odds_roi[valid_hits] += hit_inc
        else:
            # Giữ nguyên vòng lặp cho các chế độ đặc biệt (thủ công/force update) vì có logic vẽ đặc thù
            check_ray_pass_through = mask_free_roi.copy()
            for ux, uy in zip(px_roi, py_roi):
                if 0 <= uy < roi_h and 0 <= ux < roi_w:
                    if cap_nhat_vi_tri_xe == 1:
                        log_odds_roi[uy, ux] = hit_inc

                    is_noise_behind_wall = check_ray_pass_through[uy, ux] == 1 and mask_conf_occ[uy, ux]
                    if is_noise_behind_wall and update_all_point_in_map == False:
                        continue

                    if not mask_conf_free[uy, ux] or update_all_point_in_map == True:
                        log_odds_roi[uy, ux] += hit_inc

                        if update_all_point_in_map == True:
                            if cap_nhat_vi_tri_xe == 1:
                                cv2.line(log_odds_roi, (robot_x_roi, robot_y_roi), (ux, uy), 0.0, 1)
                                log_odds_roi[uy, ux] = hit_inc
                            else:
                                radius_px = int(self.do_day_tuong / self.resolution_mm)
                                cv2.circle(log_odds_roi, (ux, uy), radius_px, -0.2, thickness=-1)
                        

        np.clip(log_odds_roi, -5.0, 5.0, out=log_odds_roi)
        self.log_odds[y1:y2, x1:x2] = log_odds_roi
        return px_map, py_map

    def delete_zone_from_map(self, zone_coords_mm):
        """
        Xóa một vùng khỏi bản đồ.

        Args:
            zone_coords_mm (list): Tọa độ vùng xóa [x1, y1, x2, y2] trong hệ tọa độ thế giới (mm).
        """
        if not zone_coords_mm or len(zone_coords_mm) != 4:
            print("Tọa độ vùng xóa không hợp lệ.")
            return

        x_min_mm, y_min_mm, x_max_mm, y_max_mm = zone_coords_mm
        print(f"Đang xóa vùng: x({x_min_mm}, {x_max_mm}), y({y_min_mm}, {y_max_mm})")

        # 1. Xóa điểm trong self.global_map
        if self.global_map.shape[0] > 0:
            points = self.global_map
            # Tạo một mask để giữ lại các điểm không nằm trong vùng xóa
            # Chú ý: hệ tọa độ y của global_map ngược với hệ tọa độ của vùng chọn từ web
            mask = ~((points[:, 0] >= x_min_mm) & (points[:, 0] <= x_max_mm) &
                     (points[:, 1] >= -y_max_mm) & (points[:, 1] <= -y_min_mm))
            
            num_before = len(self.global_map)
            self.global_map = self.global_map[mask]
            num_after = len(self.global_map)
            print(f"Đã xóa {num_before - num_after} điểm khỏi global_map.")

        # 2. Cập nhật self.log_odds
        # Chuyển đổi tọa độ mm của vùng xóa sang tọa độ pixel
        ux_min, uy_max = self._world_to_pixel(x_min_mm, y_min_mm) # y_min_mm -> uy_max
        ux_max, uy_min = self._world_to_pixel(x_max_mm, y_max_mm) # y_max_mm -> uy_min

        # Đảm bảo tọa độ pixel nằm trong giới hạn bản đồ
        ux_min = max(0, ux_min)
        uy_min = max(0, uy_min)
        ux_max = min(self.pixels, ux_max)
        uy_max = min(self.pixels, uy_max)

        if ux_min < ux_max and uy_min < uy_max:
            self.log_odds[uy_min:uy_max, ux_min:ux_max] = 0.0  # Reset về xác suất 0.5 (trung tính)
            print(f"Đã reset log_odds cho vùng pixel: x({ux_min}, {ux_max}), y({uy_min}, {uy_max})")





    # ----------------- process_scan (main) -----------------
    # def process_scan(self, scan_pts0, integrate=True, distan_scan_icp = 25000, distan_scan_update = 10000, 
    #                  x_agv_pixel = 0, y_agv_pixel = 0, goc_agv_do = 0, cap_nhat_vi_tri = 0, cap_nhat_ban_do = 0,
    #                  force_update_map = 0):

    def normalize_angle_rad(self, angle):
        """
        Chuẩn hóa góc về [-pi, pi]
        Tránh trường hợp nhảy góc từ +pi sang -pi
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


    def compute_diff_drive_motion(self, v_left_mm_s, v_right_mm_s, wheel_base_mm, dt):
        """
        Tính chuyển động THỰC của AGV 2 bánh vi sai từ encoder

        Parameters
        ----------
        v_left_mm_s : float
            Vận tốc bánh trái (mm/s), có thể âm hoặc dương

        v_right_mm_s : float
            Vận tốc bánh phải (mm/s), có thể âm hoặc dương

        wheel_base_mm : float
            Khoảng cách hai bánh (mm)

        dt : float
            Thời gian vòng lặp (s)

        Returns
        -------
        dist_real_mm : float
            Quãng đường tịnh tiến thực tế trong dt (mm)

        angle_real_rad : float
            Góc quay thực tế trong dt (rad)
        """

        # Vận tốc tuyến tính của AGV
        v_linear = (v_left_mm_s + v_right_mm_s) / 2.0

        # Vận tốc góc (yaw rate)
        omega = (v_right_mm_s - v_left_mm_s) / wheel_base_mm

        dist_real_mm = abs(v_linear) * dt
        angle_real_rad = abs(omega) * dt

        return dist_real_mm, angle_real_rad


    def check_localization_error_diff_drive(self,
        pose_icp,            # ⚠️ (x_mm, y_mm, theta_rad) - pose từ ICP
        pose_icp_prev,       # ⚠️ pose ICP vòng trước
        v_left_mm_s,         # ⚠️ tốc độ bánh trái (mm/s)
        v_right_mm_s,        # ⚠️ tốc độ bánh phải (mm/s)
        wheel_base_mm,       # ⚠️ khoảng cách 2 bánh (mm)
        rmse,                # ⚠️ RMSE ICP (mm)
        dt,                  # ⚠️ thời gian vòng lặp (s)
        last_icp_time,       # ⚠️ timestamp ICP update cuối
        now_time,            # ⚠️ timestamp hiện tại
        config               # ⚠️ dict cấu hình ngưỡng
    ):
        """
        Giám sát an toàn localization cho AGV 2 bánh vi sai

        Returns
        -------
        is_error : bool
            True  -> phát hiện lỗi localization
            False -> localization bình thường

        error_name : str | None
            Tên lỗi chi tiết để log/debug
        """

        v_left_mm_s = v_left_mm_s / 10
        v_right_mm_s = v_right_mm_s / 10

        # --------------------------------------------------
        # 1️⃣ Tính thay đổi pose từ ICP
        # --------------------------------------------------
        x, y, th = pose_icp
        x0, y0, th0 = pose_icp_prev

        dx = x - x0
        dy = y - y0

        # Tính góc lệch theo radian trước, sau đó chuyển sang độ
        delta_rad = self.normalize_angle_rad(th - th0)
        delta_xoay_goc_icp = abs(delta_rad * 180 / np.pi)

        tinh_tien_icp = math.hypot(dx, dy)

        # --------------------------------------------------
        # 2️⃣ Tính chuyển động THỰC từ encoder
        # --------------------------------------------------
        tinh_tien_thuc, delta_xoay_goc_thuc_rad = self.compute_diff_drive_motion(
            v_left_mm_s,
            v_right_mm_s,
            wheel_base_mm,
            dt
        )
        delta_xoay_goc_thuc = delta_xoay_goc_thuc_rad * 180 / np.pi
        output = False
        ten_loi = ""

        

        # --------------------------------------------------
        # 4️⃣ Kiểm tra RMSE (2 mức)
        # --------------------------------------------------
        if (max(abs(AGVConfig_2.van_toc_phan_hoi_trai), abs(AGVConfig_2.van_toc_phan_hoi_phai)) > 200 and rmse == 0):
            output = True
            ten_loi = "rmse=0_mac_du_xe_chay"

        # --------------------------------------------------
        # (2) RMSE quá cao (dù xe có chạy hay không)
        # --------------------------------------------------
        if rmse >= config["rmse_stop"]:
            output = True
            ten_loi = "rmse_qua_lon" + str(rmse)
        elif rmse >= config["rmse_warn"]:
            AGVConfig_2.di_chuyen_cham = True
        elif rmse < config["rmse_warn"]:
            AGVConfig_2.di_chuyen_cham = False

        # --------------------------------------------------
        # 5️⃣ Xe đi thẳng nhưng ICP không đi
        # --------------------------------------------------
        if (
            tinh_tien_thuc > config["min_linear_motion_mm"]
            and tinh_tien_icp < config["min_icp_motion_mm"]
        ):
            output = True
            ten_loi = "agv_thuc_te_di_nhung_icp_dung_im"

        # --------------------------------------------------
        # 6️⃣ Xe quay tại chỗ nhưng ICP không quay
        # --------------------------------------------------
        if (
            delta_xoay_goc_thuc > config["min_rotation_deg"]
            and delta_xoay_goc_icp < config["min_icp_rotation_deg"]
        ):
            output = True
            ten_loi = "agv_dung_im_mac_du_xe_chay"

        # --------------------------------------------------
        # 7️⃣ ICP nhảy đột ngột
        # --------------------------------------------------
        if (
            tinh_tien_icp > config["max_pose_jump_mm"]
            or delta_xoay_goc_icp > config["max_angle_jump_deg"]
        ):
            output = True
            ten_loi = "agv_nhay_dot_ngot_do_goc_hoac_toa_do"
        
        # --------------------------------------------------
        # 3️⃣ ICP timeout (NGUY HIỂM)
        # --------------------------------------------------

        # if output == False:
        #     cfg.last_icp_time = time.time()

        # if now_time - last_icp_time > config["icp_timeout"]:
        #     output = True
        #     ten_loi = "ICP_loi_qua_thoi_gian_cho"

        # --------------------------------------------------
        # 9️⃣ Logging (Debug)
        # --------------------------------------------------
        # try:
        
        # self.log_file
        # if abs(v_left_mm_s) >  100 and abs(v_right_mm_s) > 100:
        if output == True and AGVConfig_2.luu_log_icp == 1:
            edit_csv_phay.append_csv(self.log_file, [str(now_time), str(x), str(y), str(th), 
                                                    str(v_left_mm_s), str(v_right_mm_s), str(rmse), str(dt), 
                                                    str(tinh_tien_icp), str(tinh_tien_thuc), str(delta_xoay_goc_icp), 
                                                    str(delta_xoay_goc_thuc), str(output), str(ten_loi)])
        # edit_csv_phay.append_csv(self.log_file,f"{now_time:.3f},{x:.1f},{y:.1f},{th:.3f},{v_left_mm_s:.1f},{v_right_mm_s:.1f},{rmse:.1f},{dt:.3f},{tinh_tien_icp:.1f},{tinh_tien_thuc:.1f},{delta_xoay_goc_icp:.3f},{delta_xoay_goc_thuc:.3f},{output},{ten_loi}")
            
        #     if not os.path.isfile(log_file):
        #         with open(log_file, "w") as f:
                    # f.write("timestamp,x,y,theta,v_l,v_r,rmse,dt,d_icp,d_enc,rot_icp,rot_enc,is_error,error_msg\n")
        #     with open(log_file, "a") as f:
        #         f.write(f"{now_time:.3f},{x:.1f},{y:.1f},{th:.3f},{v_left_mm_s:.1f},{v_right_mm_s:.1f},{rmse:.1f},{dt:.3f},{tinh_tien_icp:.1f},{tinh_tien_thuc:.1f},{delta_xoay_goc_icp:.3f},{delta_xoay_goc_thuc:.3f},{output},{ten_loi}\n")
        # except Exception:
        #     pass

        
        # --------------------------------------------------
        # 8️⃣ Localization OK
        # --------------------------------------------------
        return output, ten_loi


    def process_scan(self, scan_pts0, integrate=True, distan_scan_icp = 20000, distan_scan_update = 10000, 
                     x_agv_pixel = 0, y_agv_pixel = 0, goc_agv_do = 0, cap_nhat_vi_tri = 0, cap_nhat_ban_do = 0,
                     update_all_point_in_map = False):
        

        # if webserver.them_vi_tri_xe_ban_do == 1:
        delta_y = 1200
        delta_x = 500
        distances_from_center = np.linalg.norm(scan_pts0[:, :2], axis=1)

        mask = (distances_from_center >= 0) & (distances_from_center <= distan_scan_icp)
        scan_pts = scan_pts0[mask]

        cap_nhat_vi_tri_xe = 0

        if AGVConfig_2.them_vi_tri_xe_ban_do == 1:
            delta_y = 1000
            delta_x = 400
            # Trục x của robot là hướng về phía trước.
            x_coords = scan_pts0[:, 0]
            y_coords = scan_pts0[:, 1]
            mask2 = (x_coords >= -delta_x) & (x_coords <= delta_x) & \
                    (y_coords >= -delta_y) & (y_coords <= delta_y)
            cap_nhat_vi_tri_xe = 1
        else:
            mask2 = (distances_from_center >= 0) & (distances_from_center <= distan_scan_update)

        # webserver.xac_dinh_vi_tri_xe = 1
        # webserver.nang_vuong_goc = 1
        xac_dinh_vi_tri_xe = 0
        if AGVConfig_2.xac_dinh_vi_tri_xe == 1:
            if AGVConfig_2.nang_vuong_goc == 0:
                delta_y = 950
                delta_x = 500
            else:
                delta_y = 500
                delta_x = 950
            # Trục x của robot là hướng về phía trước.
            # webserver.goc_xoay
            x_coords = scan_pts0[:, 0]
            y_coords = scan_pts0[:, 1]

            # Chuyển đổi góc sang radian
            theta = np.radians(AGVConfig_2.goc_xoay)
            c, s = np.cos(theta), np.sin(theta)
            A = [0, 0]

            # Xoay tọa độ điểm ngược lại góc AGVConfig_2.goc_xoay để đưa về hệ trục của hình chữ nhật
            x_rot = (x_coords - A[0]) * c + (y_coords - A[1]) * s
            y_rot = -(x_coords - A[0]) * s + (y_coords - A[1]) * c

            mask2 = (np.abs(x_rot) <= delta_x) & (np.abs(y_rot) <= delta_y)

            # mask2 = (x_coords >= -delta_x) & (x_coords <= delta_x) & \
            #         (y_coords >= -delta_y) & (y_coords <= delta_y)
            xac_dinh_vi_tri_xe = 1
        scan_pts2 = scan_pts0[mask2]

        # --- Xử lý yêu cầu xóa vùng bản đồ từ webserver ---
        # xoa_vung_ban_do = {"vung_xoa": [], "update": 0} # dùng để xóa bót 1 số vùng bản đồ đã quét rồi muốn quét lại
        if AGVConfig.xoa_vung_ban_do.get("update") == 1:
            zone_pixel_all = AGVConfig.xoa_vung_ban_do.get("vung_xoa") # [[ux1, uy1, ux2, uy2], ...]
            if zone_pixel_all and len(zone_pixel_all) > 0:
                for zone_pixel in zone_pixel_all:
                    if len(zone_pixel) == 4:
                        # Chuyển đổi tọa độ vùng xóa từ pixel (hệ tọa độ ảnh) sang mm (hệ tọa độ thế giới)
                        # zone_pixel có dạng [ux1, uy1, ux2, uy2]
                        ux1, uy1, ux2, uy2 = zone_pixel

                        # x = (ux - center_x) * resolution
                        # y = (center_y - uy) * resolution
                        x_min_mm = (ux1 - self.center_px[0]) * self.resolution_mm
                        y_max_mm = (self.center_px[1] - uy1) * self.resolution_mm # uy1 là y nhỏ hơn -> y_max_mm lớn hơn
                        x_max_mm = (ux2 - self.center_px[0]) * self.resolution_mm
                        y_min_mm = (self.center_px[1] - uy2) * self.resolution_mm # uy2 là y lớn hơn -> y_min_mm nhỏ hơn

                        zone_to_delete_mm = [x_min_mm, y_min_mm, x_max_mm, y_max_mm]
                        self.delete_zone_from_map(zone_to_delete_mm)

            AGVConfig.xoa_vung_ban_do["update"] = 0 # Reset cờ sau khi xử lý

        rmse = 1000.0

        if cap_nhat_vi_tri == 1:
            # --- CẬP NHẬT VỊ TRÍ THỦ CÔNG TỪ NGƯỜI DÙNG ---
            print("--- Cập nhật vị trí thủ công ---")
            
            # 1. Chuyển đổi tọa độ pixel (ảnh) sang tọa độ thế giới (mm)
            world_x_mm = (x_agv_pixel - self.center_px[0]) * self.resolution_mm
            world_y_mm = (self.center_px[1] - y_agv_pixel) * self.resolution_mm

            # 2. Chuyển đổi góc (độ) sang radian và tạo ma trận xoay
            # Góc từ webserver: 0 độ là hướng sang phải (trục X), ngược chiều kim đồng hồ là dương.
            # Góc yaw trong self.pose: tính bằng atan2(R[1,0], R[0,0]).
            # Để tương thích, ta cần đổi dấu góc.
            goc_rad = -math.radians(goc_agv_do)
            cos_a = math.cos(goc_rad)
            sin_a = math.sin(goc_rad)

            # 3. Tạo ma trận pose mới và cập nhật
            self.pose = np.array([
                [cos_a, -sin_a, 0, world_x_mm],
                [sin_a,  cos_a, 0, world_y_mm],
                [0,          0, 1,          0],
                [0,          0, 0,          1]
            ], dtype=np.float64)

            # Chạy GICP một lần để kiểm tra RMSE tại vị trí mới do người dùng đặt
            # mà không cập nhật lại pose từ kết quả GICP.
            try:
                if self._target_pcd_cache is not None and len(np.asarray(self._target_pcd_cache.points)) > 10:
                    pcd_tree = o3d.geometry.KDTreeFlann(self._target_pcd_cache)
                    robot_position = self.pose[:3, 3]
                    
                    [k, idx, _] = pcd_tree.search_radius_vector_3d(robot_position, self.local_map_range_mm)
                    if k < 10:
                        raise ValueError("Không đủ điểm trên bản đồ cục bộ để tính RMSE.")
                    
                    filtered_target_points = np.asarray(self._target_pcd_cache.points)[idx, :]
                    
                    # Chỉ lấy rmse, không dùng new_pose
                    rmse, _ = gicp(
                        scan_pts,
                        filtered_target_points,
                        threshold=self.icp_dist_mm,
                        max_iteration=self.icp_max_iter,
                        trans_init=self.pose,
                        voxel_size=max(0.1, self.voxel_size_mm / 1000.0)
                    )
            except Exception as e:
                print(f"Lỗi khi tính toán RMSE thử: {e}")
                rmse = 1000.0 # Đặt RMSE cao nếu có lỗi

            print(f"Pose mới: x={world_x_mm:.1f}, y={world_y_mm:.1f}, yaw={math.degrees(-goc_rad):.1f}°")
        else:
            # --- ĐỊNH VỊ BẰNG ICP (LUỒNG BÌNH THƯỜNG) ---
            # refresh cache if necessary
            if self._target_pcd_cache is None:
                self._refresh_target_cache()
            else:
                dx = np.linalg.norm(self.pose[:2,3] - (self._target_cache_pose[:2,3] if self._target_cache_pose is not None else np.array([9999,9999])))
                if dx > (self.local_map_range_mm * 0.2):
                    self._refresh_target_cache()

            if self._target_pcd_cache is not None and len(np.asarray(self._target_pcd_cache.points)) > 5:
                check_rmse = False
                try:
                    pcd_tree = o3d.geometry.KDTreeFlann(self._target_pcd_cache)
                    robot_position = self.pose[:3, 3]
                    
                    [k, idx, _] = pcd_tree.search_radius_vector_3d(robot_position, self.local_map_range_mm)
                    if k < 10:
                        raise ValueError("Not enough points in local map for ICP.")
                    
                    filtered_target_points = np.asarray(self._target_pcd_cache.points)[idx, :]
                    
                    rmse, new_pose = gicp(
                        scan_pts,
                        filtered_target_points,
                        threshold=self.icp_dist_mm,
                        max_iteration=self.icp_max_iter,
                        trans_init=self.pose,
                        voxel_size=max(0.1, self.voxel_size_mm / 1000.0)
                    )
                    # print(rmse)
                    # print(new_pose)
                except Exception as e:
                    print(f"Lỗi ICP hoặc kết quả không hợp lệ: {e}. Giữ nguyên pose cũ.")
                    rmse = 1000.0
                if rmse != 1000:
                    AGVConfig_2.now_time = time.time()
                    # kiểm tra an toàn khi thuật toán icp lỗi, trôi map
                    self.dt1 = time.time()
                    dt = self.dt1 - self.dt0                # ⚠️ thời gian vòng lặp (s) 
                    # không có lỗi thì cfg.loi_icp == False

                    R_new = new_pose[:3,:3]
                    tvec_new = new_pose[:3,3]

                    goc_agv_new = -np.arctan2(R_new[1, 0], R_new[0, 0])
                    tam_x_mm_new = tvec_new[0] # tọa độ agv trong bản đồ gốc đơn vị mm
                    tam_y_mm_new = -tvec_new[1]

                    pose_icp = np.array([self.tam_x_mm, self.tam_y_mm, self.goc_agv])
                    pose_icp_prev = np.array([tam_x_mm_new, tam_y_mm_new, goc_agv_new])
                                
                    AGVConfig_2.loi_icp, AGVConfig_2.ten_loi = self.check_localization_error_diff_drive(pose_icp = pose_icp,                             # ⚠️ (x_mm, y_mm, theta_rad) - pose từ ICP
                                                                                        pose_icp_prev = pose_icp_prev,                   # ⚠️ pose ICP vòng trước
                                                                                        v_left_mm_s = AGVConfig_2.van_toc_phan_hoi_trai,         # ⚠️ tốc độ bánh trái (mm/s)
                                                                                        v_right_mm_s = AGVConfig_2.van_toc_phan_hoi_phai,        # ⚠️ tốc độ bánh phải (mm/s)
                                                                                        wheel_base_mm = AGVConfig_2.wheel_base_mm,               # ⚠️ khoảng cách 2 bánh (mm)
                                                                                        rmse = rmse,                                     # ⚠️ RMSE ICP (mm)
                                                                                        dt = dt,                                         # ⚠️ thời gian vòng lặp (s)
                                                                                        last_icp_time = AGVConfig_2.last_icp_time,               # ⚠️ timestamp ICP update cuối
                                                                                        now_time = AGVConfig_2.now_time,                         # ⚠️ timestamp hiện tại
                                                                                        config = AGVConfig_2.CONFIG_DIFF_DRIVE                   # ⚠️ dict cấu hình ngưỡng)
                                                                                        )
                    self.dt0 = time.time()
                    
                    # test bản đồ
                    AGVConfig_2.loi_icp = False
                    if AGVConfig_2.them_vi_tri_xe_ban_do == 0 and AGVConfig_2.loi_icp == False:
                        self.pose = new_pose
                        # print("---------------")
                    if AGVConfig_2.loi_icp == True:
                        music.data["icp_sai_vi_tri"] = 1
                    else:
                        music.data["icp_sai_vi_tri"] = 0
            else:
                rmse = 1000.0
        
        self.rmse = rmse

        self.trajectory.append(self.pose.copy())

        # transform scan points to world (map) frame using updated pose
        R = self.pose[:3,:3]
        tvec = self.pose[:3,3]
        pts_world = (scan_pts2 @ R.T) + tvec
        px_map, py_map = None, None

        if xac_dinh_vi_tri_xe == 1:
            # Lấy vị trí hiện tại của AGV để tìm điểm gần nhất
            if AGVConfig_2.nang_vuong_goc == 0:
                direction = "tren_duoi"
            else:
                direction = "trai_phai"
            
            # print(pts_world, [self.tam_x_mm, -self.tam_y_mm], direction)
            self.tam_hcn, self.goc_hcn, self.diem_gan_nhat = tam_va_goc_hcn.get_trolley_pose(pts_world, [self.tam_x_mm, -self.tam_y_mm], direction)
            # print("self.tam_hcn, self.goc_hcn, self.diem_gan_nhat ", self.tam_hcn, self.goc_hcn, self.diem_gan_nhat )
            if self.tam_hcn is not None:
                if self.diem_gan_nhat is not None:
                    self.diem_gan_nhat[1] = -self.diem_gan_nhat[1]
                AGVConfig_2.diem_gan_nhat = self.diem_gan_nhat
                self.tam_hcn[1] = -self.tam_hcn[1]
                # print("kkkkk",self.tam_hcn, self.tam_x_mm, self.tam_y_mm, self.goc_hcn)

                AGVConfig_2.toa_do_tam_xe_mm = self.tam_hcn
                if AGVConfig_2.nang_vuong_goc == 0:
                    AGVConfig_2.goc_hcn = ad.normalize_angle(90 - self.goc_hcn)
                else:
                    AGVConfig_2.goc_hcn = ad.normalize_angle(180-self.goc_hcn)


            # khoảng cách giữa điểm self.tam_hcn và tâm agv là
            if self.tam_hcn is not None:
                self.kc_tam_agv_hcn = tinh_luong_giac.calculate_distance([self.tam_x_mm, self.tam_y_mm], [self.tam_hcn[0], self.tam_hcn[1]])
            else:
                self.kc_tam_agv_hcn = None

        # Luồng 1: Khi chế độ "Chỉnh sửa vị trí xe bản đồ" được bật, thêm tất cả các điểm đã lọc vào bản đồ
        if cap_nhat_vi_tri_xe == 1:
            if pts_world.shape[0] > 0:
                if self.global_map.shape[0] == 0:
                    self.global_map = pts_world
                else:
                    self.global_map = np.vstack((self.global_map, pts_world))
                # Cập nhật occupancy map với các điểm mới, bỏ qua các bộ lọc khác
                px_map, py_map = self._update_occupancy_map(pts_world, self.pose[:3,3], force_update_map=1, cap_nhat_vi_tri_xe = cap_nhat_vi_tri_xe)
                print(f"Đã thêm {pts_world.shape[0]} điểm vào bản đồ ở chế độ chỉnh sửa.")

        # Luồng 2: Nếu không ở chế độ chỉnh sửa, tích hợp vào bản đồ theo cách thông thường
        elif cap_nhat_ban_do == 1:
            # filter dynamic-looking points (candidate filtering)
            pts_world = self._filter_dynamic_points(pts_world)

            # accumulate voxel counts and commit only after threshold
            self._accumulate_voxels_and_commit(pts_world)

            # create keyframe periodically
            self._frames_since_keyframe += 1
            moved = 0.0
            if len(self.keyframes) > 0:
                moved = np.linalg.norm(self.pose[:2,3] - self.keyframes[-1]['pose'][:2,3])
            if self._frames_since_keyframe >= self.keyframe_interval_frames or moved > self.keyframe_dist_mm:
                # build a local pcd for keyframe
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pts_world)
                pcd = pcd.voxel_down_sample(self.voxel_size_mm)
                self.keyframes.append({'pose': self.pose.copy(), 'pcd': pcd, 'timestamp': time.time()})
                self._frames_since_keyframe = 0

            # if self.executor:
            #     self.executor.submit(self._update_occupancy_map, pts_world, self.pose[:3,3], force_update_map)
            # else:
            px_map, py_map = self._update_occupancy_map(pts_world, self.pose[:3,3], update_all_point_in_map)
        self.goc_agv = -np.arctan2(R[1, 0], R[0, 0]) # đơn vị radian, 0 là hướng sang phải, ngược chiều kim đồng hồ là dương
        self.tam_x_mm = tvec[0] # tọa độ agv trong bản đồ gốc đơn vị mm
        self.tam_y_mm = -tvec[1]
        self.tam_x_pixel = self.center_px[0] + self.tam_x_mm / self.resolution_mm
        self.tam_y_pixel = self.center_px[1] + self.tam_y_mm / self.resolution_mm


        if cap_nhat_ban_do == 1:
            if AGVConfig.tao_ban_do_moi == True and AGVConfig_2.cap_nhat_ban_do_1_lan == False:
                AGVConfig_2.cap_nhat_ban_do_1_lan = True
                AGVConfig.cap_nhat_ban_do_1_lan_web = True
                self.load_state(tao_ban_do_moi=True)
            if AGVConfig.tao_ban_do_moi == False and AGVConfig_2.cap_nhat_ban_do_1_lan == True:
                AGVConfig_2.cap_nhat_ban_do_1_lan = False
        else:
            AGVConfig_2.cap_nhat_ban_do_1_lan = False




        AGVConfig.img_mapping, x_crop, y_crop = self.draw_debug(self.log_odds, window_name="map", show_scan=pts_world)

        if px_map is not None and py_map is not None:
            AGVConfig.danh_sach_diem_lidar_icp = np.vstack((px_map, py_map, np.zeros_like(px_map))).T
           
        else:
            # --- Tạo mask free-space ---
            px_map = (self.center_px[0] + pts_world[:, 0] / self.resolution_mm).astype(np.int32)
            py_map = (self.center_px[1] - pts_world[:, 1] / self.resolution_mm).astype(np.int32)
            AGVConfig.danh_sach_diem_lidar_icp = np.vstack((px_map, py_map, np.zeros_like(px_map))).T # [[x1 y1 0][x2 y2 0]]

        return self.center_px, pts_world, rmse, self.tam_x_mm, self.tam_y_mm, self.tam_x_pixel, self.tam_y_pixel, self.goc_agv, self.resolution_mm


    # ----------------- visualization helpers -----------------
    def _world_to_pixel(self, x, y):
        ux = int(self.center_px[0] + x / self.resolution_mm)
        uy = int(self.center_px[1] - y / self.resolution_mm)
        return ux, uy

    def get_occupancy_image(self, log_odds_map=None):
        log_odds = self.log_odds if log_odds_map is None else log_odds_map
        # Tối ưu: Thay thế hàm exp bằng việc so sánh ngưỡng trực tiếp trên log-odds
        # p < 0.1 tương đương L < -2.197
        # p > 0.6 tương đương L > 0.405
        img = np.full(log_odds.shape, 128, dtype=np.uint8)
        img[log_odds < -2.197] = 255    # free -> trắng
        img[log_odds > 0.405] = 0       # tường -> đen
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def draw_debug(self, log_odds_map=None, window_name="map", show_scan=None, scale=1.0, display_half_size = 500, load_map_all = 0):
        # --- Lấy bản đồ occupancy ---
        if log_odds_map is None:
            log_odds_map = self.log_odds

        h_full, w_full = log_odds_map.shape
        x_robot, y_robot, yaw_robot = pose_to_xyyaw(self.pose)
        ux_robot, uy_robot = self._world_to_pixel(x_robot, y_robot)

        x1 = max(0, ux_robot - display_half_size)
        y1 = max(0, uy_robot - display_half_size)
        x2 = min(w_full, ux_robot + display_half_size)
        y2 = min(h_full, uy_robot + display_half_size)

        if self.map_image_color is None or AGVConfig.che_do_tao_ban_do == True:
            if self.map_image_color is None:
                self.map_image_color = self.get_occupancy_image(self.log_odds)
            log_odds_crop = log_odds_map[y1:y2, x1:x2]
            img_color = self.get_occupancy_image(log_odds_crop)
            # cập nhật map_image_color
            self.map_image_color[y1:y2, x1:x2] = img_color

        return self.map_image_color, x1, y1

    # ===============================================================
    # HÀM LƯU VÀ TẢI TRẠNG THÁI
    # ===============================================================
    def save_state(self, directory_path):
        """
        Lưu trạng thái hiện tại của mapper vào một thư mục.
        Bao gồm: global_map, log_odds, pose, và keyframes.
        """
        try:
            os.makedirs(directory_path, exist_ok=True)
            print(f"Đang lưu trạng thái vào thư mục: {directory_path}")

            # 1. Lưu các mảng NumPy chính
            np.save(os.path.join(directory_path, "global_map.npy"), self.global_map)
            np.save(os.path.join(directory_path, "log_odds.npy"), self.log_odds)
            np.save(os.path.join(directory_path, "pose.npy"), self.pose)

            # 2. Lưu keyframes
            keyframes_dir = os.path.join(directory_path, "keyframes")
            os.makedirs(keyframes_dir, exist_ok=True)
            
            # Xóa các keyframe cũ trước khi lưu
            for f in os.listdir(keyframes_dir):
                os.remove(os.path.join(keyframes_dir, f))

            for i, kf in enumerate(self.keyframes):
                kf_pose_path = os.path.join(keyframes_dir, f"kf_{i}_pose.npy")
                kf_pcd_path = os.path.join(keyframes_dir, f"kf_{i}_pcd.pcd")
                np.save(kf_pose_path, kf['pose'])
                o3d.io.write_point_cloud(kf_pcd_path, kf['pcd'])

            print("Lưu trạng thái thành công.")
            return True
        except Exception as e:
            print(f"Lỗi khi lưu trạng thái: {e}")
            return False

    def load_state(self, directory_path = None, tao_ban_do_moi = False):
        """
        Tải trạng thái của mapper từ một thư mục.
        """
        print("--------------load_state------------------")
        try:
            if tao_ban_do_moi == True:
                self.log_odds = np.zeros((self.pixels, self.pixels), dtype=np.float32)
                self.global_map = np.empty((0,3), dtype=np.float64)
                self.pose = np.eye(4, dtype=np.float64)
                self.keyframes = []
            else:
                if not os.path.isdir(directory_path):
                    print(f"Lỗi: Thư mục không tồn tại: {directory_path}")
                    return False
                
                print(f"Đang tải trạng thái từ thư mục: {directory_path}")

                # 1. Tải các mảng NumPy chính
                global_map_path = os.path.join(directory_path, "global_map.npy")
                log_odds_path = os.path.join(directory_path, "log_odds.npy")
                pose_path = os.path.join(directory_path, "pose.npy")

                if not all(os.path.exists(p) for p in [global_map_path, log_odds_path, pose_path]):
                    print("Lỗi: Thiếu một trong các file global_map.npy, log_odds.npy, pose.npy.")
                    return False

                self.global_map = np.load(global_map_path)
                self.log_odds = np.load(log_odds_path)
                self.pose = np.load(pose_path)
            

                # 2. Tải keyframes
                self.keyframes = []
                keyframes_dir = os.path.join(directory_path, "keyframes")
                if os.path.isdir(keyframes_dir):
                    kf_files = os.listdir(keyframes_dir)
                    # Đếm số lượng keyframe dựa trên số file pose
                    num_keyframes = len([f for f in kf_files if f.endswith("_pose.npy")])

                    for i in range(num_keyframes):
                        kf_pose_path = os.path.join(keyframes_dir, f"kf_{i}_pose.npy")
                        kf_pcd_path = os.path.join(keyframes_dir, f"kf_{i}_pcd.pcd")

                        if os.path.exists(kf_pose_path) and os.path.exists(kf_pcd_path):
                            kf_pose = np.load(kf_pose_path)
                            kf_pcd = o3d.io.read_point_cloud(kf_pcd_path)
                            # Giả sử timestamp không quá quan trọng khi tải lại
                            self.keyframes.append({'pose': kf_pose, 'pcd': kf_pcd, 'timestamp': time.time()})
            
            # 3. Đặt lại các cache và trạng thái tạm thời
            self.map_image_color = None
            self._target_pcd_cache = None
            self._target_cache_pose = None
            self.voxel_counts = {}
            self.voxel_point = {}
            self.trajectory.clear()
            self.trajectory.append(self.pose.copy())
            self._frames_since_keyframe = 0

            print(f"Tải trạng thái thành công. {len(self.global_map)} điểm, {len(self.keyframes)} keyframes.")
            return True

        except Exception as e:
            print(f"Lỗi khi tải trạng thái: {e}")
            return False



# ----------------- demo / usage example -----------------
if __name__ == "__main__":
    
    mapper = FastLidarMapper()

    frames = None
    if os.path.exists("lidar_frames.npy"):
        try:
            frames = np.load("lidar_frames.npy", allow_pickle=True)
            print("Loaded lidar_frames.npy with", len(frames), "frames")
        except Exception as e:
            print("Failed to load lidar_frames.npy:", e)
            frames = None

    idx = 0
    stt_scan = 0
    path_folder_scan_data_2 = r"C:\tupn\phan_mem\a_agv\agv_2_lidar\data_input_output\scan_data_21"
    path_folder_scan_data_3 = r"C:\tupn\phan_mem\a_agv\agv_2_lidar\data_input_output\scan_data_31"
    chieu_ngang_xe = 530
    chieu_doc_xe = 790

    try:
        while True:
            if stt_scan < len(os.listdir(path_folder_scan_data_2))-2:
                stt_scan += 1
            scan_alpha_1 = np.load(path_folder_scan_data_2 + "/scan_"+ str(stt_scan) +".npy")
            scan_alpha_2 = np.load(path_folder_scan_data_3 + "/scan_"+ str(stt_scan) +".npy")

            ten_lidar = "duoi"
            huong_agv_khong_icp = np.array([[1, 29.30319303, 892.55780885]])
            scan, scan1, scan2, huong_agv_toa_do_xyz = convert_2_lidar.convert_scan_lidar(scan1_data_example=scan_alpha_1,
                                                                        scan2_data_example=scan_alpha_2,
                                                                        scan_an_toan = huong_agv_khong_icp,
                                                                        ten_lidar = ten_lidar,
                                                                        scaling_factor = 1,
                                                                        lidar1_orient_deg = 45,
                                                                        lidar2_orient_deg =-136,
                                                                        agv_w=chieu_ngang_xe,
                                                                        agv_l=chieu_doc_xe)
            hh = time.time()
            img_color, scan_world, x_crop, y_crop, rmse, \
                tam_x_mm, tam_y_mm, tam_x_pixel, tam_y_pixel, goc_agv = mapper.process_scan(scan, integrate=True)
            img_color = cv2.resize(img_color, None, fx=mapper.display_scale, fy=mapper.display_scale, interpolation=cv2.INTER_AREA)
            cv2.imshow("window_name", img_color)
            cv2.waitKey(1)
            # print(f"Frame {idx}: pts={out['num_points']} rmse={out['rmse']:.4f} pose=({out['pose'][0,3]:.2f},{out['pose'][1,3]:.2f})")
            idx += 1
    except KeyboardInterrupt:
        print("Stopped.")
        cv2.destroyAllWindows()
