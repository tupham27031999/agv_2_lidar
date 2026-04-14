import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from config_2 import AGVConfig_2
from config import AGVConfig

class SafetyZoneDetector:
    def detect_exclusion_zones(self, points, duong_kinh_coc=100, min_samples=1):
        """
        Tự động tìm vùng loại trừ (chân xe) khi AGV đứng ở vùng trống.
        Dùng toàn bộ số nguyên (đơn vị mm).

        Args:
            points (list | np.ndarray): Dữ liệu Lidar [[x, y, 1], ...] (mm)
            duong_kinh_coc (int): Bán kính gom cụm (mm)
            min_samples (int): Số điểm tối thiểu để tạo cụm

        Returns:
            list[tuple[int]]: [(xmin, xmax, ymin, ymax), ...] (mm)
        """
        if points is None or len(points) == 0:
            return []

        # Giữ kiểu float để làm tròn chính xác sau này
        points_xy = np.array(points, dtype=float)[:, :2]

        # ---- Bước 4: Gom cụm bằng DBSCAN ----
        # Thực hiện trực tiếp trên các điểm đầu vào vì hướng đã cố định
        clustering = DBSCAN(eps=duong_kinh_coc, min_samples=min_samples).fit(points_xy)
        labels = clustering.labels_

        # ---- Bước 5: Tạo vùng bao quanh mỗi cụm ----
        zones = []
        for label in np.unique(labels):
            if label == -1:
                continue
            cluster = points_xy[labels == label]
            xmin, ymin = np.min(cluster, axis=0)
            xmax, ymax = np.max(cluster, axis=0)
            # Làm tròn xuống cho min và làm tròn lên cho max để đảm bảo bao trọn điểm
            # Thêm một khoảng đệm nhỏ (ví dụ 1mm) để an toàn hơn
            zones.append((int(xmin) - 1, int(xmax) + 1, int(ymin) - 1, int(ymax) + 1))

        return zones


class kiem_tra_vat_can:
    def __init__(self):
        self.detector = SafetyZoneDetector()
        self.diem_vung_loai_bo = []
        

        self.khoang_cach_an_toan_tren_tools = AGVConfig_2.khoang_cach_an_toan_tren_tools
        self.khoang_cach_an_toan_ben_canh_tools = AGVConfig_2.khoang_cach_an_toan_ben_canh_tools
        self.khoang_cach_an_toan_duoi_tools = AGVConfig_2.khoang_cach_an_toan_duoi_tools

        self.khoang_cach_an_toan_duoi_nang = AGVConfig_2.khoang_cach_an_toan_duoi_nang
        self.khoang_cach_an_toan_ben_canh_nang = AGVConfig_2.khoang_cach_an_toan_ben_canh_nang
        self.khoang_cach_an_toan_tren_nang = AGVConfig_2.khoang_cach_an_toan_tren_nang

        self.khoang_cach_an_toan_tren = AGVConfig_2.khoang_cach_an_toan_tren
        self.khoang_cach_an_toan_ben_canh = AGVConfig_2.khoang_cach_an_toan_ben_canh
        self.khoang_cach_an_toan_duoi = AGVConfig_2.khoang_cach_an_toan_duoi

        self.khoang_cach_an_toan_tren_re = AGVConfig_2.khoang_cach_an_toan_tren_re
        self.khoang_cach_an_toan_ben_canh_re = AGVConfig_2.khoang_cach_an_toan_ben_canh_re
        self.khoang_cach_an_toan_duoi_re = AGVConfig_2.khoang_cach_an_toan_duoi_re


        self.khoang_cach_an_toan_duoi_xe = AGVConfig_2.khoang_cach_an_toan_duoi_xe
        self.khoang_cach_an_toan_ben_canh_xe = AGVConfig_2.khoang_cach_an_toan_ben_canh_xe
        self.khoang_cach_an_toan_tren_xe = AGVConfig_2.khoang_cach_an_toan_tren_xe

        self.khoang_cach_an_toan_tren_xe_re = AGVConfig_2.khoang_cach_an_toan_tren_xe_re
        self.khoang_cach_an_toan_ben_canh_xe_re = AGVConfig_2.khoang_cach_an_toan_ben_canh_xe_re
        self.khoang_cach_an_toan_duoi_xe_re = AGVConfig_2.khoang_cach_an_toan_duoi_xe_re

    def _polar_to_cartesian(self, scan_data):
        """Chuyển đổi từ tọa độ cực (quality, angle, distance) sang Descartes (x, y, quality)."""
        if scan_data.ndim != 2 or scan_data.shape[1] != 3 or scan_data.shape[0] == 0:
            return np.array([])
        
        quality = scan_data[:, 0]
        angles_rad = np.radians(scan_data[:, 1])
        distances = scan_data[:, 2]

        x = distances * np.cos(angles_rad)
        y = distances * np.sin(angles_rad)
        
        # Trả về mảng (x, y, quality)
        return np.vstack((x, y, quality)).T

    def _cartesian_to_polar(self, cartesian_points):
        """Chuyển đổi từ tọa độ Descartes (x, y, quality) về lại tọa độ cực (quality, angle, distance)."""
        if cartesian_points.ndim != 2 or cartesian_points.shape[1] != 3 or cartesian_points.shape[0] == 0:
            return np.array([])

        x = cartesian_points[:, 0]
        y = cartesian_points[:, 1]
        quality = cartesian_points[:, 2]

        distances = np.sqrt(x**2 + y**2)
        angles_rad = np.arctan2(y, x)
        angles_deg = np.degrees(angles_rad)
        # Chuẩn hóa góc về [0, 360)
        angles_deg = (angles_deg + 360) % 360

        return np.vstack((quality, angles_deg, distances)).T

    def _downsample_points(self, scan_data, voxel_size=50.0):
        """
        Lọc và gộp các điểm gần nhau sử dụng Voxel Grid.
        scan_data: Dữ liệu Lidar ở dạng tọa độ cực.
        voxel_size: Kích thước voxel để gộp điểm (mm).
        """
        if scan_data.shape[0] < 2:
            return scan_data

        # 1. Chuyển sang tọa độ Descartes (x, y, quality)
        cartesian_points = self._polar_to_cartesian(scan_data)
        if cartesian_points.shape[0] == 0:
            return scan_data

        # 2. Sử dụng Open3D để downsample
        pcd = o3d.geometry.PointCloud()
        # Open3D cần (x, y, z), ta sẽ dùng quality cho z
        pcd.points = o3d.utility.Vector3dVector(np.hstack((cartesian_points[:, :2], np.zeros((cartesian_points.shape[0], 1)))))
        
        # Lưu lại quality tương ứng với mỗi điểm
        pcd.colors = o3d.utility.Vector3dVector(np.vstack((cartesian_points[:, 2], np.zeros(cartesian_points.shape[0]), np.zeros(cartesian_points.shape[0]))).T)

        # Downsample và lấy trung bình các điểm trong voxel
        downsampled_pcd = pcd.voxel_down_sample_and_trace(voxel_size, pcd.get_min_bound(), pcd.get_max_bound())[0]
        downsampled_points_cartesian = np.asarray(downsampled_pcd.points)
        
        # 3. Chuyển lại tọa độ cực
        return self._cartesian_to_polar(downsampled_points_cartesian)
        
    def filter_points_in_radius(self, all_points, center, radius_max, radius_min=0):
        """
        Lọc các điểm trong một khoảng bán kính (vành khuyên) quanh tâm (center).
        all_points: np.ndarray shape (N, 3)
        center: np.array shape (3,)
        radius_max: float, bán kính ngoài (mm)
        radius_min: float, bán kính trong (mm)
        """
        if all_points.ndim != 2 or all_points.shape[0] == 0:
            return all_points

        # Dữ liệu là (quality, angle, distance), nên cột khoảng cách là cột thứ 3 (index 2)
        distances = all_points[:, 2]

        # Lọc các điểm có khoảng cách nằm trong khoảng [radius_min, radius_max]
        mask = (distances >= radius_min) & (distances <= radius_max)
        return all_points[mask]
    
    def _check_zone(self, scan_data, d1, d2, d3):
        """
        Kiểm tra một vùng an toàn cụ thể cho một lidar.
        Trả về điểm vi phạm đầu tiên nếu tìm thấy, ngược lại trả về None.
        """
        if scan_data.shape[0] > 0:
            angles = scan_data[:, 1]
            ranges = scan_data[:, 2]
            for i in range(scan_data.shape[0]):
                if self.thong_so_alpha(d1, d2, d3, angles[i], ranges[i]):
                    return scan_data[i]
        return None
    def filter_exclusion_zones(self, points, exclusion_zones):
        """Lọc các điểm nằm trong vùng loại trừ."""
        mask = np.ones(len(points), dtype=bool)
        if not exclusion_zones:
            return points

        for (xmin, xmax, ymin, ymax) in exclusion_zones:
            inside = (
                (points[:, 0] >= xmin) & (points[:, 0] <= xmax) &
                (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
            )
            mask &= ~inside  # loại bỏ điểm bên trong vùng loại trừ
        return points[mask]

    def calibrate_exclusion_zones(self, lidar_points, direction):
        # This method seems unused, but if used, it should be reviewed.
        # For now, let's assume it's not part of the main safety check logic.
        pass

    def detect(self, points, direction, di_thuan_nguoc, dang_re, che_do_lay_mau = 0, xac_dinh_vi_tri_xe = 0, di_chuyen_luon = None):
        """
        Kiểm tra xem có điểm Lidar nào nằm trong vùng an toàn quanh AGV hay không.

        Thứ tự ưu tiên vùng được xét theo thứ tự trong dict 'zones'.

        Args:
            points (list | np.ndarray): Danh sách điểm [[x, y, 1], ...]
            zones (dict): Các vùng an toàn dạng:
                {
                    "vung_1": (d1, d2, d3),
                    "vung_2": (d1, d2, d3),
                    "vung_3": (d1, d2, d3),
                }
            exclusion_zones (list): Danh sách các vùng loại trừ (chân xe), dạng:
                [
                    (xmin, xmax, ymin, ymax),
                    ...
                ]

        Returns:
            tuple: (name, [[x, y, 1]]) nếu có điểm nằm trong vùng ưu tiên cao nhất,
                hoặc ("", []) nếu không có điểm nào.
        """
        # if di_thuan_nguoc == 1:
        #     khoang_cach_tren = khoang_cach_an_toan_tren_xe_nguoc
        #     khoang_cach_ben_canh = khoang_cach_an_toan_ben_canh_xe_nguoc
        #     khoang_cach_duoi = khoang_cach_an_toan_duoi_xe_nguoc
        # else:

        # khoang_cach_tren = khoang_cach_an_toan_tren_xe
        # khoang_cach_ben_canh = khoang_cach_an_toan_ben_canh_xe
        # khoang_cach_duoi = khoang_cach_an_toan_duoi_xe
        delta0 = 0
        delta1 = 0
        if che_do_lay_mau == 1:
            khoang_cach_tren = self.khoang_cach_an_toan_tren_tools
            khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_tools
            khoang_cach_duoi = self.khoang_cach_an_toan_duoi_tools
        else:
            if xac_dinh_vi_tri_xe == 1:
                khoang_cach_tren = self.khoang_cach_an_toan_duoi_nang
                khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_nang
                khoang_cach_duoi = self.khoang_cach_an_toan_tren_nang
            else:
                if len(AGVConfig.vung_loai_bo_x1y1x2y2) == 0:
                    if dang_re == 0:
                        if di_chuyen_luon is not None and AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["di_chuyen_khong_hang"] != True: # 1
                            if di_chuyen_luon["delta"] is not None:
                                delta0 = di_chuyen_luon["delta"][0]
                                delta1 = di_chuyen_luon["delta"][1]
                        if di_thuan_nguoc == 1:
                            khoang_cach_tren = self.khoang_cach_an_toan_duoi
                            khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh
                            khoang_cach_duoi = self.khoang_cach_an_toan_tren
                        else:
                            khoang_cach_tren = self.khoang_cach_an_toan_tren
                            khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh
                            khoang_cach_duoi = self.khoang_cach_an_toan_duoi
                    else:
                        khoang_cach_tren = self.khoang_cach_an_toan_tren_re
                        khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_re
                        khoang_cach_duoi = self.khoang_cach_an_toan_duoi_re
                else:
                    if dang_re == 0:
                        if di_chuyen_luon is not None:
                            if di_chuyen_luon["delta"] is not None and AGVConfig.tin_hieu_nhan[AGVConfig.name_agv]["di_chuyen_khong_hang"] != True: # 2
                                delta0 = di_chuyen_luon["delta"][0]
                                delta1 = di_chuyen_luon["delta"][1]
                        if di_thuan_nguoc == 1:
                            khoang_cach_tren = self.khoang_cach_an_toan_duoi_xe
                            khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_xe
                            khoang_cach_duoi = self.khoang_cach_an_toan_tren_xe
                        else:
                            khoang_cach_tren = self.khoang_cach_an_toan_tren_xe
                            khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_xe
                            khoang_cach_duoi = self.khoang_cach_an_toan_duoi_xe
                    else:
                        khoang_cach_tren = self.khoang_cach_an_toan_tren_xe_re
                        khoang_cach_ben_canh = self.khoang_cach_an_toan_ben_canh_xe_re
                        khoang_cach_duoi = self.khoang_cach_an_toan_duoi_xe_re

        if len(khoang_cach_tren) < 3 or len(khoang_cach_ben_canh) < 3 or len(khoang_cach_duoi) < 3:
            return "error", []

        zones = {
            "vung_1": (khoang_cach_tren[0] - delta0, khoang_cach_ben_canh[0] - delta1, khoang_cach_duoi[0]),
            "vung_2": (khoang_cach_tren[1], khoang_cach_ben_canh[1], khoang_cach_duoi[1]),
            "vung_3": (khoang_cach_tren[2], khoang_cach_ben_canh[2], khoang_cach_duoi[2]),
        }
        
        # ---- Bước 1: Kiểm tra đầu vào rỗng ----
        points = np.array(points)
        if points.size == 0:
            return "none", []

        # ---- Bước 2: Lọc các điểm trong vùng loại trừ (chân xe) ----
        # Vùng loại trừ là cố định so với thân xe, nên ta lọc trên các điểm gốc.
        # print("----------------000------------", self.vung_loai_bo, len(self.vung_loai_bo))
        points = self.filter_exclusion_zones(points, AGVConfig.vung_loai_bo_x1y1x2y2)

        # Nếu không còn điểm sau khi lọc
        if len(points) == 0:
            return "none", []

        # ---- Bước 3: Xoay các điểm Lidar về hệ tọa độ của HƯỚNG DI CHUYỂN ----
        # Đây là một phép biến đổi tạm thời để việc kiểm tra vùng an toàn (hình chữ nhật) trở nên đơn giản.
        # Thay vì xoay hình chữ nhật, ta xoay các điểm theo chiều ngược lại.
        if direction is not None and direction.shape[0] > 0:
            xh, yh = direction[0, 0], direction[0, 1]
            theta = np.arctan2(yh, xh)
            rot_matrix = np.array([[np.cos(-theta), -np.sin(-theta)],
                                   [np.sin(-theta),  np.cos(-theta)]])
            rotated_points_xy = points[:, :2] @ rot_matrix.T
        else: # Nếu không có direction, coi như hướng thẳng, không cần xoay.
            rotated_points_xy = points[:, :2]

        # ---- Bước 4: Kiểm tra các điểm đã xoay có nằm trong vùng an toàn không ----
        for name, (d1, d2, d3) in zones.items():
            # d1: trước, d2: ngang, d3: sau
            # Hệ tọa độ xoay: x là trục dọc của AGV, y là trục ngang
            # Do đó, x tương ứng với d1 (trước) và d3 (sau)
            # y tương ứng với d2 (ngang)            
            mask = (
                (rotated_points_xy[:, 0] >= -d3) & # Giới hạn sau
                (rotated_points_xy[:, 0] <= d1) &  # Giới hạn trước
                (rotated_points_xy[:, 1] >= -d2) & # Giới hạn ngang (trái)
                (rotated_points_xy[:, 1] <= d2)   # Giới hạn ngang (phải)
            )
            idx = np.where(mask)[0]
            
            if len(idx) > 0:
                # print(idx, len(idx), np.where(mask))
                i = idx[0]  # Lấy điểm đầu tiên trong vùng
                
                # # eps = 50mm (bán kính gom cụm)
                if che_do_lay_mau == 1:
                    self.diem_vung_loai_bo.append(points[i].tolist())
                    d = AGVConfig_2.duong_kinh_coc *2
                    AGVConfig.vung_loai_bo_x1y1x2y2 = self.detector.detect_exclusion_zones(self.diem_vung_loai_bo, d, AGVConfig_2.min_samples)
                    # print("[points[i].tolist()]", [points[i].tolist()])
                else:
                    self.diem_vung_loai_bo = []

                # print("📏 exclusion_zones =", exclusion_zones)
                return name, [points[i].tolist()]

        

        # print("📏 exclusion_zones =", self.exclusion_zones)
        # ---- Bước 7: Không có điểm nào trong cả 3 vùng ----
        return "none", []

    


if __name__ == "__main__":
    detector = SafetyZoneDetector()

    # # Dữ liệu lidar: toàn bộ là mm
    lidar_points = [
        [-300, 250, 1], [-280, 270, 1], [-320, 260, 1],   # cụm trái
        [300, 200, 1], [330, 220, 1], [310, 180, 1],      # cụm phải
        [0, -150, 1], [20, -180, 1], [-20, -170, 1]       # cụm sau
    ]

    # Hướng AGV: trục X (vẫn dùng mm)
    direction = [1000, 0]

    # eps = 50mm (bán kính gom cụm)
    zones = detector.detect_exclusion_zones(lidar_points, duong_kinh_coc=50, min_samples=1)

    print("📏 exclusion_zones =", zones)
