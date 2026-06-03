import os
import cv2
import time
import threading
import numpy as np
from harvesters.core import Harvester
from libs_ngoai_vi import ket_noi_esp_loa

def list_sentech_cameras():
    h = Harvester()
    # Sử dụng đường dẫn driver từ cấu hình của bạn
    cti_path = '/opt/sentech/lib/libstgentl.cti'
    
    if not os.path.exists(cti_path):
        print(f"Không tìm thấy driver tại: {cti_path}")
        return

    h.add_file(cti_path)
    h.update()

    print(f"--- Tìm thấy {len(h.device_info_list)} thiết bị GenTL ---")
    thong_tin_cac_camera = []
    for i, info in enumerate(h.device_info_list):
        thong_tin_cac_camera.append({"Index": i, 
                                     "Model": info.model, 
                                     "Serial": info.serial_number, 
                                     "Vendor": info.vendor})
        print(f"Index: {i}")
        print(f"  Model: {info.model}")
        print(f"  Serial: {info.serial_number}")
        print(f"  Vendor: {info.vendor}")
        print("-" * 30)
    
    h.reset()
    return thong_tin_cac_camera


class AGVCamera:
    # Định nghĩa các hằng số chế độ
    MODE_RELEASE = 0  # Ngắt kết nối để làm mát
    MODE_ACTIVE  = 1  # Chạy full tính năng nhận diện

    def __init__(self, src=0, config=None, tag_size=50.0):
        """
        Khởi tạo AGV Camera
        :param src: ID camera (0, 1, ...)
        :param config: Dictionary chứa thông số cấu hình`
        :param tag_size: Kích thước thực của mã AprilTag (đơn vị: mm)
        """
        self.src = src
        self.config = config if config else {
            'fps': 120,
            'exposure': -13,
            'gain': 20,
            'width': 640,
            'height': 480,
            'mm_per_pixel': 0.33,  # Tỷ lệ mm/pixel (Xác nhận: mm = pixel * ratio)
            'cti_path': '/opt/sentech/lib/libstgentl.cti' # Đường dẫn thực tế trên hệ thống
        }

        # Thông số phục vụ tính toán
        self.tag_size = tag_size
        
        self.ia = None
        self.frame = None
        self.ret = False
        self.stopped = True
        self.mode = self.MODE_ACTIVE

        # Các biến phục vụ đo FPS thực tế
        self.fps_real = 0.0
        self._last_fps_time = time.perf_counter()
        self._frame_counter = 0
        
        # Khởi tạo Harvester một lần duy nhất để tránh lỗi Driver GenTL khi re-init
        self.h = Harvester()
        cti_path = self.config.get('cti_path', '/opt/sentech/lib/libstgentl.cti')
        if os.path.exists(cti_path):
            try:
                self.h.add_file(cti_path)
                print(f"[CAM] Đã nạp driver: {cti_path}")
            except Exception as e:
                print(f"[CAM] Lỗi nạp driver: {e}")
        else:
            print(f"[CAM] Cảnh báo: Không tìm thấy file .cti tại {cti_path}")
            
        self.last_mode = None
        
        # Khởi tạo AprilTag Detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Tối ưu nhận diện
        self.aruco_params.minMarkerPerimeterRate = 0.03 
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 15 # Giảm thêm để tăng tốc độ ngưỡng
        self.aruco_params.adaptiveThreshWinSizeStep = 10 # Tăng bước quét ngưỡng để tăng tốc
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        
        self.latest_info = [] # Lưu kết quả nhận diện mới nhất
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Quản lý ROI (Vùng quan tâm)
        self.roi = None
        self.lost_count = 0
        
        # Threading
        self.lock = threading.Lock()
        self.thread = None

    def _init_camera(self):
        """Thiết lập kết nối camera và cấu hình thông số phần cứng"""
        print(f"\n[CAM] --- Khởi tạo Sentech GenTL Camera (Index: {self.src}) ---")
        try:
            self.h.update()
            if not self.h.device_info_list:
                print("[CAM] Lỗi: Không tìm thấy thiết bị qua Harvester.")
                return False
                
            if self.src >= len(self.h.device_info_list):
                print(f"[CAM] Lỗi: Index {self.src} không tồn tại (Chỉ tìm thấy {len(self.h.device_info_list)} thiết bị).")
                return False
                
            self.ia = self.h.create(self.src)
            self._apply_mode_params() # Cấu hình thông số TRƯỚC khi start để tránh lỗi "Node is not writable"
            self.ia.start()
            print("[CAM] Kết nối thành công qua Harvester.")
            return True
        except Exception as e:
            print(f"[CAM] Lỗi khởi tạo Harvester: {e}")
            return False

    def start(self):
        """Bắt đầu luồng đọc camera"""
        if self.stopped:
            self.stopped = False
            self.thread = threading.Thread(target=self._update, args=())
            self.thread.daemon = True
            self.thread.start()
        return self

    def _update(self):
        """Vòng lặp chạy ngầm để cập nhật frame"""
        while not self.stopped:
            if self.mode == self.MODE_RELEASE:
                if self.ia is not None:
                    self.ia.stop()
                    self.ia.destroy()
                    self.ia = None
                with self.lock:
                    self.ret = False
                    self.frame = None
                    self.fps_real = 0.0
                    self._frame_counter = 0
                time.sleep(0.1)
                continue

            if self.ia is None:
                if not self._init_camera():
                    time.sleep(2)
                    continue

            try:
                # Lấy frame từ buffer với timeout
                with self.ia.fetch(timeout=0.5) as buffer:
                    component = buffer.payload.components[0]
                    
                    # Lấy dữ liệu thô từ buffer
                    raw_frame = np.frombuffer(component.data, dtype=np.uint8).reshape(
                        component.height, component.width
                    )
                    
                    # --- XỬ LÝ NHẬN DIỆN NGAY TRONG LUỒNG CAMERA ---
                    t_start_detect = time.perf_counter()
                    tag_info = []
                    gray = raw_frame # Bayer coi như ảnh xám
                    h_img, w_img = gray.shape[:2]
                    ratio = self.config.get('mm_per_pixel', 1.0)
                    
                    current_corners, current_ids = None, None
                    if self.roi is not None:
                        rx, ry, rw, rh = self.roi
                        crop = gray[ry:ry+rh, rx:rx+rw]
                        current_corners, current_ids, _ = self.detector.detectMarkers(crop)
                        if current_ids is not None:
                            current_corners = [c + [rx, ry] for c in current_corners]
                    
                    if current_ids is None:
                        current_corners, current_ids, _ = self.detector.detectMarkers(gray)

                    if current_ids is not None:
                        self.lost_count = 0
                        for i in range(len(current_ids)):
                            corners_single = current_corners[i].reshape(4, 2).astype(np.float32)
                            center_tag_x = np.mean(corners_single[:, 0])
                            center_tag_y = np.mean(corners_single[:, 1])
                            
                            mm_x = (center_tag_x - w_img/2) * ratio
                            mm_y = (center_tag_y - h_img/2) * ratio
                            
                            p0, p1 = corners_single[0], corners_single[1]
                            angle = np.degrees(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))

                            tag_info.append({
                                "id": int(current_ids[i][0]),
                                "lech_trai_phai_mm_x": round(float(mm_x), 1),
                                "lech_truoc_sau_mm_y": round(float(mm_y), 1),
                                "goc_lech_do": round(float(angle), 2),
                                "toa_do_2d": current_corners[i].astype(int).tolist()
                            })
                        
                        # Cập nhật ROI
                        all_pts = np.concatenate(current_corners, axis=0).reshape(-1, 2)
                        x_min, y_min = np.min(all_pts, axis=0)
                        x_max, y_max = np.max(all_pts, axis=0)
                        margin = 100
                        roi_x = max(0, int(x_min) - margin)
                        roi_y = max(0, int(y_min) - margin)
                        roi_w = min(w_img - roi_x, int(x_max - x_min) + 2*margin)
                        roi_h = min(h_img - roi_y, int(y_max - y_min) + 2*margin)
                        self.roi = (roi_x, roi_y, roi_w, roi_h)
                    else:
                        self.lost_count += 1
                        if self.lost_count > 10: self.roi = None

                    t_end_detect = time.perf_counter()
                    # In ra thời gian xử lý để debug
                    # print(f"[DEBUG] Detection Time: {(t_end_detect - t_start_detect)*1000:.2f} ms | ROI: {self.roi is not None}")

                    # Cập nhật dữ liệu vào biến chung dưới Lock
                    with self.lock:
                        self.ret = True
                        self.frame = raw_frame.copy()
                        self.latest_info = tag_info
                        
                        self._frame_counter += 1
                        now = time.perf_counter()
                        if now - self._last_fps_time >= 1.0:
                            self.fps_real = self._frame_counter / (now - self._last_fps_time)
                            self._frame_counter = 0
                            self._last_fps_time = now
            except Exception as e:
                self.ret = False
                # Khi xảy ra lỗi (timeout liên tục hoặc rút cáp), giải phóng để cho phép khởi tạo lại
                if self.ia is not None:
                    try:
                        self.ia.stop()
                        self.ia.destroy()
                    except:
                        pass
                    self.ia = None
                print(f"[CAM] Cảnh báo: Mất kết nối hoặc lỗi phần cứng ({e}). Đang thử kết nối lại sau 2 giây...")
                time.sleep(2)

    def _apply_mode_params(self):
        """Thay đổi thông số phần cứng dựa trên chế độ"""
        if self.ia is None: return
        try:
            # Truy cập node_map của camera qua Harvester
            nodes = self.ia.remote_device.node_map
            if self.mode == self.MODE_ACTIVE:
                print(f"[CAM] Đang cấu hình Decimation và FPS (Mục tiêu: {self.config['fps']})...")
                
                # 1. Tắt Auto Exposure & Gain
                if hasattr(nodes, 'ExposureAuto'): nodes.ExposureAuto.value = 'Off'
                if hasattr(nodes, 'GainAuto'): nodes.GainAuto.value = 'Off'
                
                # 1. Thử thiết lập Exposure Mode sang 'Timed' (hoặc 'Timer' tùy dòng máy)
                if hasattr(nodes, 'ExposureMode'):
                    for mode_val in ['Timer', 'Timed']:
                        try:
                            nodes.ExposureMode.value = mode_val
                            print(f"[CAM] Đã thiết lập ExposureMode: {mode_val}")
                            break
                        except:
                            continue
                
                # 2. Thử thiết lập thời gian phơi sáng qua ExposureTime (đơn vị us) hoặc ExposureTimeRaw
                for node_name in ['ExposureTime', 'ExposureTimeRaw']:
                    if hasattr(nodes, node_name):
                        try:
                            target_node = getattr(nodes, node_name)
                            target_node.value = 6 if node_name == 'ExposureTimeRaw' else 1000.0
                            print(f"[CAM] Đã thiết lập {node_name} thành công.")
                            break
                        except Exception as e:
                            print(f"[CAM] Lỗi khi set {node_name}: {e}")

                # 2. Cấu hình Decimation = 2 để lấy Full FOV nhưng giảm Resolution xuống (tăng FPS)
                # Reset Offset và ROI về mặc định trước khi set Decimation
                if hasattr(nodes, 'OffsetX'): nodes.OffsetX.value = 0
                if hasattr(nodes, 'OffsetY'): nodes.OffsetY.value = 0
                
                if hasattr(nodes, 'DecimationHorizontal'):
                    nodes.DecimationHorizontal.value = 2
                if hasattr(nodes, 'DecimationVertical'):
                    nodes.DecimationVertical.value = 2
                
                # Phục hồi vùng nhìn rộng nhất (Full FOV)
                if hasattr(nodes, 'Width'): 
                    nodes.Width.value = nodes.Width.max
                if hasattr(nodes, 'Height'): 
                    nodes.Height.value = nodes.Height.max
                
                # 3. Thiết lập FPS mục tiêu
                if hasattr(nodes, 'AcquisitionFrameRateEnable'):
                    nodes.AcquisitionFrameRateEnable.value = True
                if hasattr(nodes, 'AcquisitionFrameRate'):
                    # Sau khi Decimation=2, max FPS sẽ tự động tăng lên > 120
                    nodes.AcquisitionFrameRate.value = min(float(self.config['fps']), nodes.AcquisitionFrameRate.max)
                
                # 4. Thiết lập Gain
                if hasattr(nodes, 'Gain'): nodes.Gain.value = float(self.config['gain'])

                # 5. Tắt giới hạn băng thông để đạt FPS tối đa
                if hasattr(nodes, 'DeviceLinkThroughputLimitMode'):
                    nodes.DeviceLinkThroughputLimitMode.value = 'Off'
                
                # 6. Đảm bảo chỉ lấy frame mới nhất nếu có backlog
                if hasattr(nodes, 'StreamBufferHandlingMode'):
                    nodes.StreamBufferHandlingMode.value = 'NewestOnly'
                
                actual_w = nodes.Width.value if hasattr(nodes, 'Width') else 0
                actual_h = nodes.Height.value if hasattr(nodes, 'Height') else 0
                actual_fps = nodes.AcquisitionFrameRate.value if hasattr(nodes, 'AcquisitionFrameRate') else 0
                print(f"[CAM] Kết quả: {actual_w}x{actual_h} @ {actual_fps} FPS (Full FOV)")

        except Exception as e:
            print(f"[CAM] Lỗi khi cài đặt thông số phần cứng: {e}")

    def set_mode(self, mode):
        """Hàm thay đổi chế độ hoạt động (RELEASE, ACTIVE)"""
        if mode == self.mode:
            return
        print(f"[CAM] Yêu cầu chuyển chế độ: {self.mode} -> {mode}")
        self.mode = mode
        self._apply_mode_params()

    def set_mm_per_pixel(self, ratio):
        """Cập nhật tỷ lệ mm/pixel"""
        self.config['mm_per_pixel'] = ratio

    def get_data(self, get_img=True):
        """
        Hàm chính để lấy dữ liệu đầu ra
        :param get_img: Nếu False, không trả về frame (None) để tiết kiệm thời gian xử lý hiển thị
        Trả về: (frame, info, fps, is_connected)
        """
        # Trường hợp RELEASE: Trả về ảnh đen và None
        if self.mode == self.MODE_RELEASE:
            if not get_img: return None, None, 0.0, False
            blank_frame = np.ones((self.config['height'], self.config['width'], 3), dtype=np.uint8) * 128
            cv2.putText(blank_frame, "MODE_RELEASE: OFF", (50, 100), 0, 3, (0,0,255), 5)
            return blank_frame, None, 0.0, False

        with self.lock:
            if not self.ret or self.frame is None:
                return None, None, 0.0, (self.ia is not None)
            raw_bayer = self.frame.copy()

        # TỐI ƯU: Chỉ convert màu nếu người dùng cần lấy ảnh (get_img=True)
        # Nếu chỉ cần tọa độ, ta dùng ảnh xám (Bayer chính là ảnh xám) để detect luôn.
        gray = raw_bayer
        frame = cv2.cvtColor(raw_bayer, cv2.COLOR_BayerBG2BGR) if get_img else None

        tag_info = []
        # Lấy kích thước từ ảnh thô để đảm bảo tính toán đúng ngay cả khi không lấy ảnh BGR
        h_img, w_img = raw_bayer.shape[:2]
        center_img_x = w_img / 2
        center_img_y = h_img / 2
        
        # Bắt đầu đo thời gian xử lý AprilTag
        t_start = time.perf_counter()

        ratio = self.config.get('mm_per_pixel', 1.0)
        
        # Sử dụng ROI để tăng tốc nếu có
        current_corners, current_ids = None, None
        if self.roi is not None:
            x, y, w, h = self.roi
            crop = gray[y:y+h, x:x+w]
            current_corners, current_ids, _ = self.detector.detectMarkers(crop)
            if current_ids is not None:
                current_corners = [c + [x, y] for c in current_corners] # bù tọa độ
        
        # Nếu ROI không có hoặc không tìm thấy, quét toàn khung hình
        if current_ids is None:
            current_corners, current_ids, _ = self.detector.detectMarkers(gray)

        t_end = time.perf_counter()
        # detection_time_ms = (t_end - t_start) * 1000

        if current_ids is not None:
            self.lost_count = 0
            for i in range(len(current_ids)):
                corners_single = current_corners[i].reshape(4, 2).astype(np.float32)
                
                # Tọa độ trung tâm của tag (pixel)
                center_tag_x = np.mean(corners_single[:, 0])
                center_tag_y = np.mean(corners_single[:, 1])

                # Tính độ lệch so với tâm ảnh (pixel)
                dx_pixel = center_tag_x - center_img_x
                dy_pixel = center_tag_y - center_img_y

                # Chuyển sang mm dựa trên tỷ lệ
                mm_x = dx_pixel * ratio
                mm_y = dy_pixel * ratio

                # Tính góc lệch (độ) dựa trên cạnh trên của mã (từ góc 0 đến góc 1)
                # p0: Top-Left, p1: Top-Right
                p0, p1 = corners_single[0], corners_single[1]
                angle = np.degrees(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))

                # Cấu trúc dữ liệu tiếng Việt không dấu
                tag_info.append({
                    "id": int(current_ids[i][0]),
                    "lech_trai_phai_mm_x": -round(float(mm_x), 1),    # Lệch ngang (mm)
                    "lech_truoc_sau_mm_y": round(float(mm_y), 1),    # Lệch dọc (mm)
                    "goc_lech_do": round(float(angle), 2),           # Góc lệch (độ)
                    "toa_do_2d": current_corners[i].astype(int).tolist() # pixel
                })
                if get_img:
                    cv2.putText(frame, str(int(current_ids[i][0])), ((i+1)*200, 200), 0, 3, (0,255,0), 10)
                    cv2.putText(frame, str(round(self.fps_real, 1)), ((i+1)*200, 400), 0, 3, (0,255,0), 10)
            
            # Cập nhật ROI dựa trên TẤT CẢ các tag tìm thấy (ưu tiên FPS)
            all_corners_pts = np.concatenate(current_corners, axis=0).reshape(-1, 2)
            x_min, y_min = np.min(all_corners_pts, axis=0)
            x_max, y_max = np.max(all_corners_pts, axis=0)
            
            x_n, y_n = int(x_min), int(y_min)
            w_n, h_n = int(x_max - x_min), int(y_max - y_min)
            
            margin = 100 
            self.roi = (max(0, x_n - margin), max(0, y_n - margin), 
                        min(w_img - x_n, w_n + 2 * margin), 
                        min(h_img - y_n, h_n + 2 * margin))
            
        else:
            self.lost_count += 1
            
            if self.lost_count > 10:
                self.roi = None

        return (frame if get_img else None), tag_info, round(self.fps_real, 1), (self.ia is not None)

    def stop(self):
        self.stopped = True
        if self.thread:
            self.thread.join()
        if self.ia:
            self.ia.stop()
            self.ia.destroy()
            self.ia = None
        if self.h:
            self.h.reset()




# --- VÍ DỤ SỬ DỤNG ---
if __name__ == "__main__":
    list_sentech_cameras()
    # # Khởi tạo với cấu hình tùy chỉnh
    my_cam = AGVCamera(src=0).start()
    try:
        threading.Thread(target=ket_noi_esp_loa.python_esp32).start()
    except OSError as e:
        print("error 44")
        pass

    t = time.time()
    show_video = True
    ket_noi_esp_loa.py_sent_esp("bat_tat_den#bat#0")
    while True:
        ket_noi_esp_loa.check_connect()
        t = time.time()
        # Gọi get_data với tham số show_video để linh hoạt bật tắt
        img, info, fps, status = my_cam.get_data(get_img=show_video)
        print(time.time() - t)
        
        if img is not None:
            # Resize hiển thị để không tràn màn hình laptop 1080p
            h, w = img.shape[:2] 
            display_w = 1000
            display_h = int(h * (display_w / w))
            # cv2.imwrite("test.jpg", img)
            cv2.imshow("AGV_CAM_MODULE", cv2.resize(img, (display_w, display_h)))
        else:
            img = np.ones((480, 640, 3), dtype=np.uint8) * 128
            cv2.imshow("AGV_CAM_MODULE", img)
        
        print(f"\r[Conn: {status}] [FPS: {fps}] Tags detected: {len(info) if info else 0} | Video: {'ON' if show_video else 'OFF'}", end="")

        key = cv2.waitKey(1)
        if key == ord('q'): break
        elif key == ord('0'): my_cam.set_mode(AGVCamera.MODE_RELEASE)
        elif key == ord('1'): my_cam.set_mode(AGVCamera.MODE_ACTIVE)
        elif key == ord('v'): show_video = not show_video # Bấm V để bật/tắt trả về ảnh
    ket_noi_esp_loa.py_sent_esp("bat_tat_den#tat#0")
    my_cam.stop()
    cv2.destroyAllWindows()