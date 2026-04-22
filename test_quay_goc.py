import math
import time

class AGVRotationLogic:
    def __init__(self, wheel_dist, wheel_diam, kp=0.2, max_v_mm=800, min_v_mm=50, accel_limit=300):
        # Thông số cơ khí
        self.wheel_dist = wheel_dist
        self.wheel_diam = wheel_diam
        
        # Thông số điều khiển
        self.kp = kp                # Hệ số tỉ lệ
        self.max_v_mm = max_v_mm    # Tốc độ dài tối đa (mm/s)
        self.min_v_mm = min_v_mm    # Tốc độ dài tối thiểu để thắng ma sát (mm/s)
        self.accel_limit = accel_limit  # Gia tốc tối đa (mm/s^2)
        
        self.last_time = time.time()

    def calculate_v_out(self, target_angle, current_angle, current_v_feedback):
        """
        target_angle: Góc đích (độ)
        current_angle: Góc hiện tại (độ)
        current_v_feedback: Tốc độ hiện tại từ encoder/sensor (mm/s - luôn dương)
        """
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: dt = 0.02 # Tránh chia cho 0
        self.last_time = now

        # 1. Tính sai số góc (luôn lấy trị tuyệt đối vì bạn chỉ cần 1 tốc độ dương)
        error = abs(target_angle - current_angle)
        if error > 180: error = 360 - error # Lấy cung ngắn nhất

        # 2. Kiểm tra điều kiện dừng
        if error < 0.5:
            return 0.0

        # 3. Tính tốc độ mục tiêu dựa trên sai số (Hàm căn bậc hai)
        # Ở đây ta tính ra vận tốc góc robot rad/s trước rồi đổi sang mm/s
        w_robot = self.kp * math.sqrt(error)
        v_target = w_robot * (self.wheel_dist / 2)

        # 4. Giới hạn tốc độ trong khoảng [min, max]
        v_target = max(self.min_v_mm, min(v_target, self.max_v_mm))

        # 5. Kiểm soát gia tốc dựa trên current_v_feedback
        # Không cho phép v_out lệch quá nhiều so với tốc độ thực tế hiện tại
        v_diff = v_target - current_v_feedback
        max_allowed_diff = self.accel_limit * dt

        if abs(v_diff) > max_allowed_diff:
            if v_diff > 0:
                v_out = current_v_feedback + max_allowed_diff
            else:
                v_out = current_v_feedback - max_allowed_diff
        else:
            v_out = v_target

        return round(v_out, 2)

# --- MÔ PHỎNG CHẠY THỬ ---
# Khoảng cách 2 bánh 500mm, Đường kính 150mm
# Gia tốc giới hạn 200 mm/s^2
agv = AGVRotationLogic(wheel_dist=500, wheel_diam=150, accel_limit=200)

target = 90.0
current = 0.0
v_feedback = 0.0 # Ban đầu đứng im

print(f"{'Góc HT':>8} | {'Sai số':>8} | {'V_Feedback':>12} | {'V_OUT (mm/s)':>12}")
print("-" * 50)

v_out = agv.calculate_v_out(target, current, v_feedback)
print(v_out * 10)
# while True:
#     # Tính tốc độ đầu ra
#     v_out = agv.calculate_v_out(target, current, v_feedback)
    
#     print(f"{current:>8.2f}° | {abs(target-current):>8.2f}° | {v_feedback:>12.2f} | {v_out:>12.2f}")

#     if v_out == 0 and abs(target - current) < 0.5:
#         print("--- Đạt mục tiêu ---")
#         break

#     # Mô phỏng phản hồi từ hệ thống
#     # Trong thực tế, v_feedback sẽ lấy từ Encoder motor
#     v_feedback = v_out 
    
#     # Giả lập robot quay (đổi mm/s ngược lại độ/s để cập nhật góc mô phỏng)
#     w_robot_rad = (v_out / (500/2))
#     current += math.degrees(w_robot_rad) * 0.1 
    
#     time.sleep(0.1)