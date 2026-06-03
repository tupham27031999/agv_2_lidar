from harvesters.core import Harvester
import numpy as np
import cv2

# Khởi tạo Harvester
h = Harvester()

# Nạp file .cti của Sentech SDK đã tìm thấy
cti_path = '/opt/sentech/lib/GenICam/libstgentl.cti'
h.add_file(cti_path)
h.update()

# Kiểm tra xem đã nhận camera chưa
if not h.device_info_list:
    print("Không tìm thấy camera Sentech nào qua GenTL!")
else:
    print("Tìm thấy thiết bị:", h.device_info_list[0])

    # Mở thiết bị đầu tiên
    ia = h.create_image_acquirer(0)
    ia.start_acquisition()

    # Đọc frame
    try:
        with ia.fetch_buffer() as buffer:
            # Lấy dữ liệu buffer
            item = buffer.payload.components[0]
            
            # Xử lý kích thước ảnh
            width = item.width
            height = item.height
            data = item.data
            
            # Đọc thành Numpy Array (Giả định ảnh Mono hoặc Color tùy cấu hình)
            # Đối với ảnh Mono8:
            image = np.ndarray(buffer=data, dtype=np.uint8, shape=(height, width))
            
            # Chuyển đổi sang định dạng BGR để tương thích với cv2 (nếu là camera màu)
            # Hoặc sử dụng cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR) nếu cần giải mã Bayer
            cv2.imwrite('captured_frame.png', image)
            print("Đã chụp và lưu ảnh thành công!")

    finally:
        ia.stop_acquisition()
        ia.destroy()
        h.reset()