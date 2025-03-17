from ultralytics import YOLO
from djitellopy import Tello
import cv2
TELLO_IP = "192.168.137.108"
# Tải mô hình YOLOv8
model = YOLO('yolov8s.pt')  # Có thể thay bằng 'yolov8m.pt', 'yolov8l.pt', hoặc 'yolov8x.pt'

# Khởi tạo Tello
tello = Tello(host=TELLO_IP)
tello.connect()
tello.streamon()  # Bắt đầu stream video từ camera

while True:
    # Lấy frame từ camera Tello
    frame = tello.get_frame_read().frame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # Phát hiện đối tượng
    results = model(frame)

    # Hiển thị kết quả
    frame_with_boxes = results[0].plot()  # Vẽ hộp bao quanh các đối tượng

    # Hiển thị frame
    cv2.imshow('YOLOv8 Detection with Tello', frame_with_boxes)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng cửa sổ
tello.streamoff()  # Dừng stream
tello.end()  # Ngắt kết nối với drone
cv2.destroyAllWindows()