import cv2
import torch

# Tải mô hình YOLOv5 đã được huấn luyện
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Thay bằng 'yolov5m' hoặc 'yolov5l' nếu cần

# Mở camera
cap = cv2.VideoCapture(0)  # 0 là camera mặc định

while True:
    # Đọc frame từ camera
    ret, frame = cap.read()
    if not ret:
        break

    # Phát hiện đối tượng
    results = model(frame)

    # Hiển thị kết quả
    results.render()  # Vẽ bounding boxes lên frame

    # Hiển thị frame
    cv2.imshow('YOLOv5 Object Detection', frame)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng cửa sổ
cap.release()
cv2.destroyAllWindows()