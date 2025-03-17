import torch
import cv2
import numpy as np
import torchvision.transforms as T
from PIL import Image

# Tải mô hình DeepLab v3+
model = torch.hub.load('pytorch/vision', 'deeplabv3_resnet101', pretrained=True)
model.eval()

# Chuyển đổi ảnh
def preprocess(image):
    transform = T.Compose([
        T.ToPILImage(),
        T.Resize((520, 520)),  # Thay đổi kích thước
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    return transform(image).unsqueeze(0)

# Mở camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Tiền xử lý ảnh
    input_tensor = preprocess(frame)
    
    # Dự đoán
    with torch.no_grad():
        output = model(input_tensor)['out'][0]
    
    # Lấy lớp bức tường (class 3 trong COCO)
    wall_class = 3  # class ID cho bức tường trong COCO
    output_predictions = output.argmax(0)

    # Tạo mặt nạ cho bức tường
    mask = output_predictions == wall_class
    mask = mask.cpu().numpy().astype(np.uint8) * 255

    # Rescale mặt nạ về kích thước của frame gốc
    mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
    
    # Tạo mặt nạ màu
    mask_colored = cv2.applyColorMap(mask_resized, cv2.COLORMAP_JET)

    # Hiển thị kết quả
    result = cv2.addWeighted(frame, 0.5, mask_colored, 0.5, 0)

    # Hiển thị frame
    cv2.imshow('Wall Detection', result)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng cửa sổ
cap.release()
cv2.destroyAllWindows()