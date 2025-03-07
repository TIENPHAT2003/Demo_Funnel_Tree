from djitellopy import Tello
import time
import cv2
# from qrCodeScanner import scan_qr_code
import logging
import os

# Constants
BATTERY_THRESHOLD = 20
SLEEP_TIME = 1
MAX_DISTANCE = 500

def turnOnCamera(tello, should_capture=False):
    """
    Bật camera của drone
    Args:
        tello: đối tượng Tello
        should_capture: bool, có chụp ảnh hay không
    Returns:
        frame_read: đối tượng frame nếu thành công, None nếu thất bại
    """
    if should_capture:
        try:
            tello.streamoff()
            time.sleep(SLEEP_TIME)
            tello.streamon()
            print("[INFO] Video stream turned on.")
            time.sleep(SLEEP_TIME)
            return tello.get_frame_read()            
        except Exception as e:
            print(f"[ERROR] Lỗi khi chụp ảnh: {e}")
            return None
            
def capture_image(frame_read, th, input_folder):
    """
    Chụp ảnh từ frame hiện tại
    Args:
        frame_read: đối tượng frame
        th: số thứ tự ảnh
        input_folder: thư mục lưu ảnh
    Returns:
        frame: ma trận ảnh đã chụp
    """
    frame = frame_read.frame  
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image_path = os.path.join(input_folder, f"picture_{th}.png")
    cv2.imwrite(image_path, frame)          
    print(f"[INFO] Đã lưu: picture_{th}.png")
    return frame

class Coordinate:
    """Lớp quản lý tọa độ của drone"""
    def __init__(self, x=0, y=0, z=0):
        self.x = x  # forward/back
        self.y = y  # left/right
        self.z = z  # up/down
        
    def update(self, distance, direction):
        """
        Cập nhật tọa độ dựa trên hướng di chuyển
        Args:
            distance: khoảng cách di chuyển
            direction: hướng di chuyển
        """
        match direction.lower():
            case 'forward': self.x += distance
            case 'back': self.x -= distance
            case 'left': self.y -= distance
            case 'right': self.y += distance
            case 'up': self.z += distance
            case 'down': self.z -= distance
            
    def __str__(self):
        return f"(x: {self.x}, y: {self.y}, z: {self.z})"

class CustomTello(Tello):
    """Lớp mở rộng từ Tello với các chức năng bổ sung"""
    def __init__(self, ip=None):
        super().__init__() if ip is None else super().__init__(ip)
        self.MAX_DISTANCE = MAX_DISTANCE
        self.response = True
        self.is_calibrated = False

    def normalize_yaw(self, yaw):
        """
        Chuẩn hóa góc yaw về các góc chuẩn gần nhất
        Args:
            yaw: góc yaw hiện tại
        Returns:
            góc yaw chuẩn gần nhất
        """
        standard_angles = [0, 90, -90, 180, -180]
        return min(standard_angles, key=lambda x: abs(x - yaw))

    def adjust_imu(self, isTakeoff=False):
        """
        Điều chỉnh yaw về góc chuẩn gần nhất
        Args:
            isTakeoff: bool, có phải đang cất cánh không
        Returns:
            bool: True nếu thành công, False nếu thất bại
        """
        try:
            attitude = self.query_attitude()
            yaw = attitude['yaw']
            print(f"[INFO] Yaw hiện tại: {yaw}°")
            
            target_yaw = self.normalize_yaw(yaw) if not isTakeoff else 0
            adjustment = target_yaw - yaw
            
            if adjustment != 0:
                # Tối ưu hướng xoay
                if adjustment > 180: adjustment -= 360
                elif adjustment < -180: adjustment += 360
                    
                print(f"[INFO] Điều chỉnh yaw từ {yaw}° về {target_yaw}°")
                
                if adjustment > 0:
                    print(f"[INFO] Xoay theo chiều kim đồng hồ: {adjustment}°")
                    self.rotate_clockwise(adjustment)
                else:
                    print(f"[INFO] Xoay ngược chiều kim đồng hồ: {abs(adjustment)}°")
                    self.rotate_counter_clockwise(abs(adjustment))
                
                time.sleep(SLEEP_TIME)
            return True
            
        except Exception as e:
            print(f"[ERROR] Lỗi khi điều chỉnh yaw: {e}")
            return False

    def move_expand(self, distance, direction, position):
        """
        Di chuyển drone với kiểm soát và điều chỉnh hướng
        Args:
            distance: khoảng cách cần di chuyển
            direction: hướng di chuyển
            position: đối tượng Coordinate để theo dõi vị trí
        Returns:
            bool: True nếu thành công, False nếu thất bại
        """
        remaining_distance = distance
        
        while self.response and remaining_distance > 0:
            move_distance = min(remaining_distance, self.MAX_DISTANCE)
            print(f"[INFO] Di chuyển {move_distance}cm theo hướng {direction}")
            
            try:
                self.adjust_imu()
                
                # Thực hiện di chuyển
                match direction.lower():
                    case 'forward': self.response = self.move("forward", move_distance)
                    case 'back': self.response = self.move("back", move_distance)
                    case 'left': self.response = self.move("left", move_distance)
                    case 'right': self.response = self.move("right", move_distance)
                    case 'up': self.response = self.move("up", move_distance)
                    case 'down': self.response = self.move("down", move_distance)
                    case 'clockwise': self.response = self.rotate_clockwise(move_distance)
                    case 'counter_clockwise': self.response = self.rotate_counter_clockwise(move_distance)
                    case _:
                        print(f"[ERROR] Hướng không hợp lệ: {direction}")
                        return False

                time.sleep(SLEEP_TIME)
                if not self.response:
                    print(f"[ERROR] Lệnh di chuyển thất bại: {move_distance}, {direction}")
                    return False

                position.update(move_distance, direction)
                remaining_distance -= move_distance
                print(f"[INFO] Vị trí hiện tại: {position}")
                
                time.sleep(SLEEP_TIME)
                self.adjust_imu()
                
            except Exception as e:
                print(f"[ERROR] Lỗi khi di chuyển: {e}")
                return False
                
        return True

    def takeoff_with_check(self):
        """
        Cất cánh với kiểm tra an toàn
        Returns:
            bool: True nếu thành công, False nếu thất bại
        """
        try:
            battery = self.get_battery()
            if battery < BATTERY_THRESHOLD:
                print(f"[ERROR] Pin quá yếu để cất cánh: {battery}%")
                return False

            print("[INFO] Cất cánh...")
            super().takeoff()
            time.sleep(SLEEP_TIME)
            self.adjust_imu(isTakeoff=True)
            return True
            
        except Exception as e:
            print(f"[ERROR] Lỗi khi cất cánh: {e}")
            return False

def main():
    # Khởi tạo drone
    drone = CustomTello()
    
    try:
        # Kết nối với drone
        drone.connect()
        print("Pin còn lại:", drone.get_battery(), "%")
        drone.connect_to_wifi("TIENPHAT", "44444444")

        # Get the current TOF distance
        # drone.takeoff()
        # distance = drone.get_distance_tof()
        # print(f"Current TOF distance: {distance} cm")
        # drone.move_expand(50, "up")
        # distance = drone.get_distance_tof()
        # print(f"Current TOF distance: {distance} cm")
        # drone.move_expand(50, "down")
        # distance = drone.get_distance_tof()
        # print(f"Current TOF distance: {distance} cm")
        # frame_read = turnOnCamera(drone, True)
        # Cất cánh
        # drone.takeoff()
        # time.sleep(10)
        # frame = []
        
        # # # Di chuyển theo các hướng
        # drone.move_expand(100, "up")  # Di chuyển tiến 100cm
        # # time.sleep(1)
        # frame.append(capture_image(frame_read, '01'))
        
        # drone.move_expand(30, "forward")  # Bay lên 50cm
        # # time.sleep(1)
        # frame.append(capture_image(frame_read, '02'))
        # drone.move_expand(30, "back")  # Di chuyển lùi 100cm
        # # time.sleep(1)
        # frame.append(capture_image(frame_read, '03'))
        
        # print(frame)
        
    except Exception as e:
        print(f"Có lỗi xảy ra: {e}")
    
    finally:
        # Hạ cánh an toàn
        #drone.land()
        # Ngắt kết nối
        drone.end()

if __name__ == "__main__":
    main()
    