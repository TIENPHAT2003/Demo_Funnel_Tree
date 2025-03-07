from djitellopy import Tello
import time

class CustomTello(Tello):
    def __init__(self, ip=None):
        super().__init__(host=ip) if ip else super().__init__()
        self.MAX_DISTANCE = 500
        self.current_position = {"x": 0, "y": 0, "z": 0}  # Vị trí ban đầu
        self.current_yaw = 0  # Góc quay hiện tại

    def reset_position(self):
        """Đặt lại (x, y) = (0,0), đặt z = 100cm khi cất cánh"""
        self.current_position["x"] = 0
        self.current_position["y"] = 0
        self.current_position["z"] = 100  # Đặt độ cao 100cm
        self.current_yaw = 0  # Reset góc quay về 0
        print(f"[INFO] Reset vị trí ban đầu: {self.current_position}")

    def move_expand(self, distance, direction):
        """Di chuyển drone theo hướng với giới hạn khoảng cách tối đa"""
        remaining_distance = distance
        while self.is_flying and remaining_distance > 0:
            move_distance = min(remaining_distance, self.MAX_DISTANCE)
            print(f"[INFO] Di chuyển {move_distance} cm về hướng {direction}")

            try:
                match direction.lower():
                    case 'forward': self.move_forward(move_distance)
                    case 'back': self.move_back(move_distance)
                    case 'left': self.move_left(move_distance)
                    case 'right': self.move_right(move_distance)
                    case 'up': self.move_up(move_distance)
                    case 'down': self.move_down(move_distance)
                    case _:
                        print(f"[ERROR] Hướng không hợp lệ: {direction}")
                        return False

            except Exception as e:
                print(f"[ERROR] Lỗi khi di chuyển: {e}")
                return False

            remaining_distance -= move_distance
            time.sleep(1)

            # Cập nhật vị trí
            self.update_position(direction, move_distance)

        return True

    def update_position(self, direction, distance):
        """Cập nhật tọa độ (x, y, z)"""
        match direction.lower():
            case 'forward': self.current_position["x"] += distance
            case 'back': self.current_position["x"] -= distance
            case 'left': self.current_position["y"] -= distance
            case 'right': self.current_position["y"] += distance
            case 'up': self.current_position["z"] += distance
            case 'down': self.current_position["z"] -= distance

        print(f"[INFO] Vị trí mới: {self.current_position}")

    def rotate_drone(self, target_yaw):
        """Xoay drone về góc target_yaw."""
        angle_diff = target_yaw - self.current_yaw
        print(f"[INFO] Xoay drone {angle_diff}° sau khi đến điểm")

        if angle_diff > 0:
            self.rotate_clockwise(angle_diff)
        elif angle_diff < 0:
            self.rotate_counter_clockwise(abs(angle_diff))

        self.current_yaw = target_yaw  # Cập nhật yaw mới sau khi xoay

    def move_to_xyz(self, x, y, z, yaw):
        """Di chuyển drone đến tọa độ (x, y, z), sau đó xoay theo yaw"""
        print(f"[INFO] Đang di chuyển đến tọa độ ({x}, {y}, {z})")

        move_x = x - self.current_position["x"]
        move_y = y - self.current_position["y"]
        move_z = z - self.current_position["z"]

        if move_x > 0:
            self.move_expand(move_x, "forward")
        elif move_x < 0:
            self.move_expand(abs(move_x), "back")

        if move_y > 0:
            self.move_expand(move_y, "right")
        elif move_y < 0:
            self.move_expand(abs(move_y), "left")

        if move_z > 0:
            self.move_expand(move_z, "up")
        elif move_z < 0:
            self.move_expand(abs(move_z), "down")

        self.current_position["x"] = x
        self.current_position["y"] = y
        self.current_position["z"] = z
        print(f"[INFO] Đã đến tọa độ ({x}, {y}, {z})")

        # Xoay sau khi di chuyển xong
        self.rotate_drone(yaw)

def main(drone_ip, waypoints):
    drone = CustomTello(drone_ip)

    try:
        # Kết nối với drone
        drone.connect()
        print(f"[INFO] Kết nối thành công tới drone tại {drone_ip}")
        
        # Cất cánh
        print("[INFO] Drone cất cánh...")
        drone.takeoff()
        time.sleep(2)

        # Bay lên 100cm
        drone.move_expand(50, "up")
        time.sleep(2)

        # Reset tọa độ ban đầu
        drone.reset_position()

        # Di chuyển qua từng điểm trong danh sách
        for idx, (x, y, z, yaw) in enumerate(waypoints):
            print(f"[INFO] Đến điểm {idx + 1}: ({x}, {y}, {z}), Sau đó xoay {yaw}°")
            drone.move_to_xyz(x, y, z, yaw)
            time.sleep(2)

        # Hạ cánh
        print("[INFO] Hạ cánh...")
        drone.land()
        time.sleep(2)

    except KeyboardInterrupt:
        print("\n[INFO] Nhận tín hiệu ngắt (Ctrl + C), hạ cánh drone...")
        drone.land()

    except Exception as e:
        print(f"[ERROR] Có lỗi xảy ra: {e}")
        drone.land()

    finally:
        drone.end()

if __name__ == "__main__":
    drone_ip = "192.168.137.28"

    # Nhập tọa độ (x, y, z, yaw) - Xoay sau khi đến điểm
    waypoints = [
        (300, 0, 200, 45),    
        (480, 0, 100, -180),   
        (480+120, 0, 150, -45), 
        (480+120+180, 0, 100, 90), 
        (480+120+180+120, 0, 180, 90+135), 
        (480+120+180+120+220, 0, 130, 90+90), 
        (480+120+180+120+220+300, 0, 200, 0), 
        # (480+120+180+120+180, 0, 100, 90+90), 
        # (620+420, 0, 50, 0), 
        # (1140, 0, 100, 0), 
    ]

    main(drone_ip, waypoints)
