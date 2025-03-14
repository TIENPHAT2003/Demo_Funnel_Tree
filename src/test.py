from djitellopy import Tello
import time
import threading
import sys
import math
TELLO_IP = "192.168.137.108"

class PID:
    def __init__(self, Kp, Ki, Kd, sample_time, output_limits=(-200, 200)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time / 1000.0
        self.out_min, self.out_max = output_limits
        self.clear()

    def clear(self):
        self.PTerm = self.ITerm = self.DTerm = self.last_error = self.output = 0

    def compute(self, error):
        self.PTerm = self.Kp * error
        self.ITerm = max(min(self.ITerm + self.Ki * error * self.sample_time, self.out_max), self.out_min)
        self.DTerm = self.Kd * (error - self.last_error) / self.sample_time
        self.last_error = error
        self.output = max(min(self.PTerm + self.ITerm + self.DTerm, self.out_max), self.out_min)
        return self.output

def pid_control_loop(pid_outer, pid_inner, get_current_value, target, output_dict, key):
    while True:
        error = target() - get_current_value()
        outer_control = pid_outer.compute(error)
        output_dict[key] = int(pid_inner.compute(outer_control))
        time.sleep(pid_outer.sample_time)

class DroneController:
    MAX_DISTANCE = 500
    DEFAULT_SPEED = 30  # cm/s

    def __init__(self, tello):
        self.tello = tello
        self.is_flying = False
        self.current_position = {"x": 0, "y": 0, "z": 0}
        self.current_yaw = 0
        self.control_outputs = {"yaw": 0, "pitch": 0, "roll": 0, "vx": 0, "vy": 0, "vz": 0}

        self.pid_configs = {
            # "yaw": (PID(5, 5, 0, 100), PID(0.3, 0.005, 0, 100), self.tello.get_yaw, 0),
            # "roll": (PID(3, 1, 0, 100), PID(0.3, 0.005, 0, 100), self.tello.get_roll, 0),
            # "pitch": (PID(3, 1, 0, 100), PID(0.3, 0.005, 0, 100), self.tello.get_pitch, 0),
            "vx": (PID(10, 50, 0, 100), PID(1.5, 0.5, 0, 100), self.tello.get_speed_x, self.DEFAULT_SPEED),
            "vy": (PID(10, 50, 0, 100), PID(1.5, 0.5, 0, 100), self.tello.get_speed_y, self.DEFAULT_SPEED),
            "vz": (PID(10, 50, 0, 100), PID(1.5, 0.5, 0, 100), self.tello.get_speed_z, self.DEFAULT_SPEED),
        }

    def adaptive_speed(self, traveled_distance, target_distance):
        if traveled_distance >= target_distance * 0.5:  
            return max(self.DEFAULT_SPEED * 0.3, 10)
        return self.DEFAULT_SPEED

    def brake(self):
        # Lấy vận tốc hiện tại
        vx = self.tello.get_speed_x() * -2  # Đảo ngược vận tốc X
        vy = self.tello.get_speed_y() * -2  # Đảo ngược vận tốc Y
        vz = self.tello.get_speed_z() * -2  # Đảo ngược vận tốc Z
        
        print(f"🛑 Đang hãm tốc với vận tốc ngược: Vx={vx}, Vy={vy}, Vz={vz}")

        # Gửi lệnh vận tốc ngược lại
        self.tello.send_rc_control(vy, vx, vz, 0)
        time.sleep(1.5)  

        # Dừng drone hoàn toàn
        self.tello.send_rc_control(0, 0, 0, 0)
        print("✅ Drone đã dừng hẳn.")

    def move_axis(self, distance, direction):
        if distance == 0:
            return

        if distance > self.MAX_DISTANCE:
            print(f"[WARNING] Khoảng cách quá lớn: {distance} cm. Giới hạn: {self.MAX_DISTANCE} cm.")
            distance = self.MAX_DISTANCE

        print(f"[INFO] Di chuyển {direction} {distance} cm với tốc độ {self.DEFAULT_SPEED} cm/s")

        traveled_distance = 0
        start_time = time.time()
        last_time = start_time

        while traveled_distance < distance:
            if self.tello.get_battery() < 20:
                print("⚠️ Pin dưới 20%! Hạ cánh khẩn cấp!")
                self.tello.land()
                sys.exit(1)

            current_time = time.time()
            delta_time = current_time - last_time
            last_time = current_time

            speed = self.adaptive_speed(traveled_distance, distance)

            if direction == "forward":
                self.tello.send_rc_control(0, speed, 0, 0)
                velocity = self.tello.get_speed_x() * 10 
            elif direction == "back":
                self.tello.send_rc_control(0, -speed, 0, 0)
                velocity = -self.tello.get_speed_x() * 10
            elif direction == "right":
                self.tello.send_rc_control(speed, 0, 0, 0)
                velocity = self.tello.get_speed_y() * 10
            elif direction == "left":
                self.tello.send_rc_control(-speed, 0, 0, 0)
                velocity = -self.tello.get_speed_y() * 10
            elif direction == "up":
                self.tello.send_rc_control(0, 0, speed, 0)
                velocity = self.tello.get_speed_z() * 10
            elif direction == "down":
                self.tello.send_rc_control(0, 0, -speed, 0)
                velocity = -self.tello.get_speed_z() * 10

            traveled_distance += (abs(velocity) + 10) * delta_time
            sys.stdout.write(f"\r📌 Pitch: {self.tello.get_pitch()}° | Roll: {self.tello.get_roll()}° | Yaw: {self.tello.get_yaw()}° | "
                 f"💨 Speed -> Vx: {self.tello.get_speed_x()} dm/s | Vy: {self.tello.get_speed_y()} dm/s | Vz: {self.tello.get_speed_z()} dm/s | "
                 f"📏 Traveled Distance: {traveled_distance:.2f} cm / {distance} cm | "
                 f"ToF Height: {self.tello.get_distance_tof()} mm | Takeoff Height: {self.tello.get_height()} cm | 🔋 Battery: {self.tello.get_battery()}%\n")
            sys.stdout.flush()
            time.sleep(0.1)
        self.brake() 
        time.sleep(2)

    def start_pid_threads(self, pid_configs):
        for key, (pid_outer, pid_inner, get_current_value, target) in pid_configs.items():
            threading.Thread(target=pid_control_loop, args=(pid_outer, pid_inner, get_current_value, lambda: target, self.control_outputs, key), daemon=True).start()

    def move_to_xyz(self, x, y, z, yaw):
        print(f"[INFO] Di chuyển đến tọa độ: X={x}, Y={y}, Z={z}, Yaw={yaw}")
        self.move_axis(abs(x - self.current_position['x']), 'forward' if x > self.current_position['x'] else 'back')
        self.move_axis(abs(y - self.current_position['y']), 'right' if y > self.current_position['y'] else 'left')
        self.move_axis(abs(z - self.current_position['z']), 'up' if z > self.current_position['z'] else 'down')
        self.current_yaw = yaw
        self.tello.rotate_clockwise(yaw)

    def move_to_waypoints(self, waypoints):
        self.start_pid_threads(self.pid_configs)
        for waypoint in waypoints:
            x, y, z, yaw = waypoint
            self.move_to_xyz(x, y, z, yaw)
            time.sleep(2)
        print("[INFO] Drone giữ nguyên vị trí tại điểm cuối cùng.")

if __name__ == "__main__":
    tello = Tello(host=TELLO_IP)
    tello.connect()
    battery_level = tello.get_battery()
    print(f"\n🚀 Kết nối thành công! Pin còn lại: {battery_level}%\n")

    if battery_level < 20:
        print("⚠️ Cảnh báo: Pin dưới 20%, không thể bay!")
        sys.exit(1)

    drone = DroneController(tello)
    try:
        tello.takeoff()
        time.sleep(1)
        print("🛫 Drone đã cất cánh!")
        
        waypoints = [(120, 0, 0, 180), (120, 0, 0, 0)]
        drone.move_to_waypoints(waypoints)
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình... Hạ cánh!")
        tello.land()
        print("🛬 Drone đã hạ cánh.")
    except Exception as e:
        print(f"[ERROR] Có lỗi xảy ra: {e}")
        tello.land()
    finally:
        tello.end()
