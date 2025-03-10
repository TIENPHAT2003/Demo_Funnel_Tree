from djitellopy import Tello
import time
import threading

TELLO_IP = "192.168.137.28"

class PID:
    def __init__(self, Kp, Ki, Kd, sample_time, output_limits=(-200, 200)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time / 1000.0  # Chuyển đổi sang giây
        self.out_min, self.out_max = output_limits
        self.clear()
    
    def clear(self):
        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        self.last_error = 0
        self.output = 0
    
    def compute(self, error):
        self.PTerm = self.Kp * error
        self.ITerm += self.Ki * error * self.sample_time
        self.ITerm = max(min(self.ITerm, self.out_max), self.out_min)
        self.DTerm = self.Kd * (error - self.last_error) / self.sample_time
        self.last_error = error
        
        self.output = self.PTerm + self.ITerm + self.DTerm
        self.output = max(min(self.output, self.out_max), self.out_min)
        return self.output

def pid_control_loop(pid_outer, pid_inner, get_current_value, target, output_dict, key):
    while True:
        error = target - get_current_value()
        outer_control = pid_outer.compute(error)
        output_dict[key] = int(pid_inner.compute(outer_control))
        time.sleep(pid_outer.sample_time)

class DroneController:
    MAX_DISTANCE = 100
    
    def __init__(self, tello):
        self.tello = tello
        self.is_flying = False
        self.current_position = {"x": 0, "y": 0, "z": 0}
        self.current_yaw = 0
        self.control_outputs = {"yaw": 0, "pitch": 0, "roll": 0, "vx": 0, "vy": 0, "vz": 0}
        
        self.pid_configs = {
            "yaw": (PID(5, 5, 0, 100), PID(0.5, 0.005, 0, 100), self.tello.get_yaw, 0),
            "roll": (PID(3, 1, 0, 100), PID(0.5, 0.005, 0, 100), self.tello.get_roll, 0),
            "pitch": (PID(3, 1, 0, 100), PID(0.5, 0.005, 0, 100), self.tello.get_pitch, 0),
        }
        
        self.velocity_pid_configs = {
            "vx": (PID(10, 30, 0, 100), PID(1, 0.05, 0, 100), self.tello.get_speed_x, 0),
            "vy": (PID(10, 30, 0, 100), PID(1, 0.05, 0, 100), self.tello.get_speed_y, 0),
            "vz": (PID(10, 30, 0, 100), PID(1, 0.05, 0, 100), self.tello.get_speed_z, 0)
        }
        
        self.start_pid_threads(self.pid_configs)
    def move_expand(self, distance, direction):
        if distance <= 0:  # Bỏ qua nếu khoảng cách là 0
            return

        if distance > self.MAX_DISTANCE:
            print(f"[WARNING] Khoảng cách quá lớn: {distance} mm. Giới hạn: {self.MAX_DISTANCE} mm.")
            distance = self.MAX_DISTANCE
        
        print(f"[INFO] Di chuyển {direction} {distance} mm")
        
        if direction == "forward":
            self.tello.move_forward(distance)
        elif direction == "back":
            self.tello.move_back(distance)
        elif direction == "right":
            self.tello.move_right(distance)
        elif direction == "left":
            self.tello.move_left(distance)
        elif direction == "up":
            self.tello.move_up(distance)
        elif direction == "down":
            self.tello.move_down(distance)


    def start_pid_threads(self, pid_configs):
        for key, (pid_outer, pid_inner, get_current_value, target) in pid_configs.items():
            threading.Thread(target=pid_control_loop, args=(pid_outer, pid_inner, get_current_value, target, self.control_outputs, key), daemon=True).start()
    
    def move_to_xyz(self, x, y, z, yaw):
        print(f"[INFO] Di chuyển đến tọa độ: X={x}, Y={y}, Z={z}, Yaw={yaw}")
        self.move_expand(abs(x - self.current_position['x']), 'forward' if x > self.current_position['x'] else 'back')
        self.move_expand(abs(y - self.current_position['y']), 'right' if y > self.current_position['y'] else 'left')
        self.move_expand(abs(z - self.current_position['z']), 'up' if z > self.current_position['z'] else 'down')
        self.current_yaw = yaw
        self.tello.rotate_clockwise(yaw)
    
    def move_to_waypoints(self, waypoints):
        for waypoint in waypoints:
            x, y, z, yaw = waypoint
            self.move_to_xyz(x, y, z, yaw)
        
        print("[INFO] Drone giữ nguyên vị trí tại điểm cuối cùng. Nhấn Ctrl+C để thoát.")
        self.start_pid_threads(self.velocity_pid_configs)
        
        try:
            while True:
                print(f"📌 Pitch: {self.tello.get_pitch()}° (Control: {self.control_outputs['pitch']}) | "
                      f"Roll: {self.tello.get_roll()}° (Control: {self.control_outputs['roll']}) | "
                      f"Yaw: {self.tello.get_yaw()}° (Control: {self.control_outputs['yaw']})")
                print(f"💨 Speed -> Vgx: {self.tello.get_speed_x()} mm/s (Control: {self.control_outputs['vx']}) | "
                      f"Vgy: {self.tello.get_speed_y()} mm/s (Control: {self.control_outputs['vy']}) | "
                      f"Vgz: {self.tello.get_speed_z()} mm/s (Control: {self.control_outputs['vz']})")
                print(f"⚡ Acceleration -> Agx: {self.tello.get_acceleration_x()} m/s² | "
                      f"Agy: {self.tello.get_acceleration_y()} m/s² | "
                      f"Agz: {self.tello.get_acceleration_z()} m/s²")
                print(f"📏 ToF Height: {self.tello.get_distance_tof()} mm | 📡 Takeoff Height: {self.tello.get_height()} cm")
                print("=" * 50)
                time.sleep(1)
        except KeyboardInterrupt:
            print("[INFO] Kết thúc giữ vị trí, drone hạ cánh.")
            self.tello.land()

if __name__ == "__main__":
    tello = Tello(host=TELLO_IP)
    tello.connect()
    
    print(f"\n🚀 Kết nối thành công! Pin còn lại: {tello.get_battery()}%\n")
    
    drone = DroneController(tello)
    drone.is_flying = True
    tello.takeoff()
    print("🛫 Drone đã cất cánh!")
    
    waypoints = [(100, 0, 50, 90)]
    drone.move_to_waypoints(waypoints)
