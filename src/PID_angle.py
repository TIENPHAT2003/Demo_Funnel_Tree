from djitellopy import Tello
import time
import threading

TELLO_IP = "192.168.137.233"

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

def main():
    tello = Tello(host=TELLO_IP)
    tello.connect()

    print(f"\n🚀 Kết nối thành công! Pin còn lại: {tello.get_battery()}%\n")

    # Tạo các PID
    pid_configs = {
        "yaw": (PID(5, 5, 0, 100), PID(0.5, 0.005, 0, 100), tello.get_yaw, 0),
        "roll": (PID(3, 1, 0, 100), PID(0.5, 0.005, 0, 100), tello.get_roll, 0),
        "pitch": (PID(3, 1, 0, 100), PID(0.5, 0.005, 0, 100), tello.get_pitch, 0),
        "vx": (PID(10, 3, 0, 100), PID(1, 0.05, 0, 100), tello.get_speed_x, 0),
        "vy": (PID(10, 3, 0, 100), PID(1, 0.05, 0, 100), tello.get_speed_y, 0)
    }

    control_outputs = {key: 0 for key in pid_configs}
    
    # Khởi động luồng PID
    threads = []
    for key, (outer_pid, inner_pid, sensor_func, target) in pid_configs.items():
        thread = threading.Thread(target=pid_control_loop, args=(outer_pid, inner_pid, sensor_func, target, control_outputs, key))
        thread.daemon = True
        thread.start()
        threads.append(thread)

    tello.takeoff()
    print("🛫 Drone đã cất cánh!")
    
    try:
        while True:
            tello.send_rc_control(control_outputs["roll"], control_outputs["vx"], 0, control_outputs["yaw"])
            print(f"📌 Pitch: {tello.get_pitch()}° (Control: {control_outputs['pitch']}) | "
                  f"Roll: {tello.get_roll()}° (Control: {control_outputs['roll']}) | "
                  f"Yaw: {tello.get_yaw()}° (Control: {control_outputs['yaw']})")
            print(f"💨 Speed -> Vgx: {tello.get_speed_x()} mm/s (Control: {control_outputs['vx']}) | "
                  f"Vgy: {tello.get_speed_y()} mm/s (Control: {control_outputs['vy']})")
            print(f"⚡ Acceleration -> Agx: {tello.get_acceleration_x()} m/s² | "
                  f"Agy: {tello.get_acceleration_y()} m/s² | "
                  f"Agz: {tello.get_acceleration_z()} m/s²")
            print(f"📏 ToF Height: {tello.get_distance_tof()} mm | 📡 Takeoff Height: {tello.get_height()} cm")
            print("=" * 50)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình... Hạ cánh!")
        tello.land()
        print("🛬 Drone đã hạ cánh.")
        tello.end()

if __name__ == "__main__":
    main()
