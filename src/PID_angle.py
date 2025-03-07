from djitellopy import Tello
import time

# Địa chỉ IP của Tello (Cập nhật nếu cần)
TELLO_IP = "192.168.137.35"

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

def main():
    tello = Tello(host=TELLO_IP)
    tello.connect()

    print(f"\n🚀 Kết nối thành công! Pin còn lại: {tello.get_battery()}%\n")

    # PID cho Yaw
    outer_pid_yaw = PID(Kp=3, Ki=0.5, Kd=0, sample_time=100)
    inner_pid_yaw = PID(Kp=0.5, Ki=0.005, Kd=0, sample_time=100)
    
    # PID cho Roll
    outer_pid_roll = PID(Kp=3, Ki=0.5, Kd=0, sample_time=100)
    inner_pid_roll = PID(Kp=0.5, Ki=0.005, Kd=0, sample_time=100)
    
    # PID cho Pitch
    outer_pid_pitch = PID(Kp=3, Ki=0.5, Kd=0, sample_time=100)
    inner_pid_pitch = PID(Kp=0.5, Ki=0.005, Kd=0, sample_time=100)
    
    # Góc mong muốn
    target_yaw = 0  
    target_roll = 0  
    target_pitch = 0  

    tello.takeoff()
    print("🛫 Drone đã cất cánh!")

    try:
        while True:
            yaw = tello.get_yaw()
            roll = tello.get_roll()
            pitch = tello.get_pitch()

            # Tính toán PID cho Yaw
            error_yaw = target_yaw - yaw
            outer_control_yaw = outer_pid_yaw.compute(error_yaw)
            yaw_rate_control = int(inner_pid_yaw.compute(outer_control_yaw))

            # Tính toán PID cho Roll
            error_roll = target_roll - roll
            outer_control_roll = outer_pid_roll.compute(error_roll)
            roll_rate_control = int(inner_pid_roll.compute(outer_control_roll))

            # Tính toán PID cho Pitch
            error_pitch = target_pitch - pitch
            outer_control_pitch = outer_pid_pitch.compute(error_pitch)
            pitch_rate_control = int(inner_pid_pitch.compute(outer_control_pitch))

            # Điều khiển drone
            tello.send_rc_control(roll_rate_control, pitch_rate_control, 0, yaw_rate_control)
            
            # Đọc dữ liệu
            vgx = tello.get_speed_x()
            vgy = tello.get_speed_y()
            vgz = tello.get_speed_z()
            tof_distance = tello.get_distance_tof()
            height = tello.get_height()
            baro_height = tello.get_barometer()
            
            print("=" * 50)
            print(f"📌 Pitch: {pitch}° (Control: {pitch_rate_control}) | Roll: {roll}° (Control: {roll_rate_control}) | Yaw: {yaw}° (Control: {yaw_rate_control})")
            print(f"💨 Speed -> Vgx: {vgx} mm/s | Vgy: {vgy} mm/s | Vgz: {vgz} mm/s")
            print(f"📏 ToF Height: {tof_distance} mm")
            print(f"📡 Takeoff Height: {height} cm")
            print(f"🛰️ Barometer Height: {baro_height} cm")
            print("=" * 50)
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình... Hạ cánh!")
        tello.land()
        print("🛬 Drone đã hạ cánh.")
        tello.end()

if __name__ == "__main__":
    main()
