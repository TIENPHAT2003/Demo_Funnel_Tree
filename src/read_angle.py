from djitellopy import Tello
import time

# Địa chỉ IP của Tello (Cập nhật nếu cần)
TELLO_IP = "192.168.137.119"

def main():
    # Kết nối với drone
    tello = Tello(host=TELLO_IP)
    tello.connect()

    print(f"\n🚀 Kết nối thành công! Pin còn lại: {tello.get_battery()}%\n")

    # Cất cánh
    # tello.takeoff()
    # print("🛫 Drone đã cất cánh!")

    try:
        while True:
            # Đọc giá trị Roll, Pitch, Yaw
            pitch = tello.get_pitch()
            roll = tello.get_roll()
            yaw = tello.get_yaw()

            # Đọc vận tốc theo trục X, Y, Z (mm/s)
            vgx = tello.get_speed_x()
            vgy = tello.get_speed_y()
            vgz = tello.get_speed_z()

            # Đọc gia tốc theo trục X, Y, Z (mm/s²)
            agx, agy, agz = tello.get_acceleration_x(), tello.get_acceleration_y(), tello.get_acceleration_z()

            # Đọc cảm biến ToF (mm)
            tof_distance = tello.get_distance_tof()

            # Đọc độ cao từ điểm cất cánh (cm)
            height = tello.get_height()

            # Đọc độ cao từ barometer (cm)
            baro_height = tello.get_barometer()

            # Hiển thị thông tin
            print("=" * 50)
            print(f"📌 Pitch: {pitch}° | Roll: {roll}° | Yaw: {yaw}°")
            print(f"💨 Speed -> Vgx: {vgx} mm/s | Vgy: {vgy} mm/s | Vgz: {vgz} mm/s")
            print(f"📊 Accel -> Agx: {agx} mm/s² | Agy: {agy} mm/s² | Agz: {agz} mm/s²")
            print(f"📏 ToF Height: {tof_distance} mm")
            print(f"📡 Takeoff Height: {height} cm")
            print(f"🛰️ Barometer Height: {baro_height} cm")
            print("=" * 50)

            time.sleep(0.5)  # Đọc dữ liệu mỗi 0.5 giây

    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình... Hạ cánh!")
        tello.land()  # Hạ cánh
        print("🛬 Drone đã hạ cánh.")
        tello.end()

if __name__ == "__main__":
    main()
