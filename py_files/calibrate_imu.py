import serial
import serial.tools.list_ports
import time
import struct
import sys
import numpy as np
from scipy.optimize import least_squares

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "usbserial" in p.device or "SLAB_USBtoUART" in p.device or "CP210" in p.description or "CH340" in p.description:
            return p.device
    if len(ports) > 0:
        return ports[0].device
    return None

def fit_ellipsoid(data, target_norm):
    """
    Fits an ellipsoid to 3D data and returns the bias and scale vectors.
    Cost function minimizes the difference between the scaled/shifted radius and the target norm.
    """
    data = np.array(data)
    
    # Initial guess
    max_vals = np.max(data, axis=0)
    min_vals = np.min(data, axis=0)
    bias_guess = (max_vals + min_vals) / 2.0
    scale_guess = (max_vals - min_vals) / 2.0 / target_norm

    def cost_func(params, points, target):
        bx, by, bz, sx, sy, sz = params
        shifted = points - np.array([bx, by, bz])
        scaled = shifted / np.array([sx, sy, sz])
        norms = np.linalg.norm(scaled, axis=1)
        return norms - target

    initial_params = [bias_guess[0], bias_guess[1], bias_guess[2], scale_guess[0], scale_guess[1], scale_guess[2]]
    
    print("Fitting data... Please wait.")
    res = least_squares(cost_func, initial_params, args=(data, 1.0))
    
    bx, by, bz, sx, sy, sz = res.x
    return np.array([bx, by, bz]), np.array([sx, sy, sz])

def main():
    print("==================================================")
    print("      火箭 IMU 橢球校正程式 (Accel & Mag)        ")
    print("==================================================")
    print("注意：請確保你已經把火箭韌體 StorageCommManager.cpp 中的")
    print("      'Wired Telemetry Data' 註解拿掉，這樣才能抓到資料！\n")

    port_name = find_serial_port()
    if not port_name:
        print("❌ 找不到可用的序列埠，請確認 USB 有接好。")
        return

    print(f"🔌 連線到序列埠: {port_name} (Baudrate: 115200)")
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f"❌ 無法開啟序列埠: {e}")
        return

    print("\n👉 準備收集加速度計資料。")
    print("請拿著火箭，在空中**慢慢地**往各個方向旋轉，讓它經歷所有可能的姿態。")
    print("收集進度將顯示在下方...")
    time.sleep(3)

    accel_data = []
    target_samples = 500

    try:
        while len(accel_data) < target_samples:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if "Wired Telemetry Data:" in line:
                    try:
                        hex_str = line.split("Wired Telemetry Data:")[1].strip()
                        hex_list = hex_str.split()
                        byte_data = bytes([int(x, 16) for x in hex_list])
                        
                        if len(byte_data) >= 43 and byte_data[0] == 0xAA and byte_data[1] == 0xBB:
                            TYPE_ID = byte_data[2]
                            if TYPE_ID == 21:  # TYPE_IMU_DATA
                                floats = struct.unpack('<9f', byte_data[7:43])
                                ax, ay, az = floats[0:3]
                                
                                # 只保留變化較大的樣本以確保分佈均勻
                                if len(accel_data) == 0 or np.linalg.norm(np.array([ax, ay, az]) - accel_data[-1]) > 0.5:
                                    accel_data.append([ax, ay, az])
                                    sys.stdout.write(f"\r收集進度: [{len(accel_data)}/{target_samples}]")
                                    sys.stdout.flush()
                    except Exception as e:
                        pass
    except KeyboardInterrupt:
        print("\n⚠️ 使用者中斷收集。")
        ser.close()
        return

    ser.close()
    print("\n\n✅ 資料收集完成，開始計算校正矩陣...")

    accel_bias, accel_scale = fit_ellipsoid(accel_data, 9.80665)

    print("\n================= 校正結果 =================")
    print("加速度計 零偏誤差 (Bias):")
    print(f"X: {accel_bias[0]:.5f}")
    print(f"Y: {accel_bias[1]:.5f}")
    print(f"Z: {accel_bias[2]:.5f}")
    print("\n加速度計 比例誤差 (Scale Factor) [應接近 9.80665]:")
    print(f"X: {accel_scale[0]:.5f}")
    print(f"Y: {accel_scale[1]:.5f}")
    print(f"Z: {accel_scale[2]:.5f}")
    print("============================================")

    print("\n你可以在 KalmanFilter.cpp 中將 ax, ay, az 的校正改為：")
    print(f"ax = (ax - ({accel_bias[0]:.5f})) / ({accel_scale[0]/9.80665:.5f});")
    print(f"ay = (ay - ({accel_bias[1]:.5f})) / ({accel_scale[1]/9.80665:.5f});")
    print(f"az = (az - ({accel_bias[2]:.5f})) / ({accel_scale[2]/9.80665:.5f});")
    print("============================================")

if __name__ == "__main__":
    main()
