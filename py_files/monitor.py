import sys
import subprocess
import os
import datetime

# stdout 強制不緩衝，確保 daemon thread 的輸出即時可見
sys.stdout.reconfigure(line_buffering=True)

# 建立日誌資料夾 (使用絕對路徑避免 os.chdir 影響)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)
current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
JSON_LOG_FILE = os.path.join(LOG_DIR, f"telemetry_{current_time}.jsonl")
RAW_LOG_FILE  = os.path.join(LOG_DIR, f"raw_serial_{current_time}.log")

import serial
import serial.tools.list_ports
import threading
import struct
import json
import queue
import time
import math

serial_cmd_queue = queue.Queue()
BAUD_RATE = 115200

def choose_serial_port():
    import os
    if os.environ.get("SERIAL_PORT"):
        return os.environ.get("SERIAL_PORT")
        
    # 若有帶入命令列參數且不是 flags，優先使用
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
        return sys.argv[1]
        
    ports = serial.tools.list_ports.comports()
    if not ports:
        port = input("未偵測到任何 Serial Port，請手動輸入路徑 (例如 /dev/cu.usbserial-10): ").strip()
        return port if port else '/dev/cu.usbserial-10'
    
    print("\n可用的 Serial Ports:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
        
    choice = input(f"\n請選擇 Serial Port [0-{len(ports)-1}]，或直接手動輸入路徑 (直接 Enter 預設選 [0]): ").strip()
    if choice == "":
        return ports[0].device
    
    if choice.isdigit() and 0 <= int(choice) < len(ports):
        return ports[int(choice)].device
        
    return choice

_ser_global = None  # 由 main() 設定，供 mqtt_on_message 轉發指令至 Serial
current_board_freq = None  # 目前板子的頻率 (Hz)
desired_board_freq = None  # 使用者設定的目標頻率 (Hz)

# 連接埠硬體特徵追蹤
target_vid = None
target_pid = None
target_description = None
target_serial_number = None

def publish_status_update():
    try:
        payload_str = json.dumps({
            "type": "status",
            "board_freq": current_board_freq
        })
        mqtt_client.publish(TOPIC_TELEMETRY, payload_str)
    except Exception as e:
        print(f"⚠️ Status publish failed: {e}")

# STATENUM 合法範圍（對應 StateMachine.hpp enum）
_VALID_STATE_IDS = set(range(14))  # 0–13

def clean_float(val):
    if math.isnan(val) or math.isinf(val):
        return 0.0
    return val

import paho.mqtt.client as mqtt
import argparse

def load_env_mqtt_ip():
    env_path = os.path.join(os.path.dirname(__file__), "..", ".env")
    if os.path.exists(env_path):
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("VITE_MQTT_BROKER_IP="):
                    return line.split("=", 1)[1].strip()
    return "127.0.0.1"

parser = argparse.ArgumentParser(description="P2026 Ground Station Monitor")
parser.add_argument("--mqtt-ip", default=None, help="MQTT Broker IP")
args = parser.parse_args()

if args.mqtt_ip:
    MQTT_BROKER = args.mqtt_ip
else:
    default_ip = load_env_mqtt_ip()
    try:
        user_input = input(f"請輸入目標 MQTT 伺服器 IP (直接 Enter 則預設為 {default_ip}): ").strip()
        MQTT_BROKER = user_input if user_input else default_ip
    except (EOFError, KeyboardInterrupt):
        MQTT_BROKER = default_ip
        print()
MQTT_PORT = 1883
TOPIC_TELEMETRY = "fc/telemetry"
TOPIC_CMD = "fc/cmd"

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) if hasattr(mqtt, 'CallbackAPIVersion') else mqtt.Client()

def on_connect(client, userdata, flags, rc, *args):
    print(f"\n✅ MQTT 連線成功! 準備接收前端指令...")
    client.subscribe(TOPIC_CMD)

def on_disconnect(client, userdata, flags, rc, *args):
    print(f"⚠️ MQTT 連線中斷 (RC: {rc})，正在自動嘗試重新連線...", flush=True)

def on_message(client, userdata, msg):
    global desired_board_freq
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        if payload.get("type") == "cmd":
            action = payload.get("action")
            if action == "setState":
                state_id = int(payload["stateId"])
                if state_id not in _VALID_STATE_IDS:
                    print(f"[MQTT→Serial] ❌ 非法狀態碼：{state_id}", flush=True)
                    return
                if _ser_global and _ser_global.is_open:
                    serial_cmd_queue.put(f"{state_id}\n".encode('utf-8'))
                    print(f"[MQTT→Serial] 📡 切換指令發送：State {state_id}", flush=True)
                else:
                    print("[MQTT→Serial] ⚠️ Serial port 未開啟，指令丟棄", flush=True)
            elif action == "setFreq":
                freq = int(payload["frequency"])
                if freq in [433000000, 915000000]:
                    desired_board_freq = freq
                    if _ser_global and _ser_global.is_open:
                        serial_cmd_queue.put(f"SET_FREQ:{freq}\n".encode('utf-8'))
                        print(f"[MQTT→Serial] 📡 切換頻率指令發送：{freq} Hz", flush=True)
                    else:
                        print("[MQTT→Serial] ⚠️ Serial port 未開啟，指令丟棄", flush=True)
                else:
                    print(f"[MQTT→Serial] ❌ 非法頻率：{freq}", flush=True)
            elif action == "queryFreq":
                if current_board_freq is not None:
                    publish_status_update()
                if _ser_global and _ser_global.is_open:
                    serial_cmd_queue.put(b"REQ_FREQ\n")
                    print("[MQTT→Serial] 📡 查詢頻率指令發送", flush=True)
    except Exception as e:
        print(f"[MQTT CMD] ❌ 指令解析錯誤：{e}", flush=True)

mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message

def start_mqtt():
    def mqtt_connect_loop():
        connected = False
        while not connected:
            try:
                mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
                mqtt_client.loop_start() # 背景執行緒接收/發送 MQTT
                print(f"✅ MQTT 連線成功，開始監聽 {MQTT_BROKER}:{MQTT_PORT}", flush=True)
                connected = True
            except Exception as e:
                print(f"❌ 無法連線至 MQTT Broker ({MQTT_BROKER}): {e}，5 秒後重試...", flush=True)
                time.sleep(5)
    threading.Thread(target=mqtt_connect_loop, daemon=True).start()

def build_event_string(event_id, p1, p2, p3):
    EVENT_MAP = {
        1: f"State: {int(p1)} -> {int(p2)}",
        2: f"[SM] Current State: {int(p1)}, Time: {p2:.2f}",
        3: "EVENT: Powered flight start time recorded",
        4: "ACTION: System Reset to STBY_IDLE",
        5: "ACTION: Calibrating Gyro...",
        6: "ACTION: Drogue Deployed",
        7: "ACTION: Motor Sep & Main Chute",
        8: "ACTION: Main Chute (No Sep)",
        9: "EVENT: Mission Terminated",
        10: "TRIGGER: Global Time Safety - Forcing Apogee (35s)",
        11: "TRIGGER: Motor Thrust detected (IMU)",
        12: "TRIGGER: Altitude gain detected (BMP Backup)",
        13: "TRIGGER: Burnout detected (IMU)",
        14: "TRIGGER: Burnout assumed (Time Backup)",
        15: "TRIGGER: Apogee detected (Sensors)",
        16: "TRIGGER: Descent detected",
        17: "DECISION: GPS Failed, forcing Main Chute (Safety First)",
        18: "TRIGGER: Launch G-force detected",
        19: "ACTION: Gyro Calibration Started",
        20: f"ACTION: Gyro Calib Done ({p1:.3f}, {p2:.3f}, {p3:.3f})",
        21: "HW ERROR: ICM20948 (IMU) Init Failed",
        22: "HW ERROR: BMP388 (Barometer) Init Failed",
        23: "ACTION: BMP388 Calibration Completed",
        24: f"ACTION: BMP388 Calib Params (T1={p1:.2f}, P1={p2:.2f})",
        25: "ACTION: Accelerometer Calibration Done",
        26: "ACTION: Magnetometer Calibration Done",
        27: "ACTION: Temperature Calibration Done"
    }
    return EVENT_MAP.get(event_id, f"Unknown Event {event_id} ({p1:.2f}, {p2:.2f}, {p3:.2f})")

rf_buffer = bytearray()
_last_print_times = {}

def rate_limit_print(key, text, min_interval=0.2):
    now = time.time()
    last = _last_print_times.get(key, 0)
    if now - last >= min_interval:
        print(text, flush=True)
        _last_print_times[key] = now

def parse_rf_buffer(rssi=0):
    global rf_buffer, current_board_freq
    batch_data = []
    offset = 0
    parsed_count = 0
    while offset < len(rf_buffer):
        if len(rf_buffer) - offset < 8:
            break # 不夠一個 header
        
        pkt_type, ts = struct.unpack_from('<B3xI', rf_buffer, offset)
        
        if pkt_type == 4: # DATA_TYPE_LOG
            if len(rf_buffer) - offset < 24: break
            event_id, p1, p2, p3 = struct.unpack_from('<B3xfff', rf_buffer, offset + 8)
            msg = build_event_string(event_id, p1, p2, p3)
            print(f"  [LOG] {ts}ms: {msg}", flush=True)
            batch_data.append({"type": "LOG", "ts": ts, "data": {"msg": msg}})
            offset += 24
            parsed_count += 1
            
        elif pkt_type == 0: # PADDING
            offset += 1
            
        elif pkt_type == 1: # DATA_TYPE_IMU
            if len(rf_buffer) - offset < 32: break
            ax, ay, az, gx, gy, gz = struct.unpack_from('<6f', rf_buffer, offset + 8)
            ax, ay, az = clean_float(ax), clean_float(ay), clean_float(az)
            gx, gy, gz = clean_float(gx), clean_float(gy), clean_float(gz)
            rate_limit_print("IMU", f"  [IMU] {ts}ms: Acc=({ax:.2f}, {ay:.2f}, {az:.2f}) Gyro=({gx:.2f}, {gy:.2f}, {gz:.2f})")
            batch_data.append({"type": "IMU", "ts": ts, "data": {"ax":ax, "ay":ay, "az":az, "gx":gx, "gy":gy, "gz":gz}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 5: # DATA_TYPE_IMU_MAG
            if len(rf_buffer) - offset < 44: break
            ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack_from('<9f', rf_buffer, offset + 8)
            ax, ay, az = clean_float(ax), clean_float(ay), clean_float(az)
            gx, gy, gz = clean_float(gx), clean_float(gy), clean_float(gz)
            mx, my, mz = clean_float(mx), clean_float(my), clean_float(mz)
            rate_limit_print("IMU_MAG", f"  [IMU_MAG] {ts}ms: Acc=({ax:.2f}, {ay:.2f}, {az:.2f}) Gyro=({gx:.2f}, {gy:.2f}, {gz:.2f}) Mag=({mx:.2f}, {my:.2f}, {mz:.2f})")
            batch_data.append({"type": "IMU", "ts": ts, "data": {"ax":ax, "ay":ay, "az":az, "gx":gx, "gy":gy, "gz":gz, "mx":mx, "my":my, "mz":mz}})
            offset += 44
            parsed_count += 1
            
        elif pkt_type == 2: # DATA_TYPE_BMP
            if len(rf_buffer) - offset < 16: break
            pressure, temp = struct.unpack_from('<2f', rf_buffer, offset + 8)
            pressure, temp = clean_float(pressure), clean_float(temp)
            rate_limit_print("BMP", f"  [BMP] {ts}ms: Press={pressure:.2f}hPa, Temp={temp:.2f}°C")
            batch_data.append({"type": "BMP", "ts": ts, "data": {"pressure": pressure, "temp": temp}})
            offset += 16
            parsed_count += 1
            
        elif pkt_type == 3: # DATA_TYPE_GPS
            if len(rf_buffer) - offset < 32: break
            lat, lon, alt = struct.unpack_from('<ddf', rf_buffer, offset + 8)
            lat, lon, alt = clean_float(lat), clean_float(lon), clean_float(alt)
            rate_limit_print("GPS", f"  [GPS] {ts}ms: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
            batch_data.append({"type": "GPS", "ts": ts, "data": {"lat": lat, "lon": lon, "alt": alt}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 10: # KALMAN_TYPE_QUATERNION
            if len(rf_buffer) - offset < 24: break
            qw, qx, qy, qz = struct.unpack_from('<ffff', rf_buffer, offset + 8)
            qw, qx, qy, qz = clean_float(qw), clean_float(qx), clean_float(qy), clean_float(qz)
            rate_limit_print("QUAT", f"  [QUAT] {ts}ms: w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f}")
            batch_data.append({"type": "KALMAN_QUATERNION", "ts": ts, "data": {"q": [qw, qx, qy, qz]}})
            offset += 24
            parsed_count += 1
            
        elif pkt_type == 12: # KALMAN_TYPE_ALTITUDE
            if len(rf_buffer) - offset < 20: break
            alt, vz, az = struct.unpack_from('<fff', rf_buffer, offset + 8)
            alt, vz, az = clean_float(alt), clean_float(vz), clean_float(az)
            rate_limit_print("KF_ALT", f"  [KF_ALT] {ts}ms: Alt={alt:.2f}m, Vz={vz:.2f}m/s, Az={az:.2f}m/s²")
            batch_data.append({"type": "KALMAN_ALTITUDE", "ts": ts, "data": {"alt":alt, "vz":vz, "az":az}})
            offset += 20
            parsed_count += 1
            
        elif pkt_type == 11 or pkt_type == 13: # KALMAN_TYPE_POSITION (13), old GPS (11)
            if len(rf_buffer) - offset < 32: break
            lat, lon, velN, velE = struct.unpack_from('<ddff', rf_buffer, offset + 8)
            lat, lon, velN, velE = clean_float(lat), clean_float(lon), clean_float(velN), clean_float(velE)
            batch_data.append({"type": "KALMAN_POSITION" if pkt_type==13 else "KALMAN_GPS", "ts": ts, "data": {"lat":lat, "lon":lon, "vN":velN, "vE":velE}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 254: # SEQUENCE NUMBER
            batch_data.append({"type": "SEQUENCE", "ts": 0, "data": {"seq": ts}})
            offset += 8
            parsed_count += 1
            
        else:
            rate_limit_print("UNKNOWN", f"  [UNKNOWN] Type ID: {pkt_type} at offset {offset}")
            offset += 1 # 略過一個 byte 嘗試重新對齊
            
    # 從緩衝區移除已解析的部分
    rf_buffer = rf_buffer[offset:]
    
    # 斷線保護：避免緩衝區因連續解析失敗而無限增長，導致記憶體溢出
    if len(rf_buffer) > 8192:
        print("⚠️ 警告：Serial 緩衝區過大，懷疑存在大量毀損資料，執行強制清空以維持穩定性。")
        rf_buffer.clear()
    
    if parsed_count > 0:
        if batch_data:
            # 硬碟強制落地寫入 (避免前端當機導致資料遺失)
            try:
                with open(JSON_LOG_FILE, "a", encoding="utf-8") as f:
                    f.write(json.dumps({"batch": batch_data, "ts_local": time.time(), "rssi": rssi, "board_freq": current_board_freq}) + "\n")
            except Exception as e:
                print(f"⚠️ 硬碟日誌寫入失敗: {e}")
                
                
            # MQTT 推送
            try:
                seq_num = next((b["data"]["seq"] for b in batch_data if b["type"] == "SEQUENCE"), 0)
                payload_str = json.dumps({
                    "batch": batch_data, 
                    "status": "0-0", 
                    "pkt": seq_num, 
                    "rssi": rssi,
                    "board_freq": current_board_freq
                })
                mqtt_client.publish(TOPIC_TELEMETRY, payload_str)
            except Exception as e:
                print(f"⚠️ MQTT 推送失敗: {e}")


def read_from_port():
    global rf_buffer, _ser_global, current_board_freq
    last_query_time = 0
    while True:
        try:
            if _ser_global and _ser_global.is_open:
                # 定期查詢頻率 (每 10 秒，或者尚未偵測到時)
                now = time.time()
                if current_board_freq is None or now - last_query_time > 10:
                    serial_cmd_queue.put(b"REQ_FREQ\n")
                    last_query_time = now

                # 優先處理待發送指令
                while not serial_cmd_queue.empty():
                    cmd = serial_cmd_queue.get_nowait()
                    _ser_global.write(cmd)
                    _ser_global.flush()

                if _ser_global.in_waiting > 0:
                    line = _ser_global.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 原始 Serial 輸出無腦寫入硬碟
                    if line:
                        try:
                            with open(RAW_LOG_FILE, "a", encoding="utf-8") as f:
                                f.write(line + "\n")
                        except:
                            pass
                    
                    # 偵測板子頻率回應
                    if "CURRENT_FREQ:" in line:
                        try:
                            freq_str = line.split("CURRENT_FREQ:")[1].strip()
                            freq_val = int(freq_str)
                            if freq_val != current_board_freq:
                                current_board_freq = freq_val
                                print(f"📡 偵測到板子目前頻率: {current_board_freq} Hz")
                                publish_status_update()
                        except Exception as e:
                            print(f"解析頻率錯誤: {e}")

                    elif "Wired Telemetry Data:" in line:
                        try:
                            rate_limit_print("LORA_CHUNK", "\n==================== NEW LORA CHUNK ====================", min_interval=1.0)
                            parts = line.split("Wired Telemetry Data:")[1].split("RSSI:")
                            hex_str = parts[0].strip()
                            rssi = int(parts[1].strip()) if len(parts) > 1 else None
                            
                            hex_list = hex_str.split()
                            byte_data = bytes([int(x, 16) for x in hex_list])
                            rf_buffer.extend(byte_data)
                            
                            # 呼叫解析函式
                            parse_rf_buffer(rssi)
                        except Exception as e:
                            print(f"  -> [解析錯誤] {e}")
                    elif line:
                        print(line)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(1)
        except (serial.SerialException, OSError) as e:
            print(f"\n❌ [硬體異常斷線] Serial Error: {e}")
            if _ser_global:
                try:
                    _ser_global.close()
                except Exception:
                    pass
            # 進入自動重連迴圈
            while True:
                try:
                    time.sleep(2)
                    ports = serial.tools.list_ports.comports()
                    found_port = None
                    
                    # 1. 優先嘗試用相同的 port 裝置路徑
                    if any(p.device == _ser_global.port for p in ports):
                        found_port = _ser_global.port
                    
                    # 2. 如果原本的路徑不見了，嘗試使用相同的 VID/PID 找回
                    if not found_port and target_vid is not None and target_pid is not None:
                        for p in ports:
                            if p.vid == target_vid and p.pid == target_pid:
                                if target_serial_number and p.serial_number != target_serial_number:
                                    continue
                                found_port = p.device
                                break
                                
                    # 3. 如果還是找不到，但有類似 usbserial/usbmodem 的埠，自動選取它
                    if not found_port:
                        usb_ports = [p.device for p in ports if "usb" in p.device.lower() or "serial" in p.device.lower() or "usbmodem" in p.device.lower()]
                        if usb_ports:
                            found_port = usb_ports[0]

                    if found_port:
                        if _ser_global.port != found_port:
                            print(f"⚠️ 原連接埠 {_ser_global.port} 已變更，自動切換至新埠: {found_port}")
                            _ser_global.port = found_port
                        
                        print(f"🔄 嘗試重新連線至 {_ser_global.port} ...")
                        _ser_global.dtr = False
                        _ser_global.rts = False
                        _ser_global.open()
                        print(f"✅ 成功重新連線至 {_ser_global.port}!")
                        
                        # 斷電重啟防護：自動改回斷電前設定的目標頻率
                        if desired_board_freq is not None:
                            print(f"🛡️ 斷電重啟防護：正在同步發送設定頻率指令 ({desired_board_freq / 1000000} MHz)")
                            serial_cmd_queue.put(f"SET_FREQ:{desired_board_freq}\n".encode('utf-8'))
                        else:
                            serial_cmd_queue.put(b"REQ_FREQ\n")
                        break
                except Exception as ex:
                    print(f"連線失敗，繼續重試: {ex}")
        except Exception as e:
            print(f"\n❌ [未預期錯誤] read_from_port: {e}")
            time.sleep(1)

def write_to_port(ser):
    import sys
    from prompt_toolkit import PromptSession
    from prompt_toolkit.patch_stdout import patch_stdout

    print("=======================================")
    print("輸入 'T' 發送強制終止指令")
    print("輸入 'exit' 離開程式")
    print("=======================================\n")
    
    session = PromptSession()
    
    while True:
        try:
            with patch_stdout():
                user_input = session.prompt('Command > ').strip()
            
            if user_input.lower() == 'exit':
                print("Exiting...")
                ser.close()
                sys.exit(0)
            
            if user_input.upper() == 'T':
                print(">> [傳送] Force Terminate Command ('T')")
                serial_cmd_queue.put(b'T\n')
            elif user_input != "":
                serial_cmd_queue.put((user_input + '\n').encode('utf-8'))
        except KeyboardInterrupt:
            print("\nExiting...")
            ser.close()
            sys.exit(0)
        except EOFError:
            print("\nExiting...")
            ser.close()
            sys.exit(0)
        except Exception as e:
            print(f"Input error: {e}")
            break

def main():
    global _ser_global, target_vid, target_pid, target_description, target_serial_number
    serial_port = choose_serial_port()
    try:
        # 尋找匹配的連接埠硬體資訊以供重連使用
        try:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                if p.device == serial_port:
                    target_vid = p.vid
                    target_pid = p.pid
                    target_description = p.description
                    target_serial_number = p.serial_number
                    print(f"📡 已記錄連接埠硬體資訊: VID={hex(p.vid) if p.vid else 'None'}, PID={hex(p.pid) if p.pid else 'None'}, Desc='{p.description}'")
                    break
        except Exception as e:
            print(f"⚠️ 無法讀取連接埠硬體資訊: {e}")

        ser = serial.Serial()
        ser.port = serial_port
        ser.baudrate = BAUD_RATE
        ser.timeout = 1
        # 在 open 之前先設定好 DTR 和 RTS，避免一連線就觸發 ESP32 重新開機
        ser.dtr = True
        ser.rts = False
        
        try:
            ser.open()
        except Exception as e:
            if "Invalid argument" in str(e) or "termios.error" in type(e).__name__:
                print(f"\n❌ [錯誤] 無法開啟 Serial Port '{serial_port}' (Invalid argument)")
                print("💡 這在 macOS 上通常是因為您選到了「藍牙連接埠 (Bluetooth-Incoming-Port)」或虛擬連接埠。")
                print("👉 解決方法：重新執行程式，並在選單中手動選擇名稱包含 'usbserial' 或 'usbmodem' 的正確選項，不要直接按 Enter。")
                sys.exit(1)
            else:
                raise e
                
        _ser_global = ser  # 讓 ws_handler 可存取 Serial port
        print(f"成功連接 {serial_port} @ {BAUD_RATE} baud.")
        
        
        # 啟動 MQTT
        start_mqtt()
        print("📡 Ground Station Broker is listening on port 1883 (TCP) and 9001 (WebSockets)", flush=True)
        
        def serve_http():
            import http.server
            import socketserver
            import os
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            dist_dir = os.path.join(base_dir, "dist")
            
            if not os.path.exists(dist_dir):
                print(f"\n[HTTP] 找不到 {dist_dir}！")
                print("[HTTP] 若要在同一個腳本並行執行網頁，請先在 ground_station 資料夾下執行 'npm run build' 打包前端。")
                return
                
            os.chdir(dist_dir)
            handler = http.server.SimpleHTTPRequestHandler
            
            try:
                socketserver.TCPServer.allow_reuse_address = True
                with socketserver.TCPServer(("", 8080), handler) as httpd:
                    print(f"\n✅ Ground Station UI 啟動於 http://localhost:8080\n")
                    httpd.serve_forever()
            except Exception as e:
                print(f"\n[HTTP] 啟動網頁伺服器失敗: {e}\n")

        # 啟動 HTTP 靜態伺服器 (供前端使用)
        threading.Thread(target=serve_http, daemon=True).start()
        
        read_thread = threading.Thread(target=read_from_port, daemon=True)
        read_thread.start()
        
        write_to_port(ser)
    except serial.SerialException as e:
        print(f"無法打開 Serial Port {serial_port}: {e}")
        print("請確認開發板已連接，或輸入正確的 Serial Port 路徑。")

if __name__ == "__main__":
    main()

