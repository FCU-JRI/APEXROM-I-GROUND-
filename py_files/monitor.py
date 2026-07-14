import sys
import subprocess

# stdout 強制不緩衝，確保 daemon thread 的輸出即時可見
sys.stdout.reconfigure(line_buffering=True)


import serial
import serial.tools.list_ports
import threading
import struct
import json
import asyncio
import websockets
import queue
import time
import math

serial_cmd_queue = queue.Queue()
BAUD_RATE = 115200

def choose_serial_port():
    import os
    if os.environ.get("SERIAL_PORT"):
        return os.environ.get("SERIAL_PORT")
        
    # 若有帶入命令列參數，優先使用
    if len(sys.argv) > 1:
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

# WebSocket 廣播機制
ws_clients = set()
ws_loop    = None
_ser_global = None  # 由 main() 設定，供 ws_handler 轉發指令至 Serial

# STATENUM 合法範圍（對應 StateMachine.hpp enum）
_VALID_STATE_IDS = set(range(13))  # 0–12

def clean_float(val):
    if math.isnan(val) or math.isinf(val):
        return 0.0
    return val

async def ws_handler(websocket):
    """雙向 WebSocket handler：
    - 下行（FC→GND）：由 broadcast_ws 主動推送感測資料
    - 上行（GND→FC）：接收地面端指令並轉發至 Serial
    """
    ws_clients.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
                if msg.get("type") == "cmd" and msg.get("action") == "setState":
                    state_id = int(msg["stateId"])
                    if state_id not in _VALID_STATE_IDS:
                        print(f"[WS→Serial] ❌ 非法狀態碼：{state_id}", flush=True)
                        continue
                    if _ser_global and _ser_global.is_open:
                        serial_cmd_queue.put(f"{state_id}\n".encode('utf-8'))
                        print(f"[WS→Serial] 📡 切換指令發送：State {state_id}", flush=True)
                    else:
                        print("[WS→Serial] ⚠️ Serial port 未開啟，指令丟棄", flush=True)
            except Exception as e:
                print(f"[WS CMD] ❌ 解析或處理錯誤：{e}", flush=True)
    finally:
        ws_clients.discard(websocket)

async def broadcast_ws(batch_data):
    if ws_clients:
        # Extract sequence number if present, otherwise use 0
        seq_num = next((b["data"]["seq"] for b in batch_data if b["type"] == "SEQUENCE"), 0)
        msg = json.dumps({"batch": batch_data, "status": "0-0", "pkt": seq_num})
        print(f"[WS] Broadcasting to {len(ws_clients)} active clients...")
        await asyncio.gather(*[client.send(msg) for client in ws_clients])

async def _ws_main():
    global ws_loop
    ws_loop = asyncio.get_running_loop()
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()  # 永久等待，直到 thread 結束

def ws_server_thread():
    asyncio.run(_ws_main())

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

def parse_rf_buffer():
    global rf_buffer
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
            print(f"  📝 [LOG] {ts}ms: {msg}")
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
            print(f"  🔹 [IMU] {ts}ms: Acc=({ax:.2f}, {ay:.2f}, {az:.2f}) Gyro=({gx:.2f}, {gy:.2f}, {gz:.2f})")
            batch_data.append({"type": "IMU", "ts": ts, "data": {"ax":ax, "ay":ay, "az":az, "gx":gx, "gy":gy, "gz":gz}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 2: # DATA_TYPE_BMP
            if len(rf_buffer) - offset < 16: break
            pressure, temp = struct.unpack_from('<2f', rf_buffer, offset + 8)
            pressure, temp = clean_float(pressure), clean_float(temp)
            print(f"  ☁️ [BMP] {ts}ms: Press={pressure:.2f}hPa, Temp={temp:.2f}°C")
            batch_data.append({"type": "BMP", "ts": ts, "data": {"pressure": pressure, "temp": temp}})
            offset += 16
            parsed_count += 1
            
        elif pkt_type == 3: # DATA_TYPE_GPS
            if len(rf_buffer) - offset < 32: break
            lat, lon, alt = struct.unpack_from('<ddf', rf_buffer, offset + 8)
            lat, lon, alt = clean_float(lat), clean_float(lon), clean_float(alt)
            print(f"  📍 [GPS] {ts}ms: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
            batch_data.append({"type": "GPS", "ts": ts, "data": {"lat": lat, "lon": lon, "alt": alt}})
            offset += 32
            parsed_count += 1
            
        elif pkt_type == 10: # KALMAN_TYPE_QUATERNION
            if len(rf_buffer) - offset < 24: break
            qw, qx, qy, qz = struct.unpack_from('<ffff', rf_buffer, offset + 8)
            qw, qx, qy, qz = clean_float(qw), clean_float(qx), clean_float(qy), clean_float(qz)
            print(f"  [QUAT] {ts}ms: w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f}")
            batch_data.append({"type": "KALMAN_QUATERNION", "ts": ts, "data": {"q": [qw, qx, qy, qz]}})
            offset += 24
            parsed_count += 1
            
        elif pkt_type == 12: # KALMAN_TYPE_ALTITUDE
            if len(rf_buffer) - offset < 20: break
            alt, vz, az = struct.unpack_from('<fff', rf_buffer, offset + 8)
            alt, vz, az = clean_float(alt), clean_float(vz), clean_float(az)
            print(f"  🚀 [KF_ALT] {ts}ms: Alt={alt:.2f}m, Vz={vz:.2f}m/s, Az={az:.2f}m/s²")
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
            print(f"  [UNKNOWN] Type ID: {pkt_type} at offset {offset}")
            offset += 1 # 略過一個 byte 嘗試重新對齊
            
    # 從緩衝區移除已解析的部分
    rf_buffer = rf_buffer[offset:]
    
    if parsed_count > 0:
        if batch_data and ws_loop:
            asyncio.run_coroutine_threadsafe(broadcast_ws(batch_data), ws_loop)


def read_from_port(ser):
    global rf_buffer
    while True:
        try:
            # 優先處理待發送指令
            while not serial_cmd_queue.empty():
                cmd = serial_cmd_queue.get_nowait()
                ser.write(cmd)

            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if "Wired Telemetry Data:" in line:
                    try:
                        print("\n==================== NEW LORA CHUNK ====================")
                        hex_str = line.split("Wired Telemetry Data:")[1].strip()
                        hex_list = hex_str.split()
                        byte_data = bytes([int(x, 16) for x in hex_list])
                        rf_buffer.extend(byte_data)
                        
                        # 呼叫解析函式
                        parse_rf_buffer()
                    except Exception as e:
                        print(f"  -> [解析錯誤] {e}")
                else:
                    print(line)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Serial error: {e}")
            break

def write_to_port(ser):
    from prompt_toolkit import prompt
    from prompt_toolkit.patch_stdout import patch_stdout

    print("=======================================")
    print("輸入 'T' 發送強制終止指令")
    print("輸入 'exit' 離開程式")
    print("=======================================\n")
    
    with patch_stdout():
        while True:
            try:
                user_input = prompt("指令 > ")
                if user_input.lower() == 'exit':
                    print("Exiting...")
                    ser.close()
                    sys.exit(0)
                
                if user_input.upper() == 'T':
                    print(">> [傳送] Force Terminate Command ('T')")
                    serial_cmd_queue.put(b'T\n')
                elif user_input.strip() != "":
                    serial_cmd_queue.put((user_input + '\n').encode('utf-8'))
            except KeyboardInterrupt:
                print("\nExiting...")
                ser.close()
                sys.exit(0)
            except EOFError:
                import time
                time.sleep(1)
                continue
            except Exception as e:
                print(f"Input error: {e}")
                break

def main():
    global _ser_global
    serial_port = choose_serial_port()
    try:
        ser = serial.Serial()
        ser.port = serial_port
        ser.baudrate = BAUD_RATE
        ser.timeout = 1
        # 在 open 之前先設定好 DTR 和 RTS，避免一連線就觸發 ESP32 重新開機
        ser.dtr = False
        ser.rts = False
        ser.open()
        _ser_global = ser  # 讓 ws_handler 可存取 Serial port
        print(f"成功連接 {serial_port} @ {BAUD_RATE} baud.")
        
        
        # 啟動 WebSocket 伺服器
        threading.Thread(target=ws_server_thread, daemon=True).start()
        print("WebSocket Server started at ws://localhost:8765")
        
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
        
        read_thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
        read_thread.start()
        
        write_to_port(ser)
    except serial.SerialException as e:
        print(f"無法打開 Serial Port {serial_port}: {e}")
        print("請確認開發板已連接，或輸入正確的 Serial Port 路徑。")

if __name__ == "__main__":
    main()

