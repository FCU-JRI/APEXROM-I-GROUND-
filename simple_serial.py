import serial
import serial.tools.list_ports
import threading
import sys

def main():
    print("=== 簡單 Serial Monitor ===")
    
    # 1. 尋找所有 Serial Ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("❌ 找不到任何 Serial Port！請檢查 USB 連線。")
        sys.exit(1)
        
    print("\n可用的 Serial Ports:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
        
    # 2. 讓使用者選擇
    try:
        choice = input(f"\n請選擇 Port [0-{len(ports)-1}]: ").strip()
        port_index = int(choice)
        selected_port = ports[port_index].device
    except (ValueError, IndexError):
        print("❌ 無效的選擇")
        sys.exit(1)
        
    baud_rate = 115200
    
    # 3. 開啟 Serial Port
    try:
        ser = serial.Serial(selected_port, baud_rate, timeout=0.1)
        print(f"✅ 成功開啟 {selected_port} @ {baud_rate} baud")
        print("💡 提示：輸入訊息後按 Enter 即可送出。按 Ctrl+C 離開。")
        print("-" * 40)
    except Exception as e:
        print(f"❌ 開啟失敗：{e}")
        sys.exit(1)

    # 4. 建立讀取執行緒
    def read_from_port(ser_port):
        while True:
            try:
                if ser_port.in_waiting > 0:
                    line = ser_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"\n[RX] {line}")
            except Exception as e:
                print(f"\n❌ 讀取錯誤：{e}")
                break

    thread = threading.Thread(target=read_from_port, args=(ser,), daemon=True)
    thread.start()

    # 5. 主執行緒負責發送訊息
    try:
        while True:
            cmd = input()
            if cmd:
                ser.write((cmd + "\n").encode('utf-8'))
                ser.flush()
                print(f"[TX] 發送: {cmd}")
    except KeyboardInterrupt:
        print("\n\n🚪 關閉 Serial Monitor...")
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()
