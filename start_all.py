import subprocess
import sys
import atexit
import time
import os

def check_and_bootstrap_pixi():
    # 如果已經在 Pixi 環境中 (sys.prefix 包含 .pixi) 就不需要重啟
    if ".pixi" in sys.prefix or os.environ.get("PIXI_IN_SHELL"):
        return
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    if sys.platform == "win32":
        pixi_python = os.path.join(base_dir, ".pixi", "envs", "default", "python.exe")
    else:
        pixi_python = os.path.join(base_dir, ".pixi", "envs", "default", "bin", "python")
        
    if os.path.exists(pixi_python):
        # 使用 Pixi Python 重新執行當前腳本
        os.execv(pixi_python, [pixi_python] + sys.argv)
    else:
        print("❌ 錯誤：未偵測到本地 Pixi 環境，或請改用 'pixi run fullstack' 啟動。")
        sys.exit(1)

check_and_bootstrap_pixi()

def main():
    print("=======================================")
    print("✨ Starting Ground Station Fullstack...")
    print("=======================================\n")
    
    # 讀取 .env 設定檔中的 VITE_MQTT_BROKER_IP
    mqtt_ip = "127.0.0.1"
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if os.path.exists(env_path):
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("VITE_MQTT_BROKER_IP="):
                    mqtt_ip = line.split("=", 1)[1].strip()
                    break

    print(f"🔧 Loaded MQTT Broker IP from .env: {mqtt_ip}")
    is_local = (mqtt_ip == "127.0.0.1" or mqtt_ip == "localhost")
    
    mqtt_process = None
    if is_local:
        # 0. 啟動 MQTT Broker (Mosquitto)
        print("🚀 [1/3] Starting Local MQTT Broker (ws://localhost:9001)...")
        mqtt_process = subprocess.Popen(["mosquitto", "-c", "mosquitto.conf"], 
                                        stdout=subprocess.DEVNULL, 
                                        stderr=subprocess.DEVNULL)
        atexit.register(mqtt_process.terminate)
        time.sleep(1) # 等待 Broker 啟動
    else:
        print(f"🌐 [1/3] Using Remote MQTT Broker at {mqtt_ip}. Skipping local mosquitto.")
    
    # 1. 啟動 Vite 前端 (在背景執行)
    print(f"🚀 [2/3] Starting Vite Dev Server (http://localhost:5173)...")
    env = os.environ.copy()
    env["VITE_MQTT_BROKER_IP"] = mqtt_ip
    vite_process = subprocess.Popen(["npm", "run", "dev"], 
                                    stdout=subprocess.DEVNULL, 
                                    stderr=subprocess.DEVNULL,
                                    env=env)
    atexit.register(vite_process.terminate)
    
    # 2. 啟動 Python 後端 (在前台接管終端機，確保 prompt_toolkit 正常運作)
    print("🚀 [3/3] Starting Monitor Backend...\n")
    try:
        subprocess.run([sys.executable, "py_files/monitor.py", "--mqtt-ip", mqtt_ip])
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down fullstack...")
        vite_process.terminate()
        if mqtt_process:
            mqtt_process.terminate()

if __name__ == "__main__":
    main()
