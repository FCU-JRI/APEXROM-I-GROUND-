import subprocess
import sys
import os
import atexit

def main():
    print("=======================================")
    print("🚀 Starting Ground Station in PRODUCTION Mode...")
    print("=======================================\n")
    
    if not os.path.exists("dist"):
        print("❌ 找不到 dist 資料夾！請先執行 'pixi run build' 進行編譯打包。")
        sys.exit(1)

    # 1. 啟動 Caddy 伺服器 (在背景執行)
    print("🌐 [1/2] Starting Caddy Server (http://localhost:8080)...")
    caddy_process = subprocess.Popen(["caddy", "file-server", "--root", "dist", "--listen", ":8080"], 
                                     stdout=subprocess.DEVNULL, 
                                     stderr=subprocess.DEVNULL)
    atexit.register(caddy_process.terminate)
    
    # 2. 啟動 Python 後端 (在前台接管終端機)
    print("📡 [2/2] Starting Monitor Backend...\n")
    try:
        subprocess.run([sys.executable, "py_files/monitor.py"])
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down fullstack...")
        caddy_process.terminate()

if __name__ == "__main__":
    main()
