import subprocess
import sys
import atexit

def main():
    print("=======================================")
    print("✨ Starting Ground Station Fullstack...")
    print("=======================================\n")
    
    # 1. 啟動 Vite 前端 (在背景執行)
    print("🚀 [1/2] Starting Vite Dev Server (http://localhost:5173)...")
    vite_process = subprocess.Popen(["npm", "run", "dev"], 
                                    stdout=subprocess.DEVNULL, 
                                    stderr=subprocess.DEVNULL)
    atexit.register(vite_process.terminate)
    
    # 2. 啟動 Python 後端 (在前台接管終端機，確保 prompt_toolkit 正常運作)
    print("🚀 [2/2] Starting Monitor Backend...\n")
    try:
        subprocess.run([sys.executable, "py_files/monitor.py"])
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down fullstack...")
        vite_process.terminate()

if __name__ == "__main__":
    main()
