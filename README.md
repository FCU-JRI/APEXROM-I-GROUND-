# 🚀 P2026 Ground Station (Pixi 環境版)

這是一個由 React (前端) 與 Python (後端) 所組成的火箭地面站即時監控與控制系統。
本專案使用 [Pixi](https://pixi.sh/) 進行環境管理，讓你不需要手動安裝 Node.js 或 Python，就能一鍵跨平台 (Windows / macOS / Linux) 部署並執行所有依賴套件。

---

## 🛠️ 安裝與準備

### 1. 安裝 Pixi (如果你的電腦尚未安裝)
- **Windows / macOS / Linux**: 請參考 [Pixi 官方網站](https://pixi.sh/latest/)，通常可以透過終端機執行：
  ```bash
  curl -fsSL https://pixi.sh/install.sh | bash
  ```

### 2. 初始化專案 (僅第一次需要)
進入本資料夾後，執行以下指令，Pixi 會自動下載 Python、Node.js 以及所有必需的套件（包含 npm install）：
```bash
pixi run install-frontend
```

---

## 💻 開發模式 (Development Mode)
如果你需要即時修改網頁程式碼並看到變化，請使用開發模式。
此模式會啟動 Vite 的 HMR (熱更新) 伺服器，並同時啟動 Python 的 Serial/WebSocket 後端。

**一鍵啟動：**
```bash
pixi run fullstack
```
- 前端網頁將運行於：`http://localhost:5173`
- 後端 WebSocket 將運行於：`ws://localhost:8765`
*(註：在開發模式中，Python 終端機會保留原本的指令輸入介面，可用來手動切換狀態與發送終止指令)*

---

## 🏭 正式生產模式 (Production Mode) 🌟 推薦
實際出任務或發射時，強烈建議使用此模式。
它會將網頁編譯打包為靜態檔案，並透過極速的 `Caddy` 伺服器託管，能提供最流暢、穩定的監控體驗。

### 步驟 1：編譯打包前端 (原始碼修改後只需執行一次)
```bash
pixi run build
```

### 步驟 2：一鍵啟動正式伺服器
```bash
pixi run prod
```
- 前端網頁將運行於：`http://localhost:8080` (請使用瀏覽器開啟此網址)
- 後端 WebSocket 將運行於：`ws://localhost:8765`

---

## 📝 系統架構簡介
1. **Python 後端 (`py_files/monitor.py`)**: 
   - 負責監聽本機的 LoRa USB 接收模組 (Serial Port)。
   - 負責拆解從火箭傳下來的二進位封包。
   - 建立 WebSocket 伺服器，將資料即時推播給網頁前端。
2. **React 前端 (`src/`)**: 
   - 使用 Recharts 繪製即時三軸遙測數據圖表。
   - 提供 3D 火箭姿態模擬。
   - 允許直接從網頁發送指令控制火箭狀態，或觸發緊急終止。
   - 支援將全部歷史飛行數據打包匯出為 CSV 檔。
