import { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { MapContainer, TileLayer, Polyline, Marker, useMap } from 'react-leaflet';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, Legend, ReferenceLine, Label } from 'recharts';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import { Rocket3D } from './components/Rocket3D';
import { TeleCard } from './components/TeleCard';
import { SensorValue } from './components/SensorValue';

// STATENUM — 對應 StateMachine.hpp enum（值 0-17）
const STATES = {
    0:  { name:"STBY_IDLE",         label:"待機",              group:"standby", next:[1,2,10,11,12,13], critical:false },
    1:  { name:"STBY_BIT",          label:"自我測試",           group:"standby", next:[0],            critical:false },
    2:  { name:"FLIGHT_IGNITION",   label:"點火",               group:"flight",  next:[3],            critical:true  },
    3:  { name:"FLIGHT_POWERED",    label:"動力上升",           group:"flight",  next:[4],            critical:false },
    4:  { name:"FLIGHT_INERTIAL",   label:"慣性滑行",           group:"flight",  next:[5],            critical:false },
    5:  { name:"FLIGHT_APOGEE",     label:"頂點 / 減速傘",      group:"flight",  next:[6],            critical:true  },
    6:  { name:"FLIGHT_DESCENT",    label:"下降",               group:"flight",  next:[7,8],          critical:false },
    7:  { name:"MAIN_CHUTE_DEPLOY", label:"分離引擎 / 開主傘", group:"flight",  next:[9],            critical:true  },
    8:  { name:"SKIP_MAIN_CHUTE",   label:"跳過主傘",           group:"flight",  next:[9],            critical:true  },
    9:  { name:"TERMINATE",         label:"任務終止",           group:"flight",  next:[],             critical:true  },
    10: { name:"DBG_COMM",          label:"Debug: 通訊",        group:"debug",   next:[0],            critical:false },
    11: { name:"DBG_SENSOR",        label:"Debug: 感測器",      group:"debug",   next:[0],            critical:false },
    12: { name:"DBG_STORAGE",       label:"Debug: 儲存",        group:"debug",   next:[0],            critical:false },
    13: { name:"RESET_ORIENTATION", label:"重置姿態為向上",       group:"cal",     next:[0],            critical:false },
};

// === GPS MAP COMPONENT ===
const MapUpdater = ({ center }) => {
    const map = useMap();
    useEffect(() => {
        if (center[0] !== 0) map.setView(center, map.getZoom());
    }, [center, map]);
    return null;
};

const customIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
});

const GpsMap = ({ gps, trajectory }) => {
    if (gps.lat === 0 && gps.lon === 0) {
        return <div className="absolute inset-0 flex items-center justify-center text-gray-700 text-sm font-bold tracking-widest z-20 bg-black/80">AWAITING SATELLITE LOCK</div>;
    }
    return (
        <MapContainer center={[gps.lat, gps.lon]} zoom={18} style={{ height: '100%', width: '100%', backgroundColor: '#000' }} zoomControl={false} attributionControl={false}>
            <TileLayer url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png" />
            {trajectory.length > 0 && <Polyline positions={trajectory} color="#00f2fe" weight={4} opacity={0.8} />}
            <Marker position={[gps.lat, gps.lon]} icon={customIcon} />
            <MapUpdater center={[gps.lat, gps.lon]} />
        </MapContainer>
    );
};

function App() {
    const [currStateId, setCurrStateId] = useState(0);
    const [wsConnected, setWsConnected] = useState(false);
    const [isLive,      setIsLive]      = useState(true);
    const [histIdx,     setHistIdx]     = useState(0);
    const [history,     setHistory]     = useState([]);
    const [cmdLog,      setCmdLog]      = useState([]);
    const [fcLogs,      setFcLogs]      = useState([]);
    const [stateEvents, setStateEvents] = useState([]);
    const [showExport,  setShowExport]  = useState(false);
    const [exportStart, setExportStart] = useState(0);
    const [exportEnd,   setExportEnd]   = useState(600);
    const wsRef = useRef(null);

    const addCmdLog = useCallback((msg) => {
        const t = new Date().toLocaleTimeString('zh-TW', { hour12:false });
        setCmdLog(prev => [`[${t}] ${msg}`, ...prev].slice(0, 100));
    }, []);

    // WebSocket — 永久連線，僅初始化一次
    useEffect(() => {
        let ws, timer;
        let isMounted = true;
        const connect = () => {
            if (!isMounted) return;
            const host = window.location.hostname || "localhost";
            console.log(host);
            ws = new WebSocket(`ws://${host}:8765`);
            wsRef.current = ws;
            ws.onopen  = () => { if (isMounted) { setWsConnected(true);  addCmdLog("✅ 已連線至 Monitor WebSocket"); } };
            ws.onclose = () => { 
                if (isMounted) { 
                    setWsConnected(false); 
                    addCmdLog("❌ WebSocket 斷線，3 秒後重連…"); 
                    timer = setTimeout(connect, 3000); 
                }
            };
            ws.onerror = () => {};
            ws.onmessage = (e) => {
                if (!isMounted) return;
                try {
                    const d = JSON.parse(e.data);
                    if (d.batch) {
                        setHistory(h => [...h, d].slice(-1000));
                        // 攔截 LOG 封包，同步火箭端狀態
                        d.batch.forEach(b => {
                            if (b.type === "LOG" && b.data && b.data.msg) {
                                console.log("LOG RECEIVED:", b.data.msg); // Debug
                                const t = new Date().toLocaleTimeString('zh-TW', { hour12:false });
                                let shortMsg = b.data.msg;
                                shortMsg = shortMsg.replace(/\[SM\] Current State:\s*(\d+)(,\s*Time:\s*[\d.]+)?/, "STATE: $1");
                                shortMsg = shortMsg.replace(/State:\s*(\d+)\s*->\s*(\d+)/, "$1 -> $2");
                                setFcLogs(prev => [{ time: t, msg: shortMsg }, ...prev].slice(0, 5000));

                                const transitionMatch = b.data.msg.match(/State:\s*\d+\s*->\s*(\d+)/);
                                const currentMatch = b.data.msg.match(/\[SM\] Current State:\s*(\d+)/);
                                if (transitionMatch) {
                                    const nextStateId = parseInt(transitionMatch[1], 10);
                                    setCurrStateId(nextStateId);
                                    setStateEvents(prev => {
                                        const eventName = STATES[nextStateId]?.name || `STATE_${nextStateId}`;
                                        // 避免重複加入
                                        if (prev.length > 0 && prev[prev.length-1].stateId === nextStateId && Math.abs(prev[prev.length-1].ts - b.ts) < 2000) {
                                            return prev;
                                        }
                                        return [...prev, { ts: b.ts, tsSec: b.ts / 1000.0, stateId: nextStateId, name: eventName }];
                                    });
                                } else if (currentMatch) {
                                    setCurrStateId(parseInt(currentMatch[1], 10));
                                }
                            }
                        });
                    }
                } catch (err) {}
            };
        };
        connect();
        return () => { 
            isMounted = false;
            clearTimeout(timer); 
            if (ws) {
                ws.onclose = null; // IMPORTANT: Prevent onclose from firing and scheduling a reconnect!
                ws.close(); 
            }
        };
    }, [addCmdLog]);

    useEffect(() => {
        if (isLive && history.length > 0) setHistIdx(history.length - 1);
    }, [history.length, isLive]);

    // 發送狀態切換指令
    const sendStateCommand = useCallback((stateId) => {
        const s = STATES[stateId];
        if (!s) return;
        if (s.critical) {
            if (!window.confirm(`⚠️ 警告：即將發送飛行器指令\n\n→ [${stateId}] ${s.name}  (${s.label})\n\n此操作將直接影響飛行器，請再次確認！`)) return;
        }
        const ws = wsRef.current;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type:"cmd", action:"setState", stateId }));
            setCurrStateId(stateId);
            addCmdLog(`📡 [GND→FC] 切換至 ${s.name} (${stateId})`);
        } else {
            addCmdLog("❌ WebSocket 未連線，無法發送指令");
        }
    }, [addCmdLog]);

    // 衍生資料
    const currState  = STATES[currStateId];
    const nextStates = currState.next.map(id => ({ id, ...STATES[id] }));
    const normalNext   = nextStates.filter(s => !s.critical && s.group !== "debug");
    const criticalNext = nextStates.filter(s =>  s.critical);
    const debugNext    = nextStates.filter(s =>  s.group === "debug");

    const view   = isLive ? (history[history.length-1] || null) : history[histIdx];
    const getVal = (t) => view?.batch?.find(b => b.type === t)?.data;
    
    // 往前搜尋歷史紀錄，找出最近一筆指定類型的資料（解決低頻資料如 GPS 會閃動歸零的問題）
    const getLastVal = (t) => {
        const endIdx = isLive ? history.length - 1 : histIdx;
        if (endIdx < 0 || !history) return undefined;
        // 往前找最多 100 筆 (約數秒內)，避免無止盡搜尋
        const limit = Math.max(0, endIdx - 100);
        for (let i = endIdx; i >= limit; i--) {
            const d = history[i]?.batch?.find(b => b.type === t)?.data;
            if (d !== undefined) return d;
        }
        return undefined;
    };

    // 高頻資料可直接取當前 batch，或是為了穩定也使用 getLastVal
    const alt    = getLastVal("KALMAN_ALTITUDE")?.alt ?? 0;
    const vel    = getLastVal("KALMAN_ALTITUDE")?.vz  ?? 0;
    const acc    = getLastVal("IMU")?.az               ?? 0;
    const q      = getLastVal("KALMAN_QUATERNION")?.q  ?? [1,0,0,0];
    
    // GPS 是低頻資料，必須使用 getLastVal
    const gps    = getLastVal("KALMAN_GPS") ?? getLastVal("GPS") ?? { lat: 0, lon: 0 };
    
    
    const imu    = getLastVal("IMU");
    const bmp    = getLastVal("BMP");
    const gpsRaw = getLastVal("GPS");

    // 計算歷史軌跡 (Trajectory)
    const trajectory = useMemo(() => {
        const path = [];
        const endIdx = isLive ? history.length - 1 : histIdx;
        for (let i = 0; i <= endIdx; i++) {
            const batch = history[i]?.batch;
            if (!batch) continue;
            const gpsData = batch.find(b => b.type === "KALMAN_GPS" || b.type === "GPS")?.data;
            if (gpsData && gpsData.lat !== 0 && gpsData.lon !== 0) {
                const pt = [gpsData.lat, gpsData.lon];
                if (path.length === 0 || (path[path.length-1][0] !== pt[0] || path[path.length-1][1] !== pt[1])) {
                    path.push(pt);
                }
            }
        }
        return path;
    }, [history, histIdx, isLive]);

    // 計算圖表資料 (提取最近的 150 筆資料繪製)
    const chartData = useMemo(() => {
        const endIdx = isLive ? history.length - 1 : histIdx;
        if (endIdx < 0 || !history) return [];
        const startIdx = Math.max(0, endIdx - 150);
        const data = [];
        let lastAlt = 0, lastVel = 0, lastAcc = 0, lastTs = 0;
        
        for (let i = startIdx; i <= endIdx; i++) {
            const frame = history[i];
            const batch = frame?.batch;
            if (!batch) continue;
            
            const pktIndex = frame.pkt ?? i; // 使用真實的封包編號
            
            const altData = batch.find(b => b.type === "KALMAN_ALTITUDE");
            const imuData = batch.find(b => b.type === "IMU");
            
            if (altData) {
                lastAlt = altData.data.alt;
                lastVel = altData.data.vz;
                lastTs = altData.ts;
            }
            if (imuData) {
                lastAcc = imuData.data.az;
                if (!altData) lastTs = imuData.ts; // 如果沒有高頻 Alt，就用 IMU 的時間戳
            }
            
            if (lastTs > 0) {
                data.push({ time: pktIndex, tsSec: lastTs / 1000.0, alt: lastAlt, vel: lastVel, acc: lastAcc });
            }
        }
        return data;
    }, [history, histIdx, isLive]);

    // CSV 匯出邏輯
    const downloadCSV = useCallback((startSec, endSec) => {
        const rowsMap = new Map();
        const stateTimeline = [...stateEvents].sort((a,b) => a.ts - b.ts);
        const getStateAtTime = (ts) => {
            let s = "STBY_IDLE";
            for (const ev of stateTimeline) {
                if (ev.ts <= ts) s = ev.name;
                else break;
            }
            return s;
        };

        history.forEach(frame => {
            if (!frame.batch) return;
            frame.batch.forEach(b => {
                if (!b.ts) return;
                const tSec = b.ts / 1000.0;
                if (tSec >= startSec && tSec <= endSec) {
                    if (!rowsMap.has(b.ts)) {
                        rowsMap.set(b.ts, { tsSec: tSec });
                    }
                    const row = rowsMap.get(b.ts);
                    if (b.type === "IMU") {
                        row.ax = b.data.ax; row.ay = b.data.ay; row.az = b.data.az;
                        row.gx = b.data.gx; row.gy = b.data.gy; row.gz = b.data.gz;
                        row.mx = b.data.mx; row.my = b.data.my; row.mz = b.data.mz;
                    } else if (b.type === "BMP") {
                        row.pressure = b.data.pressure; row.temp = b.data.temp;
                    } else if (b.type === "GPS") {
                        row.gpsLat = b.data.lat; row.gpsLon = b.data.lon; row.gpsAlt = b.data.alt;
                    } else if (b.type === "KALMAN_ALTITUDE") {
                        row.kfAlt = b.data.alt; row.kfVz = b.data.vz; row.kfAz = b.data.az;
                    } else if (b.type === "KALMAN_QUATERNION") {
                        row.q0 = b.data.q[0]; row.q1 = b.data.q[1]; row.q2 = b.data.q[2]; row.q3 = b.data.q[3];
                    } else if (b.type === "KALMAN_GPS" || b.type === "KALMAN_POSITION") {
                        row.kfLat = b.data.lat; row.kfLon = b.data.lon; row.kfVN = b.data.vN; row.kfVE = b.data.vE;
                    }
                }
            });
        });

        const sortedTs = Array.from(rowsMap.keys()).sort((a, b) => a - b);
        if (sortedTs.length === 0) {
            alert("該時間段內沒有資料");
            return;
        }

        const headers = [
            "Time(s)", "State", 
            "AccX(g)", "AccY(g)", "AccZ(g)", "GyrX(deg/s)", "GyrY(deg/s)", "GyrZ(deg/s)", "MagX(uT)", "MagY(uT)", "MagZ(uT)",
            "Pressure(hPa)", "Temperature(C)",
            "GPS_Lat", "GPS_Lon", "GPS_Alt(m)",
            "KF_Alt(m)", "KF_Vz(m/s)", "KF_Az(m/s2)",
            "KF_QW", "KF_QX", "KF_QY", "KF_QZ",
            "KF_Lat", "KF_Lon", "KF_VN(m/s)", "KF_VE(m/s)"
        ];

        let csvContent = headers.join(",") + "\n";
        sortedTs.forEach(ts => {
            const row = rowsMap.get(ts);
            const state = getStateAtTime(ts);
            const line = [
                row.tsSec.toFixed(3), state,
                row.ax ?? "", row.ay ?? "", row.az ?? "", row.gx ?? "", row.gy ?? "", row.gz ?? "", row.mx ?? "", row.my ?? "", row.mz ?? "",
                row.pressure ?? "", row.temp ?? "",
                row.gpsLat ?? "", row.gpsLon ?? "", row.gpsAlt ?? "",
                row.kfAlt ?? "", row.kfVz ?? "", row.kfAz ?? "",
                row.q0 ?? "", row.q1 ?? "", row.q2 ?? "", row.q3 ?? "",
                row.kfLat ?? "", row.kfLon ?? "", row.kfVN ?? "", row.kfVE ?? ""
            ];
            csvContent += line.join(",") + "\n";
        });

        const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.setAttribute("href", url);
        link.setAttribute("download", `telemetry_${startSec}_to_${endSec}.csv`);
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
    }, [history, stateEvents]);

    // 計算 TIMELINE 顯示的當前秒數與真實封包編號
    const endIdx = isLive ? history.length - 1 : histIdx;
    let currentScrubberTsSec = 0;
    let currentPkt = 0;
    if (history[endIdx]) {
        currentPkt = history[endIdx].pkt ?? 0;
        if (history[endIdx].batch) {
            const b = history[endIdx].batch.find(x => x.ts);
            if (b) currentScrubberTsSec = b.ts / 1000.0;
        }
    }

    return (
        <div className="h-full flex flex-col gap-3 relative">
            <div className="scanline"></div>

            {/* Header */}
            <header className="glass-panel p-2 flex justify-between items-center glow-cyan">
                <div className="text-lg font-bold text-neon uppercase tracking-tighter">JRI P2026 GND V2.1</div>
                <div className="flex gap-3 items-center">
                    {!isLive && <div className="text-sm text-yellow-500 font-bold animate-pulse">HISTORICAL PLAYBACK</div>}
                    <div id="ws-status" className={`px-2 py-0.5 rounded text-xs font-bold border ${wsConnected ? 'bg-green-900/50 text-green-400 border-green-700' : 'bg-red-900/50 text-red-400 border-red-700 animate-pulse'}`}>
                        {wsConnected ? '● WS CONNECTED' : '○ WS DISCONNECTED'}
                    </div>
                    <div className="px-3 py-1 bg-black/60 border border-cyan-900 text-sm font-bold text-cyan-400">
                        [{currStateId}] {currState.name}
                    </div>
                </div>
            </header>

            <div className="flex-1 flex gap-3 overflow-hidden">

                {/* 左側：遙測 + 3D + GPS */}
                <div className="w-1/3 flex flex-col gap-3">
                    <div className="grid grid-cols-2 gap-2">
                        <TeleCard label="Alt"     val={alt.toFixed(3)} unit="m"   />
                        <TeleCard label="Vel"     val={vel.toFixed(3)} unit="m/s" />
                        <TeleCard label="Accel"   val={acc.toFixed(3)} unit="g"   />
                        <TeleCard label="Packets" val={Number(view?.pkt ?? 0).toFixed(3)} unit="idx" />
                    </div>
                    <div className="glass-panel flex-1 flex flex-col items-center justify-center relative min-h-[140px]">
                        <div className="text-sm text-cyan-400 font-bold uppercase absolute top-2 left-2 flex flex-col gap-0.5 z-10">
                            <span><span className="text-gray-500">QW:</span> {q[0].toFixed(3)}</span>
                            <span><span className="text-gray-500">QX:</span> {q[1].toFixed(3)}</span>
                            <span><span className="text-gray-500">QY:</span> {q[2].toFixed(3)}</span>
                            <span><span className="text-gray-500">QZ:</span> {q[3].toFixed(3)}</span>
                        </div>
                        <Rocket3D q={q} />
                        <div className="text-xs text-gray-600 uppercase absolute bottom-2">Live Orientation</div>
                    </div>
                    {/* GPS MAP */}
                    <div className="glass-panel h-[160px] flex flex-col p-2 relative overflow-hidden">
                        <div className="text-sm text-cyan-400 font-bold uppercase mb-1 flex justify-between">
                            <span>GPS MAP</span>
                            <span className="text-gray-500 font-mono text-xs">{gps.lat !== 0 ? `${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}` : 'NO FIX'}</span>
                        </div>
                        <div className="flex-1 rounded border border-cyan-900/30 overflow-hidden relative bg-black/50 z-0">
                            <GpsMap gps={gps} trajectory={trajectory} />
                        </div>
                    </div>
                </div>

                {/* 中間：Raw Sensors / 串流 / FC Log / GND CMD Log */}
                <div className="flex-1 flex flex-col gap-3 overflow-hidden">
                    
                    {/* Raw Sensor Telemetry */}
                    <div className="glass-panel p-2 flex flex-col gap-1 shrink-0">
                        <div className="p-1 bg-cyan-900/10 border-b border-cyan-900/20 text-xs font-bold text-cyan-400 uppercase mb-1">Raw Sensor Telemetry</div>
                        <div className="grid grid-cols-3 gap-2">
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="ACC_X" val={imu?.ax?.toFixed(3) ?? '0.000'} unit="g" />
                                <SensorValue label="ACC_Y" val={imu?.ay?.toFixed(3) ?? '0.000'} unit="g" />
                                <SensorValue label="ACC_Z" val={imu?.az?.toFixed(3) ?? '0.000'} unit="g" />
                            </div>
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="GYR_X" val={imu?.gx?.toFixed(3) ?? '0.000'} unit="°/s" />
                                <SensorValue label="GYR_Y" val={imu?.gy?.toFixed(3) ?? '0.000'} unit="°/s" />
                                <SensorValue label="GYR_Z" val={imu?.gz?.toFixed(3) ?? '0.000'} unit="°/s" />
                            </div>
                            <div className="flex flex-col gap-0.5">
                                <SensorValue label="MAG_X" val={imu?.mx?.toFixed(3) ?? '0.000'} unit="uT" />
                                <SensorValue label="MAG_Y" val={imu?.my?.toFixed(3) ?? '0.000'} unit="uT" />
                                <SensorValue label="MAG_Z" val={imu?.mz?.toFixed(3) ?? '0.000'} unit="uT" />
                            </div>
                        </div>
                        <div className="grid grid-cols-2 gap-2 mt-1">
                             <div className="flex flex-col gap-0.5">
                                <SensorValue label="BMP_PRES" val={bmp?.pressure?.toFixed(3) ?? '0.000'} unit="hPa" />
                                <SensorValue label="BMP_TEMP" val={bmp?.temp?.toFixed(3) ?? '0.000'} unit="°C" />
                             </div>
                             <div className="flex flex-col gap-0.5">
                                <SensorValue label="GPS_LAT" val={gpsRaw?.lat?.toFixed(6) ?? '0.000000'} unit="°" />
                                <SensorValue label="GPS_LON" val={gpsRaw?.lon?.toFixed(6) ?? '0.000000'} unit="°" />
                             </div>
                        </div>
                    </div>

                    <div className="glass-panel flex flex-col overflow-hidden shrink-0" style={{height:"200px"}}>
                        <div className="p-1.5 bg-purple-900/10 border-b border-purple-900/20 text-sm font-bold text-purple-400 uppercase">Flight Dynamics</div>
                        <div className="flex-1 p-2 w-full h-full text-xs">
                            <ResponsiveContainer width="100%" height="100%">
                                <LineChart data={chartData} margin={{ top: 15, right: 10, left: -20, bottom: 0 }}>
                                    <XAxis 
                                        dataKey="time" 
                                        type="number" 
                                        domain={['dataMin', 'dataMax']} 
                                        allowDecimals={false}
                                        tickFormatter={(v) => {
                                            const point = chartData.find(d => d.time === v);
                                            if (point && point.tsSec) return `${point.tsSec.toFixed(1)}s [${v}]`;
                                            return `[${v}]`;
                                        }} 
                                        tick={{fill: '#6b7280', fontSize: 10}} 
                                    />
                                    <YAxis yAxisId="left" domain={['auto', 'auto']} tick={{fill: '#6b7280'}} />
                                    <YAxis yAxisId="right" orientation="right" domain={['auto', 'auto']} tick={{fill: '#6b7280'}} />
                                    <Tooltip 
                                        contentStyle={{backgroundColor: 'rgba(0,0,0,0.8)', border: '1px solid #1e3a8a', borderRadius: '4px'}} 
                                        itemStyle={{fontSize: 12, padding: 0}}
                                        labelFormatter={(label) => {
                                            const point = chartData.find(d => d.time === label);
                                            if (point && point.tsSec) return `Time: ${point.tsSec.toFixed(2)}s [Pkt: ${label}]`;
                                            return `Pkt: ${label}`;
                                        }}
                                    />
                                    <Legend wrapperStyle={{fontSize: '11px', paddingTop: '4px'}} />
                                    
                                    {/* 繪製狀態切換時間線 */}
                                    {stateEvents.map((ev, idx) => {
                                        // 找出 chartData 中最接近這個事件 tsSec 的 packet index (time)
                                        let bestIdx = -1;
                                        let minDiff = Infinity;
                                        for (const d of chartData) {
                                            if (!d.tsSec) continue;
                                            const diff = Math.abs(d.tsSec - ev.tsSec);
                                            if (diff < minDiff) {
                                                minDiff = diff;
                                                bestIdx = d.time;
                                            }
                                        }
                                        if (bestIdx !== -1 && minDiff < 2.0) {
                                            return (
                                                <ReferenceLine key={idx} x={bestIdx} yAxisId="left" stroke="rgba(251, 191, 36, 0.8)" strokeDasharray="3 3" strokeWidth={2}>
                                                    <Label value={ev.name} position="insideTopLeft" fill="#fbbf24" fontSize={11} offset={5} />
                                                </ReferenceLine>
                                            );
                                        }
                                        return null;
                                    })}

                                    <Line yAxisId="left" type="monotone" dataKey="alt" stroke="#06b6d4" strokeWidth={2} dot={false} isAnimationActive={false} name="Alt (m)" />
                                    <Line yAxisId="right" type="monotone" dataKey="vel" stroke="#f59e0b" strokeWidth={2} dot={false} isAnimationActive={false} name="Vel (m/s)" />
                                    <Line yAxisId="right" type="monotone" dataKey="acc" stroke="#ef4444" strokeWidth={2} dot={false} isAnimationActive={false} name="Acc (g)" />
                                </LineChart>
                            </ResponsiveContainer>
                        </div>
                    </div>

                    <div className="glass-panel flex-1 p-2 overflow-y-auto font-mono text-sm custom-scrollbar">
                        <div className="text-xs text-gray-600 mb-1 font-bold">FC EVENT LOGS</div>
                        {fcLogs.length === 0
                            ? <div className="text-gray-800 text-xs">— 尚無資料 —</div>
                            : (
                                <table className="w-full text-left border-collapse text-xs">
                                    <thead>
                                        <tr className="border-b border-gray-700/50 text-gray-500">
                                            <th className="py-1 pr-2 w-16">Time</th>
                                            <th className="py-1">Event</th>
                                        </tr>
                                    </thead>
                                    <tbody>
                                        {fcLogs.map((log, i) => (
                                            <tr key={i} className="border-b border-gray-800/30 hover:bg-gray-800/20">
                                                <td className="py-1 pr-2 text-gray-500">{log.time}</td>
                                                <td className="py-1 text-cyan-300">{log.msg}</td>
                                            </tr>
                                        ))}
                                    </tbody>
                                </table>
                            )
                        }
                    </div>

                    <div className="glass-panel p-2 overflow-y-auto custom-scrollbar" style={{height:"100px"}}>
                        <div className="text-xs text-yellow-600 mb-1 font-bold uppercase">GND CMD Log</div>
                        {cmdLog.length === 0
                            ? <div className="text-gray-800 text-xs">— 等待操作 —</div>
                            : cmdLog.map((m,i) => <div key={i} className="text-yellow-500/70 text-xs">{m}</div>)
                        }
                    </div>
                </div>

                {/* 右側：狀態控制面板 */}
                <div className="w-1/4 flex flex-col gap-3 relative">
                    {!isLive && (
                        <div className="absolute inset-0 bg-black/80 z-20 flex items-center justify-center">
                            <button onClick={() => setIsLive(true)} className="px-4 py-1 bg-yellow-600 text-black font-bold text-sm rounded">RETURN LIVE</button>
                        </div>
                    )}

                    {/* Mission Sequence */}
                    <div className="glass-panel flex flex-col overflow-hidden flex-1">
                        <div className="p-2 bg-cyan-900/10 border-b border-cyan-900/20 text-sm font-bold text-cyan-500 uppercase">Mission Sequence</div>
                        <div className="p-2 flex flex-col gap-2 overflow-y-auto custom-scrollbar flex-1">
                            {normalNext.length === 0 && <div className="text-xs text-gray-700 text-center py-2 border border-dashed border-gray-800">NO NEXT STEP</div>}
                            {normalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-sequence w-full py-2">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {debugNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-debug w-full py-1.5">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                        </div>
                    </div>

                    {/* Critical Operations */}
                    <div className="glass-panel flex flex-col overflow-hidden" style={{minHeight:"150px"}}>
                        <div className="p-2 bg-red-900/5 border-b border-red-900/20 text-sm font-bold text-red-500 uppercase">⚠ Critical Operations</div>
                        <div className="p-2 flex flex-col gap-2 flex-1">
                            {criticalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-emergency w-full py-3">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {criticalNext.length === 0 && currStateId !== 14 && (
                                <div className="text-xs text-gray-700 text-center py-2 border border-dashed border-gray-800">— 無緊急操作 —</div>
                            )}
                            {currStateId === 14 && (
                                <div className="text-xs text-red-900 text-center py-4 border border-dashed border-red-900">☠ MISSION TERMINATED</div>
                            )}
                            {currStateId !== 14 && (
                                <button id="btn-force-terminate" onClick={() => sendStateCommand(14)}
                                    className="btn-base btn-emergency w-full py-1.5 mt-auto opacity-50 hover:opacity-100 text-sm">
                                    ☠ FORCE TERMINATE [14]
                                </button>
                            )}
                        </div>
                    </div>
                </div>
            </div>

            {/* Timeline Scrubber */}
            <div className="glass-panel p-2 flex items-center gap-3 h-9 shrink-0">
                <span className="text-xs text-gray-600 shrink-0">TIMELINE</span>
                <input type="range" min="0" max={Math.max(0, history.length-1)} value={histIdx}
                    onChange={e => { setIsLive(false); setHistIdx(parseInt(e.target.value)); }}
                    className="w-full h-1" />
                <span className="text-xs text-cyan-400 shrink-0 font-bold tracking-widest whitespace-nowrap">
                    SEQ: {currentPkt} ({currentScrubberTsSec.toFixed(1)}s)
                </span>
                
                {/* CSV Export Controls */}
                <button onClick={() => setShowExport(!showExport)} className="px-2 py-0.5 bg-blue-900/50 text-blue-400 border border-blue-700 rounded text-xs font-bold hover:bg-blue-800/50 transition-colors">
                    CSV
                </button>
                {showExport && (
                    <div className="flex items-center gap-2 border-l border-gray-700 pl-2 shrink-0">
                        <input type="number" value={exportStart} onChange={e => setExportStart(Number(e.target.value))} className="w-14 bg-black/80 text-cyan-400 border border-cyan-900 text-xs px-1 outline-none text-center rounded" placeholder="Start" title="Start Time (s)" />
                        <span className="text-gray-500 text-xs">-</span>
                        <input type="number" value={exportEnd} onChange={e => setExportEnd(Number(e.target.value))} className="w-14 bg-black/80 text-cyan-400 border border-cyan-900 text-xs px-1 outline-none text-center rounded" placeholder="End" title="End Time (s)" />
                        <span className="text-gray-500 text-xs">s</span>
                        <button onClick={() => downloadCSV(exportStart, exportEnd)} className="px-3 py-0.5 bg-green-900/50 text-green-400 border border-green-700 rounded text-xs font-bold hover:bg-green-800/50 transition-colors ml-1">
                            匯出
                        </button>
                    </div>
                )}
            </div>
        </div>
    );
}

export default App;
