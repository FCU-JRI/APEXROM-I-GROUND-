import { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { MapContainer, TileLayer, Polyline, Marker, useMap } from 'react-leaflet';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend, ReferenceLine, Label } from 'recharts';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import mqtt from 'mqtt';
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

const getLogMsgColorClass = (msg) => {
    const upper = msg.toUpperCase();
    if (upper.includes("ERROR") || upper.includes("FAILED")) return "text-red-400 font-bold";
    if (upper.includes("TRIGGER") || upper.includes("DECISION")) return "text-amber-400";
    if (upper.includes("ACTION")) return "text-cyan-400 font-semibold";
    if (upper.includes("EVENT")) return "text-emerald-400";
    return "text-slate-300";
};

function App() {
    const [currStateId, setCurrStateId] = useState(0);
    const defaultIp = import.meta.env.VITE_MQTT_BROKER_IP || window.location.hostname || "localhost";
    const [mqttUrls,    setMqttUrls]    = useState([`ws://${defaultIp}:9001`]);
    const [mqttStatuses,setMqttStatuses]= useState({});
    const [showWsModal, setShowWsModal] = useState(false);
    const [isLive,      setIsLive]      = useState(true);
    const [histIdx,     setHistIdx]     = useState(0);
    const [history,     setHistory]     = useState([]);
    const [cmdLog,      setCmdLog]      = useState([]);
    const [fcLogs,      setFcLogs]      = useState([]);
    const [stateEvents, setStateEvents] = useState([]);
    const [showExport,  setShowExport]  = useState(false);
    const [exportStart, setExportStart] = useState(0);
    const [exportEnd,   setExportEnd]   = useState(600);
    const [boardFreq,   setBoardFreq]   = useState(null);
    const mqttRefs = useRef({});
    const seenPkts = useRef(new Set());

    const addCmdLog = useCallback((msg) => {
        const t = new Date().toLocaleTimeString('zh-TW', { hour12:false });
        setCmdLog(prev => [`[${t}] ${msg}`, ...prev].slice(0, 100));
    }, []);

    // MQTT — 支援多個 Broker 連線並進行封包去重
    useEffect(() => {
        let isMounted = true;

        const connect = (url) => {
            if (!isMounted) return;
            const client = mqtt.connect(url, {
                keepalive: 30,
                reconnectPeriod: 3000
            });
            mqttRefs.current[url] = client;
            
            client.on('connect', () => { 
                if (isMounted) { 
                    setMqttStatuses(prev => ({ ...prev, [url]: true }));  
                    addCmdLog(`✅ 已連線至 MQTT Broker: ${url}`); 
                    client.subscribe('fc/telemetry');
                    client.publish('fc/cmd', JSON.stringify({ type: "cmd", action: "queryFreq" }));
                } 
            });
            client.on('offline', () => { 
                if (isMounted) { 
                    setMqttStatuses(prev => ({ ...prev, [url]: false })); 
                    addCmdLog(`❌ 斷線 ${url}，自動重連中…`); 
                }
            });
            client.on('error', () => {});
            client.on('message', (topic, message) => {
                if (!isMounted || topic !== 'fc/telemetry') return;
                try {
                    const d = JSON.parse(message.toString());
                    if (d.type === 'status') {
                        if (d.board_freq !== undefined && d.board_freq !== null) {
                            setBoardFreq(d.board_freq);
                        }
                        return;
                    }
                    if (d.batch) {
                        if (d.board_freq !== undefined && d.board_freq !== null) {
                            setBoardFreq(d.board_freq);
                        }
                        const pktId = d.pkt !== undefined ? d.pkt : Math.max(...d.batch.map(b => b.ts || 0));
                        if (seenPkts.current.has(pktId)) return; // 發現重複封包，丟棄以去重
                        seenPkts.current.add(pktId);
                        
                        // 避免 Set 無限增長
                        if (seenPkts.current.size > 5000) {
                            const arr = Array.from(seenPkts.current).slice(-2500);
                            seenPkts.current = new Set(arr);
                        }

                        setHistory(h => {
                            const newH = [...h, d].sort((a,b) => (a.pkt || 0) - (b.pkt || 0));
                            return newH.slice(-100000);
                        });
                        
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
            });
        };
        
        mqttUrls.forEach(url => connect(url));

        return () => { 
            isMounted = false;
            mqttUrls.forEach(url => {
                if (mqttRefs.current[url]) {
                    mqttRefs.current[url].end(true); 
                    delete mqttRefs.current[url];
                }
            });
        };
    }, [mqttUrls, addCmdLog]);

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
        
        let sentCount = 0;
        Object.values(mqttRefs.current).forEach(client => {
            if (client && client.connected) {
                client.publish('fc/cmd', JSON.stringify({ type:"cmd", action:"setState", stateId }));
                sentCount++;
            }
        });

        if (sentCount > 0) {
            addCmdLog(`📡 [GND→FC] 切換至 ${s.name} (${stateId}) (透過 MQTT 發送)`);
        } else {
            addCmdLog("❌ 無任何 MQTT 連線，無法發送指令");
        }
    }, [addCmdLog]);

    const handleSetFreq = useCallback((freq) => {
        let sentCount = 0;
        Object.values(mqttRefs.current).forEach(client => {
            if (client && client.connected) {
                client.publish('fc/cmd', JSON.stringify({ type: "cmd", action: "setFreq", frequency: freq }));
                sentCount++;
            }
        });

        if (sentCount > 0) {
            addCmdLog(`📡 [GND→RX] 發送切換頻率指令: ${freq / 1000000} MHz`);
        } else {
            addCmdLog("❌ 無任何 MQTT 連線，無法發送切換頻率指令");
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
    const rssi   = view?.rssi;
    
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

    const connectedCount = Object.values(mqttStatuses).filter(Boolean).length;
    const totalCount = mqttUrls.length;
    const wsStatusClass = connectedCount === totalCount && totalCount > 0 
        ? 'bg-green-900/50 text-green-400 border-green-700' 
        : (connectedCount > 0 ? 'bg-yellow-900/50 text-yellow-400 border-yellow-700' : 'bg-red-900/50 text-red-400 border-red-700 animate-pulse');

    return (
        <div className="h-full flex flex-col gap-3 relative">
            <div className="scanline"></div>

            {/* Header */}
            <header className="glass-panel p-2 flex justify-between items-center glow-cyan bg-slate-950/40">
                <div className="text-lg font-bold text-neon uppercase tracking-tighter">JRI P2026 GND V2.1</div>
                <div className="flex gap-4 items-center">
                    {/* Frequency Band Selection */}
                    <div className="flex items-center gap-2.5 px-2.5 py-1 bg-slate-950/80 border border-slate-800/80 rounded-md text-xs">
                        <span className="text-slate-400 font-bold uppercase tracking-wider text-[10px]">RX BAND:</span>
                        <div className="segment-group">
                            <div 
                                className={`segment-item ${boardFreq === 433000000 ? 'active' : ''}`}
                                onClick={() => handleSetFreq(433000000)}
                            >
                                433M
                            </div>
                            <div 
                                className={`segment-item ${boardFreq === 915000000 ? 'active' : ''}`}
                                onClick={() => handleSetFreq(915000000)}
                            >
                                915M
                            </div>
                        </div>
                    </div>

                    {!isLive && <div className="text-xs text-yellow-500 font-bold animate-pulse bg-yellow-950/30 border border-yellow-800/40 px-2 py-0.5 rounded uppercase">HISTORICAL PLAYBACK</div>}
                    <div id="ws-status" className={`px-2.5 py-1 rounded text-xs font-bold border cursor-pointer hover:opacity-80 transition-opacity ${wsStatusClass}`} onClick={() => setShowWsModal(true)} title="點擊管理多個地面站 MQTT Broker">
                        {connectedCount > 0 ? `● MQTT CONNECTED (${connectedCount}/${totalCount})` : '○ MQTT DISCONNECTED'}
                    </div>
                    <div className="px-3 py-1.5 lcd-panel text-xs font-bold text-cyan-400 font-mono tracking-wide">
                        STATUS: [{currStateId}] {currState.name}
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
                        <TeleCard label="Packets" val={Number(view?.pkt ?? 0).toFixed(0)} unit="idx" />
                        <div className="col-span-2">
                            <TeleCard label="RSSI" val={rssi !== undefined ? `${rssi}` : 'N/A'} unit="dBm" />
                        </div>
                    </div>
                    <div className="glass-panel flex-1 flex flex-col items-center justify-center relative min-h-[140px] grid-radar">
                        <div className="text-[10px] text-cyan-400/80 font-mono uppercase absolute top-2 left-2 flex flex-col gap-0.5 z-10 bg-slate-950/75 p-1 rounded border border-slate-800/50">
                            <span><span className="text-slate-500">QW:</span> {q[0].toFixed(3)}</span>
                            <span><span className="text-slate-500">QX:</span> {q[1].toFixed(3)}</span>
                            <span><span className="text-slate-500">QY:</span> {q[2].toFixed(3)}</span>
                            <span><span className="text-slate-500">QZ:</span> {q[3].toFixed(3)}</span>
                        </div>
                        <Rocket3D q={q} />
                        <div className="text-[9px] text-slate-500 font-bold uppercase absolute bottom-2 tracking-wider">Live Orientation Scope</div>
                    </div>
                    {/* GPS MAP */}
                    <div className="glass-panel h-[160px] flex flex-col p-2 relative overflow-hidden">
                        <div className="text-[10px] text-cyan-400 font-bold uppercase mb-1 flex justify-between tracking-wider">
                            <span>GPS MAP</span>
                            <span className="text-slate-500 font-mono text-xs">{gps.lat !== 0 ? `${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}` : 'NO FIX'}</span>
                        </div>
                        <div className="flex-1 rounded border border-slate-800/40 overflow-hidden relative bg-black/50 z-0">
                            <GpsMap gps={gps} trajectory={trajectory} />
                        </div>
                    </div>
                </div>

                {/* 中間：Raw Sensors / 串流 / FC Log / GND CMD Log */}
                <div className="flex-1 flex flex-col gap-3 overflow-hidden">
                    
                    {/* Raw Sensor Telemetry */}
                    <div className="glass-panel p-2 flex flex-col gap-1 shrink-0">
                        <div className="p-1 bg-cyan-950/20 border-b border-cyan-900/10 text-[10px] font-bold text-cyan-400 uppercase mb-1 tracking-wider">Raw Sensor Telemetry</div>
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
                        <div className="p-1.5 bg-slate-950/40 border-b border-slate-800/60 text-xs font-bold text-cyan-400 uppercase tracking-wider">Flight Dynamics</div>
                        <div className="flex-1 p-2 w-full h-full text-xs">
                            <ResponsiveContainer width="100%" height="100%">
                                <LineChart data={chartData} margin={{ top: 15, right: 10, left: -20, bottom: 0 }}>
                                    <CartesianGrid stroke="#1e293b" strokeDasharray="3 3" opacity={0.4} />
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
                                        tick={{fill: '#64748b', fontSize: 9, fontFamily: 'var(--font-mono)'}} 
                                    />
                                    <YAxis yAxisId="left" domain={['auto', 'auto']} tick={{fill: '#64748b', fontSize: 9, fontFamily: 'var(--font-mono)'}} />
                                    <YAxis yAxisId="right" orientation="right" domain={['auto', 'auto']} tick={{fill: '#64748b', fontSize: 9, fontFamily: 'var(--font-mono)'}} />
                                    <Tooltip 
                                        contentStyle={{
                                            backgroundColor: 'rgba(9, 15, 30, 0.95)', 
                                            border: '1px solid rgba(0, 242, 254, 0.3)', 
                                            borderRadius: '4px',
                                            boxShadow: '0 0 10px rgba(0,0,0,0.8)'
                                        }} 
                                        labelStyle={{ color: '#00f2fe', fontFamily: 'var(--font-mono)', fontWeight: 'bold' }}
                                        itemStyle={{ fontSize: 11, padding: 0 }}
                                        labelFormatter={(label) => {
                                            const point = chartData.find(d => d.time === label);
                                            if (point && point.tsSec) return `Time: ${point.tsSec.toFixed(2)}s [Pkt: ${label}]`;
                                            return `Pkt: ${label}`;
                                        }}
                                    />
                                    <Legend wrapperStyle={{fontSize: '10px', paddingTop: '4px', fontFamily: 'var(--font-sans)'}} />
                                    
                                    {/* 繪製狀態切換時間線 */}
                                    {stateEvents.map((ev, idx) => {
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
                                                <ReferenceLine key={idx} x={bestIdx} yAxisId="left" stroke="rgba(245, 158, 11, 0.8)" strokeDasharray="3 3" strokeWidth={1.5}>
                                                    <Label value={ev.name} position="insideTopLeft" fill="#f59e0b" fontSize={9} offset={5} />
                                                </ReferenceLine>
                                            );
                                        }
                                        return null;
                                    })}

                                    <Line yAxisId="left" type="monotone" dataKey="alt" stroke="#00f2fe" strokeWidth={2} dot={false} isAnimationActive={false} name="Alt (m)" />
                                    <Line yAxisId="right" type="monotone" dataKey="vel" stroke="#f59e0b" strokeWidth={2} dot={false} isAnimationActive={false} name="Vel (m/s)" />
                                    <Line yAxisId="right" type="monotone" dataKey="acc" stroke="#ef4444" strokeWidth={2} dot={false} isAnimationActive={false} name="Acc (g)" />
                                </LineChart>
                            </ResponsiveContainer>
                        </div>
                    </div>

                    <div className="glass-panel flex-1 p-2 overflow-y-auto font-mono text-xs custom-scrollbar">
                        <div className="text-[10px] text-slate-500 mb-1.5 font-bold uppercase tracking-wider">FC EVENT LOGS</div>
                        {fcLogs.length === 0
                            ? <div className="text-slate-700 text-xs">— 尚無資料 —</div>
                            : (
                                <table className="w-full text-left border-collapse text-[11px] font-mono">
                                    <thead>
                                        <tr className="border-b border-slate-800 text-slate-500">
                                            <th className="py-1 pr-2 w-16">Time</th>
                                            <th className="py-1">Event</th>
                                        </tr>
                                    </thead>
                                    <tbody>
                                        {fcLogs.map((log, i) => (
                                            <tr key={i} className="border-b border-slate-900/30 hover:bg-slate-900/10">
                                                <td className="py-1 pr-2 text-slate-500">{log.time}</td>
                                                <td className={`py-1 ${getLogMsgColorClass(log.msg)}`}>{log.msg}</td>
                                            </tr>
                                        ))}
                                    </tbody>
                                </table>
                            )
                        }
                    </div>

                    <div className="glass-panel p-2 overflow-y-auto custom-scrollbar" style={{height:"100px"}}>
                        <div className="text-[10px] text-amber-500/80 mb-1 font-bold uppercase tracking-wider">GND CMD Log</div>
                        {cmdLog.length === 0
                            ? <div className="text-slate-800 text-xs">— 等待操作 —</div>
                            : cmdLog.map((m,i) => <div key={i} className="text-amber-500/70 text-xs font-mono">{m}</div>)
                        }
                    </div>
                </div>

                {/* 右側：狀態控制面板 */}
                <div className="w-1/4 flex flex-col gap-3 relative">
                    {!isLive && (
                        <div className="absolute inset-0 bg-slate-950/80 z-20 flex items-center justify-center rounded-md">
                            <button onClick={() => setIsLive(true)} className="px-4 py-2 bg-yellow-600 hover:bg-yellow-500 text-black font-bold text-xs rounded transition-colors tracking-wide uppercase">RETURN LIVE</button>
                        </div>
                    )}

                    {/* Mission Sequence */}
                    <div className="glass-panel flex flex-col overflow-hidden flex-1">
                        <div className="p-2 bg-slate-950/40 border-b border-slate-800/60 text-xs font-bold text-cyan-400 uppercase tracking-wider">Mission Control</div>
                        
                        {/* Current State LCD display */}
                        <div className="p-2 px-2.5 m-2 lcd-panel flex flex-col gap-1.5">
                            <span className="text-[9px] text-slate-500 uppercase font-bold tracking-wider">Current State</span>
                            <div className="flex justify-between items-center">
                                <span className="text-xs font-mono text-cyan-400 font-bold">[{currStateId}] {currState.name}</span>
                                <span className="text-[10px] bg-cyan-950/70 text-cyan-300 border border-cyan-800/40 px-1.5 py-0.5 rounded font-bold">{currState.label}</span>
                            </div>
                        </div>

                        <div className="p-2 pt-0 flex flex-col gap-2 overflow-y-auto custom-scrollbar flex-1">
                            <div className="text-[9px] text-slate-500 font-bold uppercase tracking-wider mb-1">Available Actions</div>
                            {normalNext.length === 0 && debugNext.length === 0 && (
                                <div className="text-xs text-slate-600 text-center py-4 border border-dashed border-slate-800 rounded">NO NEXT TRANSITION AVAILABLE</div>
                            )}
                            {normalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-sequence w-full py-1.5 text-xs">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {debugNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-debug w-full py-1.5 text-[11px]">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                        </div>
                    </div>

                    {/* Critical Operations */}
                    <div className="glass-panel flex flex-col overflow-hidden" style={{minHeight:"150px"}}>
                        <div className="p-2 bg-red-950/20 border-b border-red-900/20 text-xs font-bold text-red-500 uppercase tracking-wider">⚠ Critical Operations</div>
                        <div className="p-2 flex flex-col gap-2 flex-1 justify-center">
                            {criticalNext.map(s => (
                                <button key={s.id} id={`btn-state-${s.id}`} onClick={() => sendStateCommand(s.id)} className="btn-base btn-emergency w-full py-2.5 text-xs">
                                    [{s.id}] {s.label}
                                </button>
                            ))}
                            {criticalNext.length === 0 && currStateId !== 9 && (
                                <div className="text-xs text-slate-600 text-center py-4 border border-dashed border-slate-800 rounded">— 無緊急操作 —</div>
                            )}
                            {currStateId === 9 && (
                                <>
                                    <div className="text-xs text-red-400 font-bold text-center py-2 border border-dashed border-red-900 rounded bg-red-950/10">☠ MISSION TERMINATED</div>
                                    <button id="btn-reset-idle" onClick={() => sendStateCommand(0)}
                                        className="btn-base btn-sequence w-full py-2 text-xs font-bold bg-amber-950/40 border border-amber-800/40 hover:bg-amber-900/40 text-amber-400 rounded transition-colors uppercase tracking-wider">
                                        🔄 RESET TO IDLE [0]
                                    </button>
                                </>
                            )}
                            {currStateId !== 9 && (
                                <button id="btn-force-terminate" onClick={() => sendStateCommand(9)}
                                    className="btn-base btn-emergency w-full py-1.5 mt-auto opacity-50 hover:opacity-100 text-xs transition-opacity duration-300">
                                    ☠ FORCE TERMINATE [9]
                                </button>
                            )}
                        </div>
                    </div>
                </div>
            </div>

            {/* Timeline Scrubber */}
            <div className="glass-panel p-2 flex items-center gap-3 h-9 shrink-0">
                <span className="text-[10px] text-slate-500 font-bold tracking-wider shrink-0">TIMELINE</span>
                <input type="range" min="0" max={Math.max(0, history.length-1)} value={histIdx}
                    onChange={e => { setIsLive(false); setHistIdx(parseInt(e.target.value)); }}
                    className="w-full h-1 cyber-slider" />
                <span className="text-xs text-cyan-400 shrink-0 font-mono font-bold tracking-wider whitespace-nowrap bg-slate-950/80 px-2 py-0.5 border border-slate-800 rounded">
                    SEQ: {currentPkt} ({currentScrubberTsSec.toFixed(1)}s)
                </span>
                
                {/* CSV Export Controls */}
                <button onClick={() => setShowExport(!showExport)} className="px-2.5 py-0.5 bg-slate-950/80 text-slate-300 border border-slate-800 rounded text-xs font-bold hover:text-cyan-400 hover:border-cyan-800 transition-colors">
                    CSV
                </button>
                {showExport && (
                    <div className="flex items-center gap-2 border-l border-slate-800 pl-2 shrink-0">
                        <input type="number" value={exportStart} onChange={e => setExportStart(Number(e.target.value))} className="w-14 bg-slate-950 text-cyan-400 border border-slate-800 text-xs px-1.5 py-0.5 outline-none text-center rounded font-mono" placeholder="Start" title="Start Time (s)" />
                        <span className="text-slate-600 text-xs">-</span>
                        <input type="number" value={exportEnd} onChange={e => setExportEnd(Number(e.target.value))} className="w-14 bg-slate-950 text-cyan-400 border border-slate-800 text-xs px-1.5 py-0.5 outline-none text-center rounded font-mono" placeholder="End" title="End Time (s)" />
                        <span className="text-slate-500 text-[10px] uppercase font-bold">sec</span>
                        <button onClick={() => downloadCSV(exportStart, exportEnd)} className="px-3 py-0.5 bg-cyan-950 text-cyan-400 border border-cyan-800/80 rounded text-xs font-bold hover:bg-cyan-800 hover:text-white transition-colors ml-1">
                            EXPORT
                        </button>
                    </div>
                )}
            </div>

            {/* MQTT Manager Modal */}
            {showWsModal && (
                <div className="absolute inset-0 bg-black/80 z-50 flex items-center justify-center">
                    <div className="glass-panel p-4 max-w-md w-full flex flex-col gap-3">
                        <h3 className="text-cyan-400 font-bold uppercase border-b border-cyan-900 pb-2">📡 追蹤多個 MQTT Broker</h3>
                        <p className="text-xs text-gray-400 mb-2">可新增多個 Ground Station 內建的 MQTT Broker，系統將根據封包編號自動過濾重複資料，防止掉包斷線。</p>
                        
                        <div className="flex flex-col gap-2 max-h-60 overflow-y-auto custom-scrollbar pr-1">
                            {mqttUrls.map((url, i) => (
                                <div key={i} className="flex gap-2 items-center bg-black/40 p-1.5 rounded border border-gray-800">
                                    <div className={`w-2.5 h-2.5 rounded-full ${mqttStatuses[url] ? 'bg-green-500 shadow-[0_0_8px_#22c55e]' : 'bg-red-500 shadow-[0_0_8px_#ef4444]'}`}></div>
                                    <input type="text" value={url} readOnly className="flex-1 bg-transparent border-none outline-none text-xs text-gray-300 font-mono" />
                                    <button onClick={() => {
                                        setMqttUrls(urls => urls.filter((_, idx) => idx !== i));
                                    }} className="px-2 py-1 bg-red-900/30 hover:bg-red-900/80 text-red-400 text-xs rounded border border-red-800 transition-colors">移除</button>
                                </div>
                            ))}
                        </div>
                        
                        <div className="flex gap-2 mt-2">
                            <input type="text" id="new-ws-url" placeholder={`ws://${window.location.hostname || "192.168.x.x"}:9001`} className="flex-1 bg-black/80 border border-cyan-900 focus:border-cyan-500 outline-none text-cyan-400 text-xs px-2 py-1.5 rounded font-mono" 
                                onKeyDown={(e) => {
                                    if (e.key === 'Enter') document.getElementById('btn-add-ws').click();
                                }}
                            />
                            <button id="btn-add-ws" onClick={() => {
                                const input = document.getElementById('new-ws-url');
                                const val = input.value.trim();
                                if (val && !mqttUrls.includes(val)) {
                                    setMqttUrls(urls => [...urls, val]);
                                    input.value = '';
                                }
                            }} className="px-4 py-1.5 bg-cyan-900/50 hover:bg-cyan-800 text-cyan-400 text-xs font-bold rounded border border-cyan-700 transition-colors">新增</button>
                        </div>
                        
                        <div className="mt-4 pt-3 border-t border-gray-800 flex justify-end">
                            <button onClick={() => setShowWsModal(false)} className="px-6 py-2 bg-gray-800 hover:bg-gray-700 text-gray-300 text-sm font-bold rounded transition-colors">完成</button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}

export default App;
