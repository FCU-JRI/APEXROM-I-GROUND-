export const SensorValue = ({ label, val, unit }) => (
    <div className="flex justify-between items-center bg-slate-950/60 px-2 py-1 rounded border border-slate-800/50 hover:border-slate-700/60 transition-colors">
        <span className="text-[10px] text-slate-500 font-bold tracking-wider">{label}</span>
        <span className="text-xs font-mono text-cyan-100 font-medium">
            {val} <span className="text-cyan-700 font-sans font-bold text-[9px] uppercase tracking-tight ml-0.5">{unit}</span>
        </span>
    </div>
);
