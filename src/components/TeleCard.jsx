export const TeleCard = ({ label, val, unit }) => (
    <div className="glass-panel p-2.5 h-16 flex flex-col justify-between border-l-2 border-l-[#00f2fe] bg-slate-900/40 hover:bg-slate-900/60 transition-colors">
        <div className="text-[10px] text-slate-500 font-bold uppercase tracking-wider">{label}</div>
        <div className="flex items-baseline gap-1 leading-none">
            <span className="text-lg font-bold font-mono text-cyan-100">{val}</span>
            <span className="text-[10px] text-cyan-600 uppercase font-bold tracking-tight">{unit}</span>
        </div>
    </div>
);
