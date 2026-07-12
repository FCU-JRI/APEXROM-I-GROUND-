export const TeleCard = ({ label, val, unit }) => (
    <div className="glass-panel p-2 h-14 flex flex-col justify-between border-l border-l-cyan-600">
        <div className="text-xs text-gray-600 font-bold uppercase">{label}</div>
        <div className="flex items-baseline gap-0.5 leading-none">
            <span className="text-base font-bold">{val}</span>
            <span className="text-xs text-cyan-900">{unit}</span>
        </div>
    </div>
);
