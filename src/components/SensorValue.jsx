export const SensorValue = ({ label, val, unit }) => (
    <div className="flex justify-between items-center bg-black/40 px-1.5 py-0.5 rounded border border-gray-800/50">
        <span className="text-xs text-gray-500 font-bold">{label}</span>
        <span className="text-xs font-mono text-cyan-100">{val} <span className="text-cyan-800">{unit}</span></span>
    </div>
);
