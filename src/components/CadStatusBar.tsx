import React from 'react';
import type { FeaResult, PartSummary } from '../types/cad';
import {
  Activity,
  CheckCircle2,
  Cpu,
  Database,
  Gauge,
  Layers,
  Shield,
  Terminal,
} from 'lucide-react';

interface CadStatusBarProps {
  selectedPart: PartSummary | null;
  feaResult: FeaResult | null;
  statusMessage: string;
  mcpToolsCount: number;
  onOpenMcpLogs: () => void;
}

export const CadStatusBar: React.FC<CadStatusBarProps> = ({
  selectedPart,
  feaResult,
  statusMessage,
  mcpToolsCount,
  onOpenMcpLogs,
}) => {
  return (
    <footer className="h-7 bg-[#0a0d17] border-t border-slate-800/90 text-slate-300 text-[11px] font-mono px-3 flex items-center justify-between shrink-0 select-none z-30">
      {/* Left: Status Message & Active Part */}
      <div className="flex items-center gap-3 truncate max-w-[45%]">
        <div className="flex items-center gap-1.5 text-emerald-400 font-semibold shrink-0">
          <span className="w-2 h-2 rounded-full bg-emerald-400 animate-pulse" />
          <span>ГОТОВ</span>
        </div>
        <span className="text-slate-600">|</span>
        <span className="text-slate-300 truncate">{statusMessage}</span>
      </div>

      {/* Center: Selected Part Metrics */}
      <div className="hidden md:flex items-center gap-3 text-slate-400">
        {selectedPart ? (
          <>
            <div className="flex items-center gap-1 text-slate-200">
              <Layers className="w-3 h-3 text-amber-400" />
              <span className="font-semibold">{selectedPart.name}</span>
            </div>
            <span className="text-slate-700">•</span>
            <span>
              Полигонов:{' '}
              <strong className="text-amber-400 font-normal">
                {selectedPart.triangle_count.toLocaleString()}
              </strong>
            </span>
            <span className="text-slate-700">•</span>
            <span>
              Масса:{' '}
              <strong className="text-emerald-400 font-normal">
                {selectedPart.mass >= 1
                  ? `${selectedPart.mass.toFixed(3)} кг`
                  : `${(selectedPart.mass * 1000).toFixed(1)} г`}
              </strong>
            </span>
            {feaResult && (
              <>
                <span className="text-slate-700">•</span>
                <div className="flex items-center gap-1">
                  <Gauge className="w-3 h-3 text-sky-400" />
                  <span>
                    FoS: <strong className="text-amber-400">{feaResult.safety_factor.toFixed(2)}</strong> (
                    {feaResult.status})
                  </span>
                </div>
              </>
            )}
          </>
        ) : (
          <span className="text-slate-500">Сцена пуста</span>
        )}
      </div>

      {/* Right: Unit standards & MCP Server */}
      <div className="flex items-center gap-3 text-slate-400 shrink-0">
        <span className="hidden sm:inline text-slate-500">СИ [м, мм, Н, МПа]</span>
        <span className="text-slate-700 hidden sm:inline">•</span>
        <div className="flex items-center gap-1 text-sky-400 font-mono">
          <Database className="w-3 h-3" />
          <span>127.0.0.1:8080</span>
        </div>
        <span className="text-slate-700">•</span>
        <button
          type="button"
          onClick={onOpenMcpLogs}
          className="flex items-center gap-1 text-slate-300 hover:text-amber-400 transition-colors cursor-pointer"
        >
          <Terminal className="w-3 h-3 text-amber-400" />
          <span>MCP ({mcpToolsCount})</span>
        </button>
      </div>
    </footer>
  );
};
