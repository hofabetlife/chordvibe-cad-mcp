import React from 'react';
import { CadPart } from '../types/cad';
import {
  Crosshair,
  Hash,
  Layers,
  CheckCircle2,
  Gauge,
  Radio,
} from 'lucide-react';

interface CadStatusBarProps {
  parts: CadPart[];
  selectedPart?: CadPart;
  isSimulatingPhysics: boolean;
  fps: number;
  mcpToolsCount: number;
  liveSyncEnabled?: boolean;
  liveSyncConnected?: boolean;
  livePartsCount?: number;
  liveLastTool?: string;
}

export const CadStatusBar: React.FC<CadStatusBarProps> = ({
  parts,
  selectedPart,
  isSimulatingPhysics,
  fps,
  mcpToolsCount,
  liveSyncEnabled = true,
  liveSyncConnected = false,
  livePartsCount = 0,
  liveLastTool,
}) => {
  const totalTriangles = parts.reduce(
    (acc, p) => acc + (p.visible ? p.mesh.triangleCount : 0),
    0
  );
  const totalVertices = parts.reduce(
    (acc, p) => acc + (p.visible ? p.mesh.vertexCount : 0),
    0
  );
  const totalWelded = parts.reduce(
    (acc, p) => acc + (p.visible ? p.mesh.weldedVertexCount : 0),
    0
  );

  const pos = selectedPart?.position ?? [0, 0, 0];

  return (
    <footer className="h-7 bg-[#12151c] border-t border-[#262b36] px-3 flex items-center justify-between text-[11px] font-mono text-[#94a3b8] select-none z-30 shrink-0">
      {/* Left: Active Part Position & Name */}
      <div className="flex items-center gap-3">
        <div className="flex items-center gap-1.5 text-[#cbd5e1]">
          <Crosshair className="w-3 h-3 text-[#38bdf8]" />
          <span>
            X: {(pos[0] * 1000).toFixed(1)} mm · Y: {(pos[1] * 1000).toFixed(1)}{' '}
            mm · Z: {(pos[2] * 1000).toFixed(1)} mm
          </span>
        </div>

        <span className="text-[#334155]">|</span>

        <div className="hidden sm:flex items-center gap-1.5 text-[#cbd5e1]">
          <Layers className="w-3 h-3 text-[#f59e0b]" />
          <span className="truncate max-w-[160px]">
            {selectedPart?.name || 'No Selection'}
          </span>
        </div>
      </div>

      {/* Center: Mesh Geometry & Welded Topology */}
      <div className="hidden md:flex items-center gap-3">
        <div className="flex items-center gap-1.5 text-[#cbd5e1]">
          <Hash className="w-3 h-3 text-[#64748b]" />
          <span>{totalTriangles.toLocaleString()} Tris</span>
          <span className="text-[#334155]">·</span>
          <span>{totalVertices.toLocaleString()} Verts</span>
          <span className="text-[#334155]">·</span>
          <span className="text-[#38bdf8]">
            {totalWelded.toLocaleString()} Welded
          </span>
        </div>
      </div>

      {/* Right: Kinematics & MCP Live Sync */}
      <div className="flex items-center gap-3">
        <div className="hidden lg:flex items-center gap-1.5 text-[#cbd5e1]">
          <Gauge className="w-3 h-3 text-[#10b981]" />
          <span>{fps} FPS</span>
          <span className="text-[#334155]">·</span>
          <span
            className={
              isSimulatingPhysics ? 'text-[#10b981] font-semibold' : 'text-[#64748b]'
            }
          >
            {isSimulatingPhysics ? 'DAE f64 ACTIVE' : 'KINEMATICS IDLE'}
          </span>
        </div>

        <span className="text-[#334155] hidden sm:inline">|</span>

        <div
          className={`flex items-center gap-1 ${
            liveSyncEnabled && livePartsCount > 0
              ? 'text-[#34d399]'
              : liveSyncEnabled && liveSyncConnected
                ? 'text-[#38bdf8]'
                : 'text-[#10b981]'
          }`}
        >
          {liveSyncConnected ? (
            <Radio className="w-3 h-3 shrink-0" />
          ) : (
            <CheckCircle2 className="w-3 h-3 text-[#10b981] shrink-0" />
          )}
          <span className="font-semibold text-[10px] truncate max-w-[260px]">
            {liveSyncEnabled
              ? livePartsCount > 0
                ? `Live MCP · ${liveLastTool || `${livePartsCount} дет.`}`
                : `Live MCP · Ожидание агента (${mcpToolsCount} Tools)`
              : `MCP ${mcpToolsCount} Tools · Пауза Live`}
          </span>
        </div>
      </div>
    </footer>
  );
};
