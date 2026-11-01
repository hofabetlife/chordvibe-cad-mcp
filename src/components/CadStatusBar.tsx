import React from 'react';
import { CadPart } from '../types/cad';
import { Crosshair, Hash, Layers, CheckCircle2, Gauge } from 'lucide-react';

interface CadStatusBarProps {
  parts: CadPart[];
  selectedPart?: CadPart;
  isSimulatingPhysics: boolean;
  fps: number;
  mcpToolsCount: number;
}

export const CadStatusBar: React.FC<CadStatusBarProps> = ({
  parts,
  selectedPart,
  isSimulatingPhysics,
  fps,
  mcpToolsCount,
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

      {/* Right: Kinematics & MCP Server Parity */}
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
            {isSimulatingPhysics ? 'RAPIER3D ACTIVE' : 'KINEMATICS IDLE'}
          </span>
        </div>

        <span className="text-[#334155] hidden sm:inline">|</span>

        <div className="flex items-center gap-1 text-[#10b981]">
          <CheckCircle2 className="w-3 h-3 text-[#10b981]" />
          <span className="font-semibold text-[10px]">
            MCP {mcpToolsCount}/{mcpToolsCount} Tools · WGPU Sync
          </span>
        </div>
      </div>
    </footer>
  );
};
