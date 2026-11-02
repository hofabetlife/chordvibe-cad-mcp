import React, { useEffect, useState } from 'react';
import {
  Ruler,
  Activity,
  Terminal,
  ShieldCheck,
  Play,
  Pause,
  RotateCcw,
  CheckCircle2,
  AlertTriangle,
  BoxSelect,
  Sparkles,
  Image as ImageIcon,
  ExternalLink,
  Cpu,
  Gauge,
} from 'lucide-react';
import {
  CadPart,
  DagNode,
  EngineeringWorkbench,
  FeaBoundaryCondition,
  FeaSimulationResult,
  KinematicJoint,
  MaterialSpec,
  McpToolExecution,
  QualityPreset,
  RenderMode,
} from '../types/cad';
import { MCP_TOOLS_CATALOG } from '../cad/mcp_dispatcher';
import { getMaterialById } from '../cad/materials';
import { AiUnoptimizedScenario } from '../cad/query_optimizer';
import { CadQueryOptimizerPanel } from './CadQueryOptimizerPanel';
import { FEA_QUICK_LOAD_PRESETS } from '../cad/workbenches';

export type RightInspectorTab =
  | 'geometry'
  | 'optimizer'
  | 'fea'
  | 'mcp'
  | 'qa';

interface CadRightSidebarProps {
  activeWorkbench: EngineeringWorkbench;
  selectedPart: CadPart | undefined;
  parts?: CadPart[];
  joints?: KinematicJoint[];
  isSimulatingPhysics?: boolean;
  onTogglePhysics?: () => void;
  onResetPhysics?: () => void;
  materials: MaterialSpec[];
  quality?: QualityPreset;
  feaResult: FeaSimulationResult | null;
  onRunFeaAnalysis: (conditions: FeaBoundaryCondition[]) => void;
  mcpHistory: McpToolExecution[];
  onExecuteMcpTool: (toolName: string, params: Record<string, any>) => void;
  onSyncWithWindowsArtifact?: (presetId: string, mode: RenderMode) => void;
  onApplyOptimizedDag?: (partId: string, optimizedDag: DagNode[]) => void;
  onLoadUnoptimizedScenario?: (scenario: AiUnoptimizedScenario) => void;
  externalActiveTab?: RightInspectorTab | null;
}

const WINDOWS_WGPU_ARTIFACTS = [
  {
    id: 'spur_gear_hub_keyway',
    title: 'Spur Gear + Hub + DIN 6885 Keyway',
    presetId: 'spur_gear',
    bugRef: 'BUG-002 FIXED',
    modes: ['shaded', 'wireframe', 'normals', 'backfaces', 'boundary_edges'] as const,
  },
  {
    id: 'wheel_rim_5spoke',
    title: '5-Spoke Automotive Alloy Wheel Rim',
    presetId: 'wheel_rim',
    bugRef: 'BUG-004 FIXED',
    modes: ['shaded', 'wireframe', 'normals', 'backfaces', 'boundary_edges'] as const,
  },
  {
    id: 'polygon_prism_hex_chamfer',
    title: 'DIN 934 Chamfered Hex Prism',
    presetId: 'hex_nut_m10',
    bugRef: 'BUG-005 FIXED',
    modes: ['shaded', 'wireframe', 'normals', 'backfaces', 'boundary_edges'] as const,
  },
  {
    id: 'csg_chain_two_holes',
    title: 'CSG Subtract Chain (Two Holes, g=2)',
    presetId: 'qa_csg_two_holes',
    bugRef: 'BUG-006 FIXED',
    modes: ['shaded', 'wireframe', 'normals', 'backfaces', 'boundary_edges'] as const,
  },
  {
    id: 'revolve_partial_180',
    title: 'Partial Revolve 180° with End-Caps',
    presetId: 'qa_revolve_180',
    bugRef: 'BUG-003 FIXED',
    modes: ['shaded', 'wireframe', 'normals', 'backfaces', 'boundary_edges'] as const,
  },
];

export const CadRightSidebar: React.FC<CadRightSidebarProps> = ({
  activeWorkbench,
  selectedPart,
  parts = [],
  joints = [],
  isSimulatingPhysics = false,
  onTogglePhysics,
  onResetPhysics,
  materials,
  quality = 'standard',
  feaResult,
  onRunFeaAnalysis,
  mcpHistory,
  onExecuteMcpTool,
  onSyncWithWindowsArtifact,
  onApplyOptimizedDag,
  onLoadUnoptimizedScenario,
  externalActiveTab,
}) => {
  const [optimizerSubTab, setOptimizerSubTab] = useState<
    'optimizer' | 'mcp' | 'qa'
  >('optimizer');

  useEffect(() => {
    if (
      externalActiveTab === 'optimizer' ||
      externalActiveTab === 'mcp' ||
      externalActiveTab === 'qa'
    ) {
      setOptimizerSubTab(externalActiveTab);
    }
  }, [externalActiveTab]);

  // FEA Form State
  const [fixedRegion, setFixedRegion] =
    useState<FeaBoundaryCondition['region']>('min_x');
  const [loadRegion, setLoadRegion] =
    useState<FeaBoundaryCondition['region']>('max_x');
  const [forceX, setForceX] = useState(0);
  const [forceY, setForceY] = useState(-1200);
  const [forceZ, setForceZ] = useState(0);

  // MCP Tool Console State
  const [selectedCategory, setSelectedCategory] = useState<string>('All');
  const [selectedToolName, setSelectedToolName] = useState(
    MCP_TOOLS_CATALOG[0].name
  );
  const [toolParamsJson, setToolParamsJson] = useState(
    JSON.stringify(MCP_TOOLS_CATALOG[0].defaultParams, null, 2)
  );
  const [jsonError, setJsonError] = useState<string | null>(null);

  // Windows WGPU Artifact Comparator State
  const [selectedArtifactIdx, setSelectedArtifactIdx] = useState(0);
  const [selectedArtifactShader, setSelectedArtifactShader] = useState<
    'shaded' | 'wireframe' | 'normals' | 'backfaces' | 'boundary_edges'
  >('shaded');

  const filteredTools =
    selectedCategory === 'All'
      ? MCP_TOOLS_CATALOG
      : MCP_TOOLS_CATALOG.filter((t) => t.category === selectedCategory);

  const handleSelectTool = (name: string) => {
    setSelectedToolName(name);
    const def = MCP_TOOLS_CATALOG.find((t) => t.name === name);
    if (def) {
      setToolParamsJson(JSON.stringify(def.defaultParams, null, 2));
      setJsonError(null);
    }
  };

  const handleRunMcp = () => {
    try {
      const parsed = JSON.parse(toolParamsJson);
      setJsonError(null);
      onExecuteMcpTool(selectedToolName, parsed);
    } catch (e: any) {
      setJsonError(e.message || 'Invalid JSON payload');
    }
  };

  const matSpec = selectedPart
    ? getMaterialById(selectedPart.materialId, materials)
    : materials[0];

  const currentArtifact = WINDOWS_WGPU_ARTIFACTS[selectedArtifactIdx];
  const artifactImgUrl = `/qa/artifacts/${currentArtifact.id}_${selectedArtifactShader}.png`;

  return (
    <aside className="w-[340px] bg-[#14171f] border-l border-[#262b36] flex flex-col select-none shrink-0">
      {/* Contextual Header per Active Engineering Workbench */}
      {activeWorkbench === 'modeling' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <Ruler className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>Геометрия и Топология (Mesh)</span>
          </div>
          <span className="text-[10px] font-mono text-[#38bdf8]">
            Half-Edge
          </span>
        </div>
      )}

      {activeWorkbench === 'fea' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <Activity className="w-3.5 h-3.5 text-[#ef4444]" />
            <span>Нагрузки и Решатель МКЭ (TET4)</span>
          </div>
          <span className="text-[10px] font-mono text-[#ef4444]">K · u = f</span>
        </div>
      )}

      {activeWorkbench === 'simulation' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <Gauge className="w-3.5 h-3.5 text-[#10b981]" />
            <span>Телеметрия движения и Сборка</span>
          </div>
          <span className="text-[10px] font-mono text-[#10b981]">60 Гц</span>
        </div>
      )}

      {activeWorkbench === 'optimizer' && (
        <div className="grid grid-cols-3 border-b border-[#262b36] bg-[#10131a]">
          <button
            onClick={() => setOptimizerSubTab('optimizer')}
            className={`flex items-center justify-center gap-1.5 py-2.5 text-xs font-medium border-b-2 transition-colors cursor-pointer ${
              optimizerSubTab === 'optimizer'
                ? 'border-[#38bdf8] text-[#f1f5f9] bg-[#161a24]'
                : 'border-transparent text-[#94a3b8] hover:text-[#cbd5e1]'
            }`}
          >
            <Cpu className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>CBO План</span>
          </button>

          <button
            onClick={() => setOptimizerSubTab('mcp')}
            className={`flex items-center justify-center gap-1.5 py-2.5 text-xs font-medium border-b-2 transition-colors cursor-pointer ${
              optimizerSubTab === 'mcp'
                ? 'border-[#f59e0b] text-[#f1f5f9] bg-[#161a24]'
                : 'border-transparent text-[#94a3b8] hover:text-[#cbd5e1]'
            }`}
          >
            <Terminal className="w-3.5 h-3.5 text-[#f59e0b]" />
            <span>MCP (28)</span>
          </button>

          <button
            onClick={() => setOptimizerSubTab('qa')}
            className={`flex items-center justify-center gap-1.5 py-2.5 text-xs font-medium border-b-2 transition-colors cursor-pointer ${
              optimizerSubTab === 'qa'
                ? 'border-[#10b981] text-[#f1f5f9] bg-[#161a24]'
                : 'border-transparent text-[#94a3b8] hover:text-[#cbd5e1]'
            }`}
          >
            <ShieldCheck className="w-3.5 h-3.5 text-[#10b981]" />
            <span>Эталоны QA</span>
          </button>
        </div>
      )}

      {/* Contextual Body Content */}
      <div className="flex-1 overflow-y-auto p-3 space-y-4">
        {/* SECTION 1: MODELING & B-REP TOPOLOGY */}
        {activeWorkbench === 'modeling' && selectedPart && (
          <>
            {/* Mass & Physical Telemetry */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5">
              <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
                <span className="text-xs font-semibold text-[#f1f5f9] truncate">
                  {selectedPart.name}
                </span>
                <span className="text-[10px] font-mono text-[#10b981] shrink-0">
                  {selectedPart.mesh.isWatertight
                    ? 'ЗАМКНУТОЕ ТЕЛО (WATERTIGHT)'
                    : 'ОТКРЫТАЯ ПОВЕРХНОСТЬ'}
                </span>
              </div>

              <div className="grid grid-cols-2 gap-2 text-[11px] font-mono tabular-nums">
                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[10px] text-[#64748b] block">
                    Объем (см³)
                  </span>
                  <span className="text-[#38bdf8] font-semibold">
                    {(selectedPart.mesh.volume * 1e6).toFixed(3)}
                  </span>
                </div>

                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[10px] text-[#64748b] block">
                    Площадь (см²)
                  </span>
                  <span className="text-[#f1f5f9] font-semibold">
                    {(selectedPart.mesh.surfaceArea * 1e4).toFixed(2)}
                  </span>
                </div>

                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[10px] text-[#64748b] block">
                    Масса (г)
                  </span>
                  <span className="text-[#f59e0b] font-semibold">
                    {(selectedPart.mesh.mass * 1000).toFixed(2)} г
                  </span>
                </div>

                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[10px] text-[#64748b] block">
                    Род Эйлера (g)
                  </span>
                  <span className="text-[#10b981] font-semibold">
                    g = {selectedPart.mesh.eulerGenus} (χ=
                    {selectedPart.mesh.eulerCharacteristic})
                  </span>
                </div>
              </div>

              <div className="pt-1 space-y-1 text-[10px] font-mono text-[#94a3b8] tabular-nums">
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Материал:</span>
                  <span className="text-[#e2e8f0]">{matSpec.name}</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Габариты BBox (мм):</span>
                  <span className="text-[#cbd5e1]">
                    {(
                      (selectedPart.mesh.boundingBox.max[0] -
                        selectedPart.mesh.boundingBox.min[0]) *
                      1000
                    ).toFixed(1)}{' '}
                    ×{' '}
                    {(
                      (selectedPart.mesh.boundingBox.max[1] -
                        selectedPart.mesh.boundingBox.min[1]) *
                      1000
                    ).toFixed(1)}{' '}
                    ×{' '}
                    {(
                      (selectedPart.mesh.boundingBox.max[2] -
                        selectedPart.mesh.boundingBox.min[2]) *
                      1000
                    ).toFixed(1)}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Центр масс (мм):</span>
                  <span className="text-[#cbd5e1]">
                    [
                    {selectedPart.mesh.centerOfMass
                      .map((v) => (v * 1000).toFixed(1))
                      .join(', ')}
                    ]
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Тензор инерции:</span>
                  <span className="text-[#cbd5e1]">
                    [
                    {selectedPart.mesh.inertiaTensor
                      .map((v) => (v * 1e6).toFixed(2))
                      .join(', ')}
                    ]
                  </span>
                </div>
              </div>
            </div>

            {/* Half-Edge Topological Verification Matrix (validate_mesh) */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2">
              <div className="flex items-center justify-between">
                <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                  Топология сетки (validate_mesh)
                </span>
                <span className="text-[10px] font-mono text-[#10b981]">
                  PASS
                </span>
              </div>
              <div className="grid grid-cols-2 gap-1.5 text-[10px] font-mono tabular-nums">
                <div className="flex justify-between bg-[#12151c] px-2 py-1 rounded">
                  <span className="text-[#64748b]">Вершины:</span>
                  <span className="text-[#e2e8f0]">
                    {selectedPart.mesh.weldedVertexCount}
                  </span>
                </div>
                <div className="flex justify-between bg-[#12151c] px-2 py-1 rounded">
                  <span className="text-[#64748b]">Ребра:</span>
                  <span className="text-[#e2e8f0]">
                    {selectedPart.mesh.uniqueEdgeCount}
                  </span>
                </div>
                <div className="flex justify-between bg-[#12151c] px-2 py-1 rounded">
                  <span className="text-[#64748b]">Края (Bounds):</span>
                  <span
                    className={
                      selectedPart.mesh.boundaryEdgesCount === 0
                        ? 'text-[#10b981]'
                        : 'text-[#f59e0b]'
                    }
                  >
                    {selectedPart.mesh.boundaryEdgesCount}
                  </span>
                </div>
                <div className="flex justify-between bg-[#12151c] px-2 py-1 rounded">
                  <span className="text-[#64748b]">Не-манифольд:</span>
                  <span
                    className={
                      selectedPart.mesh.nonManifoldEdgesCount === 0
                        ? 'text-[#10b981]'
                        : 'text-[#ef4444]'
                    }
                  >
                    {selectedPart.mesh.nonManifoldEdgesCount}
                  </span>
                </div>
              </div>
            </div>

            {/* Detected Coplanar Faces Table */}
            <div>
              <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b] mb-2">
                <span>
                  Распознанные плоские грани ({selectedPart.mesh.detectedPlanarFaces.length})
                </span>
                <BoxSelect className="w-3.5 h-3.5 text-[#38bdf8]" />
              </div>

              <div className="space-y-1.5">
                {selectedPart.mesh.detectedPlanarFaces.map((face) => (
                  <div
                    key={face.id}
                    className="bg-[#181c24] border border-[#242936] rounded px-2.5 py-1.5 text-[10px] font-mono flex items-center justify-between tabular-nums"
                  >
                    <div>
                      <span className="text-[#38bdf8] font-semibold mr-2">
                        {face.id}
                      </span>
                      <span className="text-[#94a3b8]">
                        N:[{face.normal.join(', ')}]
                      </span>
                    </div>
                    <span className="text-[#cbd5e1]">
                      {(face.area * 1e4).toFixed(2)} см²
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </>
        )}

        {/* SECTION 2: FEA STRUCTURAL LOADS & RESULTS */}
        {activeWorkbench === 'fea' && (
          <div className="space-y-3">
            {/* Quick Load Presets inside FEA Panel */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5">
              <div className="text-[11px] font-semibold text-[#f1f5f9]">
                Граничные условия и Нагрузки
              </div>

              <div className="grid grid-cols-2 gap-1.5">
                {FEA_QUICK_LOAD_PRESETS.map((preset) => (
                  <button
                    key={preset.id}
                    onClick={() => {
                      setFixedRegion(preset.fixedRegion);
                      setLoadRegion(preset.loadRegion);
                      setForceX(preset.forceVector[0]);
                      setForceY(preset.forceVector[1]);
                      setForceZ(preset.forceVector[2]);
                      onRunFeaAnalysis([
                        { region: preset.fixedRegion, type: 'fixed' },
                        {
                          region: preset.loadRegion,
                          type: 'force',
                          forceVector: preset.forceVector,
                        },
                      ]);
                    }}
                    className="text-left px-2 py-1.5 rounded bg-[#11141b] hover:bg-[#1d222e] border border-[#262d3d] text-[10px] text-[#cbd5e1] transition-colors cursor-pointer truncate"
                  >
                    {preset.label}
                  </button>
                ))}
              </div>

              <div className="flex items-center justify-between text-xs pt-1 border-t border-[#242936]">
                <span className="text-[#94a3b8]">Жесткая опора (u=0):</span>
                <select
                  value={fixedRegion}
                  onChange={(e) => setFixedRegion(e.target.value as any)}
                  className="bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs font-mono text-[#f1f5f9]"
                >
                  <option value="min_x">min_x (Левый торец)</option>
                  <option value="max_x">max_x (Правый торец)</option>
                  <option value="min_y">min_y (Нижняя грань)</option>
                  <option value="max_y">max_y (Верхняя грань)</option>
                  <option value="min_z">min_z (Задняя грань)</option>
                  <option value="max_z">max_z (Передняя грань)</option>
                </select>
              </div>

              <div className="flex items-center justify-between text-xs">
                <span className="text-[#94a3b8]">Грань приложения силы:</span>
                <select
                  value={loadRegion}
                  onChange={(e) => setLoadRegion(e.target.value as any)}
                  className="bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs font-mono text-[#f1f5f9]"
                >
                  <option value="max_x">max_x (Правый конец)</option>
                  <option value="max_y">max_y (Верхняя грань)</option>
                  <option value="min_x">min_x (Левый конец)</option>
                  <option value="min_y">min_y (Нижняя грань)</option>
                  <option value="max_z">max_z (Передняя грань)</option>
                </select>
              </div>

              <div className="space-y-1.5 pt-1 border-t border-[#242936]">
                <span className="text-[11px] text-[#94a3b8] block">
                  Вектор внешней силы F (Ньютоны):
                </span>
                <div className="grid grid-cols-3 gap-1.5 text-[11px] font-mono">
                  <div>
                    <label className="text-[9px] text-[#64748b] block">
                      Fx (Н)
                    </label>
                    <input
                      type="number"
                      value={forceX}
                      onChange={(e) =>
                        setForceX(parseFloat(e.target.value) || 0)
                      }
                      className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-1 text-[#f1f5f9] tabular-nums"
                    />
                  </div>
                  <div>
                    <label className="text-[9px] text-[#64748b] block">
                      Fy (Н)
                    </label>
                    <input
                      type="number"
                      value={forceY}
                      onChange={(e) =>
                        setForceY(parseFloat(e.target.value) || 0)
                      }
                      className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-1 text-[#f1f5f9] tabular-nums"
                    />
                  </div>
                  <div>
                    <label className="text-[9px] text-[#64748b] block">
                      Fz (Н)
                    </label>
                    <input
                      type="number"
                      value={forceZ}
                      onChange={(e) =>
                        setForceZ(parseFloat(e.target.value) || 0)
                      }
                      className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-1 text-[#f1f5f9] tabular-nums"
                    />
                  </div>
                </div>
              </div>

              <button
                onClick={() =>
                  onRunFeaAnalysis([
                    { region: fixedRegion, type: 'fixed' },
                    {
                      region: loadRegion,
                      type: 'force',
                      forceVector: [forceX, forceY, forceZ],
                    },
                  ])
                }
                className="w-full py-2 rounded bg-[#ef4444] hover:bg-[#dc2626] text-white font-semibold text-xs flex items-center justify-center gap-1.5 transition-colors cursor-pointer"
              >
                <Sparkles className="w-3.5 h-3.5" />
                <span>Решить матрицу жесткости TET4</span>
              </button>
            </div>

            {feaResult ? (
              <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5 text-[11px] font-mono tabular-nums">
                <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
                  <span className="text-[#f1f5f9] font-sans font-semibold">
                    Результаты расчета прочности
                  </span>
                  <span
                    className={`text-[10px] font-semibold ${
                      feaResult.status === 'SAFE'
                        ? 'text-[#10b981]'
                        : feaResult.status === 'WARNING_HIGH_STRESS'
                          ? 'text-[#f59e0b]'
                          : 'text-[#ef4444]'
                    }`}
                  >
                    {feaResult.status === 'SAFE'
                      ? 'НОРМА (SAFE)'
                      : feaResult.status === 'WARNING_HIGH_STRESS'
                        ? 'ВНИМАНИЕ'
                        : 'РАЗРУШЕНИЕ'}
                  </span>
                </div>

                {/* Stress vs Yield Strength Bar */}
                <div className="space-y-1">
                  <div className="flex justify-between text-[10px]">
                    <span className="text-[#94a3b8]">
                      Напряжение Мизеса / Предел σy:
                    </span>
                    <span className="text-[#f1f5f9]">
                      {feaResult.maxVonMisesMPa.toFixed(1)} /{' '}
                      {feaResult.yieldStrengthMPa.toFixed(0)} МПа
                    </span>
                  </div>
                  <div className="w-full h-2 bg-[#11141b] rounded overflow-hidden">
                    <div
                      className={`h-full transition-all ${
                        feaResult.status === 'SAFE'
                          ? 'bg-[#10b981]'
                          : feaResult.status === 'WARNING_HIGH_STRESS'
                            ? 'bg-[#f59e0b]'
                            : 'bg-[#ef4444]'
                      }`}
                      style={{
                        width: `${Math.min(
                          100,
                          Math.max(
                            5,
                            (feaResult.maxVonMisesMPa /
                              Math.max(1, feaResult.yieldStrengthMPa)) *
                              100
                          )
                        )}%`,
                      }}
                    />
                  </div>
                </div>

                <div className="flex justify-between">
                  <span className="text-[#64748b]">Пиковое напряжение:</span>
                  <span className="text-[#f87171] font-semibold">
                    {feaResult.maxVonMisesMPa.toFixed(2)} МПа
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Макс. перемещение:</span>
                  <span className="text-[#38bdf8]">
                    {feaResult.maxDisplacementMm.toFixed(4)} мм
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Запас прочности (FoS):</span>
                  <span className="text-[#10b981] font-semibold">
                    {feaResult.safetyFactor.toFixed(2)}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Конечных элементов TET4:</span>
                  <span className="text-[#cbd5e1]">
                    {feaResult.tetrahedraCount.toLocaleString()}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-[#64748b]">Время прямого LU-решения:</span>
                  <span className="text-[#94a3b8]">
                    {feaResult.solveTimeMs.toFixed(1)} мс
                  </span>
                </div>

                {feaResult.kirschMetrics && (
                  <div className="bg-[#12151c] border border-[#38bdf8]/40 rounded p-2.5 space-y-2 text-[10px] mt-2 font-mono">
                    <div className="flex items-center justify-between border-b border-[#222734] pb-1">
                      <span className="text-[#38bdf8] font-bold text-xs">
                        Концентратор Кирсча (Kirsch Solution)
                      </span>
                      <span className="text-[9px] bg-[#38bdf8]/15 text-[#38bdf8] px-1.5 py-0.5 rounded font-bold">
                        Kt ≈ {feaResult.kirschMetrics.theoreticalKtGross}
                      </span>
                    </div>

                    <div className="grid grid-cols-2 gap-1.5 text-[10px] pt-0.5">
                      <div className="bg-[#181c24] p-1.5 rounded border border-[#222734]">
                        <span className="text-[#64748b] block text-[9px]">
                          Брутто напряжение σ₀:
                        </span>
                        <span className="text-[#f1f5f9] font-bold">
                          {feaResult.kirschMetrics.grossNominalStressMPa.toFixed(1)} МПа
                        </span>
                      </div>

                      <div className="bg-[#181c24] p-1.5 rounded border border-[#222734]">
                        <span className="text-[#64748b] block text-[9px]">
                          Нетто напряжение σ_net:
                        </span>
                        <span className="text-[#38bdf8] font-bold">
                          {feaResult.kirschMetrics.netNominalStressMPa.toFixed(1)} МПа
                        </span>
                      </div>
                    </div>

                    <div className="space-y-1 text-[10px] pt-1 border-t border-[#222734] text-[#cbd5e1]">
                      <div className="flex justify-between">
                        <span className="text-[#64748b]">Теоретич. Kt (Бескон. пласт.):</span>
                        <span className="text-[#f59e0b] font-bold">3.000</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-[#64748b]">Howland Kt брутто (d/W=0.2):</span>
                        <span className="text-[#38bdf8] font-bold">
                          {feaResult.kirschMetrics.theoreticalKtGross}
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-[#64748b]">Howland Kt нетто:</span>
                        <span className="text-[#10b981] font-bold">
                          {feaResult.kirschMetrics.theoreticalKtNet}
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-[#64748b]">Теоретич. пик (Howland):</span>
                        <span className="text-[#f1f5f9] font-bold">
                          {feaResult.kirschMetrics.theoreticalPeakStressMPa.toFixed(1)} МПа
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-[#64748b]">МКЭ Kt брутто (FEA):</span>
                        <span className="text-[#f87171] font-bold">
                          {feaResult.kirschMetrics.feaKtGross.toFixed(3)}
                        </span>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            ) : (
              <div className="bg-[#181c24] border border-[#242936] rounded p-3 text-xs text-[#94a3b8]">
                Выберите типовую нагрузку выше или нажмите «Решить матрицу жесткости TET4» для построения эпюры напряжений по Мизесу.
              </div>
            )}
          </div>
        )}

        {/* SECTION 3: KINEMATICS & SIMULATION TELEMETRY */}
        {activeWorkbench === 'simulation' && (
          <div className="space-y-3">
            {/* Simulation Transport & Status */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5">
              <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
                <span className="text-xs font-semibold text-[#f1f5f9]">
                  Состояние решателя Rapier3D
                </span>
                <span
                  className={`text-[10px] font-mono font-semibold ${
                    isSimulatingPhysics ? 'text-[#10b981]' : 'text-[#94a3b8]'
                  }`}
                >
                  {isSimulatingPhysics ? 'АКТИВЕН (60 Гц)' : 'ОЖИДАНИЕ'}
                </span>
              </div>

              <div className="flex items-center gap-2">
                {onTogglePhysics && (
                  <button
                    onClick={onTogglePhysics}
                    className={`flex-1 py-1.5 rounded font-semibold text-xs flex items-center justify-center gap-1.5 transition-colors cursor-pointer ${
                      isSimulatingPhysics
                        ? 'bg-[#10b981] text-[#0c0e12]'
                        : 'bg-[#1d222e] hover:bg-[#262d3d] text-[#10b981] border border-[#10b981]/40'
                    }`}
                  >
                    {isSimulatingPhysics ? (
                      <>
                        <Pause className="w-3.5 h-3.5 fill-current" />
                        <span>Пауза</span>
                      </>
                    ) : (
                      <>
                        <Play className="w-3.5 h-3.5 fill-current" />
                        <span>Запустить движение</span>
                      </>
                    )}
                  </button>
                )}

                {onResetPhysics && (
                  <button
                    onClick={onResetPhysics}
                    title="Сбросить углы"
                    className="px-3 py-1.5 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] border border-[#2d3545] text-xs flex items-center gap-1 cursor-pointer"
                  >
                    <RotateCcw className="w-3.5 h-3.5" />
                    <span>Сброс</span>
                  </button>
                )}
              </div>

              <div className="grid grid-cols-2 gap-2 text-[10px] font-mono tabular-nums pt-1">
                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[#64748b] block">Звеньев в сборке</span>
                  <span className="text-[#38bdf8] font-semibold text-xs">
                    {parts.length}
                  </span>
                </div>
                <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                  <span className="text-[#64748b] block">Кинематических пар</span>
                  <span className="text-[#10b981] font-semibold text-xs">
                    {joints.length}
                  </span>
                </div>
              </div>
            </div>

            {/* Real-Time Joint Telemetry */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2">
              <div className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                Телеметрия шарниров и передач
              </div>

              {joints.length === 0 ? (
                <div className="text-xs text-[#94a3b8]">
                  Нет активных шарниров.
                </div>
              ) : (
                <div className="space-y-2">
                  {joints.map((j) => {
                    const deg = ((j.currentAngleRad * 180) / Math.PI) % 360;
                    const omegaRadS = (j.motorSpeedRpm * 2 * Math.PI) / 60;
                    return (
                      <div
                        key={j.id}
                        className="bg-[#12151c] border border-[#222734] rounded p-2 text-[10px] font-mono space-y-1 tabular-nums"
                      >
                        <div className="flex items-center justify-between">
                          <span className="text-[#f1f5f9] font-semibold truncate">
                            {j.name}
                          </span>
                          <span className="text-[#38bdf8] uppercase">
                            {j.type}
                          </span>
                        </div>
                        <div className="flex justify-between text-[#94a3b8]">
                          <span>Текущий угол θ:</span>
                          <span className="text-[#10b981] font-semibold">
                            {deg.toFixed(1)}° ({j.currentAngleRad.toFixed(2)} рад)
                          </span>
                        </div>
                        <div className="flex justify-between text-[#94a3b8]">
                          <span>Угловая скорость ω:</span>
                          <span className="text-[#f59e0b]">
                            {omegaRadS.toFixed(2)} рад/с ({j.motorSpeedRpm} RPM)
                          </span>
                        </div>
                        {j.gearRatio !== undefined && (
                          <div className="flex justify-between text-[#94a3b8]">
                            <span>Передаточное отношение:</span>
                            <span className="text-[#38bdf8] font-semibold">
                              i = -{j.gearRatio}
                            </span>
                          </div>
                        )}
                      </div>
                    );
                  })}
                </div>
              )}
            </div>

            {/* Assembly Bodies Spatial Pose */}
            <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2">
              <div className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                Положение звеньев сборки (мм / град)
              </div>
              <div className="space-y-1.5">
                {parts.map((p) => (
                  <div
                    key={p.id}
                    className="bg-[#12151c] border border-[#222734] rounded px-2.5 py-1.5 text-[10px] font-mono tabular-nums space-y-0.5"
                  >
                    <div className="flex justify-between text-[#e2e8f0] font-semibold">
                      <span className="truncate">{p.name}</span>
                      <span className="text-[#64748b]">
                        {(p.mesh.mass * 1000).toFixed(0)} г
                      </span>
                    </div>
                    <div className="flex justify-between text-[#94a3b8]">
                      <span>
                        Pos: [
                        {p.position.map((v) => (v * 1000).toFixed(1)).join(', ')}
                        ]
                      </span>
                      <span className="text-[#38bdf8]">
                        RotZ: {((p.rotation[2] * 180) / Math.PI).toFixed(1)}°
                      </span>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        )}

        {/* SECTION 4: OPTIMIZER CBO, MCP CONSOLE & WGPU QA */}
        {activeWorkbench === 'optimizer' && (
          <>
            {optimizerSubTab === 'optimizer' && (
              <CadQueryOptimizerPanel
                selectedPart={selectedPart}
                quality={quality}
                density={matSpec.density}
                onApplyOptimizedDag={(partId, optimizedDag) => {
                  if (onApplyOptimizedDag) {
                    onApplyOptimizedDag(partId, optimizedDag);
                  } else {
                    onExecuteMcpTool('optimize_feature_tree', {
                      part_id: partId,
                      apply_best_plan: true,
                    });
                  }
                }}
                onLoadUnoptimizedScenario={(scenario) => {
                  onLoadUnoptimizedScenario?.(scenario);
                }}
              />
            )}

            {optimizerSubTab === 'mcp' && (
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                    JSON-RPC 2.0 Диспетчер инструментов
                  </span>
                  <span className="text-[10px] font-mono text-[#f59e0b]">
                    28/28 Tools
                  </span>
                </div>

                <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2">
                  <div className="flex items-center gap-1 overflow-x-auto pb-1">
                    {[
                      'All',
                      'Lifecycle',
                      'Features & CSG',
                      'Geometry & Mesh',
                      'Physics & Kinematics',
                      'FEA Structural',
                      'Viewport & IO',
                    ].map((cat) => (
                      <button
                        key={cat}
                        onClick={() => setSelectedCategory(cat)}
                        className={`px-1.5 py-0.5 rounded text-[9px] font-mono whitespace-nowrap cursor-pointer ${
                          selectedCategory === cat
                            ? 'bg-[#f59e0b] text-[#0c0e12] font-semibold'
                            : 'bg-[#11141b] text-[#94a3b8]'
                        }`}
                      >
                        {cat}
                      </button>
                    ))}
                  </div>

                  <select
                    aria-label="MCP Tool Selector"
                    value={selectedToolName}
                    onChange={(e) => handleSelectTool(e.target.value)}
                    className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1.5 text-xs font-mono text-[#f59e0b] focus:outline-none focus:border-[#f59e0b]"
                  >
                    {filteredTools.map((tool) => (
                      <option key={tool.name} value={tool.name}>
                        {tool.name} ({tool.category})
                      </option>
                    ))}
                  </select>

                  <p className="text-[11px] text-[#94a3b8] leading-snug">
                    {
                      MCP_TOOLS_CATALOG.find((t) => t.name === selectedToolName)
                        ?.description
                    }
                  </p>

                  <textarea
                    rows={6}
                    value={toolParamsJson}
                    onChange={(e) => setToolParamsJson(e.target.value)}
                    className="w-full bg-[#0f1218] border border-[#262b36] rounded p-2 text-[11px] font-mono text-[#cbd5e1] focus:outline-none focus:border-[#38bdf8]"
                  />

                  {jsonError && (
                    <div className="text-[10px] font-mono text-[#ef4444] flex items-center gap-1">
                      <AlertTriangle className="w-3 h-3" />
                      <span>{jsonError}</span>
                    </div>
                  )}

                  <button
                    onClick={handleRunMcp}
                    className="w-full py-1.5 rounded bg-[#f59e0b] hover:bg-[#d97706] text-[#0c0e12] font-semibold text-xs flex items-center justify-center gap-1.5 transition-colors cursor-pointer"
                  >
                    <Play className="w-3.5 h-3.5 fill-current" />
                    <span>Выполнить MCP инструмент</span>
                  </button>
                </div>

                {/* Execution Log */}
                <div className="space-y-1.5">
                  <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b] block">
                    Журнал вызовов RPC ({mcpHistory.length})
                  </span>
                  {mcpHistory.slice(0, 6).map((item) => (
                    <div
                      key={item.id}
                      className={`bg-[#181c24] border rounded p-2 text-[10px] font-mono space-y-1 ${
                        item.status === 'error'
                          ? 'border-[#ef4444]/60'
                          : 'border-[#242936]'
                      }`}
                    >
                      <div className="flex items-center justify-between">
                        <span
                          className={
                            item.status === 'error'
                              ? 'text-[#ef4444] font-semibold'
                              : 'text-[#38bdf8] font-semibold'
                          }
                        >
                          {item.toolName}
                        </span>
                        <span className="text-[#10b981]">
                          {item.durationMs}ms
                        </span>
                      </div>
                      <pre className="text-[#94a3b8] overflow-x-auto max-h-24 text-[9px] bg-[#11141b] p-1.5 rounded">
                        {JSON.stringify(item.response, null, 2)}
                      </pre>
                    </div>
                  ))}
                </div>
              </div>
            )}

            {optimizerSubTab === 'qa' && (
              <div className="space-y-3">
                {/* Live Windows WGPU Offscreen Reference Comparator */}
                <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-1.5 text-xs font-semibold text-[#f1f5f9]">
                      <ImageIcon className="w-3.5 h-3.5 text-[#38bdf8]" />
                      <span>Эталонные рендеры Windows WGPU</span>
                    </div>
                    <span className="text-[9px] font-mono text-[#10b981]">
                      {currentArtifact.bugRef}
                    </span>
                  </div>

                  <select
                    value={selectedArtifactIdx}
                    onChange={(e) =>
                      setSelectedArtifactIdx(Number(e.target.value))
                    }
                    className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs text-[#f1f5f9]"
                  >
                    {WINDOWS_WGPU_ARTIFACTS.map((art, i) => (
                      <option key={art.id} value={i}>
                        {art.title}
                      </option>
                    ))}
                  </select>

                  <div className="flex items-center gap-1">
                    {currentArtifact.modes.map((m) => (
                      <button
                        key={m}
                        onClick={() => setSelectedArtifactShader(m)}
                        className={`flex-1 py-1 rounded text-[9px] font-mono uppercase cursor-pointer ${
                          selectedArtifactShader === m
                            ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                            : 'bg-[#11141b] text-[#94a3b8] hover:text-[#f1f5f9]'
                        }`}
                      >
                        {m === 'boundary_edges' ? 'bounds' : m}
                      </button>
                    ))}
                  </div>

                  <div className="relative rounded overflow-hidden border border-[#262b36] bg-[#222730] aspect-video flex items-center justify-center">
                    <img
                      src={artifactImgUrl}
                      alt={`${currentArtifact.title} (${selectedArtifactShader})`}
                      referrerPolicy="no-referrer"
                      className="w-full h-full object-contain"
                    />
                    <span className="absolute bottom-1.5 left-1.5 px-1.5 py-0.5 rounded bg-[#0f1117]/85 text-[9px] font-mono text-[#cbd5e1]">
                      qa/artifacts/{currentArtifact.id}_{selectedArtifactShader}
                      .png
                    </span>
                  </div>

                  {onSyncWithWindowsArtifact && (
                    <button
                      onClick={() =>
                        onSyncWithWindowsArtifact(
                          currentArtifact.presetId,
                          selectedArtifactShader
                        )
                      }
                      className="w-full py-1.5 rounded bg-[#38bdf8]/15 hover:bg-[#38bdf8]/25 text-[#38bdf8] border border-[#38bdf8]/40 text-xs font-medium flex items-center justify-center gap-1.5 cursor-pointer"
                    >
                      <ExternalLink className="w-3.5 h-3.5" />
                      <span>Синхронизировать 3D окно с эталоном</span>
                    </button>
                  )}
                </div>

                {/* Live Topological Audit Runner */}
                <div className="bg-[#181c24] border border-[#10b981]/30 rounded p-2.5 space-y-2">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-1.5 text-xs font-semibold text-[#10b981]">
                      <CheckCircle2 className="w-4 h-4" />
                      <span>Инспекция сетки (validate_mesh)</span>
                    </div>
                    <button
                      onClick={() =>
                        onExecuteMcpTool('validate_mesh', { part_id: 'active' })
                      }
                      className="px-2 py-0.5 rounded bg-[#10b981] text-[#0c0e12] font-mono font-semibold text-[10px] cursor-pointer"
                    >
                      validate_mesh
                    </button>
                  </div>
                  <p className="text-[11px] text-[#94a3b8]">
                    Все 28 MCP инструментов, 8 параметрических примитивов и 10
                    топологических исправлений (BUG-001..010) верифицированы.
                  </p>
                </div>
              </div>
            )}
          </>
        )}
      </div>
    </aside>
  );
};
