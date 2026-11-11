import React, { useEffect, useState } from 'react';
import {
  GitBranch,
  Layers,
  Palette,
  Eye,
  EyeOff,
  Trash2,
  ChevronRight,
  ChevronDown,
  Link2,
  Zap,
  Plus,
  Check,
  RotateCw,
  ShieldAlert,
} from 'lucide-react';
import {
  CadPart,
  DagNode,
  EngineeringWorkbench,
  KinematicJoint,
  MaterialSpec,
} from '../types/cad';
import { getMaterialById } from '../cad/materials';

interface CadLeftSidebarProps {
  activeWorkbench: EngineeringWorkbench;
  parts: CadPart[];
  selectedPartId: string;
  onSelectPart: (id: string) => void;
  onTogglePartVisibility: (id: string) => void;
  onDeletePart: (id: string) => void;
  onUpdateDagNodeParams: (
    partId: string,
    nodeId: string,
    newParams: Record<string, any>
  ) => void;
  onToggleDagNode: (partId: string, nodeId: string) => void;
  onDeleteDagNode: (partId: string, nodeId: string) => void;
  materials: MaterialSpec[];
  onAssignMaterial: (partId: string, materialId: string) => void;
  onDefineMaterial: (mat: MaterialSpec) => void;
  joints: KinematicJoint[];
  onUpdateJointMotor: (
    jointId: string,
    speedRpm: number,
    torqueNm: number,
    enabled: boolean
  ) => void;
  onAddJoint: (joint: KinematicJoint) => void;
  onDeleteJoint: (jointId: string) => void;
}

export const CadLeftSidebar: React.FC<CadLeftSidebarProps> = ({
  activeWorkbench,
  parts,
  selectedPartId,
  onSelectPart,
  onTogglePartVisibility,
  onDeletePart,
  onUpdateDagNodeParams,
  onToggleDagNode,
  onDeleteDagNode,
  materials,
  onAssignMaterial,
  onDefineMaterial,
  joints,
  onUpdateJointMotor,
  onAddJoint,
  onDeleteJoint,
}) => {
  const [modelingSubTab, setModelingSubTab] = useState<'dag' | 'materials'>(
    'dag'
  );
  const [expandedNodes, setExpandedNodes] = useState<Record<string, boolean>>({
    node_root: true,
    node_1: true,
    node_2: true,
  });

  // Inline Create Joint state
  const [showNewJointForm, setShowNewJointForm] = useState(false);
  const [newJointType, setNewJointType] =
    useState<KinematicJoint['type']>('revolute');
  const [newJointPartA, setNewJointPartA] = useState('world');
  const [newJointPartB, setNewJointPartB] = useState('');
  const [newJointAxis, setNewJointAxis] = useState<'X' | 'Y' | 'Z'>('Z');
  const [newJointRpm, setNewJointRpm] = useState(45);

  // Inline Define Material state
  const [showNewMatForm, setShowNewMatForm] = useState(false);
  const [newMatName, setNewMatName] = useState('Inconel 718 Superalloy');
  const [newMatDensity, setNewMatDensity] = useState(8190);
  const [newMatE, setNewMatE] = useState(200);
  const [newMatNu, setNewMatNu] = useState(0.29);
  const [newMatYield, setNewMatYield] = useState(1035);
  const [newMatColor, setNewMatColor] = useState('#8b95a5');

  useEffect(() => {
    if (activeWorkbench === 'modeling') {
      setModelingSubTab('dag');
    }
  }, [activeWorkbench]);

  const selectedPart =
    parts.find((p) => p.id === selectedPartId) || parts[0];
  const activeMaterial = selectedPart
    ? getMaterialById(selectedPart.materialId, materials)
    : materials[0];

  const toggleNodeExpand = (nodeId: string) => {
    setExpandedNodes((prev) => ({ ...prev, [nodeId]: !prev[nodeId] }));
  };

  const handleCreateJointSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const targetB =
      newJointPartB || selectedPart?.id || parts[0]?.id || 'part_1';
    const axisVec: [number, number, number] =
      newJointAxis === 'X'
        ? [1, 0, 0]
        : newJointAxis === 'Y'
          ? [0, 1, 0]
          : [0, 0, 1];
    const targetPartObj = parts.find((p) => p.id === targetB);
    onAddJoint({
      id: `joint_${Date.now().toString().slice(-4)}`,
      name: `${newJointType.toUpperCase()} (${targetB})`,
      type: newJointType,
      partA: newJointPartA,
      partB: targetB,
      anchor: targetPartObj ? [...targetPartObj.position] : [0, 0, 0],
      axis: axisVec,
      motorSpeedRpm: newJointRpm,
      motorTorqueNm: 45,
      motorEnabled: newJointType === 'revolute' || newJointType === 'prismatic',
      currentAngleRad: 0,
    });
    setShowNewJointForm(false);
  };

  const handleDefineMaterialSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const id = `custom_${newMatName
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, '_')}_${Date.now().toString().slice(-3)}`;
    const spec: MaterialSpec = {
      id,
      name: newMatName || 'Custom Alloy',
      category: 'Custom',
      density: Math.max(100, Number(newMatDensity)),
      youngsModulus: Math.max(0.1, Number(newMatE)) * 1e9,
      poissonsRatio: Math.max(0.05, Math.min(0.49, Number(newMatNu))),
      yieldStrength: Math.max(1, Number(newMatYield)) * 1e6,
      colorHex: newMatColor,
      roughness: 0.3,
      metalness: 0.85,
    };
    onDefineMaterial(spec);
    if (selectedPart) {
      onAssignMaterial(selectedPart.id, spec.id);
    }
    setShowNewMatForm(false);
  };

  const renderPartsList = (compactLabel = 'Детали сцены') => (
    <div>
      <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b] mb-2">
        <span>
          {compactLabel} ({parts.length})
        </span>
        <Layers className="w-3 h-3" />
      </div>

      <div className="space-y-1">
        {parts.map((part) => {
          const isSelected = part.id === selectedPartId;
          return (
            <div
              key={part.id}
              onClick={() => onSelectPart(part.id)}
              className={`group flex items-center justify-between px-2.5 py-1.5 rounded text-xs cursor-pointer border transition-colors ${
                isSelected
                  ? 'bg-[#1e2533] border-[#38bdf8]/60 text-[#f1f5f9]'
                  : 'bg-[#181c24] border-[#242936] text-[#94a3b8] hover:bg-[#1b202b]'
              }`}
            >
              <div className="flex items-center gap-2 truncate">
                <span
                  className={`w-2 h-2 rounded-full shrink-0 ${
                    isSelected ? 'bg-[#38bdf8]' : 'bg-[#64748b]'
                  }`}
                />
                <span className="truncate font-medium">{part.name}</span>
              </div>

              <div className="flex items-center gap-1 shrink-0">
                <span className="text-[10px] font-mono text-[#64748b] tabular-nums">
                  {part.mesh.triangleCount}Δ
                </span>
                <button
                  onClick={(e) => {
                    e.stopPropagation();
                    onTogglePartVisibility(part.id);
                  }}
                  title="Скрыть / Показать деталь"
                  className="p-1 text-[#64748b] hover:text-[#f1f5f9] cursor-pointer"
                >
                  {part.visible ? (
                    <Eye className="w-3 h-3" />
                  ) : (
                    <EyeOff className="w-3 h-3 text-[#ef4444]" />
                  )}
                </button>
                {parts.length > 1 && (
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      onDeletePart(part.id);
                    }}
                    title="Удалить деталь"
                    className="p-1 text-[#64748b] hover:text-[#ef4444] cursor-pointer"
                  >
                    <Trash2 className="w-3 h-3" />
                  </button>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );

  const renderDagTree = () => {
    if (!selectedPart) return null;
    return (
      <div>
        <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b] mb-2 pt-2 border-t border-[#262b36]">
          <span className="truncate">Дерево операций: {selectedPart.name}</span>
          <span className="text-[#f59e0b] shrink-0">
            {selectedPart.dag.length} оп.
          </span>
        </div>

        <div className="space-y-2">
          {selectedPart.dag.map((node: DagNode, idx: number) => {
            const isExpanded = expandedNodes[node.id] ?? idx === 0;
            return (
              <div
                key={node.id}
                className={`rounded border overflow-hidden transition-opacity ${
                  node.enabled
                    ? 'bg-[#181c24] border-[#282e3d]'
                    : 'bg-[#14171e] border-[#222632] opacity-60'
                }`}
              >
                <div className="flex items-center justify-between px-2.5 py-1.5 bg-[#1b202b]">
                  <div
                    onClick={() => toggleNodeExpand(node.id)}
                    className="flex items-center gap-1.5 text-xs font-medium text-[#e2e8f0] cursor-pointer flex-1 truncate"
                  >
                    {isExpanded ? (
                      <ChevronDown className="w-3.5 h-3.5 text-[#94a3b8] shrink-0" />
                    ) : (
                      <ChevronRight className="w-3.5 h-3.5 text-[#94a3b8] shrink-0" />
                    )}
                    <span className="text-[10px] font-mono px-1 py-0.2 rounded bg-[#262d3d] text-[#38bdf8]">
                      #{idx + 1}
                    </span>
                    <span className="truncate">{node.name}</span>
                  </div>

                  <div className="flex items-center gap-1 ml-1">
                    <input
                      type="checkbox"
                      checked={node.enabled}
                      onChange={() =>
                        onToggleDagNode(selectedPart.id, node.id)
                      }
                      title="Включить / Подавить операцию в дереве DAG"
                      className="accent-[#f59e0b] cursor-pointer w-3 h-3"
                    />
                    {selectedPart.dag.length > 1 && (
                      <button
                        onClick={() =>
                          onDeleteDagNode(selectedPart.id, node.id)
                        }
                        title="Удалить операцию"
                        className="p-0.5 text-[#64748b] hover:text-[#ef4444] cursor-pointer"
                      >
                        <Trash2 className="w-3 h-3" />
                      </button>
                    )}
                  </div>
                </div>

                {isExpanded && (
                  <div className="p-2.5 space-y-2 border-t border-[#242936]">
                    {Object.entries(node.params).map(([key, val]) => {
                      if (typeof val === 'number') {
                        const step =
                          key.includes('teeth') ||
                          key.includes('sides') ||
                          key.includes('starts') ||
                          key.includes('spokes') ||
                          key.includes('count')
                            ? 1
                            : Math.abs(val) < 0.02
                              ? 0.0005
                              : 0.002;
                        return (
                          <div
                            key={key}
                            className="flex items-center justify-between gap-2 text-[11px]"
                          >
                            <label className="text-[#94a3b8] font-mono truncate">
                              {key}:
                            </label>
                            <input
                              type="number"
                              step={step}
                              value={val}
                              onChange={(e) => {
                                const num = parseFloat(e.target.value);
                                if (!Number.isNaN(num)) {
                                  onUpdateDagNodeParams(
                                    selectedPart.id,
                                    node.id,
                                    {
                                      ...node.params,
                                      [key]: num,
                                    }
                                  );
                                }
                              }}
                              className="w-24 bg-[#11141b] border border-[#2d3545] focus:border-[#f59e0b] rounded px-1.5 py-0.5 text-right font-mono text-[#f1f5f9] focus:outline-none tabular-nums"
                            />
                          </div>
                        );
                      }

                      if (
                        Array.isArray(val) &&
                        val.length === 3 &&
                        typeof val[0] === 'number'
                      ) {
                        return (
                          <div key={key} className="space-y-1 text-[10px]">
                            <span className="text-[#94a3b8] font-mono block">
                              {key} [x, y, z]:
                            </span>
                            <div className="grid grid-cols-3 gap-1">
                              {[0, 1, 2].map((axisIdx) => (
                                <input
                                  key={axisIdx}
                                  type="number"
                                  step={0.002}
                                  value={val[axisIdx]}
                                  onChange={(e) => {
                                    const num = parseFloat(e.target.value);
                                    if (!Number.isNaN(num)) {
                                      const nextArr = [...val];
                                      nextArr[axisIdx] = num;
                                      onUpdateDagNodeParams(
                                        selectedPart.id,
                                        node.id,
                                        {
                                          ...node.params,
                                          [key]: nextArr,
                                        }
                                      );
                                    }
                                  }}
                                  className="bg-[#11141b] border border-[#2d3545] rounded px-1 py-0.5 text-right font-mono text-[#cbd5e1] tabular-nums"
                                />
                              ))}
                            </div>
                          </div>
                        );
                      }

                      return (
                        <div
                          key={key}
                          className="flex items-center justify-between text-[11px]"
                        >
                          <span className="text-[#94a3b8] font-mono">
                            {key}:
                          </span>
                          <span className="text-[#cbd5e1] font-mono truncate max-w-[120px]">
                            {JSON.stringify(val)}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </div>
    );
  };

  const renderMaterialsSection = () => {
    if (!selectedPart) return null;
    return (
      <div className="space-y-2.5">
        <div className="flex items-center justify-between">
          <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b] truncate">
            Материал: {selectedPart.name}
          </span>
          <button
            onClick={() => setShowNewMatForm((v) => !v)}
            className="flex items-center gap-1 px-2 py-0.5 rounded bg-[#10b981]/15 hover:bg-[#10b981]/25 text-[#10b981] border border-[#10b981]/40 text-[10px] font-mono cursor-pointer shrink-0"
          >
            <Plus className="w-3 h-3" />
            <span>Свой сплав</span>
          </button>
        </div>

        {showNewMatForm && (
          <form
            onSubmit={handleDefineMaterialSubmit}
            className="bg-[#181c24] border border-[#10b981]/50 rounded p-2.5 space-y-2 text-xs"
          >
            <div className="text-[11px] font-semibold text-[#10b981]">
              Новый изотропный материал
            </div>
            <input
              type="text"
              placeholder="Название сплава"
              value={newMatName}
              onChange={(e) => setNewMatName(e.target.value)}
              className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs text-[#f1f5f9]"
            />
            <div className="grid grid-cols-2 gap-1.5 text-[10px] font-mono">
              <div>
                <label className="text-[#94a3b8] block">Плотность (кг/м³)</label>
                <input
                  type="number"
                  value={newMatDensity}
                  onChange={(e) => setNewMatDensity(Number(e.target.value))}
                  className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-0.5 text-[#f1f5f9]"
                />
              </div>
              <div>
                <label className="text-[#94a3b8] block">Модуль Юнга (ГПа)</label>
                <input
                  type="number"
                  value={newMatE}
                  onChange={(e) => setNewMatE(Number(e.target.value))}
                  className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-0.5 text-[#f1f5f9]"
                />
              </div>
              <div>
                <label className="text-[#94a3b8] block">Текучесть (МПа)</label>
                <input
                  type="number"
                  value={newMatYield}
                  onChange={(e) => setNewMatYield(Number(e.target.value))}
                  className="w-full bg-[#11141b] border border-[#2d3545] rounded px-1.5 py-0.5 text-[#f1f5f9]"
                />
              </div>
              <div>
                <label className="text-[#94a3b8] block">Пуассон ν и Цвет</label>
                <div className="flex gap-1">
                  <input
                    type="number"
                    step={0.01}
                    value={newMatNu}
                    onChange={(e) => setNewMatNu(Number(e.target.value))}
                    className="w-14 bg-[#11141b] border border-[#2d3545] rounded px-1 py-0.5 text-[#f1f5f9]"
                  />
                  <input
                    type="color"
                    value={newMatColor}
                    onChange={(e) => setNewMatColor(e.target.value)}
                    className="w-7 h-5 bg-transparent cursor-pointer"
                  />
                </div>
              </div>
            </div>
            <button
              type="submit"
              className="w-full py-1 rounded bg-[#10b981] hover:bg-[#10b981]/90 text-[#0c0e12] font-semibold text-xs cursor-pointer"
            >
              Сохранить и назначить
            </button>
          </form>
        )}

        {materials.map((mat) => {
          const isAssigned = selectedPart.materialId === mat.id;
          return (
            <div
              key={mat.id}
              onClick={() => onAssignMaterial(selectedPart.id, mat.id)}
              className={`p-2.5 rounded border cursor-pointer transition-colors ${
                isAssigned
                  ? 'bg-[#1d2534] border-[#10b981] text-[#f1f5f9]'
                  : 'bg-[#181c24] border-[#262b36] text-[#cbd5e1] hover:bg-[#1c212c]'
              }`}
            >
              <div className="flex items-center justify-between mb-1.5">
                <div className="flex items-center gap-2 min-w-0">
                  <span
                    className="w-3.5 h-3.5 rounded-full border border-white/20 shrink-0"
                    style={{ backgroundColor: mat.colorHex }}
                  />
                  <span className="text-xs font-semibold truncate">
                    {mat.name}
                  </span>
                </div>
                <span className="text-[10px] font-mono text-[#94a3b8] shrink-0">
                  {mat.category}
                </span>
              </div>

              <div className="grid grid-cols-3 gap-1 text-[10px] font-mono text-[#94a3b8] tabular-nums">
                <div>
                  <span className="text-[#64748b] block">Плотность</span>
                  {(mat.density / 1000).toFixed(2)} г/см³
                </div>
                <div>
                  <span className="text-[#64748b] block">Модуль E</span>
                  {(mat.youngsModulus / 1e9).toFixed(1)} ГПа
                </div>
                <div>
                  <span className="text-[#64748b] block">Предел σy</span>
                  {(mat.yieldStrength / 1e6).toFixed(0)} МПа
                </div>
              </div>
            </div>
          );
        })}
      </div>
    );
  };

  return (
    <aside className="w-72 bg-[#14171f] border-r border-[#262b36] flex flex-col select-none shrink-0">
      {/* Contextual Top Header per Active Engineering Workbench */}
      {activeWorkbench === 'modeling' && (
        <div className="grid grid-cols-2 border-b border-[#262b36] bg-[#10131a]">
          <button
            onClick={() => setModelingSubTab('dag')}
            className={`flex items-center justify-center gap-1.5 py-2.5 text-xs font-medium border-b-2 transition-colors cursor-pointer ${
              modelingSubTab === 'dag'
                ? 'border-[#38bdf8] text-[#f1f5f9] bg-[#161a24]'
                : 'border-transparent text-[#94a3b8] hover:text-[#cbd5e1]'
            }`}
          >
            <GitBranch className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>Дерево DAG</span>
          </button>

          <button
            onClick={() => setModelingSubTab('materials')}
            className={`flex items-center justify-center gap-1.5 py-2.5 text-xs font-medium border-b-2 transition-colors cursor-pointer ${
              modelingSubTab === 'materials'
                ? 'border-[#10b981] text-[#f1f5f9] bg-[#161a24]'
                : 'border-transparent text-[#94a3b8] hover:text-[#cbd5e1]'
            }`}
          >
            <Palette className="w-3.5 h-3.5 text-[#10b981]" />
            <span>Материалы</span>
          </button>
        </div>
      )}

      {activeWorkbench === 'fea' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <ShieldAlert className="w-3.5 h-3.5 text-[#ef4444]" />
            <span>Деталь и Свойства материала</span>
          </div>
          <span className="text-[10px] font-mono text-[#ef4444]">E, ν, σy</span>
        </div>
      )}

      {activeWorkbench === 'simulation' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <Link2 className="w-3.5 h-3.5 text-[#10b981]" />
            <span>Кинематические пары ({joints.length})</span>
          </div>
          <span className="text-[10px] font-mono text-[#10b981]">Rapier3D</span>
        </div>
      )}

      {activeWorkbench === 'optimizer' && (
        <div className="px-3 py-2.5 border-b border-[#262b36] bg-[#10131a] flex items-center justify-between">
          <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
            <GitBranch className="w-3.5 h-3.5 text-[#f59e0b]" />
            <span>Исходное / Оптим. дерево DAG</span>
          </div>
          <span className="text-[10px] font-mono text-[#f59e0b]">
            {selectedPart?.dag.length ?? 0} узлов
          </span>
        </div>
      )}

      {/* Contextual Body Content */}
      <div className="flex-1 overflow-y-auto p-3 space-y-4">
        {activeWorkbench === 'modeling' &&
          (modelingSubTab === 'dag' ? (
            <>
              {renderPartsList()}
              {renderDagTree()}
            </>
          ) : (
            <>
              {renderPartsList()}
              {renderMaterialsSection()}
            </>
          ))}

        {activeWorkbench === 'fea' && (
          <>
            {renderPartsList('Исследуемая деталь')}

            {/* Active Material Mechanical Tensor Summary */}
            {selectedPart && (
              <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2">
                <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
                  <span className="text-[11px] font-semibold text-[#f1f5f9]">
                    Тензор упругости (Закон Гука)
                  </span>
                  <span className="text-[10px] font-mono text-[#ef4444]">
                    {activeMaterial.name.split(' ')[0]}
                  </span>
                </div>
                <div className="grid grid-cols-2 gap-1.5 text-[10px] font-mono tabular-nums">
                  <div className="bg-[#12151c] p-1.5 rounded border border-[#222734]">
                    <span className="text-[#64748b] block">Модуль Юнга E</span>
                    <span className="text-[#38bdf8] font-semibold">
                      {(activeMaterial.youngsModulus / 1e9).toFixed(1)} ГПа
                    </span>
                  </div>
                  <div className="bg-[#12151c] p-1.5 rounded border border-[#222734]">
                    <span className="text-[#64748b] block">Предел текуч. σy</span>
                    <span className="text-[#ef4444] font-semibold">
                      {(activeMaterial.yieldStrength / 1e6).toFixed(0)} МПа
                    </span>
                  </div>
                  <div className="bg-[#12151c] p-1.5 rounded border border-[#222734]">
                    <span className="text-[#64748b] block">Коэф. Пуассона ν</span>
                    <span className="text-[#cbd5e1]">
                      {activeMaterial.poissonsRatio.toFixed(2)}
                    </span>
                  </div>
                  <div className="bg-[#12151c] p-1.5 rounded border border-[#222734]">
                    <span className="text-[#64748b] block">Масса детали</span>
                    <span className="text-[#f59e0b]">
                      {(selectedPart.mesh.mass * 1000).toFixed(1)} г
                    </span>
                  </div>
                </div>
              </div>
            )}

            {renderMaterialsSection()}
          </>
        )}

        {activeWorkbench === 'simulation' && (
          <div className="space-y-3">
            {renderPartsList('Звенья механизма')}

            <div className="pt-2 border-t border-[#262b36] flex items-center justify-between">
              <span className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                Шарниры и Приводы ({joints.length})
              </span>
              <button
                onClick={() => setShowNewJointForm((v) => !v)}
                className="flex items-center gap-1 px-2 py-0.5 rounded bg-[#10b981]/15 hover:bg-[#10b981]/25 text-[#10b981] border border-[#10b981]/40 text-[10px] font-mono cursor-pointer"
              >
                <Plus className="w-3 h-3" />
                <span>Добавить шарнир</span>
              </button>
            </div>

            {showNewJointForm && (
              <form
                onSubmit={handleCreateJointSubmit}
                className="bg-[#181c24] border border-[#10b981]/50 rounded p-2.5 space-y-2 text-xs"
              >
                <div className="text-[11px] font-semibold text-[#10b981] mb-1">
                  Новая кинематическая пара (create_joint)
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-[11px] text-[#94a3b8]">Тип связи:</span>
                  <select
                    value={newJointType}
                    onChange={(e) =>
                      setNewJointType(e.target.value as KinematicJoint['type'])
                    }
                    className="bg-[#11141b] border border-[#2d3545] rounded px-2 py-0.5 text-xs text-[#f1f5f9]"
                  >
                    <option value="revolute">Вращательная (Revolute)</option>
                    <option value="prismatic">Поступательная (Ползун)</option>
                    <option value="gear">Зубчатая передача (Gear)</option>
                    <option value="fixed">Жесткая фиксация (Fixed)</option>
                    <option value="spherical">Шаровой шарнир</option>
                  </select>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-[11px] text-[#94a3b8]">Звено A:</span>
                  <select
                    value={newJointPartA}
                    onChange={(e) => setNewJointPartA(e.target.value)}
                    className="bg-[#11141b] border border-[#2d3545] rounded px-2 py-0.5 text-xs text-[#f1f5f9]"
                  >
                    <option value="world">world (Станина)</option>
                    {parts.map((p) => (
                      <option key={p.id} value={p.id}>
                        {p.name}
                      </option>
                    ))}
                  </select>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-[11px] text-[#94a3b8]">Звено B:</span>
                  <select
                    value={newJointPartB || selectedPart?.id || ''}
                    onChange={(e) => setNewJointPartB(e.target.value)}
                    className="bg-[#11141b] border border-[#2d3545] rounded px-2 py-0.5 text-xs text-[#f1f5f9]"
                  >
                    {parts.map((p) => (
                      <option key={p.id} value={p.id}>
                        {p.name}
                      </option>
                    ))}
                  </select>
                </div>

                <div className="flex items-center justify-between">
                  <span className="text-[11px] text-[#94a3b8]">Ось и об/мин:</span>
                  <div className="flex items-center gap-1">
                    {(['X', 'Y', 'Z'] as const).map((ax) => (
                      <button
                        type="button"
                        key={ax}
                        onClick={() => setNewJointAxis(ax)}
                        className={`px-1.5 py-0.5 rounded text-[10px] font-mono ${
                          newJointAxis === ax
                            ? 'bg-[#10b981] text-[#0c0e12] font-bold'
                            : 'bg-[#11141b] text-[#94a3b8]'
                        }`}
                      >
                        {ax}
                      </button>
                    ))}
                    <input
                      type="number"
                      value={newJointRpm}
                      onChange={(e) => setNewJointRpm(Number(e.target.value))}
                      className="w-14 bg-[#11141b] border border-[#2d3545] rounded px-1 py-0.5 text-right font-mono text-[11px] text-[#f1f5f9]"
                    />
                  </div>
                </div>

                <button
                  type="submit"
                  className="w-full py-1 rounded bg-[#10b981] hover:bg-[#10b981]/90 text-[#0c0e12] font-semibold text-xs flex items-center justify-center gap-1 cursor-pointer"
                >
                  <Check className="w-3.5 h-3.5" />
                  <span>Создать шарнир</span>
                </button>
              </form>
            )}

            {joints.length === 0 && (
              <div className="bg-[#181c24] border border-[#242936] rounded p-3 text-xs text-[#94a3b8]">
                В текущей сцене нет активных шарниров. Нажмите «Добавить шарнир» или выберите сборку в верхней панели.
              </div>
            )}

            {joints.map((joint) => (
              <div
                key={joint.id}
                className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2"
              >
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-1.5 min-w-0">
                    <Zap
                      className={`w-3.5 h-3.5 shrink-0 ${
                        joint.motorEnabled ? 'text-[#f59e0b]' : 'text-[#64748b]'
                      }`}
                    />
                    <span className="text-xs font-medium text-[#f1f5f9] truncate">
                      {joint.name}
                    </span>
                  </div>
                  <div className="flex items-center gap-1 shrink-0">
                    <span className="text-[10px] font-mono uppercase text-[#38bdf8]">
                      {joint.type}
                    </span>
                    <button
                      onClick={() => onDeleteJoint(joint.id)}
                      title="Удалить шарнир"
                      className="p-0.5 text-[#64748b] hover:text-[#ef4444] cursor-pointer"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </div>
                </div>

                <div className="text-[11px] font-mono text-[#94a3b8] flex justify-between tabular-nums">
                  <span className="truncate">
                    {joint.partA} → {joint.partB}
                  </span>
                  <span className="text-[#10b981] shrink-0">
                    θ = {((joint.currentAngleRad * 180) / Math.PI).toFixed(1)}°
                  </span>
                </div>

                {(joint.type === 'revolute' || joint.type === 'prismatic') && (
                  <div className="pt-2 border-t border-[#242936] space-y-2">
                    <div className="flex items-center justify-between text-[11px]">
                      <span className="text-[#94a3b8]">Привод активен</span>
                      <div className="flex items-center gap-1.5">
                        <button
                          onClick={() =>
                            onUpdateJointMotor(
                              joint.id,
                              -joint.motorSpeedRpm,
                              joint.motorTorqueNm,
                              joint.motorEnabled
                            )
                          }
                          title="Реверс направления вращения"
                          className="p-0.5 rounded bg-[#222836] hover:bg-[#2d3545] text-[#cbd5e1] cursor-pointer"
                        >
                          <RotateCw className="w-3 h-3" />
                        </button>
                        <input
                          type="checkbox"
                          checked={joint.motorEnabled}
                          onChange={(e) =>
                            onUpdateJointMotor(
                              joint.id,
                              joint.motorSpeedRpm,
                              joint.motorTorqueNm,
                              e.target.checked
                            )
                          }
                          className="accent-[#10b981] cursor-pointer"
                        />
                      </div>
                    </div>

                    <div className="space-y-1">
                      <div className="flex justify-between text-[10px] font-mono tabular-nums">
                        <span className="text-[#64748b]">Скорость привода:</span>
                        <span className="text-[#f59e0b]">
                          {joint.motorSpeedRpm} об/мин
                        </span>
                      </div>
                      <input
                        type="range"
                        min={-120}
                        max={120}
                        step={5}
                        value={joint.motorSpeedRpm}
                        onChange={(e) =>
                          onUpdateJointMotor(
                            joint.id,
                            parseFloat(e.target.value),
                            joint.motorTorqueNm,
                            true
                          )
                        }
                        className="w-full accent-[#10b981] h-1 bg-[#262b36] rounded cursor-pointer"
                      />
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>
        )}

        {activeWorkbench === 'optimizer' && (
          <>
            {renderPartsList('Анализируемые тела')}
            {renderDagTree()}
          </>
        )}
      </div>
    </aside>
  );
};
