import React, { useState } from 'react';
import type { FeaResult, PartSummary, PhysicsSimulationResult } from '../types/cad';
import {
  Activity,
  CheckCircle2,
  Cpu,
  Gauge,
  Play,
  ShieldAlert,
  Sliders,
} from 'lucide-react';

interface EngineeringInspectorProps {
  parts: PartSummary[];
  selectedPartId: string | null;
  feaResult: FeaResult | null;
  physicsResult: PhysicsSimulationResult | null;
  geometryQueryResult: Record<string, unknown> | null;
  meshValidationResult: Record<string, unknown> | null;
  isBusy: boolean;
  onRunFea: (params: {
    part_id: string;
    mesh_density: 'coarse' | 'medium' | 'fine';
    fixed_faces: string[];
    force_vector: [number, number, number];
  }) => Promise<void>;
  onRunPhysics: (durationSeconds: number, stepSizeMs: number) => Promise<void>;
  onQueryGeometry: (partId: string) => Promise<void>;
  onValidateMesh: (partId: string) => Promise<void>;
  onTessellatePart: (partId: string, quality: 'coarse' | 'medium' | 'fine') => Promise<void>;
}

export const EngineeringInspector: React.FC<EngineeringInspectorProps> = ({
  parts,
  selectedPartId,
  feaResult,
  physicsResult,
  geometryQueryResult,
  meshValidationResult,
  isBusy,
  onRunFea,
  onRunPhysics,
  onQueryGeometry,
  onValidateMesh,
  onTessellatePart,
}) => {
  const [activeTab, setActiveTab] = useState<'fea' | 'physics' | 'qa'>('fea');

  // FEA form state
  const [meshDensity, setMeshDensity] = useState<'coarse' | 'medium' | 'fine'>('medium');
  const [fixedFace, setFixedFace] = useState('min_x');
  const [forceX, setForceX] = useState(0);
  const [forceY, setForceY] = useState(-800);
  const [forceZ, setForceZ] = useState(0);

  // Physics form state
  const [simDuration, setSimDuration] = useState(2.0);
  const [simStepMs, setSimStepMs] = useState(16.67);

  const selectedPart = parts.find((p) => p.id === selectedPartId) || parts[0] || null;

  const handleTriggerFea = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!selectedPart) return;
    await onRunFea({
      part_id: selectedPart.id,
      mesh_density: meshDensity,
      fixed_faces: [fixedFace],
      force_vector: [forceX, forceY, forceZ],
    });
  };

  return (
    <aside className="w-80 shrink-0 h-full bg-[#111827] border-l border-slate-800 flex flex-col overflow-hidden">
      {/* Inspector Tabs */}
      <div className="p-2 border-b border-slate-800 flex items-center gap-1 bg-[#0B0F17]/60">
        <button
          type="button"
          onClick={() => setActiveTab('fea')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeTab === 'fea'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          МКЭ (FEA)
        </button>
        <button
          type="button"
          onClick={() => setActiveTab('physics')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeTab === 'physics'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          Кинематика
        </button>
        <button
          type="button"
          onClick={() => setActiveTab('qa')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeTab === 'qa'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          Аудит сетки
        </button>
      </div>

      <div className="flex-1 overflow-y-auto p-3.5 space-y-4">
        {/* TAB 1: FEA Linear Elasticity Solver */}
        {activeTab === 'fea' && (
          <div className="space-y-4">
            <form
              onSubmit={handleTriggerFea}
              className="p-3 rounded-lg bg-slate-900/70 border border-slate-800 space-y-3"
            >
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Gauge className="w-3.5 h-3.5 text-amber-400" />
                <span>Расчёт прочности МКЭ (TET4)</span>
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Плотность сетки тетраэдров</label>
                <select
                  value={meshDensity}
                  onChange={(e) => setMeshDensity(e.target.value as 'coarse' | 'medium' | 'fine')}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-950 border border-slate-800 rounded-md text-white"
                >
                  <option value="coarse">Coarse (Быстрый)</option>
                  <option value="medium">Medium (Сбалансированный)</option>
                  <option value="fine">Fine (Высокая точность)</option>
                </select>
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Закреплённая грань Дирихле</label>
                <select
                  value={fixedFace}
                  onChange={(e) => setFixedFace(e.target.value)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-950 border border-slate-800 rounded-md text-white"
                >
                  <option value="min_x">Min X (-X)</option>
                  <option value="max_x">Max X (+X)</option>
                  <option value="min_y">Min Y (-Y)</option>
                  <option value="max_y">Max Y (+Y)</option>
                  <option value="min_z">Min Z (-Z)</option>
                  <option value="max_z">Max Z (+Z)</option>
                </select>
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">
                  Вектор внешней нагрузки [Fx, Fy, Fz] (Н)
                </label>
                <div className="grid grid-cols-3 gap-2">
                  <input
                    type="number"
                    aria-label="Сила Fx"
                    value={forceX}
                    onChange={(e) => setForceX(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                  <input
                    type="number"
                    aria-label="Сила Fy"
                    value={forceY}
                    onChange={(e) => setForceY(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                  <input
                    type="number"
                    aria-label="Сила Fz"
                    value={forceZ}
                    onChange={(e) => setForceZ(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                </div>
              </div>

              <button
                type="submit"
                disabled={isBusy || !selectedPart}
                className="w-full py-2 px-3 text-xs font-semibold bg-amber-500 hover:bg-amber-400 disabled:opacity-40 text-slate-950 rounded-md transition-colors flex items-center justify-center gap-1.5"
              >
                <Play className="w-3.5 h-3.5" />
                <span>Запустить МКЭ + Тепловую карту</span>
              </button>
            </form>

            {/* FEA Telemetry Report */}
            {feaResult ? (
              <div className="p-3.5 rounded-lg bg-slate-900 border border-slate-800 space-y-3">
                <div className="flex items-center justify-between border-b border-slate-800 pb-2">
                  <span className="text-xs font-semibold text-white">Вердикт прочности</span>
                  <span
                    className={`text-xs font-mono font-semibold flex items-center gap-1 ${
                      feaResult.status === 'PASS' ? 'text-emerald-400' : 'text-red-400'
                    }`}
                  >
                    {feaResult.status === 'PASS' ? (
                      <CheckCircle2 className="w-3.5 h-3.5" />
                    ) : (
                      <ShieldAlert className="w-3.5 h-3.5" />
                    )}
                    {feaResult.status}
                  </span>
                </div>

                <div className="grid grid-cols-2 gap-2.5 text-xs">
                  <div className="p-2 rounded bg-slate-950 border border-slate-800/80">
                    <div className="text-[10px] text-slate-400">Запас прочности (FoS)</div>
                    <div className="text-sm font-mono font-semibold text-amber-400 mt-0.5">
                      {Number(feaResult.safety_factor).toFixed(2)}
                    </div>
                  </div>
                  <div className="p-2 rounded bg-slate-950 border border-slate-800/80">
                    <div className="text-[10px] text-slate-400">Макс. Мизес (σ_vM)</div>
                    <div className="text-sm font-mono font-semibold text-white mt-0.5">
                      {Number(feaResult.max_von_mises_mpa).toFixed(2)} МПа
                    </div>
                  </div>
                  <div className="p-2 rounded bg-slate-950 border border-slate-800/80">
                    <div className="text-[10px] text-slate-400">Предел текучести</div>
                    <div className="text-xs font-mono text-slate-300 mt-0.5">
                      {Number(feaResult.yield_strength_mpa).toFixed(1)} МПа
                    </div>
                  </div>
                  <div className="p-2 rounded bg-slate-950 border border-slate-800/80">
                    <div className="text-[10px] text-slate-400">Макс. прогиб</div>
                    <div className="text-xs font-mono text-slate-300 mt-0.5">
                      {Number(feaResult.max_displacement_mm).toFixed(4)} мм
                    </div>
                  </div>
                </div>

                <div className="text-[11px] font-mono text-slate-400 flex items-center justify-between pt-1 tabular-nums">
                  <span>Узлов: {feaResult.node_count}</span>
                  <span>Тетраэдров: {feaResult.element_count}</span>
                  <span>{Math.round(Number(feaResult.computation_time_ms || 0))} мс</span>
                </div>
              </div>
            ) : (
              <div className="p-3 rounded-lg bg-slate-900/50 border border-slate-800/80 text-xs text-slate-400">
                Выберите деталь и нажмите «Запустить МКЭ», чтобы построить TET4-сетку, решить систему $K \cdot u = F$ и отрендерить 3D тепловую карту напряжений.
              </div>
            )}
          </div>
        )}

        {/* TAB 2: Rapier3D Kinematics & Gear Transmission */}
        {activeTab === 'physics' && (
          <div className="space-y-4">
            <div className="p-3 rounded-lg bg-slate-900/70 border border-slate-800 space-y-3">
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Activity className="w-3.5 h-3.5 text-amber-400" />
                <span>Динамика и зубчатые передачи (Rapier3D)</span>
              </div>

              <div className="grid grid-cols-2 gap-2">
                <div>
                  <label className="block text-[10px] text-slate-400 mb-1">Длительность (с)</label>
                  <input
                    type="number"
                    step="0.5"
                    min="0.1"
                    value={simDuration}
                    onChange={(e) => setSimDuration(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                </div>
                <div>
                  <label className="block text-[10px] text-slate-400 mb-1">Шаг интеграции (мс)</label>
                  <input
                    type="number"
                    step="1"
                    min="1"
                    value={simStepMs}
                    onChange={(e) => setSimStepMs(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                </div>
              </div>

              <button
                type="button"
                disabled={isBusy || parts.length === 0}
                onClick={() => onRunPhysics(simDuration, simStepMs)}
                className="w-full py-2 px-3 text-xs font-semibold bg-amber-500 hover:bg-amber-400 disabled:opacity-40 text-slate-950 rounded-md transition-colors flex items-center justify-center gap-1.5"
              >
                <Play className="w-3.5 h-3.5" />
                <span>Запустить симуляцию (`run_physics_simulation`)</span>
              </button>
            </div>

            {physicsResult ? (
              <div className="p-3.5 rounded-lg bg-slate-900 border border-slate-800 space-y-3">
                <div className="flex items-center justify-between border-b border-slate-800 pb-2 text-xs">
                  <span className="font-semibold text-white">Отчёт Rapier3D</span>
                  <span className="font-mono text-emerald-400">
                    {physicsResult.steps_simulated} шагов
                  </span>
                </div>

                {/* Peak Joint Forces */}
                {Object.keys(physicsResult.peak_joint_forces || {}).length > 0 && (
                  <div className="space-y-1">
                    <div className="text-[11px] font-medium text-slate-400">
                      Пиковые реакции в шарнирах (Н):
                    </div>
                    {Object.entries(physicsResult.peak_joint_forces).map(([jId, force]) => (
                      <div
                        key={jId}
                        className="flex items-center justify-between text-xs font-mono bg-slate-950 px-2.5 py-1 rounded border border-slate-800"
                      >
                        <span className="truncate text-slate-300">{jId.slice(0, 14)}</span>
                        <span className="text-amber-400">{Number(force).toFixed(2)} Н</span>
                      </div>
                    ))}
                  </div>
                )}

                {/* Trajectories Summary */}
                <div className="space-y-1.5">
                  <div className="text-[11px] font-medium text-slate-400">
                    Конечные состояния звеньев:
                  </div>
                  {Object.entries(physicsResult.trajectory_summary || {}).map(([pId, traj]) => {
                    const last = traj.samples?.[traj.samples.length - 1];
                    const partName = parts.find((p) => p.id === pId)?.name || pId.slice(0, 10);
                    return (
                      <div
                        key={pId}
                        className="p-2 rounded bg-slate-950 border border-slate-800 text-[11px] font-mono space-y-0.5"
                      >
                        <div className="text-white font-sans font-semibold">{partName}</div>
                        {last && (
                          <>
                            <div className="text-slate-400">
                              pos: [{last.position.map((v) => v.toFixed(3)).join(', ')}] м
                            </div>
                            <div className="text-amber-400">
                              ω: [{last.angular_velocity.map((v) => v.toFixed(2)).join(', ')}] рад/с
                            </div>
                          </>
                        )}
                      </div>
                    );
                  })}
                </div>
              </div>
            ) : (
              <div className="p-3 rounded-lg bg-slate-900/50 border border-slate-800/80 text-xs text-slate-400">
                Запустите сценарий «02. Зубчатая передача» или «03. Кривошипно-ползунный привод» слева, чтобы увидеть расчёт кинематики, угловых скоростей и реакций в опорах.
              </div>
            )}
          </div>
        )}

        {/* TAB 3: Mesh QA, Tessellation & Geometry Query */}
        {activeTab === 'qa' && (
          <div className="space-y-4">
            <div className="p-3 rounded-lg bg-slate-900/70 border border-slate-800 space-y-2.5">
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Sliders className="w-3.5 h-3.5 text-amber-400" />
                <span>Адаптивная тесселяция и топология</span>
              </div>

              <div className="grid grid-cols-3 gap-1.5">
                {(['coarse', 'medium', 'fine'] as const).map((q) => (
                  <button
                    key={q}
                    type="button"
                    disabled={isBusy || !selectedPart}
                    onClick={() => selectedPart && onTessellatePart(selectedPart.id, q)}
                    className="py-1.5 px-2 text-xs font-medium bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-200 rounded transition-colors capitalize"
                  >
                    {q}
                  </button>
                ))}
              </div>

              <div className="grid grid-cols-2 gap-2 pt-1">
                <button
                  type="button"
                  disabled={isBusy || !selectedPart}
                  onClick={() => selectedPart && onQueryGeometry(selectedPart.id)}
                  className="py-1.5 px-2.5 text-xs font-medium bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-white rounded-md transition-colors"
                >
                  Метрики геометрии
                </button>
                <button
                  type="button"
                  disabled={isBusy || !selectedPart}
                  onClick={() => selectedPart && onValidateMesh(selectedPart.id)}
                  className="py-1.5 px-2.5 text-xs font-medium bg-amber-500 hover:bg-amber-400 disabled:opacity-40 text-slate-950 font-semibold rounded-md transition-colors"
                >
                  Проверить сетку
                </button>
              </div>
            </div>

            {meshValidationResult && (
              <div className="p-3 rounded-lg bg-slate-900 border border-slate-800 space-y-1.5">
                <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                  <Cpu className="w-3.5 h-3.5 text-emerald-400" />
                  <span>Результат `validate_mesh`</span>
                </div>
                <pre className="text-[10px] font-mono text-slate-300 bg-slate-950 p-2.5 rounded overflow-x-auto">
                  {JSON.stringify(meshValidationResult, null, 2)}
                </pre>
              </div>
            )}

            {geometryQueryResult && (
              <div className="p-3 rounded-lg bg-slate-900 border border-slate-800 space-y-1.5">
                <div className="text-xs font-semibold text-white">
                  Результат `query_part_geometry`
                </div>
                <pre className="text-[10px] font-mono text-slate-300 bg-slate-950 p-2.5 rounded overflow-x-auto">
                  {JSON.stringify(geometryQueryResult, null, 2)}
                </pre>
              </div>
            )}
          </div>
        )}
      </div>
    </aside>
  );
};
