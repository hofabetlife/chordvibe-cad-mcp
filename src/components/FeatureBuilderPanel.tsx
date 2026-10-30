import React, { useState } from 'react';
import type { DagNode, PartSummary } from '../types/cad';
import {
  Box,
  Cpu,
  FolderOpen,
  GitBranch,
  Layers,
  Play,
  Plus,
  Scissors,
  Trash2,
  Wrench,
} from 'lucide-react';

const MATERIALS = [
  { id: 'aluminum_6061_t6', label: 'Aluminum 6061-T6 (2700 кг/м³)' },
  { id: 'steel_structural_aisi1020', label: 'Steel AISI 1020 (7850 кг/м³)' },
  { id: 'steel_high_strength_4140', label: 'Steel 4140 (7850 кг/м³)' },
  { id: 'titanium_grade5_ti6al4v', label: 'Titanium Ti-6Al-4V (4430 кг/м³)' },
  { id: 'brass_c360', label: 'Brass C360 (8500 кг/м³)' },
  { id: 'pla_filament_3dprint', label: 'PLA 3D Print (1240 кг/м³)' },
  { id: 'petg_filament_3dprint', label: 'PETG 3D Print (1270 кг/м³)' },
  { id: 'nylon_pa12', label: 'Nylon PA12 (1010 кг/м³)' },
];

export type ScenarioPresetId =
  | 'gear_transmission'
  | 'aerospace_bracket'
  | 'crank_mechanism'
  | 'forged_wheel_rim'
  | 'din934_hex_nut';

interface FeatureBuilderPanelProps {
  parts: PartSummary[];
  selectedPartId: string | null;
  dagNodes: DagNode[];
  outputStls: Array<{ name: string; filepath: string; sizeBytes: number }>;
  isBusy: boolean;
  onSelectPart: (partId: string) => void;
  onDeletePart: (partId: string) => void;
  onClearAllParts: () => void;
  onRunScenario: (preset: ScenarioPresetId) => void;
  onImportStl: (filepath: string, material: string) => void;
  onCallTool: (toolName: string, args: Record<string, unknown>) => Promise<void>;
}

export const FeatureBuilderPanel: React.FC<FeatureBuilderPanelProps> = ({
  parts,
  selectedPartId,
  dagNodes,
  outputStls,
  isBusy,
  onSelectPart,
  onDeletePart,
  onClearAllParts,
  onRunScenario,
  onImportStl,
  onCallTool,
}) => {
  const [activeSection, setActiveSection] = useState<'scenarios' | 'builder' | 'dag'>('scenarios');

  // Create Part state
  const [newPartName, setNewPartName] = useState('custom_component');
  const [newPartMaterial, setNewPartMaterial] = useState('aluminum_6061_t6');
  const [newPartFixed, setNewPartFixed] = useState(false);

  // Add Primitive state
  const [primitiveType, setPrimitiveType] = useState<
    'box' | 'cylinder' | 'sphere' | 'cone' | 'polygon_prism' | 'spur_gear' | 'wheel_rim'
  >('spur_gear');

  // Primitive params
  const [boxW, setBoxW] = useState(0.12);
  const [boxH, setBoxH] = useState(0.02);
  const [boxD, setBoxD] = useState(0.06);

  const [cylR, setCylR] = useState(0.025);
  const [cylH, setCylH] = useState(0.08);

  const [gearModule, setGearModule] = useState(0.0025);
  const [gearTeeth, setGearTeeth] = useState(24);
  const [gearFaceWidth, setGearFaceWidth] = useState(0.015);
  const [gearBoreD, setGearBoreD] = useState(0.014);
  const [gearHubD, setGearHubD] = useState(0.028);

  const [prismSides, setPrismSides] = useState(6);
  const [prismWaf, setPrismWaf] = useState(0.016);
  const [prismH, setPrismH] = useState(0.008);
  const [prismChamfer, setPrismChamfer] = useState(0.0012);

  // Cut Hole state
  const [holePos, setHolePos] = useState<[number, number, number]>([0, 0, 0]);
  const [holeRadius, setHoleRadius] = useState(0.005);
  const [holeDepth, setHoleDepth] = useState(0.04);
  const [holeKeywayW, setHoleKeywayW] = useState(0);
  const [holeKeywayD, setHoleKeywayD] = useState(0);

  const handleCreatePart = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!newPartName.trim()) return;
    await onCallTool('create_part', {
      name: newPartName.trim(),
      material: newPartMaterial,
      is_fixed: newPartFixed,
    });
  };

  const handleAddPrimitive = async () => {
    if (!selectedPartId) return;
    let params: Record<string, unknown> = {};
    if (primitiveType === 'box') {
      params = { width: boxW, height: boxH, depth: boxD };
    } else if (primitiveType === 'cylinder') {
      params = { radius: cylR, height: cylH, segments: 48 };
    } else if (primitiveType === 'sphere') {
      params = { radius: cylR, sectors: 36, stacks: 24 };
    } else if (primitiveType === 'cone') {
      params = { radius_bottom: cylR, radius_top: cylR * 0.4, height: cylH, segments: 48 };
    } else if (primitiveType === 'spur_gear') {
      params = {
        module: gearModule,
        teeth: gearTeeth,
        pressure_angle_deg: 20.0,
        face_width: gearFaceWidth,
        bore_diameter: gearBoreD,
        hub_diameter: gearHubD,
        hub_length: gearFaceWidth * 0.5,
        keyway_width: gearBoreD * 0.28,
        keyway_depth: gearBoreD * 0.14,
      };
    } else if (primitiveType === 'polygon_prism') {
      params = {
        sides: prismSides,
        width_across_flats: prismWaf,
        height: prismH,
        chamfer: prismChamfer,
      };
    } else if (primitiveType === 'wheel_rim') {
      params = {
        rim_diameter: 0.4572,
        rim_width: 0.2032,
        flange_diameter: 0.49,
        hub_diameter: 0.16,
        hub_bore_diameter: 0.066,
        pcd_diameter: 0.1143,
        bolt_holes_count: 5,
        bolt_hole_diameter: 0.014,
        spokes_count: 5,
        spoke_width: 0.028,
        spoke_thickness: 0.018,
        offset_et: 0.035,
      };
    }

    await onCallTool('add_feature_primitive', {
      part_id: selectedPartId,
      primitive_type: primitiveType,
      params,
    });
  };

  const handleCutHole = async () => {
    if (!selectedPartId) return;
    const args: Record<string, unknown> = {
      part_id: selectedPartId,
      position: holePos,
      normal: [0, 1, 0],
      radius: holeRadius,
      depth: holeDepth,
    };
    if (holeKeywayW > 0 && holeKeywayD > 0) {
      args.keyway_width = holeKeywayW;
      args.keyway_depth = holeKeywayD;
    }
    await onCallTool('cut_hole', args);
  };

  return (
    <aside className="w-80 shrink-0 h-full bg-[#111827] border-r border-slate-800 flex flex-col overflow-hidden">
      {/* Navigation Tabs */}
      <div className="p-2 border-b border-slate-800 flex items-center gap-1 bg-[#0B0F17]/60">
        <button
          type="button"
          onClick={() => setActiveSection('scenarios')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeSection === 'scenarios'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          Сценарии
        </button>
        <button
          type="button"
          onClick={() => setActiveSection('builder')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeSection === 'builder'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          Конструктор
        </button>
        <button
          type="button"
          onClick={() => setActiveSection('dag')}
          className={`flex-1 py-1.5 px-2 text-xs font-medium rounded-md transition-colors whitespace-nowrap ${
            activeSection === 'dag'
              ? 'bg-slate-800 text-amber-400 font-semibold'
              : 'text-slate-400 hover:text-slate-200'
          }`}
        >
          Дерево DAG ({dagNodes.length})
        </button>
      </div>

      {/* Scrollable Body */}
      <div className="flex-1 overflow-y-auto p-3.5 space-y-5">
        {/* Active Parts List (Always visible at top) */}
        <div>
          <div className="flex items-center justify-between mb-2">
            <h2 className="text-xs font-semibold text-slate-300 flex items-center gap-1.5">
              <Layers className="w-3.5 h-3.5 text-amber-400" />
              <span>Детали в сцене ({parts.length})</span>
            </h2>
            {parts.length > 0 && (
              <button
                type="button"
                onClick={onClearAllParts}
                disabled={isBusy}
                className="text-[11px] text-slate-400 hover:text-red-400 transition-colors"
              >
                Очистить сцену
              </button>
            )}
          </div>

          {parts.length === 0 ? (
            <div className="p-3 rounded-lg bg-slate-900/70 border border-slate-800/80 text-xs text-slate-400">
              Деталей пока нет. Запустите готовый сценарий ниже или создайте деталь во вкладке «Конструктор».
            </div>
          ) : (
            <div className="space-y-1.5">
              {parts.map((part) => {
                const isSelected = part.id === selectedPartId;
                return (
                  <div
                    key={part.id}
                    className={`flex items-center justify-between p-2.5 rounded-lg border transition-colors ${
                      isSelected
                        ? 'bg-amber-500/10 border-amber-500/50 text-white'
                        : 'bg-slate-900/60 border-slate-800 text-slate-300 hover:border-slate-700'
                    }`}
                  >
                    <button
                      type="button"
                      onClick={() => onSelectPart(part.id)}
                      className="flex-1 text-left min-w-0"
                    >
                      <div className="text-xs font-semibold truncate">{part.name}</div>
                      <div className="text-[11px] text-slate-400 font-mono truncate mt-0.5">
                        {part.material_name} · {part.triangle_count} tris
                      </div>
                    </button>
                    <button
                      type="button"
                      aria-label={`Удалить деталь ${part.name}`}
                      onClick={() => onDeletePart(part.id)}
                      disabled={isBusy}
                      className="p-1.5 text-slate-500 hover:text-red-400 rounded-md transition-colors shrink-0"
                    >
                      <Trash2 className="w-3.5 h-3.5" />
                    </button>
                  </div>
                );
              })}
            </div>
          )}
        </div>

        {/* SECTION 1: Engineering Scenarios & Prebuilt STL Loader */}
        {activeSection === 'scenarios' && (
          <div className="space-y-4">
            <div>
              <h3 className="text-xs font-semibold text-slate-300 mb-2 flex items-center gap-1.5">
                <Cpu className="w-3.5 h-3.5 text-amber-400" />
                <span>Инженерные сценарии (Rust E2E)</span>
              </h3>
              <div className="space-y-2">
                <button
                  type="button"
                  disabled={isBusy}
                  onClick={() => onRunScenario('aerospace_bracket')}
                  className="w-full text-left p-3 rounded-lg bg-slate-900 hover:bg-slate-800/90 border border-slate-800 hover:border-amber-500/40 transition-colors group"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-semibold text-white group-hover:text-amber-400">
                      01. Силовой кронштейн + МКЭ (FEA)
                    </span>
                    <Play className="w-3.5 h-3.5 text-amber-400 shrink-0" />
                  </div>
                  <p className="mt-1 text-[11px] text-slate-400 leading-snug">
                    Алюминий 6061-T6 (120×15×60 мм), 2 отверстия M8, нагрузка 800 Н, расчёт по Мизесу и тепловая карта.
                  </p>
                </button>

                <button
                  type="button"
                  disabled={isBusy}
                  onClick={() => onRunScenario('gear_transmission')}
                  className="w-full text-left p-3 rounded-lg bg-slate-900 hover:bg-slate-800/90 border border-slate-800 hover:border-amber-500/40 transition-colors group"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-semibold text-white group-hover:text-amber-400">
                      02. Зубчатая передача (z1=20, z2=40)
                    </span>
                    <Play className="w-3.5 h-3.5 text-amber-400 shrink-0" />
                  </div>
                  <p className="mt-1 text-[11px] text-slate-400 leading-snug">
                    Неподвижная рама (`is_fixed`), ведущая шестерня (20 зубьев), ведомое колесо (40 зубьев), сочленение `gear` и симуляция.
                  </p>
                </button>

                <button
                  type="button"
                  disabled={isBusy}
                  onClick={() => onRunScenario('crank_mechanism')}
                  className="w-full text-left p-3 rounded-lg bg-slate-900 hover:bg-slate-800/90 border border-slate-800 hover:border-amber-500/40 transition-colors group"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-semibold text-white group-hover:text-amber-400">
                      03. Кривошипно-ползунный привод
                    </span>
                    <Play className="w-3.5 h-3.5 text-amber-400 shrink-0" />
                  </div>
                  <p className="mt-1 text-[11px] text-slate-400 leading-snug">
                    Станина, кривошип, шатун и латунный ползун с сервомотором 6.28 рад/с и телеметрией Rapier3D.
                  </p>
                </button>

                <button
                  type="button"
                  disabled={isBusy}
                  onClick={() => onRunScenario('din934_hex_nut')}
                  className="w-full text-left p-3 rounded-lg bg-slate-900 hover:bg-slate-800/90 border border-slate-800 hover:border-amber-500/40 transition-colors group"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-semibold text-white group-hover:text-amber-400">
                      04. Гайка M10 DIN 934 с резьбой 60°
                    </span>
                    <Play className="w-3.5 h-3.5 text-amber-400 shrink-0" />
                  </div>
                  <p className="mt-1 text-[11px] text-slate-400 leading-snug">
                    Шестигранная призма с фасками DIN 934, отверстием и винтовой канавкой `metric_60` (ISO 68-1).
                  </p>
                </button>

                <button
                  type="button"
                  disabled={isBusy}
                  onClick={() => onRunScenario('forged_wheel_rim')}
                  className="w-full text-left p-3 rounded-lg bg-slate-900 hover:bg-slate-800/90 border border-slate-800 hover:border-amber-500/40 transition-colors group"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-xs font-semibold text-white group-hover:text-amber-400">
                      05. Кованый диск 18&quot; (`wheel_rim`)
                    </span>
                    <Play className="w-3.5 h-3.5 text-amber-400 shrink-0" />
                  </div>
                  <p className="mt-1 text-[11px] text-slate-400 leading-snug">
                    Параметрический легкосплавный диск с профилем обода, ступицей, PCD 5×114.3 и 5 спицами.
                  </p>
                </button>
              </div>
            </div>

            {/* Pre-built STL Models from /output */}
            {outputStls.length > 0 && (
              <div>
                <h3 className="text-xs font-semibold text-slate-300 mb-2 flex items-center gap-1.5">
                  <FolderOpen className="w-3.5 h-3.5 text-amber-400" />
                  <span>Импорт эталонных моделей (`/output`)</span>
                </h3>
                <div className="space-y-1.5">
                  {outputStls.map((stl) => (
                    <button
                      key={stl.name}
                      type="button"
                      disabled={isBusy}
                      onClick={() => onImportStl(stl.filepath, 'aluminum_6061_t6')}
                      className="w-full flex items-center justify-between px-3 py-2 rounded-lg bg-slate-900/70 hover:bg-slate-800 border border-slate-800 text-xs text-slate-200 transition-colors"
                    >
                      <span className="font-mono truncate">{stl.name}</span>
                      <span className="text-[11px] text-slate-400 font-mono shrink-0">
                        {(stl.sizeBytes / 1024).toFixed(0)} КБ
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>
        )}

        {/* SECTION 2: Parametric CAD Builder */}
        {activeSection === 'builder' && (
          <div className="space-y-5">
            {/* Step 1: Create Part */}
            <form onSubmit={handleCreatePart} className="space-y-2.5 p-3 rounded-lg bg-slate-900/70 border border-slate-800">
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Plus className="w-3.5 h-3.5 text-amber-400" />
                <span>1. Создать деталь (`create_part`)</span>
              </div>
              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Имя детали</label>
                <input
                  type="text"
                  value={newPartName}
                  onChange={(e) => setNewPartName(e.target.value)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-950 border border-slate-800 rounded-md text-white focus:outline-none focus:border-amber-500"
                />
              </div>
              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Материал из реестра</label>
                <select
                  value={newPartMaterial}
                  onChange={(e) => setNewPartMaterial(e.target.value)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-950 border border-slate-800 rounded-md text-white focus:outline-none focus:border-amber-500"
                >
                  {MATERIALS.map((m) => (
                    <option key={m.id} value={m.id}>
                      {m.label}
                    </option>
                  ))}
                </select>
              </div>
              <label className="flex items-center gap-2 text-xs text-slate-300 cursor-pointer">
                <input
                  type="checkbox"
                  checked={newPartFixed}
                  onChange={(e) => setNewPartFixed(e.target.checked)}
                  className="rounded border-slate-700 text-amber-500 focus:ring-0"
                />
                <span>Неподвижная станина (`is_fixed: true`)</span>
              </label>
              <button
                type="submit"
                disabled={isBusy}
                className="w-full py-1.5 px-3 text-xs font-semibold bg-amber-500 hover:bg-amber-400 text-slate-950 rounded-md transition-colors"
              >
                Создать деталь
              </button>
            </form>

            {/* Step 2: Add Primitive to Selected Part */}
            <div className="space-y-2.5 p-3 rounded-lg bg-slate-900/70 border border-slate-800">
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Box className="w-3.5 h-3.5 text-amber-400" />
                <span>2. Добавить примитив в DAG</span>
              </div>

              <div>
                <label className="block text-[11px] text-slate-400 mb-1">Тип примитива</label>
                <select
                  value={primitiveType}
                  onChange={(e) => setPrimitiveType(e.target.value as typeof primitiveType)}
                  className="w-full px-2.5 py-1.5 text-xs bg-slate-950 border border-slate-800 rounded-md text-white focus:outline-none focus:border-amber-500"
                >
                  <option value="spur_gear">Spur Gear (Шестерня DIN 867)</option>
                  <option value="box">Box (Параллелепипед)</option>
                  <option value="cylinder">Cylinder (Цилиндр)</option>
                  <option value="polygon_prism">Polygon Prism (Гайка / Призма DIN 934)</option>
                  <option value="wheel_rim">Wheel Rim (Колёсный диск)</option>
                  <option value="cone">Cone (Конус / Усечённый конус)</option>
                  <option value="sphere">Sphere (Сфера)</option>
                </select>
              </div>

              {primitiveType === 'box' && (
                <div className="grid grid-cols-3 gap-2">
                  <div>
                    <label className="block text-[10px] text-slate-400">Width (м)</label>
                    <input
                      type="number"
                      step="0.01"
                      value={boxW}
                      onChange={(e) => setBoxW(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Height (м)</label>
                    <input
                      type="number"
                      step="0.005"
                      value={boxH}
                      onChange={(e) => setBoxH(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Depth (м)</label>
                    <input
                      type="number"
                      step="0.01"
                      value={boxD}
                      onChange={(e) => setBoxD(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                </div>
              )}

              {(primitiveType === 'cylinder' || primitiveType === 'sphere' || primitiveType === 'cone') && (
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="block text-[10px] text-slate-400">Radius (м)</label>
                    <input
                      type="number"
                      step="0.005"
                      value={cylR}
                      onChange={(e) => setCylR(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  {primitiveType !== 'sphere' && (
                    <div>
                      <label className="block text-[10px] text-slate-400">Height (м)</label>
                      <input
                        type="number"
                        step="0.01"
                        value={cylH}
                        onChange={(e) => setCylH(Number(e.target.value))}
                        className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                      />
                    </div>
                  )}
                </div>
              )}

              {primitiveType === 'spur_gear' && (
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="block text-[10px] text-slate-400">Модуль m (м)</label>
                    <input
                      type="number"
                      step="0.0005"
                      value={gearModule}
                      onChange={(e) => setGearModule(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Число зубьев z</label>
                    <input
                      type="number"
                      step="1"
                      min="6"
                      value={gearTeeth}
                      onChange={(e) => setGearTeeth(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Ширина венца (м)</label>
                    <input
                      type="number"
                      step="0.002"
                      value={gearFaceWidth}
                      onChange={(e) => setGearFaceWidth(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Отверстие вала (м)</label>
                    <input
                      type="number"
                      step="0.002"
                      value={gearBoreD}
                      onChange={(e) => setGearBoreD(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                </div>
              )}

              {primitiveType === 'polygon_prism' && (
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="block text-[10px] text-slate-400">Грани (sides)</label>
                    <input
                      type="number"
                      min="3"
                      value={prismSides}
                      onChange={(e) => setPrismSides(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                  <div>
                    <label className="block text-[10px] text-slate-400">Размер под ключ S (м)</label>
                    <input
                      type="number"
                      step="0.001"
                      value={prismWaf}
                      onChange={(e) => setPrismWaf(Number(e.target.value))}
                      className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                    />
                  </div>
                </div>
              )}

              <button
                type="button"
                disabled={isBusy || !selectedPartId}
                onClick={handleAddPrimitive}
                className="w-full py-1.5 px-3 text-xs font-semibold bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-white rounded-md transition-colors"
              >
                Добавить примитив в выбранную деталь
              </button>
            </div>

            {/* Step 3: Cut Hole with Optional DIN 6885 Keyway */}
            <div className="space-y-2.5 p-3 rounded-lg bg-slate-900/70 border border-slate-800">
              <div className="text-xs font-semibold text-white flex items-center gap-1.5">
                <Scissors className="w-3.5 h-3.5 text-amber-400" />
                <span>3. Отверстие / шпоночный паз (`cut_hole`)</span>
              </div>
              <div className="grid grid-cols-2 gap-2">
                <div>
                  <label className="block text-[10px] text-slate-400">Радиус (м)</label>
                  <input
                    type="number"
                    step="0.001"
                    value={holeRadius}
                    onChange={(e) => setHoleRadius(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                </div>
                <div>
                  <label className="block text-[10px] text-slate-400">Глубина (м)</label>
                  <input
                    type="number"
                    step="0.005"
                    value={holeDepth}
                    onChange={(e) => setHoleDepth(Number(e.target.value))}
                    className="w-full px-2 py-1 text-xs font-mono bg-slate-950 border border-slate-800 rounded text-white"
                  />
                </div>
              </div>
              <button
                type="button"
                disabled={isBusy || !selectedPartId}
                onClick={handleCutHole}
                className="w-full py-1.5 px-3 text-xs font-semibold bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-white rounded-md transition-colors"
              >
                Вырезать отверстие
              </button>
            </div>
          </div>
        )}

        {/* SECTION 3: FeatureTree DAG History */}
        {activeSection === 'dag' && (
          <div className="space-y-2.5">
            <h3 className="text-xs font-semibold text-slate-300 flex items-center gap-1.5">
              <GitBranch className="w-3.5 h-3.5 text-amber-400" />
              <span>История операций `FeatureTree`</span>
            </h3>

            {!selectedPartId ? (
              <p className="text-xs text-slate-400">Выберите деталь для просмотра графа операций.</p>
            ) : dagNodes.length === 0 ? (
              <div className="p-3 rounded-lg bg-slate-900/70 border border-slate-800 text-xs text-slate-400">
                У выбранной детали пустое дерево DAG (например, деталь была создана через `import_stl` или ещё не содержит операций).
              </div>
            ) : (
              <div className="space-y-2">
                {dagNodes.map((node, idx) => (
                  <div
                    key={node.id}
                    className="p-2.5 rounded-lg bg-slate-900 border border-slate-800 text-xs space-y-1"
                  >
                    <div className="flex items-center justify-between">
                      <span className="font-semibold text-amber-400">
                        #{idx + 1} {node.name || 'FeatureOp'}
                      </span>
                      <span className="text-[10px] font-mono text-slate-500">
                        {node.id.slice(0, 8)}
                      </span>
                    </div>
                    <pre className="text-[10px] font-mono text-slate-300 bg-slate-950 p-2 rounded overflow-x-auto">
                      {JSON.stringify(node.op, null, 2)}
                    </pre>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </aside>
  );
};
