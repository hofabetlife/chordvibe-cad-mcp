import React, { useState } from 'react';
import type { ScenarioPresetId } from './FeatureBuilderPanel';
import {
  Activity,
  Box,
  CheckCircle2,
  Circle,
  Cpu,
  Download,
  FileDown,
  FolderOpen,
  Gauge,
  Layers,
  Move,
  Play,
  Plus,
  RotateCw,
  Scissors,
  Shield,
  Sliders,
  Sparkles,
  Zap,
} from 'lucide-react';

interface CadRibbonToolbarProps {
  activePartId: string | null;
  activePartName: string | null;
  activeMaterial: string;
  isBusy: boolean;
  onRunScenario: (preset: ScenarioPresetId) => void;
  onAddPrimitive: (
    type: 'spur_gear' | 'box' | 'cylinder' | 'polygon_prism' | 'wheel_rim' | 'cone' | 'sphere'
  ) => void;
  onCutHole: () => void;
  onCutHelicalThread: () => void;
  onRunFeaQuick: () => void;
  onRunPhysicsQuick: () => void;
  onImportStlQuick: (filepath: string) => void;
  onTessellateQuick: (quality: 'coarse' | 'medium' | 'fine') => void;
  onQueryGeometryQuick: () => void;
}

export const CadRibbonToolbar: React.FC<CadRibbonToolbarProps> = ({
  activePartId,
  activePartName,
  activeMaterial,
  isBusy,
  onRunScenario,
  onAddPrimitive,
  onCutHole,
  onCutHelicalThread,
  onRunFeaQuick,
  onRunPhysicsQuick,
  onImportStlQuick,
  onTessellateQuick,
  onQueryGeometryQuick,
}) => {
  const [activeTab, setActiveTab] = useState<'modeling' | 'fea' | 'physics' | 'materials' | 'scenarios'>(
    'modeling'
  );

  return (
    <div className="bg-[#111726] border-b border-slate-800 text-slate-200 select-none shrink-0 shadow-md">
      {/* Top Tab Bar Header */}
      <div className="flex items-center gap-1 px-3 pt-1 border-b border-slate-800/80 bg-[#0d121f]">
        <button
          type="button"
          onClick={() => setActiveTab('modeling')}
          className={`px-3 py-1.5 text-xs font-semibold rounded-t-md border-t border-x transition-colors cursor-pointer flex items-center gap-1.5 ${
            activeTab === 'modeling'
              ? 'bg-[#111726] border-slate-700 text-amber-400 font-bold'
              : 'border-transparent text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'
          }`}
        >
          <Box className="w-3.5 h-3.5 text-amber-400" />
          <span>3D Моделирование (DAG)</span>
        </button>

        <button
          type="button"
          onClick={() => setActiveTab('fea')}
          className={`px-3 py-1.5 text-xs font-semibold rounded-t-md border-t border-x transition-colors cursor-pointer flex items-center gap-1.5 ${
            activeTab === 'fea'
              ? 'bg-[#111726] border-slate-700 text-amber-400 font-bold'
              : 'border-transparent text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'
          }`}
        >
          <Gauge className="w-3.5 h-3.5 text-sky-400" />
          <span>Расчёт прочности МКЭ</span>
        </button>

        <button
          type="button"
          onClick={() => setActiveTab('physics')}
          className={`px-3 py-1.5 text-xs font-semibold rounded-t-md border-t border-x transition-colors cursor-pointer flex items-center gap-1.5 ${
            activeTab === 'physics'
              ? 'bg-[#111726] border-slate-700 text-amber-400 font-bold'
              : 'border-transparent text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'
          }`}
        >
          <Activity className="w-3.5 h-3.5 text-emerald-400" />
          <span>Кинематика Rapier3D</span>
        </button>

        <button
          type="button"
          onClick={() => setActiveTab('materials')}
          className={`px-3 py-1.5 text-xs font-semibold rounded-t-md border-t border-x transition-colors cursor-pointer flex items-center gap-1.5 ${
            activeTab === 'materials'
              ? 'bg-[#111726] border-slate-700 text-amber-400 font-bold'
              : 'border-transparent text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'
          }`}
        >
          <Sliders className="w-3.5 h-3.5 text-cyan-400" />
          <span>Материалы & QA</span>
        </button>

        <button
          type="button"
          onClick={() => setActiveTab('scenarios')}
          className={`px-3 py-1.5 text-xs font-semibold rounded-t-md border-t border-x transition-colors cursor-pointer flex items-center gap-1.5 ${
            activeTab === 'scenarios'
              ? 'bg-[#111726] border-slate-700 text-amber-400 font-bold'
              : 'border-transparent text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'
          }`}
        >
          <Zap className="w-3.5 h-3.5 text-amber-400" />
          <span>E2E Сценарии (1-Click)</span>
        </button>
      </div>

      {/* Ribbon Content Body */}
      <div className="px-3 py-2 flex items-center gap-4 overflow-x-auto min-h-[58px]">
        {/* TAB 1: 3D MODELING */}
        {activeTab === 'modeling' && (
          <div className="flex items-stretch gap-4 divide-x divide-slate-800 text-xs">
            {/* Section: Primitives */}
            <div className="flex flex-col gap-1 pr-4">
              <span className="text-[10px] font-mono uppercase text-slate-400 font-semibold tracking-wider">
                1. Примитивы DAG
              </span>
              <div className="flex items-center gap-1.5">
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={() => onAddPrimitive('spur_gear')}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <RotateCw className="w-3.5 h-3.5 text-amber-400" />
                  <span>Шестерня DIN867</span>
                </button>
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={() => onAddPrimitive('polygon_prism')}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <Box className="w-3.5 h-3.5 text-amber-400" />
                  <span>Гайка DIN934</span>
                </button>
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={() => onAddPrimitive('box')}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <Box className="w-3.5 h-3.5 text-sky-400" />
                  <span>Параллелепипед</span>
                </button>
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={() => onAddPrimitive('wheel_rim')}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <Circle className="w-3.5 h-3.5 text-emerald-400" />
                  <span>Диск Rim R18</span>
                </button>
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={() => onAddPrimitive('cylinder')}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <Circle className="w-3.5 h-3.5 text-purple-400" />
                  <span>Цилиндр</span>
                </button>
              </div>
            </div>

            {/* Section: Modifiers */}
            <div className="flex flex-col gap-1 px-4">
              <span className="text-[10px] font-mono uppercase text-slate-400 font-semibold tracking-wider">
                2. Обработка & Вырезы
              </span>
              <div className="flex items-center gap-1.5">
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={onCutHole}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <Scissors className="w-3.5 h-3.5 text-amber-400" />
                  <span>Отверстие + Шпонка DIN6885</span>
                </button>
                <button
                  type="button"
                  disabled={isBusy || !activePartId}
                  onClick={onCutHelicalThread}
                  className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
                >
                  <RotateCw className="w-3.5 h-3.5 text-sky-400" />
                  <span>Резьба ISO 68-1 (60°)</span>
                </button>
              </div>
            </div>

            {/* Active Part indicator */}
            <div className="flex flex-col justify-center px-4">
              <span className="text-[10px] text-slate-400">Активная деталь:</span>
              <span className="text-xs font-semibold text-amber-400 truncate max-w-[160px]">
                {activePartName || 'Выберите деталь'}
              </span>
            </div>
          </div>
        )}

        {/* TAB 2: FEA STRESS ANALYSIS */}
        {activeTab === 'fea' && (
          <div className="flex items-center gap-3 text-xs">
            <button
              type="button"
              disabled={isBusy || !activePartId}
              onClick={onRunFeaQuick}
              className="px-3 py-1.5 rounded bg-amber-500 hover:bg-amber-400 disabled:opacity-40 text-slate-950 font-bold flex items-center gap-1.5 shadow-md transition-colors cursor-pointer"
            >
              <Gauge className="w-4 h-4" />
              <span>Запустить МКЭ Расчёт + Тепловую Карту (TET4)</span>
            </button>
            <div className="text-[11px] text-slate-400 leading-tight border-l border-slate-800 pl-3">
              Решает уравнение упругости K · u = F на 3D тетраэдрах, рассчитывает
              эквивалентные напряжения по Мизесу (σ_vM) и запас прочности (FoS).
            </div>
          </div>
        )}

        {/* TAB 3: KINEMATICS & PHYSICS */}
        {activeTab === 'physics' && (
          <div className="flex items-center gap-3 text-xs">
            <button
              type="button"
              disabled={isBusy}
              onClick={onRunPhysicsQuick}
              className="px-3 py-1.5 rounded bg-amber-500 hover:bg-amber-400 disabled:opacity-40 text-slate-950 font-bold flex items-center gap-1.5 shadow-md transition-colors cursor-pointer"
            >
              <Activity className="w-4 h-4" />
              <span>Запустить симуляцию механизмов (Rapier3D)</span>
            </button>
            <div className="text-[11px] text-slate-400 leading-tight border-l border-slate-800 pl-3">
              Интегрирует сочленения (Revolute, Prismatic, Gear Joint) с шагом $\Delta t=16.67$ мс и
              телеметрией реакций в шарнирах.
            </div>
          </div>
        )}

        {/* TAB 4: MATERIALS & QA */}
        {activeTab === 'materials' && (
          <div className="flex items-center gap-2 text-xs">
            <button
              type="button"
              disabled={isBusy || !activePartId}
              onClick={onQueryGeometryQuick}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
            >
              <CheckCircle2 className="w-3.5 h-3.5 text-emerald-400" />
              <span>Инспекция геометрии (query_part_geometry)</span>
            </button>
            <button
              type="button"
              disabled={isBusy || !activePartId}
              onClick={() => onTessellateQuick('medium')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
            >
              <Sliders className="w-3.5 h-3.5 text-sky-400" />
              <span>Адаптивная тесселяция (Medium)</span>
            </button>
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onImportStlQuick('./output/spur_gear.stl')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 disabled:opacity-40 text-slate-100 flex items-center gap-1 font-medium transition-colors cursor-pointer"
            >
              <FolderOpen className="w-3.5 h-3.5 text-amber-400" />
              <span>Импортировать spur_gear.stl</span>
            </button>
          </div>
        )}

        {/* TAB 5: E2E SCENARIOS */}
        {activeTab === 'scenarios' && (
          <div className="flex items-center gap-2 overflow-x-auto text-xs">
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onRunScenario('gear_transmission')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-100 font-semibold flex items-center gap-1 transition-colors cursor-pointer whitespace-nowrap"
            >
              <Play className="w-3 h-3 text-amber-400" />
              <span>1. Зубчатая передача Z20:Z40</span>
            </button>
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onRunScenario('aerospace_bracket')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-100 font-semibold flex items-center gap-1 transition-colors cursor-pointer whitespace-nowrap"
            >
              <Play className="w-3 h-3 text-amber-400" />
              <span>2. Кронштейн Al 6061-T6 + МКЭ</span>
            </button>
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onRunScenario('crank_mechanism')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-100 font-semibold flex items-center gap-1 transition-colors cursor-pointer whitespace-nowrap"
            >
              <Play className="w-3 h-3 text-amber-400" />
              <span>3. Кривошипно-ползунный привод</span>
            </button>
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onRunScenario('din934_hex_nut')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-100 font-semibold flex items-center gap-1 transition-colors cursor-pointer whitespace-nowrap"
            >
              <Play className="w-3 h-3 text-amber-400" />
              <span>4. Гайка M10 DIN934</span>
            </button>
            <button
              type="button"
              disabled={isBusy}
              onClick={() => onRunScenario('forged_wheel_rim')}
              className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-100 font-semibold flex items-center gap-1 transition-colors cursor-pointer whitespace-nowrap"
            >
              <Play className="w-3 h-3 text-amber-400" />
              <span>5. Диск R18 WheelRim</span>
            </button>
          </div>
        )}
      </div>
    </div>
  );
};
