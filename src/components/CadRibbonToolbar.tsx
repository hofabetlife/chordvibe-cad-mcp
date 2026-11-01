import React, { useRef } from 'react';
import {
  Box,
  Scissors,
  Eye,
  Activity,
  Play,
  Pause,
  RotateCcw,
  Layers,
  Sparkles,
  Maximize2,
  Sliders,
  Upload,
  Download,
  Camera,
  Cpu,
  ShieldCheck,
  FastForward,
  Grid,
} from 'lucide-react';
import {
  DagNodeType,
  EngineeringWorkbench,
  FeaBoundaryCondition,
  ProjectionMode,
  QualityPreset,
  RenderMode,
  ViewportLayout,
} from '../types/cad';
import {
  FEA_QUICK_LOAD_PRESETS,
  buildBoundaryConditionsFromPreset,
} from '../cad/workbenches';

interface CadRibbonToolbarProps {
  activeWorkbench: EngineeringWorkbench;
  renderMode: RenderMode;
  onChangeRenderMode: (mode: RenderMode) => void;
  showCadEdges: boolean;
  onToggleCadEdges: () => void;
  showReferenceGrid?: boolean;
  onToggleReferenceGrid?: () => void;
  projectionMode: ProjectionMode;
  onToggleProjectionMode: () => void;
  viewportLayout?: ViewportLayout;
  onChangeViewportLayout?: (layout: ViewportLayout) => void;
  onFitCamera: () => void;
  quality: QualityPreset;
  onChangeQuality: (q: QualityPreset) => void;
  isSimulatingPhysics: boolean;
  onTogglePhysics: () => void;
  onResetPhysics: () => void;
  onStepPhysics?: () => void;
  onRunQuickFea: (conditions?: FeaBoundaryCondition[], label?: string) => void;
  deformScale: number;
  onChangeDeformScale: (scale: number) => void;
  onAddPrimitive: (type: DagNodeType) => void;
  onOpenCutHoleModal: () => void;
  onOpenHelicalModal: () => void;
  onApplyTwoHoleCsgChain: () => void;
  onSelectPreset: (presetId: string) => void;
  onImportStlFile: (file: File) => void;
  onExportStl: () => void;
  onCaptureViewport: () => void;
  onOptimizeCurrentPart?: () => void;
  onValidateMesh?: () => void;
  hasCompiledStlMapping?: boolean;
  useCompiledRustStl?: boolean;
  onToggleRustStlSource?: () => void;
}

const RENDER_MODES: {
  id: RenderMode;
  label: string;
  hotkey: string;
}[] = [
  { id: 'shaded', label: '[1] Shaded (Стандарт)', hotkey: '1' },
  { id: 'flat_shading', label: '[2] Flat (Грани)', hotkey: '2' },
  { id: 'wireframe', label: '[3] Wireframe (Каркас)', hotkey: '3' },
  { id: 'normals', label: '[4] Normals (Нормали)', hotkey: '4' },
  { id: 'backfaces', label: '[5] Backfaces (Внутр.)', hotkey: '5' },
  { id: 'boundary_edges', label: '[6] Границы (Открытые)', hotkey: '6' },
  { id: 'fea_heatmap', label: '[7] Эпюра МКЭ (MPa)', hotkey: '7' },
];

const PRIMARY_PRIMITIVE_BTNS: {
  type: DagNodeType;
  label: string;
  title: string;
}[] = [
  { type: 'box', label: '+ Блок', title: 'Добавить параметрический параллелепипед' },
  { type: 'cylinder', label: '+ Цилиндр', title: 'Добавить параметрический цилиндр' },
  { type: 'polygon_prism', label: '+ Призма', title: 'Добавить шестигранную призму с фаской DIN 934' },
  { type: 'spur_gear', label: '+ Шестерня', title: 'Добавить эвольвентную шестерню со ступицей и шпонкой' },
];

const EXTRA_PRIMITIVE_OPTIONS: {
  type: DagNodeType;
  label: string;
}[] = [
  { type: 'sphere', label: '+ Сфера' },
  { type: 'cone', label: '+ Конус' },
  { type: 'revolve', label: '+ Тело вращения' },
  { type: 'wheel_rim', label: '+ Автодиск' },
];

export const CadRibbonToolbar: React.FC<CadRibbonToolbarProps> = ({
  activeWorkbench,
  renderMode,
  onChangeRenderMode,
  showCadEdges,
  onToggleCadEdges,
  showReferenceGrid,
  onToggleReferenceGrid,
  projectionMode,
  onToggleProjectionMode,
  viewportLayout,
  onChangeViewportLayout,
  onFitCamera,
  quality,
  onChangeQuality,
  isSimulatingPhysics,
  onTogglePhysics,
  onResetPhysics,
  onStepPhysics,
  onRunQuickFea,
  deformScale,
  onChangeDeformScale,
  onAddPrimitive,
  onOpenCutHoleModal,
  onOpenHelicalModal,
  onApplyTwoHoleCsgChain,
  onSelectPreset,
  onImportStlFile,
  onExportStl,
  onCaptureViewport,
  onOptimizeCurrentPart,
  onValidateMesh,
  hasCompiledStlMapping,
  useCompiledRustStl,
  onToggleRustStlSource,
}) => {
  const fileInputRef = useRef<HTMLInputElement | null>(null);

  return (
    <div className="h-10 bg-[#151922] border-b border-[#252b38] px-2.5 flex items-center justify-between gap-1.5 select-none shrink-0 overflow-x-auto">
      {/* Contextual Left & Center Controls depending on activeWorkbench */}
      {activeWorkbench === 'modeling' && (
        <div className="flex items-center gap-1 shrink-0">
          <span className="text-[11px] font-medium text-[#94a3b8] flex items-center gap-1 whitespace-nowrap">
            <Box className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>Тела:</span>
          </span>
          {PRIMARY_PRIMITIVE_BTNS.map((p) => (
            <button
              key={p.type}
              onClick={() => onAddPrimitive(p.type)}
              title={p.title}
              className="px-1.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
            >
              {p.label}
            </button>
          ))}

          <select
            aria-label="Дополнительные примитивы"
            defaultValue=""
            onChange={(e) => {
              const val = e.target.value as DagNodeType;
              if (!val) return;
              onAddPrimitive(val);
              e.target.value = '';
            }}
            className="bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] text-[11px] font-medium px-1.5 py-1 rounded border border-[#2d3545] focus:outline-none cursor-pointer shrink-0"
          >
            <option value="" disabled>
              + Еще...
            </option>
            {EXTRA_PRIMITIVE_OPTIONS.map((opt) => (
              <option key={opt.type} value={opt.type} className="bg-[#181c26]">
                {opt.label}
              </option>
            ))}
          </select>

          <div className="h-4 w-[1px] bg-[#262b36] mx-0.5 shrink-0" />

          <span className="text-[11px] font-medium text-[#94a3b8] whitespace-nowrap">
            CSG:
          </span>
          <button
            onClick={onOpenCutHoleModal}
            title="Вырезать цилиндрическое отверстие со шпоночным пазом DIN 6885"
            className="flex items-center gap-1 px-1.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#f1f5f9] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Scissors className="w-3 h-3 text-[#ef4444]" />
            <span>Отв.</span>
          </button>

          <button
            onClick={onOpenHelicalModal}
            title="Нарезать метрическую резьбу ISO 68-1 или спиральную канавку"
            className="flex items-center gap-1 px-1.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#f1f5f9] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Sparkles className="w-3 h-3 text-[#a855f7]" />
            <span>Резьба</span>
          </button>

          <button
            onClick={onApplyTwoHoleCsgChain}
            title="Применить цепочку из 2 сквозных отверстий (род поверхности g + 2)"
            className="flex items-center gap-1 px-1.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Layers className="w-3 h-3 text-[#38bdf8]" />
            <span>2 Отв.</span>
          </button>

          <div className="h-4 w-[1px] bg-[#262b36] mx-0.5 shrink-0" />

          {/* Quality & STL IO */}
          <div className="flex items-center bg-[#10131a] p-0.5 rounded border border-[#252b38] shrink-0">
            {(
              [
                { id: 'draft', label: 'Черн.' },
                { id: 'standard', label: 'Станд.' },
                { id: 'fine', label: 'Точно' },
              ] as { id: QualityPreset; label: string }[]
            ).map((q) => (
              <button
                key={q.id}
                onClick={() => onChangeQuality(q.id)}
                className={`px-1.5 py-0.5 text-[10px] font-mono rounded transition-colors cursor-pointer whitespace-nowrap ${
                  quality === q.id
                    ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                    : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                }`}
              >
                {q.label}
              </button>
            ))}
          </div>

          <input
            ref={fileInputRef}
            type="file"
            accept=".stl"
            className="hidden"
            onChange={(e) => {
              const file = e.target.files?.[0];
              if (file) {
                onImportStlFile(file);
                e.target.value = '';
              }
            }}
          />
          <button
            onClick={() => fileInputRef.current?.click()}
            title="Импортировать локальный STL файл"
            className="flex items-center gap-1 px-1.5 py-1 rounded text-[11px] font-medium bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] border border-[#2d3545] transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Upload className="w-3 h-3 text-[#10b981]" />
            <span>Импорт</span>
          </button>

          <button
            onClick={onExportStl}
            title="Экспортировать текущую деталь в бинарный STL"
            className="flex items-center gap-1 px-1.5 py-1 rounded text-[11px] font-medium bg-[#1d222e] hover:bg-[#262d3d] text-[#f59e0b] border border-[#2d3545] transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Download className="w-3 h-3" />
            <span>STL</span>
          </button>
        </div>
      )}

      {activeWorkbench === 'fea' && (
        <div className="flex items-center gap-1.5 shrink-0">
          <span className="text-[11px] font-medium text-[#94a3b8] flex items-center gap-1 whitespace-nowrap">
            <Activity className="w-3.5 h-3.5 text-[#ef4444]" />
            <span>Нагрузки:</span>
          </span>

          {FEA_QUICK_LOAD_PRESETS.map((preset) => (
            <button
              key={preset.id}
              onClick={() =>
                onRunQuickFea(
                  buildBoundaryConditionsFromPreset(preset.id),
                  preset.label
                )
              }
              title={preset.description}
              className="px-2 py-1 rounded bg-[#1d222e] hover:bg-[#282f3f] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
            >
              {preset.label}
            </button>
          ))}

          <div className="h-4 w-[1px] bg-[#262b36] mx-0.5 shrink-0" />

          <button
            onClick={() => onRunQuickFea()}
            title="Решить систему линейной упругости TET4 (K*u=f) и построить карту напряжений по Мизесу"
            className="flex items-center gap-1.5 px-2.5 py-1 rounded bg-[#ef4444] hover:bg-[#dc2626] text-white text-xs font-semibold transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Sparkles className="w-3.5 h-3.5" />
            <span>Рассчитать МКЭ</span>
          </button>

          <div className="flex items-center gap-1.5 bg-[#10131a] px-2 py-1 rounded border border-[#252b38] shrink-0">
            <span className="text-[11px] text-[#94a3b8] whitespace-nowrap">
              Деформ.:
            </span>
            <input
              type="range"
              aria-label="Масштаб деформации МКЭ"
              min={1}
              max={50}
              step={1}
              value={deformScale}
              onChange={(e) => onChangeDeformScale(Number(e.target.value))}
              className="w-16 accent-[#ef4444] h-1 bg-[#262b36] rounded cursor-pointer"
            />
            <span className="text-[11px] font-mono text-[#f87171] font-semibold w-7 text-right">
              {deformScale}x
            </span>
          </div>

          <button
            onClick={() =>
              onChangeRenderMode(
                renderMode === 'fea_heatmap' ? 'shaded' : 'fea_heatmap'
              )
            }
            className={`px-2 py-1 rounded border text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
              renderMode === 'fea_heatmap'
                ? 'bg-[#ef4444]/20 border-[#ef4444] text-[#fca5a5]'
                : 'bg-[#1d222e] border-[#2d3545] text-[#cbd5e1]'
            }`}
          >
            {renderMode === 'fea_heatmap' ? 'Эпюра: ВКЛ' : 'Эпюра МКЭ'}
          </button>
        </div>
      )}

      {activeWorkbench === 'simulation' && (
        <div className="flex items-center gap-2 shrink-0">
          <div className="flex items-center gap-1 bg-[#10131a] p-0.5 rounded border border-[#252b38] shrink-0">
            <button
              onClick={onTogglePhysics}
              title="Запуск / Пауза кинематической симуляции Rapier3D (Клавиша P)"
              className={`flex items-center gap-1.5 px-3 py-1 rounded text-xs font-semibold transition-colors cursor-pointer whitespace-nowrap ${
                isSimulatingPhysics
                  ? 'bg-[#10b981] text-[#0c0e12]'
                  : 'bg-[#1d222e] text-[#10b981] hover:bg-[#262d3d]'
              }`}
            >
              {isSimulatingPhysics ? (
                <>
                  <Pause className="w-3.5 h-3.5 fill-current" />
                  <span>Пауза [P]</span>
                </>
              ) : (
                <>
                  <Play className="w-3.5 h-3.5 fill-current" />
                  <span>Старт симуляции [P]</span>
                </>
              )}
            </button>

            {onStepPhysics && (
              <button
                onClick={onStepPhysics}
                title="Выполнить один шаг кинематики (+1/15 сек)"
                className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium text-[#cbd5e1] hover:bg-[#1d222e] transition-colors cursor-pointer whitespace-nowrap"
              >
                <FastForward className="w-3.5 h-3.5 text-[#38bdf8]" />
                <span>Шаг</span>
              </button>
            )}

            <button
              onClick={onResetPhysics}
              title="Сбросить углы поворота и исходные положения звеньев"
              className="flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium text-[#94a3b8] hover:text-[#f1f5f9] hover:bg-[#1d222e] transition-colors cursor-pointer whitespace-nowrap"
            >
              <RotateCcw className="w-3.5 h-3.5" />
              <span>Сброс</span>
            </button>
          </div>

          <div className="h-4 w-[1px] bg-[#262b36] mx-0.5 shrink-0" />

          <span className="text-[11px] font-medium text-[#94a3b8] whitespace-nowrap">
            Сборки и механизмы:
          </span>

          <button
            onClick={() => onSelectPreset('gear_transmission')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Редуктор Z1=20 / Z2=40
          </button>

          <button
            onClick={() => onSelectPreset('crank_mechanism')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Кривошипно-шатунный
          </button>

          <button
            onClick={() => onSelectPreset('spur_gear')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Привод шестерни
          </button>

          <button
            onClick={() => onSelectPreset('twist_drill')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Шпиндель сверла
          </button>
        </div>
      )}

      {activeWorkbench === 'optimizer' && (
        <div className="flex items-center gap-2 shrink-0">
          <span className="text-[11px] font-medium text-[#94a3b8] flex items-center gap-1 whitespace-nowrap">
            <Cpu className="w-3.5 h-3.5 text-[#f59e0b]" />
            <span>Цепочки ИИ до оптимизации:</span>
          </span>

          <button
            onClick={() => onSelectPreset('ai_unopt_bushing')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Втулка ИИ (5 → 1 оп.)
          </button>

          <button
            onClick={() => onSelectPreset('ai_unopt_flange_plate')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Плита ИИ (8 → 3 оп.)
          </button>

          <button
            onClick={() => onSelectPreset('ai_unopt_hex_nut')}
            className="px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            Гайка М10 ИИ (5 → 1 оп.)
          </button>

          <div className="h-4 w-[1px] bg-[#262b36] mx-0.5 shrink-0" />

          {onOptimizeCurrentPart && (
            <button
              onClick={onOptimizeCurrentPart}
              className="flex items-center gap-1.5 px-3 py-1 rounded bg-[#10b981] hover:bg-[#059669] text-[#0c0e12] text-xs font-semibold transition-colors cursor-pointer whitespace-nowrap shrink-0"
            >
              <Sparkles className="w-3.5 h-3.5" />
              <span>Применить лучший план CBO</span>
            </button>
          )}

          {onValidateMesh && (
            <button
              onClick={onValidateMesh}
              className="flex items-center gap-1 px-2.5 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#38bdf8] border border-[#2d3545] text-[11px] font-medium transition-colors cursor-pointer whitespace-nowrap shrink-0"
            >
              <ShieldCheck className="w-3.5 h-3.5" />
              <span>Проверить топологию</span>
            </button>
          )}

          {hasCompiledStlMapping && onToggleRustStlSource && (
            <button
              onClick={onToggleRustStlSource}
              className={`px-2.5 py-1 rounded border text-[11px] font-mono transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
                useCompiledRustStl
                  ? 'bg-[#f59e0b]/20 border-[#f59e0b] text-[#fbbf24] font-semibold'
                  : 'bg-[#1d222e] border-[#2d3545] text-[#cbd5e1]'
              }`}
            >
              {useCompiledRustStl ? 'Источник: Rust STL' : 'Источник: Ядро WGPU'}
            </button>
          )}
        </div>
      )}

      {/* Right Zone: Always-Accessible Compact Viewport & Shader Controls */}
      <div className="flex items-center gap-1.5 shrink-0">
        <div className="flex items-center gap-1 bg-[#10131a] px-2 py-0.5 rounded border border-[#252b38]">
          <Eye className="w-3.5 h-3.5 text-[#38bdf8] shrink-0" />
          <select
            aria-label="Шейдер отображения"
            value={renderMode}
            onChange={(e) => onChangeRenderMode(e.target.value as RenderMode)}
            className="bg-transparent text-[#f1f5f9] text-[11px] font-mono py-0.5 focus:outline-none cursor-pointer"
          >
            {RENDER_MODES.map((m) => (
              <option
                key={m.id}
                value={m.id}
                className="bg-[#151922] text-[#f1f5f9]"
              >
                {m.label}
              </option>
            ))}
          </select>
        </div>

        <button
          onClick={onToggleCadEdges}
          title="Показать / скрыть контурные ребра поверх модели"
          className={`px-2 py-1 rounded border text-[11px] font-mono transition-colors cursor-pointer whitespace-nowrap ${
            showCadEdges
              ? 'bg-[#38bdf8]/15 border-[#38bdf8]/60 text-[#38bdf8]'
              : 'bg-[#1d222e] border-[#2d3545] text-[#94a3b8] hover:text-[#e2e8f0]'
          }`}
        >
          Ребра
        </button>

        {onToggleReferenceGrid && (
          <button
            onClick={onToggleReferenceGrid}
            title="Показать / скрыть координатную сетку и оси"
            className={`p-1 rounded border transition-colors cursor-pointer ${
              showReferenceGrid
                ? 'bg-[#38bdf8]/15 border-[#38bdf8]/60 text-[#38bdf8]'
                : 'bg-[#1d222e] border-[#2d3545] text-[#94a3b8] hover:text-[#e2e8f0]'
            }`}
          >
            <Grid className="w-3.5 h-3.5" />
          </button>
        )}

        <button
          onClick={onToggleProjectionMode}
          title="Переключить проекцию: Перспектива / Ортогональная (Клавиша O)"
          className="px-2 py-1 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#e2e8f0] border border-[#2d3545] text-[11px] font-mono transition-colors cursor-pointer whitespace-nowrap"
        >
          {projectionMode === 'perspective' ? 'Персп.' : 'Орто'}
        </button>

        {onChangeViewportLayout && (
          <button
            onClick={() =>
              onChangeViewportLayout(
                viewportLayout === 'quad' ? 'single' : 'quad'
              )
            }
            title="Переключить раскладку 1 Вид / 4 Вида (Cinema 4D)"
            className={`px-2 py-1 rounded border text-[11px] font-mono transition-colors cursor-pointer whitespace-nowrap ${
              viewportLayout === 'quad'
                ? 'bg-[#f59e0b]/20 border-[#f59e0b] text-[#fbbf24] font-semibold'
                : 'bg-[#1d222e] border-[#2d3545] text-[#cbd5e1]'
            }`}
          >
            {viewportLayout === 'quad' ? '4 Вида' : '1 Вид'}
          </button>
        )}

        <button
          onClick={onFitCamera}
          title="Центрировать камеру по габаритам модели (Клавиша R / F)"
          className="p-1.5 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#38bdf8] border border-[#2d3545] transition-colors cursor-pointer"
        >
          <Maximize2 className="w-3.5 h-3.5" />
        </button>

        <button
          onClick={onCaptureViewport}
          title="Сделать PNG скриншот 3D окна"
          className="p-1.5 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] border border-[#2d3545] transition-colors cursor-pointer"
        >
          <Camera className="w-3.5 h-3.5" />
        </button>
      </div>
    </div>
  );
};
