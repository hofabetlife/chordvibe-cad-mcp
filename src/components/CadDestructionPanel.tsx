import React, { useEffect, useMemo, useState } from 'react';
import {
  Flame,
  Zap,
  RotateCcw,
  Download,
  ArrowLeftRight,
  AlertTriangle,
  Database,
  Clock,
  Activity,
  Layers,
  Cpu,
  Plug,
  Square,
  X,
} from 'lucide-react';
import type {
  DeformationScalePreset,
  DestructionCache,
  DestructionFieldMode,
  DestructionScenarioMode,
  DestructionSimConfig,
  DestructionTimelineEvent,
  SimulationSolverBackend,
} from '../types/cad';
import type { RustAccuracyLevel } from '../cad/rust_scene_sync';
import { formatCacheBytes } from '../cad/destruction_engine';
import {
  buildEnergyBalanceSeries,
  DEFORMATION_SCALE_PRESETS,
} from '../cad/field_colormaps';

interface CadDestructionPanelProps {
  config: DestructionSimConfig;
  cache: DestructionCache | null;
  currentFrame: number;
  isBaking: boolean;
  bakeProgressPct: number;
  bakeStageLabel: string;
  isCacheStale: boolean;
  fieldMode: DestructionFieldMode;
  deformScale: DeformationScalePreset;
  nativeRustConnected?: boolean;
  rustEndpointUrl?: string;
  rustDiagnosticMessage?: string;
  isProbingRust?: boolean;
  rustAccuracyLevel?: RustAccuracyLevel;
  nativeBakeError?: string | null;
  canCancelBake?: boolean;
  onChangeFieldMode: (mode: DestructionFieldMode) => void;
  onChangeDeformScale: (scale: DeformationScalePreset) => void;
  onChangeConfig: (next: DestructionSimConfig) => void;
  onSelectScenario: (mode: DestructionScenarioMode) => void;
  onBakeCache: () => void;
  onClearCache: () => void;
  onSeekFrame: (frame: number) => void;
  onExportCacheJson: () => void;
  onChangeRustAccuracyLevel?: (level: RustAccuracyLevel) => void;
  onConnectRustServer?: (endpointUrl: string) => void;
  onCancelBake?: () => void;
  onDismissNativeBakeError?: () => void;
}

const SCENARIO_OPTIONS: {
  id: DestructionScenarioMode;
  title: string;
  subtitle: string;
}[] = [
  {
    id: 'counter_clash',
    title: 'Встречное вращение шестерён',
    subtitle: 'Лобовой удар зубьев, пластическое смятие и локальный скол',
  },
  {
    id: 'torque_overload',
    title: 'Перегрузка редуктора (Разрыв)',
    subtitle: 'Вращение в зацеплении с критическим моментом > σy',
  },
  {
    id: 'impact_crush',
    title: 'Ударный краш-тест детали',
    subtitle: 'Динамическое деформирование и разрушение CAD-модели',
  },
];

const FIELD_MODE_OPTIONS: {
  id: DestructionFieldMode;
  short: string;
  label: string;
}[] = [
  {
    id: 'shaded',
    short: 'Shaded',
    label: 'Shaded (Реальная деформация)',
  },
  {
    id: 'von_mises',
    short: 'σ_vM',
    label: 'Напряжения Von Mises (σ_vM, МПа)',
  },
  {
    id: 'plastic_strain',
    short: 'ε_p',
    label: 'Пластическая деформация (ε_p, %)',
  },
  {
    id: 'damage',
    short: 'D',
    label: 'Повреждаемость (D, 0..1)',
  },
];

function eventBadgeMeta(type: DestructionTimelineEvent['type']): {
  tag: string;
  colorClass: string;
} {
  switch (type) {
    case 'contact':
      return {
        tag: 'Контакт',
        colorClass: 'bg-[#0c2435] text-[#38bdf8] border-[#38bdf8]/40',
      };
    case 'yield_onset':
      return {
        tag: 'Текучесть',
        colorClass: 'bg-[#2c240a] text-[#facc15] border-[#eab308]/50',
      };
    case 'tooth_shear':
      return {
        tag: 'Скол',
        colorClass: 'bg-[#2d180c] text-[#fb923c] border-[#f97316]/50',
      };
    case 'fragment_detachment':
      return {
        tag: 'Фрагмент',
        colorClass: 'bg-[#2c1914] text-[#fdba74] border-[#fb923c]/50',
      };
    case 'body_fracture':
      return {
        tag: 'Раскол',
        colorClass: 'bg-[#2e1015] text-[#f87171] border-[#ef4444]/50',
      };
    case 'floor_bounce':
      return {
        tag: 'Стенд',
        colorClass: 'bg-[#12262b] text-[#67e8f9] border-[#22d3ee]/40',
      };
  }
}

const RUST_ACCURACY_OPTIONS: {
  id: RustAccuracyLevel;
  label: string;
  hint: string;
}[] = [
  { id: 'standard', label: 'Standard', hint: 'TET4 + быстрый контакт' },
  { id: 'high', label: 'High', hint: 'Сглаженная сетка (Snap)' },
  { id: 'ultra', label: 'Ultra', hint: 'Квадратичные TET10 (точнее)' },
];

export const CadDestructionPanel: React.FC<CadDestructionPanelProps> = ({
  config,
  cache,
  currentFrame,
  isBaking,
  bakeProgressPct,
  bakeStageLabel,
  isCacheStale,
  fieldMode,
  deformScale,
  nativeRustConnected = false,
  rustEndpointUrl = '',
  rustDiagnosticMessage,
  isProbingRust = false,
  rustAccuracyLevel = 'standard',
  nativeBakeError = null,
  canCancelBake = false,
  onChangeFieldMode,
  onChangeDeformScale,
  onChangeConfig,
  onSelectScenario,
  onBakeCache,
  onClearCache,
  onSeekFrame,
  onExportCacheJson,
  onChangeRustAccuracyLevel,
  onConnectRustServer,
  onCancelBake,
  onDismissNativeBakeError,
}) => {
  const [endpointDraft, setEndpointDraft] = useState<string>(rustEndpointUrl);

  useEffect(() => {
    setEndpointDraft(rustEndpointUrl);
  }, [rustEndpointUrl]);

  const frameData = cache
    ? cache.frames[Math.max(0, Math.min(cache.frames.length - 1, currentFrame))]
    : null;

  const activeSolverBackend: SimulationSolverBackend =
    cache?.solverBackend ??
    (nativeRustConnected ? 'rust_fem_f64' : 'web_local_preview');
  const activeSolverLabel =
    cache?.solverLabel ??
    (nativeRustConnected ? 'Rust FEM f64' : 'Web Local Preview');

  const isHeadOnClash =
    Math.sign(config.motorRpmLeft) !== 0 &&
    Math.sign(config.motorRpmLeft) === Math.sign(config.motorRpmRight);

  const stressPct = frameData
    ? Math.min(
        100,
        Math.max(
          2,
          (frameData.metrics.peakStressMPa /
            Math.max(1, frameData.metrics.yieldLimitMPa)) *
            100
        )
      )
    : 0;

  const energySeries = useMemo(
    () => (cache ? buildEnergyBalanceSeries(cache.frames, 100, 48, 4) : null),
    [cache]
  );

  const playheadX =
    cache && cache.totalFrames > 0
      ? (currentFrame / cache.totalFrames) * 100
      : 0;

  return (
    <div className="space-y-3">
      {/* 0. Explicit Solver Backend Indicator & Rust Server Bridge */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-2.5 space-y-2">
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center gap-1.5 min-w-0">
            <Cpu
              className={`w-3.5 h-3.5 shrink-0 ${
                activeSolverBackend === 'rust_fem_f64'
                  ? 'text-[#10b981]'
                  : 'text-[#f59e0b]'
              }`}
            />
            <div className="min-w-0">
              <div className="text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
                Решатель симуляции
              </div>
              <div className="text-[11px] font-mono text-[#e2e8f0] truncate">
                {activeSolverBackend === 'rust_fem_f64'
                  ? 'HHT-α FEM (f64, нативный сервер)'
                  : 'Локальный TS-превью решатель'}
              </div>
            </div>
          </div>
          <span
            data-testid="solver-backend-badge"
            className={`px-2 py-0.5 rounded text-[10px] font-mono font-semibold border shrink-0 ${
              activeSolverBackend === 'rust_fem_f64'
                ? 'bg-[#052e1e] border-[#10b981]/60 text-[#34d399]'
                : 'bg-[#2a1d12] border-[#f59e0b]/60 text-[#fbbf24]'
            }`}
          >
            {activeSolverLabel}
          </span>
        </div>

        {/* Rust mcp-cad-server Endpoint Connection Bar */}
        {onConnectRustServer && (
          <div className="pt-1.5 border-t border-[#242936] space-y-1.5">
            <div className="flex items-center gap-1.5">
              <input
                type="text"
                data-testid="rust-endpoint-input"
                aria-label="URL сервера Rust mcp-cad-server"
                placeholder="auto (/api) или http://127.0.0.1:8080"
                value={endpointDraft}
                onChange={(e) => setEndpointDraft(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' && !isProbingRust) {
                    onConnectRustServer(endpointDraft);
                  }
                }}
                className="flex-1 min-w-0 bg-[#11141b] border border-[#2d3545] focus:border-[#38bdf8] focus:outline-none rounded px-2 py-1 text-[10px] font-mono text-[#e2e8f0] placeholder:text-[#64748b]"
              />
              <button
                type="button"
                data-testid="rust-connect-btn"
                disabled={isProbingRust}
                onClick={() => onConnectRustServer(endpointDraft)}
                className="px-2 py-1 rounded bg-[#12151c] hover:bg-[#1f2636] border border-[#2d3545] text-[10px] font-mono text-[#38bdf8] flex items-center gap-1 shrink-0 cursor-pointer disabled:opacity-60"
              >
                <Plug className="w-3 h-3" />
                <span>
                  {isProbingRust
                    ? 'Проверка...'
                    : nativeRustConnected
                      ? 'Обновить'
                      : 'Подключить'}
                </span>
              </button>
            </div>

            {rustDiagnosticMessage && (
              <div
                data-testid="rust-diagnostic-msg"
                className={`text-[10px] font-mono leading-snug ${
                  nativeRustConnected ? 'text-[#34d399]' : 'text-[#94a3b8]'
                }`}
              >
                {rustDiagnosticMessage}
              </div>
            )}
          </div>
        )}

        {/* Rust HHT-α Accuracy Preset */}
        {onChangeRustAccuracyLevel && (
          <div className="pt-1.5 border-t border-[#242936] flex items-center justify-between gap-2 text-[10px]">
            <span className="text-[#94a3b8] shrink-0">Точность FEM:</span>
            <div className="flex items-center gap-1 bg-[#11141b] p-0.5 rounded border border-[#262d3d]">
              {RUST_ACCURACY_OPTIONS.map((opt) => (
                <button
                  key={opt.id}
                  type="button"
                  title={opt.hint}
                  data-testid={`rust-accuracy-${opt.id}`}
                  onClick={() => onChangeRustAccuracyLevel(opt.id)}
                  className={`px-1.5 py-0.5 rounded text-[10px] font-mono cursor-pointer transition-colors ${
                    rustAccuracyLevel === opt.id
                      ? 'bg-[#10b981] text-[#0c0e12] font-semibold'
                      : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                  }`}
                >
                  {opt.label}
                </button>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Native Rust Bake Error / Fallback Banner */}
      {nativeBakeError && (
        <div
          data-testid="native-bake-error-banner"
          className="bg-[#2a1715] border border-[#ef4444]/60 rounded p-2.5 text-[10px] font-mono text-[#fca5a5] flex items-start justify-between gap-2"
        >
          <div className="flex items-start gap-1.5 min-w-0">
            <AlertTriangle className="w-3.5 h-3.5 text-[#f87171] shrink-0 mt-0.5" />
            <div className="leading-snug break-words">
              <span className="text-[#f87171] font-semibold block">
                Сбой Rust FEM (переход на Web Preview):
              </span>
              <span>{nativeBakeError}</span>
            </div>
          </div>
          {onDismissNativeBakeError && (
            <button
              type="button"
              aria-label="Скрыть предупреждение"
              onClick={onDismissNativeBakeError}
              className="text-[#fca5a5] hover:text-white shrink-0 cursor-pointer"
            >
              <X className="w-3.5 h-3.5" />
            </button>
          )}
        </div>
      )}

      {/* 1. Scenario Mode Selection */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2">
        <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
          <span className="text-xs font-semibold text-[#f1f5f9] flex items-center gap-1.5">
            <Flame className="w-3.5 h-3.5 text-[#f97316]" />
            <span>Сценарий разрушения</span>
          </span>
          <span className="text-[10px] font-mono text-[#94a3b8]">
            FEM / 480 Гц
          </span>
        </div>

        <div className="space-y-1.5">
          {SCENARIO_OPTIONS.map((sc) => {
            const active = config.scenarioMode === sc.id;
            return (
              <button
                key={sc.id}
                type="button"
                onClick={() => onSelectScenario(sc.id)}
                className={`w-full text-left px-2.5 py-2 rounded border transition-colors cursor-pointer ${
                  active
                    ? 'bg-[#231915] border-[#f97316]/70 text-[#f8fafc]'
                    : 'bg-[#12151c] border-[#232938] text-[#cbd5e1] hover:bg-[#1a1f2b]'
                }`}
              >
                <div className="text-xs font-semibold">{sc.title}</div>
                <div className="text-[10px] text-[#94a3b8] mt-0.5 leading-snug">
                  {sc.subtitle}
                </div>
              </button>
            );
          })}
        </div>
      </div>

      {/* 2. Field Colormap & Deformation Scale Controls */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5 text-[11px]">
        <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
          <span className="text-xs font-semibold text-[#f1f5f9] flex items-center gap-1.5">
            <Layers className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span>Визуализация полей и масштаба</span>
          </span>
          <span className="text-[10px] font-mono text-[#38bdf8] tabular-nums">
            {deformScale}×
          </span>
        </div>

        {/* Field Mode Radio List */}
        <div className="space-y-1">
          {FIELD_MODE_OPTIONS.map((opt) => {
            const active = fieldMode === opt.id;
            return (
              <button
                key={opt.id}
                type="button"
                data-testid={`field-mode-${opt.id}`}
                onClick={() => onChangeFieldMode(opt.id)}
                className={`w-full text-left px-2.5 py-1.5 rounded border text-[11px] flex items-center justify-between transition-colors cursor-pointer ${
                  active
                    ? 'bg-[#231915] border-[#f97316]/70 text-[#f8fafc] font-semibold'
                    : 'bg-[#12151c] border-[#232938] text-[#cbd5e1] hover:bg-[#1a1f2b]'
                }`}
              >
                <span className="truncate">{opt.label}</span>
                <span
                  className={`font-mono text-[10px] shrink-0 ml-2 ${
                    active ? 'text-[#f97316]' : 'text-[#64748b]'
                  }`}
                >
                  {opt.short}
                </span>
              </button>
            );
          })}
        </div>

        {/* Deformation Scale Switcher: 1x / 5x / 20x */}
        <div className="flex items-center justify-between pt-1 border-t border-[#242936]">
          <span className="text-[#94a3b8]">Масштаб деформаций:</span>
          <div className="flex items-center gap-1 bg-[#11141b] p-0.5 rounded border border-[#262d3d]">
            {DEFORMATION_SCALE_PRESETS.map((sc) => (
              <button
                key={sc}
                type="button"
                data-testid={`deform-scale-${sc}`}
                onClick={() => onChangeDeformScale(sc)}
                className={`px-2 py-0.5 rounded text-[10px] font-mono tabular-nums cursor-pointer ${
                  deformScale === sc
                    ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                    : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                }`}
              >
                {sc}×{sc === 1 ? ' (1:1)' : ''}
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* 3. Solver & Fracture Parameters */}
      <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5 text-[11px]">
        <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
          <span className="text-xs font-semibold text-[#f1f5f9]">
            Параметры предрасчёта (Кэш)
          </span>
          <span className="text-[10px] font-mono text-[#f97316] tabular-nums">
            {Math.round(config.durationSec * config.fps)} кадр.
          </span>
        </div>

        {/* Dual Gear Drive RPMs */}
        {config.scenarioMode !== 'impact_crush' && (
          <div className="space-y-1.5 pb-1.5 border-b border-[#242936]">
            <div className="flex items-center justify-between">
              <span className="text-[#94a3b8]">Приводы шестерён (RPM):</span>
              <button
                type="button"
                onClick={() =>
                  onChangeConfig({
                    ...config,
                    motorRpmRight: -config.motorRpmRight || -120,
                  })
                }
                title="Инвертировать направление вращения правой шестерни"
                className="flex items-center gap-1 px-1.5 py-0.5 rounded bg-[#12151c] hover:bg-[#1f2636] border border-[#2d3545] text-[10px] font-mono text-[#38bdf8] cursor-pointer"
              >
                <ArrowLeftRight className="w-3 h-3" />
                <span>Инверт. ω₂</span>
              </button>
            </div>
            <div className="grid grid-cols-2 gap-2 font-mono tabular-nums">
              <div>
                <label className="text-[10px] text-[#64748b] block mb-0.5">
                  Левая ω₁ (RPM)
                </label>
                <input
                  type="number"
                  step={15}
                  value={config.motorRpmLeft}
                  onChange={(e) =>
                    onChangeConfig({
                      ...config,
                      motorRpmLeft: Number(e.target.value) || 0,
                    })
                  }
                  className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-[#f1f5f9]"
                />
              </div>
              <div>
                <label className="text-[10px] text-[#64748b] block mb-0.5">
                  Правая ω₂ (RPM)
                </label>
                <input
                  type="number"
                  step={15}
                  value={config.motorRpmRight}
                  onChange={(e) =>
                    onChangeConfig({
                      ...config,
                      motorRpmRight: Number(e.target.value) || 0,
                    })
                  }
                  className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-[#f1f5f9]"
                />
              </div>
            </div>
            <div className="text-[10px] font-mono text-[#94a3b8]">
              Режим зубьев:{' '}
              <span
                className={
                  isHeadOnClash
                    ? 'text-[#f97316] font-semibold'
                    : 'text-[#38bdf8]'
                }
              >
                {isHeadOnClash
                  ? 'Встречный удар (зуб в зуб)'
                  : 'Разнонаправленное зацепление (перегрузка)'}
              </span>
            </div>
          </div>
        )}

        {/* Impact Torque */}
        <div className="space-y-1">
          <div className="flex justify-between font-mono tabular-nums">
            <label htmlFor="destr-torque" className="text-[#94a3b8] font-sans">
              Крутящий момент / Удар:
            </label>
            <span className="text-[#f97316] font-semibold">
              {config.impactTorqueNm} Н·м
            </span>
          </div>
          <input
            id="destr-torque"
            type="range"
            min={60}
            max={450}
            step={10}
            value={config.impactTorqueNm}
            onChange={(e) =>
              onChangeConfig({
                ...config,
                impactTorqueNm: Number(e.target.value),
              })
            }
            className="w-full accent-[#f97316] h-1 bg-[#11141b] rounded cursor-pointer"
          />
        </div>

        {/* Material Brittleness */}
        <div className="space-y-1">
          <div className="flex justify-between font-mono tabular-nums">
            <label
              htmlFor="destr-brittleness"
              className="text-[#94a3b8] font-sans"
            >
              Хрупкость (порог скола):
            </label>
            <span className="text-[#e2e8f0]">
              {Math.round(config.brittleness * 100)}%
            </span>
          </div>
          <input
            id="destr-brittleness"
            type="range"
            min={20}
            max={98}
            step={2}
            value={Math.round(config.brittleness * 100)}
            onChange={(e) =>
              onChangeConfig({
                ...config,
                brittleness: Number(e.target.value) / 100,
              })
            }
            className="w-full accent-[#f97316] h-1 bg-[#11141b] rounded cursor-pointer"
          />
        </div>

        {/* Shard Count & Duration */}
        <div className="grid grid-cols-2 gap-2 pt-0.5 font-mono tabular-nums">
          <div>
            <label
              htmlFor="destr-shards"
              className="text-[10px] text-[#94a3b8] font-sans block mb-0.5"
            >
              Осколков / деталь
            </label>
            <select
              id="destr-shards"
              value={config.shardCountPerPart}
              onChange={(e) =>
                onChangeConfig({
                  ...config,
                  shardCountPerPart: Number(e.target.value),
                })
              }
              className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs text-[#f1f5f9] cursor-pointer"
            >
              <option value={14}>14 (быстро)</option>
              <option value={24}>24 (стандарт)</option>
              <option value={32}>32 (высоко)</option>
              <option value={42}>42 (детально)</option>
            </select>
          </div>
          <div>
            <label
              htmlFor="destr-duration"
              className="text-[10px] text-[#94a3b8] font-sans block mb-0.5"
            >
              Длительность кэша
            </label>
            <select
              id="destr-duration"
              value={config.durationSec}
              onChange={(e) =>
                onChangeConfig({
                  ...config,
                  durationSec: Number(e.target.value),
                })
              }
              className="w-full bg-[#11141b] border border-[#2d3545] rounded px-2 py-1 text-xs text-[#f1f5f9] cursor-pointer"
            >
              <option value={1.5}>1.5 с (90 кадр.)</option>
              <option value={2.5}>2.5 с (150 кадр.)</option>
              <option value={4.0}>4.0 с (240 кадр.)</option>
            </select>
          </div>
        </div>

        {/* Gravity Switch */}
        <div className="flex items-center justify-between pt-1">
          <span className="text-[#94a3b8]">Гравитация:</span>
          <div className="flex items-center gap-1 bg-[#11141b] p-0.5 rounded border border-[#262d3d]">
            <button
              type="button"
              onClick={() => onChangeConfig({ ...config, gravityY: -9.81 })}
              className={`px-2 py-0.5 rounded text-[10px] font-mono cursor-pointer ${
                config.gravityY < -1
                  ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                  : 'text-[#94a3b8] hover:text-[#f1f5f9]'
              }`}
            >
              9.81 м/с²
            </button>
            <button
              type="button"
              onClick={() => onChangeConfig({ ...config, gravityY: 0 })}
              className={`px-2 py-0.5 rounded text-[10px] font-mono cursor-pointer ${
                Math.abs(config.gravityY) < 0.1
                  ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                  : 'text-[#94a3b8] hover:text-[#f1f5f9]'
              }`}
            >
              0 G (Космос)
            </button>
          </div>
        </div>

        {/* Primary Bake CTA */}
        <div className="pt-1.5 space-y-1.5">
          <button
            type="button"
            data-testid="bake-destruction-btn"
            disabled={isBaking}
            onClick={onBakeCache}
            className={`w-full py-2 rounded font-semibold text-xs flex items-center justify-center gap-1.5 transition-colors cursor-pointer disabled:opacity-60 ${
              isCacheStale && cache
                ? 'bg-[#f59e0b] hover:bg-[#d97706] text-[#0c0e12]'
                : 'bg-[#f97316] hover:bg-[#ea580c] text-[#0c0e12]'
            }`}
          >
            <Zap className="w-3.5 h-3.5" />
            <span>
              {isBaking
                ? `Расчёт кэша (${bakeProgressPct}%)...`
                : cache
                  ? isCacheStale
                    ? 'Перезапечь кэш (изменено)'
                    : 'Перезапечь кэш симуляции'
                  : 'Запечь кэш симуляции'}
            </span>
          </button>

          {isBaking && (
            <div className="space-y-1.5">
              <div className="w-full h-1.5 rounded bg-[#0c0f15] overflow-hidden">
                <div
                  className="h-full bg-[#f97316] transition-all duration-150"
                  style={{ width: `${bakeProgressPct}%` }}
                />
              </div>
              <div className="flex items-center justify-between gap-2">
                <div className="text-[10px] font-mono text-[#94a3b8] truncate">
                  {bakeStageLabel}
                </div>
                {canCancelBake && onCancelBake && (
                  <button
                    type="button"
                    data-testid="cancel-bake-btn"
                    onClick={onCancelBake}
                    className="px-2 py-0.5 rounded bg-[#2a1418] hover:bg-[#3f1a20] border border-[#ef4444]/60 text-[10px] font-mono text-[#fca5a5] flex items-center gap-1 shrink-0 cursor-pointer"
                  >
                    <Square className="w-2.5 h-2.5 fill-current" />
                    <span>Отмена</span>
                  </button>
                )}
              </div>
            </div>
          )}

          {cache && !isBaking && (
            <div className="grid grid-cols-2 gap-1.5">
              <button
                type="button"
                onClick={onExportCacheJson}
                className="py-1.5 px-2 rounded bg-[#12151c] hover:bg-[#1e2433] border border-[#2a3142] text-[10px] font-mono text-[#cbd5e1] flex items-center justify-center gap-1 cursor-pointer"
              >
                <Download className="w-3 h-3 text-[#38bdf8]" />
                <span>Экспорт JSON</span>
              </button>
              <button
                type="button"
                onClick={onClearCache}
                className="py-1.5 px-2 rounded bg-[#12151c] hover:bg-[#1e2433] border border-[#2a3142] text-[10px] font-mono text-[#94a3b8] hover:text-[#f87171] flex items-center justify-center gap-1 cursor-pointer"
              >
                <RotateCcw className="w-3 h-3" />
                <span>Очистить кэш</span>
              </button>
            </div>
          )}
        </div>
      </div>

      {/* 4. Live Cached Frame Telemetry & Energy Balance Chart */}
      {cache && frameData ? (
        <>
          <div className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2.5 text-[11px] font-mono tabular-nums">
            <div className="flex items-center justify-between border-b border-[#242936] pb-1.5">
              <span className="font-sans text-xs font-semibold text-[#f1f5f9] flex items-center gap-1.5">
                <Activity className="w-3.5 h-3.5 text-[#f97316]" />
                <span>Телеметрия кадра #{frameData.frameIndex}</span>
              </span>
              <span className="text-[10px] text-[#38bdf8]">
                {frameData.timeSec.toFixed(2)} с
              </span>
            </div>

            {/* Stress vs Yield bar */}
            <div className="space-y-1">
              <div className="flex justify-between text-[10px]">
                <span className="text-[#94a3b8] font-sans">
                  Напряжение σ_vM / σ_y:
                </span>
                <span
                  className={
                    frameData.metrics.peakStressMPa >=
                    frameData.metrics.yieldLimitMPa
                      ? 'text-[#ef4444] font-semibold'
                      : 'text-[#f1f5f9]'
                  }
                >
                  {frameData.metrics.peakStressMPa.toFixed(0)} /{' '}
                  {frameData.metrics.yieldLimitMPa.toFixed(0)} МПа
                </span>
              </div>
              <div className="w-full h-2 bg-[#11141b] rounded overflow-hidden">
                <div
                  className={`h-full transition-all duration-75 ${
                    stressPct >= 95
                      ? 'bg-[#ef4444]'
                      : stressPct >= 65
                        ? 'bg-[#f97316]'
                        : 'bg-[#10b981]'
                  }`}
                  style={{ width: `${stressPct}%` }}
                />
              </div>
            </div>

            <div className="grid grid-cols-2 gap-1.5 text-[10px]">
              <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                <span className="text-[#64748b] font-sans block">
                  Прогиб зуба u_max
                </span>
                <span className="text-[#38bdf8] font-semibold text-xs">
                  {frameData.metrics.maxDisplacementMm.toFixed(3)} мм
                </span>
              </div>
              <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                <span className="text-[#64748b] font-sans block">
                  Пластич. деформ. ε_p
                </span>
                <span className="text-[#e879f9] font-semibold text-xs">
                  {frameData.metrics.maxPlasticStrainPct.toFixed(2)} %
                </span>
              </div>
              <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                <span className="text-[#64748b] font-sans block">
                  Повреждаемость D
                </span>
                <span className="text-[#f87171] font-semibold text-xs">
                  {frameData.metrics.maxDamage.toFixed(3)}
                </span>
              </div>
              <div className="bg-[#12151c] p-2 rounded border border-[#222734]">
                <span className="text-[#64748b] font-sans block">
                  Фрагментов / Кэш
                </span>
                <span className="text-[#f97316] font-semibold text-xs">
                  {frameData.metrics.activeShardsCount}/
                  {frameData.metrics.totalShardsCount} ·{' '}
                  {formatCacheBytes(cache.cacheBytes)}
                </span>
              </div>
            </div>
          </div>

          {/* 5. Multi-Series Energy Balance Chart (E_k, E_e, E_p, E_f) */}
          {energySeries && (
            <div
              data-testid="energy-balance-chart"
              className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2 text-[10px] font-mono tabular-nums"
            >
              <div className="flex items-center justify-between border-b border-[#242936] pb-1">
                <span className="font-sans text-xs font-semibold text-[#f1f5f9]">
                  Баланс энергий (E_k, E_e, E_p, E_f)
                </span>
                <span className="text-[#94a3b8]">
                  max {energySeries.maxEnergyJ.toFixed(2)} Дж
                </span>
              </div>

              <div className="relative h-14 rounded bg-[#0c0f15] border border-[#232938] overflow-hidden">
                <svg
                  viewBox="0 0 100 48"
                  preserveAspectRatio="none"
                  className="w-full h-full"
                  aria-label="График баланса энергий"
                >
                  <line
                    x1="0"
                    y1="24"
                    x2="100"
                    y2="24"
                    stroke="#1e2433"
                    strokeWidth="0.6"
                  />
                  {energySeries.kineticPoints && (
                    <polyline
                      fill="none"
                      stroke="#38bdf8"
                      strokeWidth="1.5"
                      vectorEffect="non-scaling-stroke"
                      points={energySeries.kineticPoints}
                    />
                  )}
                  {energySeries.elasticPoints && (
                    <polyline
                      fill="none"
                      stroke="#10b981"
                      strokeWidth="1.5"
                      vectorEffect="non-scaling-stroke"
                      points={energySeries.elasticPoints}
                    />
                  )}
                  {energySeries.plasticPoints && (
                    <polyline
                      fill="none"
                      stroke="#f59e0b"
                      strokeWidth="1.5"
                      vectorEffect="non-scaling-stroke"
                      points={energySeries.plasticPoints}
                    />
                  )}
                  {energySeries.fracturePoints && (
                    <polyline
                      fill="none"
                      stroke="#f43f5e"
                      strokeWidth="1.5"
                      vectorEffect="non-scaling-stroke"
                      points={energySeries.fracturePoints}
                    />
                  )}
                  <line
                    x1={playheadX}
                    y1="0"
                    x2={playheadX}
                    y2="48"
                    stroke="#f8fafc"
                    strokeOpacity="0.85"
                    strokeWidth="0.9"
                    vectorEffect="non-scaling-stroke"
                  />
                </svg>
              </div>

              <div className="grid grid-cols-2 gap-1.5">
                <div className="flex items-center justify-between bg-[#12151c] px-2 py-1 rounded border border-[#222734]">
                  <span className="text-[#38bdf8]">E_k кинет.</span>
                  <span className="text-[#e2e8f0]">
                    {frameData.metrics.kineticEnergyJ.toFixed(3)} Дж
                  </span>
                </div>
                <div className="flex items-center justify-between bg-[#12151c] px-2 py-1 rounded border border-[#222734]">
                  <span className="text-[#10b981]">E_e упруг.</span>
                  <span className="text-[#e2e8f0]">
                    {frameData.metrics.elasticEnergyJ.toFixed(3)} Дж
                  </span>
                </div>
                <div className="flex items-center justify-between bg-[#12151c] px-2 py-1 rounded border border-[#222734]">
                  <span className="text-[#f59e0b]">E_p пласт.</span>
                  <span className="text-[#e2e8f0]">
                    {frameData.metrics.plasticEnergyJ.toFixed(3)} Дж
                  </span>
                </div>
                <div className="flex items-center justify-between bg-[#12151c] px-2 py-1 rounded border border-[#222734]">
                  <span className="text-[#f43f5e]">E_f разруш.</span>
                  <span className="text-[#e2e8f0]">
                    {frameData.metrics.fractureEnergyJ.toFixed(3)} Дж
                  </span>
                </div>
              </div>
            </div>
          )}

          {/* 6. Clickable Physical Event Log */}
          <div
            data-testid="destruction-event-log"
            className="bg-[#181c24] border border-[#282e3d] rounded p-3 space-y-2"
          >
            <div className="flex items-center justify-between text-[10px] font-mono uppercase tracking-wider text-[#64748b]">
              <span className="flex items-center gap-1">
                <Clock className="w-3 h-3 text-[#f97316]" />
                <span>Журнал событий ({cache.events.length})</span>
              </span>
              <span className="text-[#94a3b8]">Клик → кадр</span>
            </div>

            {cache.events.length === 0 ? (
              <div className="text-xs text-[#94a3b8] flex items-center gap-1.5">
                <AlertTriangle className="w-3.5 h-3.5 text-[#f59e0b]" />
                <span>
                  Детали выдержали нагрузку в упругой области. Увеличьте момент или хрупкость.
                </span>
              </div>
            ) : (
              <div className="space-y-1 max-h-48 overflow-y-auto pr-0.5">
                {cache.events.map((ev, idx) => {
                  const isCurrent = Math.abs(ev.frameIndex - currentFrame) <= 2;
                  const meta = eventBadgeMeta(ev.type);
                  return (
                    <button
                      key={`${ev.type}_${ev.frameIndex}_${idx}`}
                      type="button"
                      onClick={() => onSeekFrame(ev.frameIndex)}
                      className={`w-full text-left px-2 py-1.5 rounded border text-[11px] flex items-center justify-between gap-2 transition-colors cursor-pointer ${
                        isCurrent
                          ? 'bg-[#2a1d17] border-[#f97316]/70 text-[#f8fafc]'
                          : 'bg-[#12151c] border-[#222734] text-[#cbd5e1] hover:bg-[#1b202c]'
                      }`}
                    >
                      <div className="flex items-center gap-1.5 min-w-0">
                        <span
                          className={`px-1 py-0.5 rounded border text-[9px] font-mono shrink-0 ${meta.colorClass}`}
                        >
                          {meta.tag}
                        </span>
                        <span className="truncate">{ev.label}</span>
                      </div>
                      <span className="font-mono text-[10px] text-[#f97316] tabular-nums shrink-0">
                        #{ev.frameIndex} ({ev.timeSec.toFixed(2)}с)
                      </span>
                    </button>
                  );
                })}
              </div>
            )}
          </div>
        </>
      ) : (
        <div className="bg-[#181c24] border border-[#242936] rounded p-3 text-xs text-[#94a3b8] space-y-1.5">
          <div className="flex items-center gap-1.5 text-[#e2e8f0] font-semibold">
            <Database className="w-3.5 h-3.5 text-[#f97316]" />
            <span>Кэш симуляции пуст</span>
          </div>
          <p className="leading-relaxed">
            Настройте приводы, момент удара и хрупкость, затем нажмите «Запечь кэш симуляции». После предрасчёта доступны поля σ_vM, ε_p, D, масштаб деформаций 1×/5×/20×, график энергий и журнал событий.
          </p>
        </div>
      )}
    </div>
  );
};
