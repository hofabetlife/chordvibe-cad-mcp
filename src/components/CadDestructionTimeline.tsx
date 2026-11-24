import React, { useMemo, useState } from 'react';
import {
  Play,
  Pause,
  SkipBack,
  SkipForward,
  ChevronLeft,
  ChevronRight,
  Repeat,
  Flame,
  Sparkles,
  ChevronDown,
  ChevronUp,
  Database,
  Zap,
  RotateCcw,
  Radio,
  Square,
} from 'lucide-react';
import type {
  DeformationScalePreset,
  DestructionCache,
  DestructionFieldMode,
  DestructionTimelineEvent,
} from '../types/cad';
import { formatCacheBytes } from '../cad/destruction_engine';
import { buildEnergyBalanceSeries } from '../cad/field_colormaps';
import {
  formatDurationMsRu,
  type LiveFemSolveStatus,
} from '../cad/coupled_sim_stream';

export type PlaybackSpeed = 0.1 | 0.25 | 0.5 | 1 | 2;

const PLAYBACK_SPEEDS: PlaybackSpeed[] = [0.1, 0.25, 0.5, 1, 2];

interface CadDestructionTimelineProps {
  cache: DestructionCache | null;
  isBaking: boolean;
  bakeProgressPct: number;
  bakeStageLabel: string;
  liveSolveStatus?: LiveFemSolveStatus | null;
  followLiveFrame?: boolean;
  isCacheStale: boolean;
  currentFrame: number;
  isPlaying: boolean;
  playbackSpeed: PlaybackSpeed;
  loopEnabled: boolean;
  showStressGlow: boolean;
  showSparks: boolean;
  fieldMode?: DestructionFieldMode;
  deformScale?: DeformationScalePreset;
  onChangeFieldMode?: (mode: DestructionFieldMode) => void;
  onChangeDeformScale?: (scale: DeformationScalePreset) => void;
  onSeekFrame: (frame: number) => void;
  onTogglePlay: () => void;
  onStepFrame: (delta: number) => void;
  onChangeSpeed: (speed: PlaybackSpeed) => void;
  onToggleLoop: () => void;
  onToggleStressGlow: () => void;
  onToggleSparks: () => void;
  onToggleFollowLive?: () => void;
  onBakeCache: () => void;
  onCancelBake?: () => void;
  onClearCache?: () => void;
}

export const CadDestructionTimeline: React.FC<CadDestructionTimelineProps> = ({
  cache,
  isBaking,
  bakeProgressPct,
  bakeStageLabel,
  liveSolveStatus = null,
  followLiveFrame = true,
  isCacheStale,
  currentFrame,
  isPlaying,
  playbackSpeed,
  loopEnabled,
  showStressGlow,
  showSparks,
  fieldMode = 'shaded',
  onChangeFieldMode,
  onSeekFrame,
  onTogglePlay,
  onStepFrame,
  onChangeSpeed,
  onToggleLoop,
  onToggleStressGlow,
  onToggleSparks,
  onToggleFollowLive,
  onBakeCache,
  onCancelBake,
  onClearCache,
}) => {
  const [collapsed, setCollapsed] = useState(false);
  const [trackOverlayMode, setTrackOverlayMode] = useState<'energy' | 'stress'>(
    'energy'
  );

  const isSolveActive = isBaking || Boolean(liveSolveStatus?.active);
  const effectivePct = Math.max(
    0,
    Math.min(
      100,
      liveSolveStatus?.active ? liveSolveStatus.progressPct : bakeProgressPct
    )
  );
  const effectiveStage = liveSolveStatus?.stageLabel || bakeStageLabel;
  const maxPlayableFrame = cache ? Math.max(0, cache.frames.length - 1) : 0;

  // Build normalized SVG polyline for stress curve across cached frames
  const stressSparklinePoints = useMemo(() => {
    if (!cache || cache.frames.length < 2) return '';
    const maxS = Math.max(1, cache.maxStressOverallMPa);
    const total = cache.frames.length - 1;
    const pts: string[] = [];
    for (let i = 0; i < cache.frames.length; i++) {
      const x = (i / total) * 100;
      const norm = Math.min(1, cache.frames[i].metrics.peakStressMPa / maxS);
      const y = 30 - norm * 24;
      pts.push(`${x.toFixed(2)},${y.toFixed(2)}`);
    }
    return pts.join(' ');
  }, [cache]);

  // Build multi-series energy balance curves (E_k, E_e, E_p, E_f) for timeline track
  const energyTrackSeries = useMemo(
    () => (cache ? buildEnergyBalanceSeries(cache.frames, 100, 32, 3) : null),
    [cache]
  );

  const activeFrameData = cache
    ? cache.frames[Math.max(0, Math.min(cache.frames.length - 1, currentFrame))]
    : null;

  const progressPct =
    cache && cache.totalFrames > 0
      ? (currentFrame / cache.totalFrames) * 100
      : 0;

  const eventColor = (type: DestructionTimelineEvent['type']) => {
    switch (type) {
      case 'contact':
        return '#38bdf8';
      case 'yield_onset':
        return '#eab308';
      case 'tooth_shear':
        return '#f97316';
      case 'fragment_detachment':
        return '#fb923c';
      case 'body_fracture':
        return '#ef4444';
      case 'floor_bounce':
        return '#22d3ee';
    }
  };

  const etaStr = liveSolveStatus
    ? formatDurationMsRu(liveSolveStatus.estimatedRemainingMs)
    : '—';
  const elapsedStr = liveSolveStatus
    ? formatDurationMsRu(liveSolveStatus.elapsedMs)
    : '—';

  if (collapsed) {
    return (
      <div
        data-testid="destruction-timeline-collapsed"
        className="absolute bottom-3 left-1/2 -translate-x-1/2 z-30 flex items-center gap-2 px-3 py-1.5 rounded-md bg-[#12151cf5] border border-[#2a3142] shadow-lg text-xs font-mono text-[#e2e8f0]"
      >
        <Database className="w-3.5 h-3.5 text-[#f97316]" />
        <span className="tabular-nums">
          {isSolveActive
            ? `FEM ${effectivePct}% · Кадр ${(liveSolveStatus?.framesCached ?? (cache ? cache.frames.length : 0))}${
                liveSolveStatus?.frameCount ? `/${liveSolveStatus.frameCount}` : ''
              }${liveSolveStatus?.estimatedRemainingMs != null ? ` · ~${etaStr}` : ''}`
            : cache
              ? `${cache.solverLabel} · Кадр ${currentFrame}/${maxPlayableFrame}`
              : 'Симуляция разрушения (не запечено)'}
        </span>
        {isSolveActive && liveSolveStatus?.cancellable && onCancelBake && (
          <button
            type="button"
            onClick={onCancelBake}
            className="px-2 py-0.5 rounded bg-[#2a1418] hover:bg-[#3f1a20] border border-[#ef4444]/60 text-[#fca5a5] font-semibold cursor-pointer"
          >
            Отмена
          </button>
        )}
        {cache && !isSolveActive && (
          <button
            type="button"
            onClick={onTogglePlay}
            aria-label={isPlaying ? 'Пауза воспроизведения кэша' : 'Воспроизвести кэш'}
            className="px-2 py-0.5 rounded bg-[#f97316] hover:bg-[#ea580c] text-[#0c0e12] font-semibold cursor-pointer"
          >
            {isPlaying ? 'Пауза' : 'Пуск'}
          </button>
        )}
        <button
          type="button"
          onClick={() => setCollapsed(false)}
          aria-label="Развернуть таймлайн разрушения"
          className="flex items-center gap-1 px-2 py-0.5 rounded bg-[#1d222e] hover:bg-[#262d3d] text-[#cbd5e1] border border-[#2d3545] cursor-pointer"
        >
          <ChevronUp className="w-3.5 h-3.5" />
          <span>Таймлайн</span>
        </button>
      </div>
    );
  }

  return (
    <div
      data-testid="destruction-timeline"
      className="absolute bottom-3 left-1/2 -translate-x-1/2 z-30 w-[min(940px,calc(100%-24px))] rounded-lg bg-[#11141cf5] border border-[#2b3345] shadow-[0_14px_40px_rgba(0,0,0,0.65)] text-[#f1f5f9] select-none"
    >
      {/* Top thin header tab */}
      <div className="flex items-center justify-between px-3 py-1.5 border-b border-[#222836] bg-[#151923] rounded-t-lg gap-2">
        <div className="flex items-center gap-2 text-[11px] font-mono min-w-0">
          <div className="flex items-center gap-1.5 text-[#f97316] font-semibold shrink-0">
            <Flame className="w-3.5 h-3.5" />
            <span>Кэш симуляции</span>
          </div>

          {(cache || liveSolveStatus) && (
            <span
              data-testid="timeline-solver-badge"
              className={`px-1.5 py-0.5 rounded text-[9px] font-mono font-semibold border shrink-0 ${
                cache?.solverBackend === 'rust_fem_f64' ||
                liveSolveStatus?.source === 'rust_fem' ||
                liveSolveStatus?.source === 'live_sync'
                  ? 'bg-[#052e1e] border-[#10b981]/60 text-[#34d399]'
                  : 'bg-[#2a1d12] border-[#f59e0b]/60 text-[#fbbf24]'
              }`}
            >
              {cache?.solverLabel ??
                (liveSolveStatus?.source === 'web_local'
                  ? 'Web Local Preview'
                  : 'Rust FEM f64')}
            </span>
          )}

          {isSolveActive ? (
            <span className="text-[#38bdf8] truncate tabular-nums">
              {cache
                ? `В кэше: ${cache.frames.length} кадр.`
                : `Идёт расчёт FEM (${effectivePct}%)`}
            </span>
          ) : cache ? (
            <span
              className="text-[#94a3b8] tabular-nums truncate pr-1"
              title={`Запечено за ${cache.bakeTimeMs.toFixed(0)} мс (${cache.totalFrames + 1} кадров @ ${cache.fps} FPS)`}
            >
              {cache.frames.length} кадр. · {cache.fps} FPS ·{' '}
              {formatCacheBytes(cache.cacheBytes)}
              {isCacheStale ? ' · Устарел' : ''}
            </span>
          ) : (
            <span className="text-[#94a3b8] truncate pr-1">
              Кэш не рассчитан — нажмите «Запечь кэш» для расчёта деформаций и разрушения
            </span>
          )}
        </div>

        <div className="flex items-center gap-2 shrink-0 pl-2 border-l border-[#232938]">
          {cache && activeFrameData && (
            <span className="hidden md:inline text-[10px] font-mono tabular-nums text-[#cbd5e1] truncate max-w-[260px]">
              {activeFrameData.metrics.statusLabel} · σ=
              <strong className="text-[#f97316]">
                {activeFrameData.metrics.peakStressMPa.toFixed(0)}
              </strong>{' '}
              МПа
            </span>
          )}
          <button
            type="button"
            onClick={() => setCollapsed(true)}
            aria-label="Свернуть таймлайн разрушения"
            title="Свернуть таймлайн для чистого обзора 3D сцены"
            className="flex items-center gap-1 px-2 py-0.5 rounded bg-[#1d222e] hover:bg-[#262d3d] border border-[#2d3545] text-[10px] font-mono text-[#94a3b8] hover:text-[#f1f5f9] cursor-pointer whitespace-nowrap"
          >
            <ChevronDown className="w-3 h-3" />
            <span>Свернуть</span>
          </button>
        </div>
      </div>

      {/* Body */}
      <div className="px-3 py-2 space-y-2">
        {isSolveActive && (
          <div
            data-testid="live-fem-progress-hud"
            className="space-y-1.5 px-2.5 py-2 rounded bg-[#0e131d] border border-[#253147]"
          >
            <div className="flex flex-wrap items-center justify-between gap-2 text-[11px] font-mono tabular-nums">
              <div className="flex items-center gap-2 min-w-0">
                <span className="px-1.5 py-0.5 rounded bg-[#0c273b] border border-[#38bdf8]/45 text-[#38bdf8] text-[9px] font-semibold uppercase tracking-wider shrink-0">
                  LIVE FEM
                </span>
                <span className="text-[#e2e8f0] truncate max-w-[360px]">
                  {effectiveStage}
                </span>
              </div>

              <div className="flex items-center gap-2.5 shrink-0 flex-wrap">
                {liveSolveStatus && (
                  <>
                    <span className="text-[#cbd5e1]">
                      Кадр{' '}
                      <strong className="text-[#f97316]">
                        {liveSolveStatus.framesCached}
                      </strong>
                      {liveSolveStatus.frameCount > 0
                        ? `/${liveSolveStatus.frameCount}`
                        : ''}
                    </span>
                    <span className="text-[#94a3b8]">
                      Шаг {liveSolveStatus.stepCount}
                    </span>
                    <span className="text-[#38bdf8]">
                      t={liveSolveStatus.currentTimeSec.toFixed(3)}
                      {liveSolveStatus.totalTimeSec > 0
                        ? `/${liveSolveStatus.totalTimeSec.toFixed(2)}с`
                        : 'с'}
                    </span>
                    <span className="text-[#fb923c]">
                      σ_max={liveSolveStatus.maxVonMisesMPa.toFixed(0)} МПа
                    </span>
                    <span className="text-[#94a3b8]">
                      {elapsedStr}
                      {liveSolveStatus.estimatedRemainingMs != null
                        ? ` · ~${etaStr}`
                        : ''}
                    </span>
                  </>
                )}

                <span
                  data-testid="live-fem-progress-pct"
                  className="text-[#38bdf8] font-bold"
                >
                  {effectivePct}%
                </span>

                {onToggleFollowLive && (
                  <button
                    type="button"
                    data-testid="follow-live-btn"
                    onClick={onToggleFollowLive}
                    aria-pressed={followLiveFrame}
                    title="Автоматически переключать 3D вид на каждый новый рассчитанный кадр"
                    className={`flex items-center gap-1 px-2 py-0.5 rounded border text-[10px] cursor-pointer whitespace-nowrap ${
                      followLiveFrame
                        ? 'bg-[#052e1e] border-[#10b981]/60 text-[#34d399]'
                        : 'bg-[#1a1f2b] border-[#2d3545] text-[#94a3b8] hover:text-[#e2e8f0]'
                    }`}
                  >
                    <Radio className="w-3 h-3" />
                    <span>Авто-кадр</span>
                  </button>
                )}

                {(liveSolveStatus?.cancellable || isBaking) && onCancelBake && (
                  <button
                    type="button"
                    data-testid="timeline-cancel-bake-btn"
                    onClick={onCancelBake}
                    className="flex items-center gap-1 px-2 py-0.5 rounded bg-[#2a1418] hover:bg-[#3f1a20] border border-[#ef4444]/60 text-[#fca5a5] text-[10px] cursor-pointer whitespace-nowrap"
                  >
                    <Square className="w-2.5 h-2.5 fill-current" />
                    <span>Отмена</span>
                  </button>
                )}
              </div>
            </div>

            <div className="w-full h-1.5 rounded bg-[#080b10] border border-[#222b3d] overflow-hidden">
              <div
                className="h-full bg-[#38bdf8] transition-all duration-150"
                style={{ width: `${effectivePct}%` }}
              />
            </div>
          </div>
        )}

        {!cache ? (
          !isSolveActive && (
            <div className="flex flex-wrap items-center justify-between gap-2 py-0.5">
              <p className="text-xs text-[#94a3b8]">
                Предварительный расчёт деформаций, напряжений σ_vM, пластичности ε_p, повреждаемости D и локальных сколов сохраняет кадры в кэш.
              </p>
              <button
                type="button"
                onClick={onBakeCache}
                className="flex items-center gap-1.5 px-3.5 py-1.5 rounded bg-[#f97316] hover:bg-[#ea580c] text-[#0c0e12] text-xs font-semibold cursor-pointer whitespace-nowrap shrink-0 transition-colors"
              >
                <Zap className="w-3.5 h-3.5" />
                <span>Запечь кэш симуляции</span>
              </button>
            </div>
          )
        ) : (
          <>
            {/* Compact Energy Balance & Overlay Switcher Legend */}
            {activeFrameData && (
              <div className="flex flex-wrap items-center justify-between gap-2 text-[10px] font-mono tabular-nums">
                <div className="flex items-center gap-2.5 flex-wrap">
                  <span className="text-[#38bdf8]">
                    E_k: {activeFrameData.metrics.kineticEnergyJ.toFixed(3)} Дж
                  </span>
                  <span className="text-[#10b981]">
                    E_e: {activeFrameData.metrics.elasticEnergyJ.toFixed(3)} Дж
                  </span>
                  <span className="text-[#f59e0b]">
                    E_p: {activeFrameData.metrics.plasticEnergyJ.toFixed(3)} Дж
                  </span>
                  <span className="text-[#f43f5e]">
                    E_f: {activeFrameData.metrics.fractureEnergyJ.toFixed(3)} Дж
                  </span>
                </div>

                <div className="flex items-center gap-1.5">
                  {onChangeFieldMode && (
                    <div
                      role="group"
                      aria-label="Поле визуализации 3D"
                      className="flex items-center gap-0.5 bg-[#0b0e14] p-0.5 rounded border border-[#232a3a]"
                    >
                      {(
                        [
                          { id: 'shaded', label: 'Shaded', activeCls: 'bg-[#cbd5e1] text-[#0c0e12]' },
                          { id: 'von_mises', label: 'σ_vM', activeCls: 'bg-[#f97316] text-[#0c0e12]' },
                          { id: 'plastic_strain', label: 'ε_p', activeCls: 'bg-[#c084fc] text-[#0c0e12]' },
                          { id: 'damage', label: 'D', activeCls: 'bg-[#ef4444] text-[#ffffff]' },
                        ] as const
                      ).map((item) => (
                        <button
                          key={item.id}
                          type="button"
                          onClick={() => onChangeFieldMode(item.id)}
                          data-testid={`timeline-field-${item.id}`}
                          className={`px-1.5 py-0.5 rounded text-[9px] font-mono cursor-pointer transition-colors ${
                            fieldMode === item.id
                              ? `${item.activeCls} font-semibold`
                              : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                          }`}
                        >
                          {item.label}
                        </button>
                      ))}
                    </div>
                  )}

                  <div className="flex items-center gap-1 bg-[#0b0e14] p-0.5 rounded border border-[#232a3a]">
                    <button
                      type="button"
                      onClick={() => setTrackOverlayMode('energy')}
                      className={`px-1.5 py-0.5 rounded text-[9px] cursor-pointer ${
                        trackOverlayMode === 'energy'
                          ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                          : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                      }`}
                    >
                      Энергии
                    </button>
                    <button
                      type="button"
                      onClick={() => setTrackOverlayMode('stress')}
                      className={`px-1.5 py-0.5 rounded text-[9px] cursor-pointer ${
                        trackOverlayMode === 'stress'
                          ? 'bg-[#f97316] text-[#0c0e12] font-semibold'
                          : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                      }`}
                    >
                      График σ
                    </button>
                  </div>
                </div>
              </div>
            )}

            {/* Scrubber Track with embedded Energy Balance / Stress Curves & Event Markers */}
            <div className="relative pt-0.5 pb-0.5">
              <div className="relative h-9 rounded bg-[#0b0e14] border border-[#232a3a] overflow-hidden">
                {/* Cached progress fill */}
                <div
                  className="absolute inset-y-0 left-0 bg-[#f97316]/12 border-r border-[#f97316]/75"
                  style={{ width: `${progressPct}%` }}
                />

                {/* Embedded Energy Balance or Stress Sparkline */}
                <svg
                  viewBox="0 0 100 32"
                  preserveAspectRatio="none"
                  className="absolute inset-0 w-full h-full pointer-events-none"
                  aria-hidden="true"
                >
                  {trackOverlayMode === 'energy' && energyTrackSeries ? (
                    <>
                      {energyTrackSeries.kineticPoints && (
                        <polyline
                          fill="none"
                          stroke="#38bdf8"
                          strokeOpacity="0.85"
                          strokeWidth="1.2"
                          vectorEffect="non-scaling-stroke"
                          points={energyTrackSeries.kineticPoints}
                        />
                      )}
                      {energyTrackSeries.elasticPoints && (
                        <polyline
                          fill="none"
                          stroke="#10b981"
                          strokeOpacity="0.85"
                          strokeWidth="1.2"
                          vectorEffect="non-scaling-stroke"
                          points={energyTrackSeries.elasticPoints}
                        />
                      )}
                      {energyTrackSeries.plasticPoints && (
                        <polyline
                          fill="none"
                          stroke="#f59e0b"
                          strokeOpacity="0.9"
                          strokeWidth="1.2"
                          vectorEffect="non-scaling-stroke"
                          points={energyTrackSeries.plasticPoints}
                        />
                      )}
                      {energyTrackSeries.fracturePoints && (
                        <polyline
                          fill="none"
                          stroke="#f43f5e"
                          strokeOpacity="0.9"
                          strokeWidth="1.2"
                          vectorEffect="non-scaling-stroke"
                          points={energyTrackSeries.fracturePoints}
                        />
                      )}
                    </>
                  ) : (
                    stressSparklinePoints && (
                      <polyline
                        fill="none"
                        stroke="#f97316"
                        strokeOpacity="0.8"
                        strokeWidth="1.3"
                        vectorEffect="non-scaling-stroke"
                        points={stressSparklinePoints}
                      />
                    )
                  )}
                </svg>

                {/* Event Marker Ticks */}
                {cache.events.map((ev, idx) => {
                  const leftPct =
                    cache.totalFrames > 0
                      ? (ev.frameIndex / cache.totalFrames) * 100
                      : 0;
                  return (
                    <button
                      key={`${ev.type}_${ev.frameIndex}_${idx}`}
                      type="button"
                      onClick={(e) => {
                        e.stopPropagation();
                        onSeekFrame(ev.frameIndex);
                      }}
                      title={`${ev.label} (кадр ${ev.frameIndex}, ${ev.timeSec.toFixed(2)} с)`}
                      aria-label={`Перейти к событию: ${ev.label} на кадре ${ev.frameIndex}`}
                      style={{
                        left: `${Math.max(1, Math.min(99, leftPct))}%`,
                        backgroundColor: eventColor(ev.type),
                      }}
                      className="absolute top-0 bottom-0 w-1 -translate-x-1/2 z-20 opacity-85 hover:opacity-100 hover:w-1.5 transition-all cursor-pointer"
                    />
                  );
                })}

                {/* Native Range Input overlay for precise dragging & keyboard accessibility */}
                <input
                  type="range"
                  aria-label="Таймлайн кэша симуляции разрушения"
                  min={0}
                  max={cache.totalFrames}
                  step={1}
                  value={currentFrame}
                  onChange={(e) => onSeekFrame(Number(e.target.value))}
                  className="absolute inset-0 w-full h-full opacity-0 cursor-ew-resize z-10"
                />

                {/* Playhead handle indicator */}
                <div
                  className="absolute top-0 bottom-0 w-0.5 bg-[#f8fafc] pointer-events-none z-10"
                  style={{ left: `${progressPct}%` }}
                />
              </div>
            </div>

            {/* Transport & Speed Controls Row */}
            <div className="flex flex-wrap items-center justify-between gap-2">
              {/* Left: Transport Buttons */}
              <div className="flex items-center gap-1">
                <button
                  type="button"
                  onClick={() => onSeekFrame(0)}
                  aria-label="В начало кэша"
                  title="В начало (кадр 0)"
                  className="p-1.5 rounded bg-[#1a1f2b] hover:bg-[#252c3d] border border-[#2a3142] text-[#cbd5e1] cursor-pointer"
                >
                  <SkipBack className="w-3.5 h-3.5" />
                </button>
                <button
                  type="button"
                  onClick={() => onStepFrame(-1)}
                  aria-label="Предыдущий кадр"
                  title="Назад на 1 кадр (Стрелка влево)"
                  className="p-1.5 rounded bg-[#1a1f2b] hover:bg-[#252c3d] border border-[#2a3142] text-[#cbd5e1] cursor-pointer"
                >
                  <ChevronLeft className="w-3.5 h-3.5" />
                </button>

                <button
                  type="button"
                  onClick={onTogglePlay}
                  aria-label={isPlaying ? 'Пауза' : 'Воспроизвести кэш'}
                  className={`flex items-center gap-1.5 px-3 py-1.5 rounded text-xs font-semibold cursor-pointer whitespace-nowrap transition-colors ${
                    isPlaying
                      ? 'bg-[#ef4444] hover:bg-[#dc2626] text-white'
                      : 'bg-[#f97316] hover:bg-[#ea580c] text-[#0c0e12]'
                  }`}
                >
                  {isPlaying ? (
                    <>
                      <Pause className="w-3.5 h-3.5 fill-current" />
                      <span>Пауза</span>
                    </>
                  ) : (
                    <>
                      <Play className="w-3.5 h-3.5 fill-current" />
                      <span>Пуск</span>
                    </>
                  )}
                </button>

                <button
                  type="button"
                  onClick={() => onStepFrame(1)}
                  aria-label="Следующий кадр"
                  title="Вперёд на 1 кадр (Стрелка вправо)"
                  className="p-1.5 rounded bg-[#1a1f2b] hover:bg-[#252c3d] border border-[#2a3142] text-[#cbd5e1] cursor-pointer"
                >
                  <ChevronRight className="w-3.5 h-3.5" />
                </button>
                <button
                  type="button"
                  onClick={() => onSeekFrame(cache.totalFrames)}
                  aria-label="В конец кэша"
                  title="В конец симуляции"
                  className="p-1.5 rounded bg-[#1a1f2b] hover:bg-[#252c3d] border border-[#2a3142] text-[#cbd5e1] cursor-pointer"
                >
                  <SkipForward className="w-3.5 h-3.5" />
                </button>

                <button
                  type="button"
                  onClick={onToggleLoop}
                  aria-label="Зациклить воспроизведение"
                  title="Зациклить воспроизведение кэша"
                  className={`p-1.5 rounded border cursor-pointer transition-colors ${
                    loopEnabled
                      ? 'bg-[#38bdf8]/20 border-[#38bdf8]/60 text-[#38bdf8]'
                      : 'bg-[#1a1f2b] border-[#2a3142] text-[#64748b] hover:text-[#cbd5e1]'
                  }`}
                >
                  <Repeat className="w-3.5 h-3.5" />
                </button>

                <span className="ml-2 text-xs font-mono tabular-nums text-[#e2e8f0] whitespace-nowrap">
                  Кадр{' '}
                  <strong className="text-[#f97316]">
                    {String(currentFrame).padStart(3, '0')}
                  </strong>
                  /{cache.totalFrames}
                  <span className="text-[#64748b] mx-1.5">·</span>
                  {(activeFrameData?.timeSec ?? 0).toFixed(2)} /{' '}
                  {cache.durationSec.toFixed(2)} с
                </span>
              </div>

              {/* Right: Speed Selector, Visual Toggles & Re-bake */}
              <div className="flex items-center gap-1.5">
                <div
                  role="group"
                  aria-label="Скорость воспроизведения"
                  className="flex items-center bg-[#0c0f15] p-0.5 rounded border border-[#232a3a]"
                >
                  {PLAYBACK_SPEEDS.map((spd) => (
                    <button
                      key={spd}
                      type="button"
                      onClick={() => onChangeSpeed(spd)}
                      className={`px-1.5 py-0.5 rounded text-[10px] font-mono tabular-nums cursor-pointer transition-colors whitespace-nowrap ${
                        playbackSpeed === spd
                          ? 'bg-[#f97316] text-[#0c0e12] font-semibold'
                          : 'text-[#94a3b8] hover:text-[#f1f5f9]'
                      }`}
                    >
                      {spd}×
                    </button>
                  ))}
                </div>

                <button
                  type="button"
                  onClick={onToggleStressGlow}
                  title="Подсвечивать тепловую зону напряжений и свежих разломов (в режиме Shaded)"
                  className={`flex items-center gap-1 px-2 py-1 rounded border text-[10px] font-mono cursor-pointer whitespace-nowrap ${
                    showStressGlow
                      ? 'bg-[#ef4444]/20 border-[#ef4444]/60 text-[#fca5a5]'
                      : 'bg-[#1a1f2b] border-[#2a3142] text-[#94a3b8]'
                  }`}
                >
                  <Flame className="w-3 h-3" />
                  <span>Разлом</span>
                </button>

                <button
                  type="button"
                  onClick={onToggleSparks}
                  title="Показать / скрыть металлические искры при ударе"
                  className={`flex items-center gap-1 px-2 py-1 rounded border text-[10px] font-mono cursor-pointer whitespace-nowrap ${
                    showSparks
                      ? 'bg-[#f59e0b]/20 border-[#f59e0b]/60 text-[#fcd34d]'
                      : 'bg-[#1a1f2b] border-[#2a3142] text-[#94a3b8]'
                  }`}
                >
                  <Sparkles className="w-3 h-3" />
                  <span>Искры</span>
                </button>

                {isCacheStale && (
                  <button
                    type="button"
                    onClick={onBakeCache}
                    className="flex items-center gap-1 px-2 py-1 rounded bg-[#f97316] hover:bg-[#ea580c] text-[#0c0e12] text-[10px] font-semibold cursor-pointer whitespace-nowrap"
                  >
                    <Zap className="w-3 h-3" />
                    <span>Перезапечь</span>
                  </button>
                )}

                {onClearCache && (
                  <button
                    type="button"
                    onClick={onClearCache}
                    title="Очистить кэш и вернуть исходные целые детали"
                    aria-label="Очистить кэш симуляции"
                    className="p-1.5 rounded bg-[#1a1f2b] hover:bg-[#252c3d] border border-[#2a3142] text-[#94a3b8] hover:text-[#f87171] cursor-pointer"
                  >
                    <RotateCcw className="w-3.5 h-3.5" />
                  </button>
                )}
              </div>
            </div>
          </>
        )}
      </div>
    </div>
  );
};
