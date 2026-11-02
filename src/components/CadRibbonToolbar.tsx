import React, { useRef, useState, useEffect } from 'react';
import {
  Box,
  ChevronDown,
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
  X,
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

interface CadRibbonToolbarProps {
  activeWorkbench: EngineeringWorkbench;
  activePreset?: string;
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

export interface PrimitiveMatrixItem {
  type: DagNodeType;
  label: string;
  title: string;
  icon: React.ReactNode;
}

const PRIMITIVE_MATRIX_ITEMS: PrimitiveMatrixItem[] = [
  {
    type: 'box',
    label: 'Блок',
    title: 'Добавить параметрический параллелепипед (Куб)',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <path d="M12 2.5L3.5 7.5v9L12 21.5l8.5-5v-9L12 2.5z" strokeLinecap="round" strokeLinejoin="round" />
        <path d="M12 21.5V12" strokeLinecap="round" strokeLinejoin="round" />
        <path d="M20.5 7.5L12 12 3.5 7.5" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    ),
  },
  {
    type: 'cylinder',
    label: 'Цилиндр',
    title: 'Добавить параметрический цилиндр',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <ellipse cx="12" cy="6" rx="8" ry="3.2" />
        <path d="M4 6v12c0 1.77 3.58 3.2 8 3.2s8-1.43 8-3.2V6" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    ),
  },
  {
    type: 'sphere',
    label: 'Сфера',
    title: 'Добавить параметрическую сферу',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <circle cx="12" cy="12" r="9" />
        <path d="M3 12c0 2.5 4 4.5 9 4.5s9-2 9-4.5-4-4.5-9-4.5-9 2-9 4.5z" strokeDasharray="2 2" />
        <path d="M12 3c-2.5 0-4.5 4-4.5 9s2 9 4.5 9 4.5-4 4.5-9-2-9-4.5-9z" strokeDasharray="2 2" />
      </svg>
    ),
  },
  {
    type: 'cone',
    label: 'Конус',
    title: 'Добавить параметрический конус',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <ellipse cx="12" cy="18" rx="8" ry="3" />
        <path d="M4 18L12 3l8 15" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    ),
  },
  {
    type: 'polygon_prism',
    label: 'Призма',
    title: 'Добавить шестигранную призму с фаской DIN 934',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <path d="M12 2.5L18.5 5.5v5L12 13.5L5.5 10.5v-5L12 2.5z" strokeLinecap="round" strokeLinejoin="round" />
        <path d="M12 13.5v8" strokeLinecap="round" strokeLinejoin="round" />
        <path d="M18.5 10.5v8L12 21.5L5.5 18.5v-8" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    ),
  },
  {
    type: 'spur_gear',
    label: 'Шестерня',
    title: 'Добавить эвольвентную шестерню со ступицей и шпонкой',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <circle cx="12" cy="12" r="3" />
        <path d="M12 2v2.5M12 19.5V22M2 12h2.5M19.5 12H22M4.93 4.93l1.77 1.77M17.3 17.3l1.77 1.77M4.93 19.07l1.77-1.77M17.3 6.7l1.77-1.77" strokeLinecap="round" />
        <circle cx="12" cy="12" r="7.5" />
      </svg>
    ),
  },
  {
    type: 'revolve',
    label: 'Вращение',
    title: 'Добавить тело вращения по замкнутому профилю',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <path d="M12 3v18" strokeDasharray="3 2" />
        <path d="M6 8c0-2.2 2.7-4 6-4s6 1.8 6 4-2.7 4-6 4" strokeLinecap="round" />
        <path d="M18 16c0 2.2-2.7 4-6 4s-6-1.8-6-4 2.7-4 6-4" strokeLinecap="round" />
        <path d="M15 14l3 2-3 2" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    ),
  },
  {
    type: 'wheel_rim',
    label: 'Автодиск',
    title: 'Добавить параметрический автомобильный диск',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
        <circle cx="12" cy="12" r="9.5" />
        <circle cx="12" cy="12" r="7.5" opacity="0.6" />
        <circle cx="12" cy="12" r="2.8" />
        <path d="M12 4.5v4.7M19.1 9.7l-4.5 1.5M16.4 18.1l-3.8-2.8M7.6 18.1l3.8-2.8M4.9 9.7l4.5 1.5" strokeLinecap="round" strokeWidth="1.6" />
      </svg>
    ),
  },
];

export interface AssemblyPresetItem {
  id: string;
  label: string;
  description: string;
  icon: React.ReactNode;
}

const ASSEMBLY_PRESETS: AssemblyPresetItem[] = [
  {
    id: 'gear_transmission',
    label: 'Редуктор Z1=20 / Z2=40',
    description: 'Двухступенчатая зубчатая передача',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-4 h-4 text-[#10b981]">
        <circle cx="9" cy="12" r="4" />
        <circle cx="17" cy="12" r="2.5" />
        <path d="M9 6v2M9 16v2M3 12h2M13 12h2" strokeLinecap="round" />
      </svg>
    ),
  },
  {
    id: 'crank_mechanism',
    label: 'Кривошипно-шатунный',
    description: 'Кривошипно-ползунный механизм (Rapier3D)',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-4 h-4 text-[#38bdf8]">
        <circle cx="6" cy="14" r="2" />
        <path d="M6 14l6-6 8 3" strokeLinecap="round" strokeLinejoin="round" />
        <rect x="18" y="9" width="4" height="4" rx="1" />
      </svg>
    ),
  },
  {
    id: 'spur_gear',
    label: 'Привод шестерни',
    description: 'Эвольвентная шестерня со ступицей',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-4 h-4 text-[#f59e0b]">
        <circle cx="12" cy="12" r="3" />
        <circle cx="12" cy="12" r="7.5" />
      </svg>
    ),
  },
  {
    id: 'twist_drill',
    label: 'Шпиндель сверла',
    description: 'Спиральное сверло со шпинделем',
    icon: (
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-4 h-4 text-[#a855f7]">
        <path d="M12 2v20M8 6l8 4M8 12l8 4" strokeLinecap="round" />
      </svg>
    ),
  },
];

export const CadRibbonToolbar: React.FC<CadRibbonToolbarProps> = ({
  activeWorkbench,
  activePreset,
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

  // Assemblies Popover State
  const [isAssembliesOpen, setIsAssembliesOpen] = useState(false);
  const [assembliesMenuCoords, setAssembliesMenuCoords] = useState<{ top: number; left: number } | null>(null);
  const assembliesMenuRef = useRef<HTMLDivElement | null>(null);

  // Bodies Popover State
  const [isBodiesOpen, setIsBodiesOpen] = useState(false);
  const [menuCoords, setMenuCoords] = useState<{ top: number; left: number } | null>(null);
  const bodiesMenuRef = useRef<HTMLDivElement | null>(null);

  // CSG Popover State
  const [isCsgOpen, setIsCsgOpen] = useState(false);
  const [csgMenuCoords, setCsgMenuCoords] = useState<{ top: number; left: number } | null>(null);
  const csgMenuRef = useRef<HTMLDivElement | null>(null);

  const handleToggleBodies = () => {
    if (!isBodiesOpen && bodiesMenuRef.current) {
      const rect = bodiesMenuRef.current.getBoundingClientRect();
      setMenuCoords({ top: rect.bottom + 6, left: rect.left });
    }
    setIsBodiesOpen((prev) => !prev);
    setIsCsgOpen(false);
    setIsAssembliesOpen(false);
  };

  const handleToggleCsg = () => {
    if (!isCsgOpen && csgMenuRef.current) {
      const rect = csgMenuRef.current.getBoundingClientRect();
      setCsgMenuCoords({ top: rect.bottom + 6, left: rect.left });
    }
    setIsCsgOpen((prev) => !prev);
    setIsBodiesOpen(false);
    setIsAssembliesOpen(false);
  };

  const handleToggleAssemblies = () => {
    if (!isAssembliesOpen && assembliesMenuRef.current) {
      const rect = assembliesMenuRef.current.getBoundingClientRect();
      setAssembliesMenuCoords({ top: rect.bottom + 6, left: rect.left });
    }
    setIsAssembliesOpen((prev) => !prev);
    setIsBodiesOpen(false);
    setIsCsgOpen(false);
  };

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as Node;
      if (
        bodiesMenuRef.current &&
        !bodiesMenuRef.current.contains(target)
      ) {
        setIsBodiesOpen(false);
      }
      if (
        csgMenuRef.current &&
        !csgMenuRef.current.contains(target)
      ) {
        setIsCsgOpen(false);
      }
      if (
        assembliesMenuRef.current &&
        !assembliesMenuRef.current.contains(target)
      ) {
        setIsAssembliesOpen(false);
      }
    };
    if (isBodiesOpen || isCsgOpen || isAssembliesOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isBodiesOpen, isCsgOpen, isAssembliesOpen]);

  return (
    <div className="h-10 bg-[#151922] border-b border-[#252b38] px-2.5 flex items-center justify-between gap-1.5 select-none shrink-0 overflow-x-auto">
      {/* Contextual Left & Center Controls depending on activeWorkbench */}
      {activeWorkbench === 'modeling' && (
        <div className="flex items-center gap-1 shrink-0">
          {/* Minimalist Seamless Primitive Matrix Submenu */}
          <div className="relative shrink-0" ref={bodiesMenuRef}>
            <button
              onClick={handleToggleBodies}
              title="Открыть матрицу примитивов"
              className={`flex items-center gap-1.5 px-2.5 py-1 rounded border text-[11px] font-semibold transition-all cursor-pointer whitespace-nowrap ${
                isBodiesOpen
                  ? 'bg-[#38bdf8]/20 border-[#38bdf8] text-[#38bdf8] shadow-[0_0_10px_rgba(56,189,248,0.25)]'
                  : 'bg-[#1d222e] hover:bg-[#262d3d] border-[#2d3545] text-[#f1f5f9]'
              }`}
            >
              <Box className="w-3.5 h-3.5 text-[#38bdf8]" />
              <span>Тела</span>
              <ChevronDown
                className={`w-3 h-3 text-[#94a3b8] transition-transform duration-200 ${
                  isBodiesOpen ? 'rotate-180 text-[#38bdf8]' : ''
                }`}
              />
            </button>

            {isBodiesOpen && (
              <div
                style={{
                  top: menuCoords ? `${menuCoords.top}px` : '42px',
                  left: menuCoords ? `${menuCoords.left}px` : '10px',
                }}
                className="fixed z-[9999] bg-[#141824] border border-[#262f40] rounded-lg shadow-[0_12px_36px_rgba(0,0,0,0.85)] p-1 overflow-hidden select-none"
              >
                <div className="grid grid-cols-4 gap-0.5">
                  {PRIMITIVE_MATRIX_ITEMS.map((item) => (
                    <button
                      key={item.type}
                      onClick={() => {
                        onAddPrimitive(item.type);
                        setIsBodiesOpen(false);
                      }}
                      title={item.title}
                      className="group flex flex-col items-center justify-center w-20 h-16 rounded-md hover:bg-[#222b3d] transition-all cursor-pointer text-center"
                    >
                      <div className="text-[#94a3b8] group-hover:text-[#38bdf8] transition-colors mb-1 group-hover:scale-110">
                        {item.icon}
                      </div>
                      <span className="text-[11px] font-medium text-[#cbd5e1] group-hover:text-white truncate w-full px-1">
                        {item.label}
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>

          {/* Minimalist Seamless CSG Matrix Submenu */}
          <div className="relative shrink-0" ref={csgMenuRef}>
            <button
              onClick={handleToggleCsg}
              title="Открыть операции CSG (Отверстия, Резьба, Вырезы)"
              className={`flex items-center gap-1.5 px-2.5 py-1 rounded border text-[11px] font-semibold transition-all cursor-pointer whitespace-nowrap ${
                isCsgOpen
                  ? 'bg-[#ef4444]/20 border-[#ef4444] text-[#f87171] shadow-[0_0_10px_rgba(239,68,68,0.25)]'
                  : 'bg-[#1d222e] hover:bg-[#262d3d] border-[#2d3545] text-[#f1f5f9]'
              }`}
            >
              <Scissors className="w-3.5 h-3.5 text-[#ef4444]" />
              <span>CSG</span>
              <ChevronDown
                className={`w-3 h-3 text-[#94a3b8] transition-transform duration-200 ${
                  isCsgOpen ? 'rotate-180 text-[#ef4444]' : ''
                }`}
              />
            </button>

            {isCsgOpen && (
              <div
                style={{
                  top: csgMenuCoords ? `${csgMenuCoords.top}px` : '42px',
                  left: csgMenuCoords ? `${csgMenuCoords.left}px` : '85px',
                }}
                className="fixed z-[9999] bg-[#141824] border border-[#262f40] rounded-lg shadow-[0_12px_36px_rgba(0,0,0,0.85)] p-1 overflow-hidden select-none"
              >
                <div className="grid grid-cols-3 gap-0.5">
                  <button
                    onClick={() => {
                      onOpenCutHoleModal();
                      setIsCsgOpen(false);
                    }}
                    title="Вырезать цилиндрическое отверстие со шпоночным пазом DIN 6885"
                    className="group flex flex-col items-center justify-center w-20 h-16 rounded-md hover:bg-[#222b3d] transition-all cursor-pointer text-center"
                  >
                    <div className="text-[#94a3b8] group-hover:text-[#ef4444] transition-colors mb-1 group-hover:scale-110">
                      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
                        <path d="M10 4.5h4v3h-4v-3z" strokeLinecap="round" strokeLinejoin="round" />
                        <circle cx="12" cy="13.5" r="7" />
                      </svg>
                    </div>
                    <span className="text-[11px] font-medium text-[#cbd5e1] group-hover:text-white truncate w-full px-1">
                      Отверстие
                    </span>
                  </button>

                  <button
                    onClick={() => {
                      onOpenHelicalModal();
                      setIsCsgOpen(false);
                    }}
                    title="Нарезать метрическую резьбу ISO 68-1 или спиральную канавку"
                    className="group flex flex-col items-center justify-center w-20 h-16 rounded-md hover:bg-[#222b3d] transition-all cursor-pointer text-center"
                  >
                    <div className="text-[#94a3b8] group-hover:text-[#a855f7] transition-colors mb-1 group-hover:scale-110">
                      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
                        <path d="M7 4.5h10v15H7z" strokeLinecap="round" strokeLinejoin="round" />
                        <path d="M7 8.5l10-2M7 12.5l10-2M7 16.5l10-2" strokeLinecap="round" />
                      </svg>
                    </div>
                    <span className="text-[11px] font-medium text-[#cbd5e1] group-hover:text-white truncate w-full px-1">
                      Резьба
                    </span>
                  </button>

                  <button
                    onClick={() => {
                      onApplyTwoHoleCsgChain();
                      setIsCsgOpen(false);
                    }}
                    title="Применить цепочку из 2 сквозных отверстий (род поверхности g + 2)"
                    className="group flex flex-col items-center justify-center w-20 h-16 rounded-md hover:bg-[#222b3d] transition-all cursor-pointer text-center"
                  >
                    <div className="text-[#94a3b8] group-hover:text-[#38bdf8] transition-colors mb-1 group-hover:scale-110">
                      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="w-5 h-5">
                        <rect x="3" y="6" width="18" height="12" rx="2" />
                        <circle cx="8.5" cy="12" r="2.5" />
                        <circle cx="15.5" cy="12" r="2.5" />
                      </svg>
                    </div>
                    <span className="text-[11px] font-medium text-[#cbd5e1] group-hover:text-white truncate w-full px-1">
                      2 Отв.
                    </span>
                  </button>
                </div>
              </div>
            )}
          </div>

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
          <button
            onClick={() => onSelectPreset('kirsch_plate')}
            title="Спроектировать стальную пластину 200x200x10 мм с отверстием R=20мм (Kirsch benchmark)"
            className="flex items-center gap-1.5 px-2.5 py-1 rounded bg-[#38bdf8]/20 border border-[#38bdf8] text-[#38bdf8] text-xs font-semibold hover:bg-[#38bdf8]/30 transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <Activity className="w-3.5 h-3.5" />
            <span>Пластина Кирсча 200×200</span>
          </button>

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

          {/* Assemblies & Mechanisms Popover Dropdown */}
          <div className="relative shrink-0" ref={assembliesMenuRef}>
            <button
              onClick={handleToggleAssemblies}
              title="Открыть список сборок и механизмов"
              className={`flex items-center gap-1.5 px-2.5 py-1 rounded border text-[11px] font-semibold transition-all cursor-pointer whitespace-nowrap ${
                isAssembliesOpen
                  ? 'bg-[#10b981]/20 border-[#10b981] text-[#10b981] shadow-[0_0_10px_rgba(16,185,129,0.25)]'
                  : 'bg-[#1d222e] hover:bg-[#262d3d] border-[#2d3545] text-[#f1f5f9]'
              }`}
            >
              <Layers className="w-3.5 h-3.5 text-[#10b981]" />
              <span>Сборки и механизмы</span>
              <ChevronDown
                className={`w-3 h-3 text-[#94a3b8] transition-transform duration-200 ${
                  isAssembliesOpen ? 'rotate-180 text-[#10b981]' : ''
                }`}
              />
            </button>

            {isAssembliesOpen && (
              <div
                style={{
                  top: assembliesMenuCoords ? `${assembliesMenuCoords.top}px` : '42px',
                  left: assembliesMenuCoords ? `${assembliesMenuCoords.left}px` : '180px',
                }}
                className="fixed z-[9999] bg-[#141824] border border-[#262f40] rounded-lg shadow-[0_12px_36px_rgba(0,0,0,0.85)] p-1.5 w-64 overflow-hidden select-none"
              >
                <div className="text-[10px] font-mono uppercase tracking-wider text-[#64748b] px-2 py-1 mb-1 border-b border-[#232a3a]">
                  Выпадающий список сборок:
                </div>
                <div className="space-y-1">
                  {ASSEMBLY_PRESETS.map((item) => {
                    const isSelected = activePreset === item.id;
                    return (
                      <button
                        key={item.id}
                        onClick={() => {
                          onSelectPreset(item.id);
                          setIsAssembliesOpen(false);
                        }}
                        className={`w-full flex items-start gap-2.5 px-2.5 py-2 rounded-md transition-all cursor-pointer text-left ${
                          isSelected
                            ? 'bg-[#1e2a3a] border border-[#10b981]/50 text-white'
                            : 'hover:bg-[#1f2736] text-[#cbd5e1] border border-transparent'
                        }`}
                      >
                        <div className="mt-0.5 shrink-0">{item.icon}</div>
                        <div className="flex-1 min-w-0">
                          <div className="text-xs font-medium truncate flex items-center justify-between">
                            <span>{item.label}</span>
                            {isSelected && (
                              <span className="w-1.5 h-1.5 rounded-full bg-[#10b981]" />
                            )}
                          </div>
                          <div className="text-[10px] text-[#8492a6] truncate mt-0.5">
                            {item.description}
                          </div>
                        </div>
                      </button>
                    );
                  })}
                </div>
              </div>
            )}
          </div>

          {/* Quick Select Dropdown for direct access */}
          <div className="flex items-center gap-1.5 bg-[#10131a] px-2 py-0.5 rounded border border-[#252b38] shrink-0">
            <select
              aria-label="Сборки и механизмы (выпадающий список)"
              value={activePreset || ''}
              onChange={(e) => onSelectPreset(e.target.value)}
              className="bg-transparent text-[#10b981] font-semibold text-[11px] font-mono py-0.5 focus:outline-none cursor-pointer max-w-[170px] truncate"
            >
              {ASSEMBLY_PRESETS.map((p) => (
                <option key={p.id} value={p.id} className="bg-[#151922] text-[#f1f5f9]">
                  {p.label}
                </option>
              ))}
            </select>
          </div>
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
