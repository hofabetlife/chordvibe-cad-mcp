import React from 'react';
import {
  Cpu,
  FolderOpen,
  HelpCircle,
  Box,
  Activity,
  Play,
  Monitor,
  PanelLeft,
  PanelRight,
} from 'lucide-react';
import { EngineeringWorkbench, QualityPreset } from '../types/cad';
import { ENGINEERING_WORKBENCHES } from '../cad/workbenches';

interface CadTitleBarProps {
  activeWorkbench: EngineeringWorkbench;
  onSelectWorkbench: (wb: EngineeringWorkbench) => void;
  activePreset: string;
  onSelectPreset: (presetId: string) => void;
  quality: QualityPreset;
  onChangeQuality: (q: QualityPreset) => void;
  onCaptureViewport: () => void;
  onExportStl: () => void;
  onImportStlFile: (file: File) => void;
  onLoadNativeStlUrl: (stlUrl: string, label: string) => void;
  isWatertight: boolean;
  eulerGenus: number;
  showLeftPanel: boolean;
  onToggleLeftPanel: () => void;
  showRightPanel: boolean;
  onToggleRightPanel: () => void;
  onSwitchToPreviewWindow: () => void;
  onOpenHelp?: () => void;
}

const PRESET_SCENES = [
  { id: 'spur_gear', label: '1. Шестерня Z=20, m=2.0 (Шпонка DIN 6885)' },
  { id: 'hex_nut_m10', label: '2. Гайка М10 (DIN 934 / Резьба ISO 68-1)' },
  { id: 'twist_drill', label: '3. Спиральное сверло Ø10мм (2 канавки)' },
  { id: 'wheel_rim', label: '4. Литой автодиск 18" (5 спиц)' },
  { id: 'lightbulb_e27', label: '5. Лампа E27 A60 (Вращение + Цоколь)' },
  { id: 'gear_transmission', label: '6. Сборка: Редуктор Z1=20 / Z2=40' },
  { id: 'bracket_fea', label: '7. Прочность: Консольная балка (МКЭ)' },
  { id: 'crank_mechanism', label: '8. Кинематика: Кривошипно-шатунный механизм' },
  { id: 'ai_unopt_bushing', label: '9. ИИ-цепочка: Втулка (5 оп. → Revolve)' },
  { id: 'ai_unopt_flange_plate', label: '10. ИИ-цепочка: Плита (8 оп. → De Morgan)' },
  { id: 'ai_unopt_hex_nut', label: '11. ИИ-цепочка: Гайка М10 (Kernel Pushdown)' },
  { id: 'qa_hex_chamfer', label: '12. QA Эталон: Шестигранная призма с фаской' },
  { id: 'qa_revolve_180', label: '13. QA Эталон: Тело вращения 180° (BUG-003)' },
  { id: 'qa_csg_two_holes', label: '14. QA Эталон: Блок с 2 отверстиями (g=2)' },
];

export const CadTitleBar: React.FC<CadTitleBarProps> = ({
  activeWorkbench,
  onSelectWorkbench,
  activePreset,
  onSelectPreset,
  showLeftPanel,
  onToggleLeftPanel,
  showRightPanel,
  onToggleRightPanel,
  onSwitchToPreviewWindow,
  onOpenHelp,
}) => {
  const getWorkbenchIcon = (id: EngineeringWorkbench) => {
    switch (id) {
      case 'modeling':
        return <Box className="w-3.5 h-3.5 shrink-0" />;
      case 'fea':
        return <Activity className="w-3.5 h-3.5 shrink-0" />;
      case 'simulation':
        return <Play className="w-3.5 h-3.5 shrink-0" />;
      case 'optimizer':
        return <Cpu className="w-3.5 h-3.5 shrink-0" />;
    }
  };

  return (
    <header className="h-11 bg-[#11141c] border-b border-[#252b38] px-3 flex items-center justify-between gap-4 select-none shrink-0">
      {/* Zone 1: Brand & Scene Model Selector */}
      <div className="flex items-center gap-2.5 min-w-0 shrink-0">
        <div className="flex items-center gap-2 shrink-0">
          <div className="w-6 h-6 rounded bg-[#38bdf8]/15 border border-[#38bdf8]/40 flex items-center justify-center text-[#38bdf8]">
            <Box className="w-3.5 h-3.5" />
          </div>
          <span className="font-semibold text-xs tracking-wide text-[#f1f5f9] whitespace-nowrap">
            MCP-CAD Инженерная студия
          </span>
        </div>

        <div className="h-4 w-[1px] bg-[#252b38] shrink-0" />

        <div className="flex items-center gap-1.5">
          <FolderOpen className="w-3.5 h-3.5 text-[#38bdf8] shrink-0" />
          <select
            aria-label="Выбор инженерной сцены"
            value={activePreset}
            onChange={(e) => onSelectPreset(e.target.value)}
            className="bg-[#181c26] text-[#f1f5f9] text-xs font-medium px-2 py-1 rounded border border-[#2d3545] hover:border-[#38bdf8]/60 focus:outline-none focus:border-[#38bdf8] transition-colors cursor-pointer max-w-[230px]"
          >
            {PRESET_SCENES.map((s) => (
              <option
                key={s.id}
                value={s.id}
                className="bg-[#181c26] text-[#f1f5f9]"
              >
                {s.label}
              </option>
            ))}
          </select>
        </div>
      </div>

      {/* Zone 2: Logical Engineering Workbenches (Разделы функционала) */}
      <nav
        aria-label="Инженерные разделы"
        className="flex items-center gap-1 bg-[#0b0d13] p-1 rounded-lg border border-[#232936] shrink-0"
      >
        {ENGINEERING_WORKBENCHES.map((wb) => {
          const isActive = activeWorkbench === wb.id;
          const activeStyles =
            wb.id === 'modeling'
              ? 'bg-[#38bdf8] text-[#0c111d] font-semibold shadow-sm'
              : wb.id === 'fea'
                ? 'bg-[#ef4444] text-white font-semibold shadow-sm'
                : wb.id === 'simulation'
                  ? 'bg-[#10b981] text-[#0c111d] font-semibold shadow-sm'
                  : 'bg-[#f59e0b] text-[#0c111d] font-semibold shadow-sm';

          return (
            <button
              key={wb.id}
              onClick={() => onSelectWorkbench(wb.id)}
              title={wb.subtitle}
              className={`flex items-center gap-1.5 px-3 py-1 rounded-md text-xs transition-colors cursor-pointer whitespace-nowrap shrink-0 ${
                isActive
                  ? activeStyles
                  : 'text-[#94a3b8] hover:text-[#f1f5f9] hover:bg-[#161b26]'
              }`}
            >
              {getWorkbenchIcon(wb.id)}
              <span>{wb.label}</span>
            </button>
          );
        })}
      </nav>

      {/* Zone 3: Workspace Panel Toggles & Return to Simple WGPU Preview */}
      <div className="flex items-center gap-1.5 shrink-0">
        <div className="hidden xl:flex items-center gap-1 bg-[#161a24] p-0.5 rounded border border-[#262c3a]">
          <button
            onClick={onToggleLeftPanel}
            title={
              showLeftPanel
                ? 'Скрыть левую панель'
                : 'Показать левую панель'
            }
            aria-label="Переключить левую панель"
            className={`p-1 rounded text-xs transition-colors cursor-pointer ${
              showLeftPanel
                ? 'bg-[#252d3d] text-[#38bdf8]'
                : 'text-[#64748b] hover:text-[#cbd5e1]'
            }`}
          >
            <PanelLeft className="w-3.5 h-3.5" />
          </button>
          <button
            onClick={onToggleRightPanel}
            title={
              showRightPanel
                ? 'Скрыть правую панель инспектора'
                : 'Показать правую панель инспектора'
            }
            aria-label="Переключить правую панель"
            className={`p-1 rounded text-xs transition-colors cursor-pointer ${
              showRightPanel
                ? 'bg-[#252d3d] text-[#38bdf8]'
                : 'text-[#64748b] hover:text-[#cbd5e1]'
            }`}
          >
            <PanelRight className="w-3.5 h-3.5" />
          </button>
        </div>

        {onOpenHelp && (
          <button
            onClick={onOpenHelp}
            title="Справка по управлению и горячим клавишам (F1)"
            className="flex items-center gap-1 px-2.5 py-1 rounded text-xs font-medium bg-[#181c26] hover:bg-[#222836] text-[#cbd5e1] border border-[#2d3545] transition-colors cursor-pointer whitespace-nowrap shrink-0"
          >
            <HelpCircle className="w-3.5 h-3.5 text-[#38bdf8]" />
            <span className="hidden md:inline">Справка</span>
          </button>
        )}

        <button
          onClick={onSwitchToPreviewWindow}
          title="Вернуться в минималистичное окно 3D предпросмотра (Клавиша Tab)"
          className="flex items-center gap-1.5 px-3 py-1 rounded text-xs font-semibold bg-[#1e293b] hover:bg-[#334155] text-[#38bdf8] border border-[#38bdf8]/40 transition-colors cursor-pointer whitespace-nowrap shrink-0"
        >
          <Monitor className="w-3.5 h-3.5" />
          <span>3D Просмотр</span>
        </button>
      </div>
    </header>
  );
};
