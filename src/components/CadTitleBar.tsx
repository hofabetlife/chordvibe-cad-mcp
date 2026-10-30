import React, { useState } from 'react';
import {
  Box,
  CheckCircle2,
  ChevronDown,
  Cpu,
  Download,
  FileCode,
  FolderOpen,
  HelpCircle,
  Layers,
  Maximize2,
  Minus,
  Moon,
  RefreshCw,
  Save,
  Settings,
  Shield,
  Sun,
  Terminal,
  Wrench,
  X,
} from 'lucide-react';

interface CadTitleBarProps {
  activePartName: string | null;
  mcpToolCount: number;
  statusMessage: string;
  theme: 'dark' | 'light' | 'high_contrast';
  onThemeChange: (theme: 'dark' | 'light' | 'high_contrast') => void;
  onOpenMcpOps: () => void;
  onCaptureWgpu: () => void;
  onResetScene: () => void;
  isBusy: boolean;
}

export const CadTitleBar: React.FC<CadTitleBarProps> = ({
  activePartName,
  mcpToolCount,
  statusMessage,
  theme,
  onThemeChange,
  onOpenMcpOps,
  onCaptureWgpu,
  onResetScene,
  isBusy,
}) => {
  const [activeMenu, setActiveMenu] = useState<string | null>(null);

  const toggleMenu = (menuName: string) => {
    setActiveMenu((prev) => (prev === menuName ? null : menuName));
  };

  const closeMenus = () => setActiveMenu(null);

  return (
    <header className="h-11 bg-[#0d121f] text-slate-200 border-b border-slate-800 flex items-center justify-between px-2 select-none z-40 relative">
      {/* Left: Application Icon, Brand, and Windows CAD Menu Bar */}
      <div className="flex items-center gap-2">
        {/* Windows CAD App Icon */}
        <div className="w-6 h-6 rounded bg-gradient-to-br from-amber-500 to-amber-700 flex items-center justify-center text-slate-950 shadow-md font-bold text-xs">
          <Box className="w-3.5 h-3.5" />
        </div>

        {/* CAD Brand & Version */}
        <div className="flex items-center gap-1.5 mr-2">
          <span className="font-semibold text-xs tracking-tight text-white font-sans">
            MCP-CAD Engine
          </span>
          <span className="text-[10px] font-mono px-1.5 py-0.5 rounded bg-amber-500/20 text-amber-300 border border-amber-500/30">
            Win32/Web Sync v1.2
          </span>
        </div>

        {/* Windows CAD Top Menu Bar */}
        <div className="hidden md:flex items-center gap-0.5 text-xs text-slate-300 relative">
          {/* File Menu */}
          <div className="relative">
            <button
              type="button"
              onClick={() => toggleMenu('file')}
              className={`px-2.5 py-1 rounded text-[11px] font-medium transition-colors hover:bg-slate-800 hover:text-white ${
                activeMenu === 'file' ? 'bg-slate-800 text-amber-400 font-semibold' : ''
              }`}
            >
              Файл
            </button>
            {activeMenu === 'file' && (
              <div
                className="absolute top-full left-0 mt-1 w-56 bg-[#131b2e] border border-slate-700 rounded-md shadow-2xl py-1 z-50 text-xs"
                onMouseLeave={closeMenus}
              >
                <button
                  type="button"
                  onClick={() => {
                    onResetScene();
                    closeMenus();
                  }}
                  className="w-full text-left px-3 py-1.5 hover:bg-slate-800 flex items-center gap-2 text-slate-200"
                >
                  <RefreshCw className="w-3.5 h-3.5 text-amber-400" />
                  <span>Новая сцена (Очистить)</span>
                </button>
                <div className="my-1 border-t border-slate-800" />
                <button
                  type="button"
                  onClick={() => {
                    onOpenMcpOps();
                    closeMenus();
                  }}
                  className="w-full text-left px-3 py-1.5 hover:bg-slate-800 flex items-center gap-2 text-slate-200"
                >
                  <FileCode className="w-3.5 h-3.5 text-sky-400" />
                  <span>Менеджер проектов & STL</span>
                </button>
              </div>
            )}
          </div>

          {/* Edit Menu */}
          <div className="relative">
            <button
              type="button"
              onClick={() => toggleMenu('edit')}
              className={`px-2.5 py-1 rounded text-[11px] font-medium transition-colors hover:bg-slate-800 hover:text-white ${
                activeMenu === 'edit' ? 'bg-slate-800 text-amber-400 font-semibold' : ''
              }`}
            >
              Правка
            </button>
            {activeMenu === 'edit' && (
              <div
                className="absolute top-full left-0 mt-1 w-52 bg-[#131b2e] border border-slate-700 rounded-md shadow-2xl py-1 z-50 text-xs"
                onMouseLeave={closeMenus}
              >
                <div className="px-3 py-1 text-[10px] text-slate-400 uppercase font-mono">
                  История DAG
                </div>
                <button
                  type="button"
                  onClick={() => {
                    onOpenMcpOps();
                    closeMenus();
                  }}
                  className="w-full text-left px-3 py-1.5 hover:bg-slate-800 flex items-center gap-2 text-slate-200"
                >
                  <Wrench className="w-3.5 h-3.5 text-amber-400" />
                  <span>Параметры текущей детали</span>
                </button>
              </div>
            )}
          </div>

          {/* Modeling Menu */}
          <div className="relative">
            <button
              type="button"
              onClick={() => toggleMenu('modeling')}
              className={`px-2.5 py-1 rounded text-[11px] font-medium transition-colors hover:bg-slate-800 hover:text-white ${
                activeMenu === 'modeling' ? 'bg-slate-800 text-amber-400 font-semibold' : ''
              }`}
            >
              3D Моделирование
            </button>
            {activeMenu === 'modeling' && (
              <div
                className="absolute top-full left-0 mt-1 w-60 bg-[#131b2e] border border-slate-700 rounded-md shadow-2xl py-1 z-50 text-xs"
                onMouseLeave={closeMenus}
              >
                <div className="px-3 py-1 text-[10px] text-slate-400 uppercase font-mono">
                  Примитивы & Операции
                </div>
                <div className="px-3 py-1 text-[11px] text-slate-300">
                  Шестерня ISO/DIN 867 • Гайка M10 • Диск WheelRim • Отверстие M8 • Резьба ISO 68-1
                </div>
              </div>
            )}
          </div>

          {/* Simulation & CAE Menu */}
          <div className="relative">
            <button
              type="button"
              onClick={() => toggleMenu('sim')}
              className={`px-2.5 py-1 rounded text-[11px] font-medium transition-colors hover:bg-slate-800 hover:text-white ${
                activeMenu === 'sim' ? 'bg-slate-800 text-amber-400 font-semibold' : ''
              }`}
            >
              Расчет МКЭ & Физика
            </button>
            {activeMenu === 'sim' && (
              <div
                className="absolute top-full left-0 mt-1 w-64 bg-[#131b2e] border border-slate-700 rounded-md shadow-2xl py-1 z-50 text-xs"
                onMouseLeave={closeMenus}
              >
                <div className="px-3 py-1 text-[10px] text-slate-400 uppercase font-mono">
                  CAE Инструменты
                </div>
                <div className="px-3 py-1 text-[11px] text-slate-300">
                  3D Тетраэдральный решатель TET4 ($K \cdot u = F$) • Мизес • Запас FoS • Rapier3D Kinematics
                </div>
              </div>
            )}
          </div>

          {/* Tools & MCP Menu */}
          <div className="relative">
            <button
              type="button"
              onClick={() => toggleMenu('mcp')}
              className={`px-2.5 py-1 rounded text-[11px] font-medium transition-colors hover:bg-slate-800 hover:text-white ${
                activeMenu === 'mcp' ? 'bg-slate-800 text-amber-400 font-semibold' : ''
              }`}
            >
              MCP Transport & AI
            </button>
            {activeMenu === 'mcp' && (
              <div
                className="absolute top-full left-0 mt-1 w-64 bg-[#131b2e] border border-slate-700 rounded-md shadow-2xl py-1 z-50 text-xs"
                onMouseLeave={closeMenus}
              >
                <button
                  type="button"
                  onClick={() => {
                    onOpenMcpOps();
                    closeMenus();
                  }}
                  className="w-full text-left px-3 py-1.5 hover:bg-slate-800 flex items-center gap-2 text-amber-400 font-medium"
                >
                  <Terminal className="w-3.5 h-3.5" />
                  <span>Диспетчер JSON-RPC ({mcpToolCount} MCP Tools)</span>
                </button>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Center: Active Document Name & Rust Status */}
      <div className="hidden lg:flex items-center gap-2 text-xs bg-slate-900/80 px-3 py-1 rounded-md border border-slate-800/80">
        <span className="font-semibold text-white">
          {activePartName ? `[${activePartName}]` : '[Без названия]'}
        </span>
        <span className="text-slate-500">•</span>
        <span className="text-emerald-400 font-mono text-[11px] flex items-center gap-1">
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse" />
          Rust Kernel Connected (stdio / SSE :8080)
        </span>
      </div>

      {/* Right: Status text, WGPU Snapshot trigger, Theme switcher, and Windows Controls */}
      <div className="flex items-center gap-2">
        {/* Status text */}
        <span className="text-[11px] text-slate-400 hidden xl:inline truncate max-w-[220px]">
          {statusMessage}
        </span>

        {/* WGPU Capture Quick Button */}
        <button
          type="button"
          onClick={onCaptureWgpu}
          disabled={isBusy}
          className="px-2.5 py-1 rounded bg-amber-500/15 hover:bg-amber-500/25 border border-amber-500/40 text-amber-300 text-xs font-medium flex items-center gap-1 transition-colors cursor-pointer"
        >
          <Cpu className="w-3.5 h-3.5 text-amber-400" />
          <span>WGPU Снимок</span>
        </button>

        {/* MCP Manager Launcher */}
        <button
          type="button"
          onClick={onOpenMcpOps}
          className="px-2.5 py-1 rounded bg-slate-800 hover:bg-slate-700 border border-slate-700 text-slate-200 text-xs font-medium flex items-center gap-1 transition-colors cursor-pointer"
        >
          <Terminal className="w-3.5 h-3.5 text-sky-400" />
          <span className="hidden sm:inline">MCP ({mcpToolCount})</span>
        </button>

        {/* CAD Theme Switcher */}
        <button
          type="button"
          title="Сменить тему CAD"
          onClick={() => {
            const next = theme === 'dark' ? 'light' : theme === 'light' ? 'high_contrast' : 'dark';
            onThemeChange(next);
          }}
          className="p-1 rounded text-slate-400 hover:text-white hover:bg-slate-800 transition-colors"
        >
          {theme === 'dark' ? (
            <Moon className="w-3.5 h-3.5 text-amber-400" />
          ) : theme === 'light' ? (
            <Sun className="w-3.5 h-3.5 text-amber-500" />
          ) : (
            <Shield className="w-3.5 h-3.5 text-cyan-400" />
          )}
        </button>

        {/* Windows Title Bar Window Controls Mock */}
        <div className="flex items-center gap-1 ml-1 pl-2 border-l border-slate-800 text-slate-400">
          <button
            type="button"
            title="Свернуть"
            className="w-6 h-6 rounded flex items-center justify-center hover:bg-slate-800 hover:text-white text-xs"
          >
            <Minus className="w-3 h-3" />
          </button>
          <button
            type="button"
            title="Развернуть"
            className="w-6 h-6 rounded flex items-center justify-center hover:bg-slate-800 hover:text-white text-xs"
          >
            <Maximize2 className="w-3 h-3" />
          </button>
          <button
            type="button"
            title="Закрыть"
            className="w-6 h-6 rounded flex items-center justify-center hover:bg-red-600 hover:text-white text-xs transition-colors"
          >
            <X className="w-3 h-3" />
          </button>
        </div>
      </div>
    </header>
  );
};
