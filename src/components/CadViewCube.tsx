import React from 'react';
import { Maximize2, RotateCcw } from 'lucide-react';

interface CadViewCubeProps {
  activePreset: 'iso' | 'front' | 'top' | 'right';
  onSelectPreset: (preset: 'iso' | 'front' | 'top' | 'right') => void;
  onFitView: () => void;
}

export const CadViewCube: React.FC<CadViewCubeProps> = ({
  activePreset,
  onSelectPreset,
  onFitView,
}) => {
  return (
    <div className="absolute top-14 right-4 z-20 flex flex-col items-end gap-1.5 pointer-events-auto select-none">
      {/* 3D View Cube Orientation Widget */}
      <div className="w-20 h-20 bg-[#131b2e]/90 backdrop-blur border border-slate-700/80 rounded-xl shadow-2xl p-1 flex flex-col items-center justify-between relative group">
        {/* Top Face Button */}
        <button
          type="button"
          onClick={() => onSelectPreset('top')}
          className={`w-full text-[10px] font-bold font-mono py-0.5 rounded transition-colors ${
            activePreset === 'top'
              ? 'bg-amber-500 text-slate-950 shadow-sm'
              : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700 hover:text-white'
          }`}
        >
          TOP / СВЕРХУ
        </button>

        {/* Middle Row: Front & Right */}
        <div className="flex items-center justify-between w-full gap-1">
          <button
            type="button"
            onClick={() => onSelectPreset('front')}
            className={`flex-1 text-[9px] font-bold font-mono py-1 rounded transition-colors ${
              activePreset === 'front'
                ? 'bg-amber-500 text-slate-950 shadow-sm'
                : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700 hover:text-white'
            }`}
          >
            FRONT
          </button>
          <button
            type="button"
            onClick={() => onSelectPreset('right')}
            className={`flex-1 text-[9px] font-bold font-mono py-1 rounded transition-colors ${
              activePreset === 'right'
                ? 'bg-amber-500 text-slate-950 shadow-sm'
                : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700 hover:text-white'
            }`}
          >
            RIGHT
          </button>
        </div>

        {/* Bottom Row: Isometric Preset */}
        <button
          type="button"
          onClick={() => onSelectPreset('iso')}
          className={`w-full text-[10px] font-bold font-mono py-0.5 rounded transition-colors ${
            activePreset === 'iso'
              ? 'bg-amber-500 text-slate-950 shadow-sm'
              : 'bg-slate-800/80 text-slate-300 hover:bg-slate-700 hover:text-white'
          }`}
        >
          ISO / ИЗО
        </button>
      </div>

      {/* Quick Action Button Bar */}
      <div className="flex items-center gap-1 bg-[#131b2e]/90 backdrop-blur border border-slate-700/80 p-1 rounded-lg shadow-xl">
        <button
          type="button"
          onClick={onFitView}
          title="Вписать всю сцену в экран (Fit Scene)"
          className="p-1 text-slate-300 hover:text-amber-400 hover:bg-slate-800 rounded transition-colors"
        >
          <Maximize2 className="w-3.5 h-3.5" />
        </button>
        <button
          type="button"
          onClick={() => onSelectPreset('iso')}
          title="Сбросить на Изометрию"
          className="p-1 text-slate-300 hover:text-amber-400 hover:bg-slate-800 rounded transition-colors"
        >
          <RotateCcw className="w-3.5 h-3.5" />
        </button>
      </div>
    </div>
  );
};
