import { EngineeringWorkbench, FeaBoundaryCondition } from '../types/cad';

export interface WorkbenchMetadata {
  id: EngineeringWorkbench;
  label: string;
  shortLabel: string;
  subtitle: string;
  accentColor: string;
  leftPanelTitle: string;
  rightPanelTitle: string;
}

export const ENGINEERING_WORKBENCHES: WorkbenchMetadata[] = [
  {
    id: 'modeling',
    label: 'Моделирование',
    shortLabel: '1. Моделирование',
    subtitle: 'Параметрические тела, булевы операции CSG, дерево DAG и топология TriangleMesh',
    accentColor: '#38bdf8',
    leftPanelTitle: 'Дерево построения и Материалы',
    rightPanelTitle: 'Геометрия и Топология (Mesh)',
  },
  {
    id: 'fea',
    label: 'Нагрузки и МКЭ',
    shortLabel: '2. Нагрузки (МКЭ)',
    subtitle: 'Расчет напряжений по Мизесу (TET4), граничные условия, опоры и запас прочности',
    accentColor: '#ef4444',
    leftPanelTitle: 'Упругие свойства и Материал',
    rightPanelTitle: 'Нагрузки и Результаты МКЭ',
  },
  {
    id: 'simulation',
    label: 'Симуляция',
    shortLabel: '3. Симуляция',
    subtitle: 'Кинематика механизмов Rapier3D, шарниры, приводы, зубчатые зацепления',
    accentColor: '#10b981',
    leftPanelTitle: 'Шарниры и Приводы (Joints)',
    rightPanelTitle: 'Телеметрия движения и Сборка',
  },
  {
    id: 'optimizer',
    label: 'Оптимизатор и MCP',
    shortLabel: '4. Оптимизатор / MCP',
    subtitle: 'Cost-Based оптимизация дерева DAG (SQL CBO), 28 MCP-инструментов и QA эталоны',
    accentColor: '#f59e0b',
    leftPanelTitle: 'Операции дерева DAG',
    rightPanelTitle: 'Оптимизатор CBO, MCP и QA',
  },
];

export interface FeaQuickLoadPreset {
  id: string;
  label: string;
  description: string;
  fixedRegion: FeaBoundaryCondition['region'];
  loadRegion: FeaBoundaryCondition['region'];
  forceVector: [number, number, number];
}

export const FEA_QUICK_LOAD_PRESETS: FeaQuickLoadPreset[] = [
  {
    id: 'kirsch_tension',
    label: 'Пластина Кирсча 100 кН',
    description: 'Заделка min_x, продольное растяжение max_x Fx = +100 кН (Пластина 200x200x10 мм с отверстием R=20мм)',
    fixedRegion: 'min_x',
    loadRegion: 'max_x',
    forceVector: [100000, 0, 0],
  },
  {
    id: 'cantilever_bending',
    label: 'Изгиб 1.2 кН',
    description: 'Жесткая заделка слева (min_x), поперечная сила Fy = -1200 Н на правом торце',
    fixedRegion: 'min_x',
    loadRegion: 'max_x',
    forceVector: [0, -1200, 0],
  },
  {
    id: 'axial_tension',
    label: 'Растяжение 2.5 кН',
    description: 'Заделка слева (min_x), продольная растягивающая сила Fx = +2500 Н',
    fixedRegion: 'min_x',
    loadRegion: 'max_x',
    forceVector: [2500, 0, 0],
  },
  {
    id: 'transverse_shear',
    label: 'Сдвиг 0.8 кН',
    description: 'Заделка основания (min_y), боковая нагрузка Fz = -800 Н по оси Z',
    fixedRegion: 'min_y',
    loadRegion: 'max_z',
    forceVector: [0, 0, -800],
  },
  {
    id: 'heavy_press',
    label: 'Пресс 4.5 кН',
    description: 'Заделка нижней грани (min_y), вертикальное сжатие Fy = -4500 Н сверху',
    fixedRegion: 'min_y',
    loadRegion: 'max_y',
    forceVector: [0, -4500, 0],
  },
];

export function getWorkbenchMetadata(
  workbench: EngineeringWorkbench
): WorkbenchMetadata {
  return (
    ENGINEERING_WORKBENCHES.find((w) => w.id === workbench) ||
    ENGINEERING_WORKBENCHES[0]
  );
}

export function buildBoundaryConditionsFromPreset(
  presetId: string
): FeaBoundaryCondition[] {
  const preset =
    FEA_QUICK_LOAD_PRESETS.find((p) => p.id === presetId) ||
    FEA_QUICK_LOAD_PRESETS[0];
  return [
    { region: preset.fixedRegion, type: 'fixed' },
    {
      region: preset.loadRegion,
      type: 'force',
      forceVector: [...preset.forceVector] as [number, number, number],
    },
  ];
}
