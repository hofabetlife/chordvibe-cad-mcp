import { MaterialSpec } from '../types/cad';

export const MATERIALS_LIBRARY: MaterialSpec[] = [
  {
    id: 'steel_1045',
    name: 'AISI 1045 Carbon Steel',
    category: 'Metal',
    density: 7850,
    youngsModulus: 205e9,
    poissonsRatio: 0.29,
    yieldStrength: 350e6,
    colorHex: '#8b9bb4',
    roughness: 0.32,
    metalness: 0.85,
  },
  {
    id: 'steel_4140',
    name: 'AISI 4140 Alloy Steel',
    category: 'Metal',
    density: 7850,
    youngsModulus: 210e9,
    poissonsRatio: 0.3,
    yieldStrength: 655e6,
    colorHex: '#6c7a89',
    roughness: 0.28,
    metalness: 0.9,
  },
  {
    id: 'alu_6061',
    name: 'Aluminum 6061-T6 (Aerospace)',
    category: 'Metal',
    density: 2700,
    youngsModulus: 68.9e9,
    poissonsRatio: 0.33,
    yieldStrength: 276e6,
    colorHex: '#c4cdd6',
    roughness: 0.24,
    metalness: 0.82,
  },
  {
    id: 'ti_6al_4v',
    name: 'Titanium Grade 5 (Ti-6Al-4V)',
    category: 'Metal',
    density: 4430,
    youngsModulus: 114e9,
    poissonsRatio: 0.34,
    yieldStrength: 880e6,
    colorHex: '#9aa0a6',
    roughness: 0.28,
    metalness: 0.88,
  },
  {
    id: 'brass_c360',
    name: 'Free-Cutting Brass C360',
    category: 'Metal',
    density: 8500,
    youngsModulus: 100e9,
    poissonsRatio: 0.34,
    yieldStrength: 310e6,
    colorHex: '#d4af37',
    roughness: 0.28,
    metalness: 0.85,
  },
  {
    id: 'pla_3d',
    name: 'PLA Filament (3D Print)',
    category: 'Polymer',
    density: 1240,
    youngsModulus: 3.5e9,
    poissonsRatio: 0.36,
    yieldStrength: 60e6,
    colorHex: '#38bdf8',
    roughness: 0.55,
    metalness: 0.05,
  },
  {
    id: 'petg_3d',
    name: 'PETG Filament (3D Print)',
    category: 'Polymer',
    density: 1270,
    youngsModulus: 2.1e9,
    poissonsRatio: 0.38,
    yieldStrength: 50e6,
    colorHex: '#f97316',
    roughness: 0.5,
    metalness: 0.1,
  },
  {
    id: 'nylon_pa12',
    name: 'Engineering Nylon PA12',
    category: 'Polymer',
    density: 1010,
    youngsModulus: 1.8e9,
    poissonsRatio: 0.4,
    yieldStrength: 45e6,
    colorHex: '#e2e8f0',
    roughness: 0.6,
    metalness: 0.05,
  },
  {
    id: 'borosilicate_glass',
    name: 'Borosilicate Glass E27',
    category: 'Ceramic',
    density: 2500,
    youngsModulus: 64e9,
    poissonsRatio: 0.2,
    yieldStrength: 120e6,
    colorHex: '#dbeafe',
    roughness: 0.12,
    metalness: 0.15,
  },
];

export const DEFAULT_MATERIALS = MATERIALS_LIBRARY;

export function getMaterialById(
  id: string,
  customList: MaterialSpec[] = MATERIALS_LIBRARY
): MaterialSpec {
  return (
    customList.find((m) => m.id === id) ||
    MATERIALS_LIBRARY.find((m) => m.id === id) ||
    MATERIALS_LIBRARY[0]
  );
}
