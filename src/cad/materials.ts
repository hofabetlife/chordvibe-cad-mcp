import { MaterialFailureMode, MaterialSpec } from '../types/cad';

export interface MaterialValidationResult {
  valid: boolean;
  errors: string[];
}

export interface NonlinearMaterialDefaults {
  ultimateStrengthPa: number;
  hardeningModulusPa: number;
  fractureStrain: number;
  fractureEnergyJM2: number;
  frictionCoeff: number;
  failureMode: MaterialFailureMode;
}

/**
 * Computes physically consistent default Phase 5 nonlinear plasticity, fracture,
 * and friction parameters from linear-elastic properties (used for legacy projects
 * and custom materials when advanced fields are omitted).
 */
export function deriveNonlinearMaterialDefaults(
  youngsModulusPa: number,
  yieldStrengthPa: number,
  failureMode: MaterialFailureMode = 'ductile'
): NonlinearMaterialDefaults {
  const safeYield = Number.isFinite(yieldStrengthPa) && yieldStrengthPa > 0 ? yieldStrengthPa : 250e6;
  const safeE = Number.isFinite(youngsModulusPa) && youngsModulusPa > 0 ? youngsModulusPa : 200e9;

  if (failureMode === 'brittle') {
    return {
      ultimateStrengthPa: safeYield,
      hardeningModulusPa: 0,
      fractureStrain: 0.002,
      fractureEnergyJM2: 50,
      frictionCoeff: 0.2,
      failureMode: 'brittle',
    };
  }

  return {
    ultimateStrengthPa: safeYield * 1.25,
    hardeningModulusPa: safeE * 0.01,
    fractureStrain: 0.15,
    fractureEnergyJM2: 15_000,
    frictionCoeff: 0.25,
    failureMode: 'ductile',
  };
}

/**
 * Standard engineering material library with linear-elastic, J2 elastoplastic hardening,
 * continuum fracture energy (G_f), critical strain (ε_f), and dry friction constants (ADR-005).
 */
export const MATERIALS_LIBRARY: MaterialSpec[] = [
  {
    id: 'steel_1045',
    name: 'AISI 1045 Carbon Steel',
    category: 'Metal',
    density: 7850,
    youngsModulus: 205e9,
    poissonsRatio: 0.29,
    yieldStrength: 350e6,
    ultimateStrengthPa: 585e6,
    hardeningModulusPa: 1.8e9,
    fractureStrain: 0.16,
    fractureEnergyJM2: 28_000,
    frictionCoeff: 0.25,
    failureMode: 'ductile',
    colorHex: '#8b9bb4',
    roughness: 0.32,
    metalness: 0.85,
  },
  {
    id: 'steel_structural_aisi1020',
    name: 'AISI 1020 Structural Steel',
    category: 'Metal',
    density: 7850,
    youngsModulus: 205e9,
    poissonsRatio: 0.29,
    yieldStrength: 250e6,
    ultimateStrengthPa: 420e6,
    hardeningModulusPa: 1.2e9,
    fractureStrain: 0.22,
    fractureEnergyJM2: 28_000,
    frictionCoeff: 0.25,
    failureMode: 'ductile',
    colorHex: '#7c8ea8',
    roughness: 0.36,
    metalness: 0.82,
  },
  {
    id: 'steel_4140',
    name: 'AISI 4140 Alloy Steel',
    category: 'Metal',
    density: 7850,
    youngsModulus: 210e9,
    poissonsRatio: 0.3,
    yieldStrength: 655e6,
    ultimateStrengthPa: 1020e6,
    hardeningModulusPa: 2.6e9,
    fractureStrain: 0.14,
    fractureEnergyJM2: 35_000,
    frictionCoeff: 0.22,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 310e6,
    hardeningModulusPa: 0.45e9,
    fractureStrain: 0.12,
    fractureEnergyJM2: 11_000,
    frictionCoeff: 0.3,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 950e6,
    hardeningModulusPa: 0.9e9,
    fractureStrain: 0.1,
    fractureEnergyJM2: 44_000,
    frictionCoeff: 0.32,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 400e6,
    hardeningModulusPa: 0.85e9,
    fractureStrain: 0.15,
    fractureEnergyJM2: 14_000,
    frictionCoeff: 0.2,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 65e6,
    hardeningModulusPa: 0.18e9,
    fractureStrain: 0.04,
    fractureEnergyJM2: 3_000,
    frictionCoeff: 0.35,
    failureMode: 'brittle',
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
    ultimateStrengthPa: 55e6,
    hardeningModulusPa: 0.12e9,
    fractureStrain: 0.18,
    fractureEnergyJM2: 7_200,
    frictionCoeff: 0.32,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 55e6,
    hardeningModulusPa: 0.15e9,
    fractureStrain: 0.25,
    fractureEnergyJM2: 9_500,
    frictionCoeff: 0.2,
    failureMode: 'ductile',
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
    ultimateStrengthPa: 120e6,
    hardeningModulusPa: 0,
    fractureStrain: 0.002,
    fractureEnergyJM2: 8.5,
    frictionCoeff: 0.18,
    failureMode: 'brittle',
    colorHex: '#dbeafe',
    roughness: 0.12,
    metalness: 0.15,
  },
];

export const DEFAULT_MATERIALS = MATERIALS_LIBRARY;

const MATERIAL_ALIAS_MAP: Record<string, string> = {
  steel_structural_aisi1020: 'steel_structural_aisi1020',
  steel_1020: 'steel_structural_aisi1020',
  aisi_1020: 'steel_structural_aisi1020',
  aisi1020: 'steel_structural_aisi1020',
  steel: 'steel_structural_aisi1020',
  structural_steel: 'steel_structural_aisi1020',
  steel_structural: 'steel_structural_aisi1020',

  steel_carbon_aisi1045: 'steel_1045',
  steel_1045: 'steel_1045',
  aisi_1045: 'steel_1045',
  aisi1045: 'steel_1045',
  '1045': 'steel_1045',
  carbon_steel_1045: 'steel_1045',
  steel_carbon_1045: 'steel_1045',

  steel_high_strength_4140: 'steel_4140',
  steel_4140: 'steel_4140',
  aisi_4140: 'steel_4140',
  aisi4140: 'steel_4140',
  '4140': 'steel_4140',
  alloy_steel_4140: 'steel_4140',

  aluminum_6061_t6: 'alu_6061',
  alu_6061: 'alu_6061',
  alu_6061_t6: 'alu_6061',
  al6061_t6: 'alu_6061',
  al_6061_t6: 'alu_6061',
  al6061: 'alu_6061',
  al_6061: 'alu_6061',
  aluminum_6061: 'alu_6061',
  aluminum: 'alu_6061',
  aluminium_6061_t6: 'alu_6061',
  aluminium_6061: 'alu_6061',
  aluminium: 'alu_6061',

  titanium_grade5_ti6al4v: 'ti_6al_4v',
  ti_6al_4v: 'ti_6al_4v',
  ti6al4v: 'ti_6al_4v',
  titanium_ti6al4v: 'ti_6al_4v',
  titanium_grade5: 'ti_6al_4v',
  titanium_grade_5: 'ti_6al_4v',
  ti_grade5: 'ti_6al_4v',
  titanium: 'ti_6al_4v',

  brass_c360: 'brass_c360',
  brass_c36000: 'brass_c360',
  c360: 'brass_c360',
  c36000: 'brass_c360',
  brass: 'brass_c360',

  pla_filament_3dprint: 'pla_3d',
  pla_3d: 'pla_3d',
  pla_plastic: 'pla_3d',
  pla_filament: 'pla_3d',
  pla: 'pla_3d',

  petg_filament_3dprint: 'petg_3d',
  petg_3d: 'petg_3d',
  petg_plastic: 'petg_3d',
  petg_filament: 'petg_3d',
  petg: 'petg_3d',

  nylon_pa12: 'nylon_pa12',
  nylon_pa6: 'nylon_pa12',
  nylon: 'nylon_pa12',
  pa12: 'nylon_pa12',
  pa6: 'nylon_pa12',
  polyamide_12: 'nylon_pa12',

  borosilicate_glass: 'borosilicate_glass',
  borosilicate: 'borosilicate_glass',
  glass_borosilicate: 'borosilicate_glass',
  pyrex: 'borosilicate_glass',
  glass: 'borosilicate_glass',
};

/**
 * Validates physical constitutive invariants of a MaterialSpec:
 * - density > 0, E > 0, 0 < ν < 0.5
 * - σ_u >= σ_y > 0
 * - H >= 0, ε_f > 0, G_f > 0, μ >= 0
 * - failureMode ∈ {'ductile', 'brittle'}
 */
export function validateMaterialSpec(mat: MaterialSpec): MaterialValidationResult {
  const errors: string[] = [];

  if (!mat.id || !mat.id.trim()) {
    errors.push('Material id must be non-empty');
  }
  if (!Number.isFinite(mat.density) || mat.density <= 0) {
    errors.push(`density must be finite and > 0 (got ${mat.density})`);
  }
  if (!Number.isFinite(mat.youngsModulus) || mat.youngsModulus <= 0) {
    errors.push(`youngsModulus must be finite and > 0 (got ${mat.youngsModulus})`);
  }
  if (!Number.isFinite(mat.poissonsRatio) || mat.poissonsRatio <= 0 || mat.poissonsRatio >= 0.5) {
    errors.push(`poissonsRatio must be finite and strictly in (0, 0.5) (got ${mat.poissonsRatio})`);
  }
  if (!Number.isFinite(mat.yieldStrength) || mat.yieldStrength <= 0) {
    errors.push(`yieldStrength must be finite and > 0 (got ${mat.yieldStrength})`);
  }
  if (!Number.isFinite(mat.ultimateStrengthPa) || mat.ultimateStrengthPa < mat.yieldStrength) {
    errors.push(
      `ultimateStrengthPa (${mat.ultimateStrengthPa}) must be finite and >= yieldStrength (${mat.yieldStrength})`
    );
  }
  if (!Number.isFinite(mat.hardeningModulusPa) || mat.hardeningModulusPa < 0) {
    errors.push(`hardeningModulusPa must be finite and >= 0 (got ${mat.hardeningModulusPa})`);
  }
  if (!Number.isFinite(mat.fractureStrain) || mat.fractureStrain <= 0) {
    errors.push(`fractureStrain must be finite and > 0 (got ${mat.fractureStrain})`);
  }
  if (!Number.isFinite(mat.fractureEnergyJM2) || mat.fractureEnergyJM2 <= 0) {
    errors.push(`fractureEnergyJM2 must be finite and > 0 (got ${mat.fractureEnergyJM2})`);
  }
  if (!Number.isFinite(mat.frictionCoeff) || mat.frictionCoeff < 0) {
    errors.push(`frictionCoeff must be finite and >= 0 (got ${mat.frictionCoeff})`);
  }
  if (mat.failureMode !== 'ductile' && mat.failureMode !== 'brittle') {
    errors.push(`failureMode must be 'ductile' or 'brittle' (got ${String(mat.failureMode)})`);
  }

  return { valid: errors.length === 0, errors };
}

/**
 * Throws an Error if a MaterialSpec violates any physical invariant.
 */
export function assertValidMaterialSpec(mat: MaterialSpec): void {
  const res = validateMaterialSpec(mat);
  if (!res.valid) {
    throw new Error(`Invalid material '${mat.id}': ${res.errors.join('; ')}`);
  }
}

/**
 * Normalizes a legacy or snake_case (Rust JSON) partial material object into a complete
 * Phase 5 `MaterialSpec` with backward-compatible physical defaults.
 */
export function normalizeMaterialSpec(
  raw: Partial<MaterialSpec> & Record<string, unknown>
): MaterialSpec {
  const density = Number(raw.density ?? raw.density_kg_m3 ?? 7850);
  const youngsModulus =
    raw.youngsModulus !== undefined
      ? Number(raw.youngsModulus)
      : raw.youngs_modulus_pa !== undefined
        ? Number(raw.youngs_modulus_pa)
        : raw.youngs_modulus_gpa !== undefined
          ? Number(raw.youngs_modulus_gpa) * 1e9
          : 205e9;
  const poissonsRatio = Number(raw.poissonsRatio ?? raw.poissons_ratio ?? 0.29);
  const yieldStrength =
    raw.yieldStrength !== undefined
      ? Number(raw.yieldStrength)
      : raw.yield_strength_pa !== undefined
        ? Number(raw.yield_strength_pa)
        : raw.yield_strength_mpa !== undefined
          ? Number(raw.yield_strength_mpa) * 1e6
          : 250e6;

  const rawMode = String(raw.failureMode ?? raw.failure_mode ?? 'ductile')
    .trim()
    .toLowerCase();
  const failureMode: MaterialFailureMode = rawMode === 'brittle' ? 'brittle' : 'ductile';
  const defaults = deriveNonlinearMaterialDefaults(youngsModulus, yieldStrength, failureMode);

  const ultimateStrengthPa =
    raw.ultimateStrengthPa !== undefined
      ? Number(raw.ultimateStrengthPa)
      : raw.ultimate_strength_pa !== undefined
        ? Number(raw.ultimate_strength_pa)
        : raw.ultimate_strength_mpa !== undefined
          ? Number(raw.ultimate_strength_mpa) * 1e6
          : defaults.ultimateStrengthPa;

  const hardeningModulusPa =
    raw.hardeningModulusPa !== undefined
      ? Number(raw.hardeningModulusPa)
      : raw.hardening_modulus_pa !== undefined
        ? Number(raw.hardening_modulus_pa)
        : defaults.hardeningModulusPa;

  const fractureStrain =
    raw.fractureStrain !== undefined
      ? Number(raw.fractureStrain)
      : raw.fracture_strain !== undefined
        ? Number(raw.fracture_strain)
        : defaults.fractureStrain;

  const fractureEnergyJM2 =
    raw.fractureEnergyJM2 !== undefined
      ? Number(raw.fractureEnergyJM2)
      : raw.fracture_energy_j_m2 !== undefined
        ? Number(raw.fracture_energy_j_m2)
        : defaults.fractureEnergyJM2;

  const frictionCoeff =
    raw.frictionCoeff !== undefined
      ? Number(raw.frictionCoeff)
      : raw.friction_coeff !== undefined
        ? Number(raw.friction_coeff)
        : defaults.frictionCoeff;

  return {
    id: String(raw.id ?? raw.name ?? 'custom_material'),
    name: String(raw.name ?? raw.id ?? 'Custom Material'),
    category: (raw.category as MaterialSpec['category']) || 'Custom',
    density,
    youngsModulus,
    poissonsRatio,
    yieldStrength,
    ultimateStrengthPa,
    hardeningModulusPa,
    fractureStrain,
    fractureEnergyJM2,
    frictionCoeff,
    failureMode,
    colorHex: String(raw.colorHex ?? raw.color_hex ?? '#7a8a9e'),
    roughness: raw.roughness !== undefined ? Number(raw.roughness) : 0.3,
    metalness: raw.metalness !== undefined ? Number(raw.metalness) : 0.8,
  };
}

export function resolveMaterialAlias(id: string): string {
  const normalized = String(id || '')
    .trim()
    .toLowerCase()
    .replace(/[-\s]+/g, '_');
  return MATERIAL_ALIAS_MAP[normalized] || id;
}

export function getMaterialById(
  id: string,
  customList: MaterialSpec[] = MATERIALS_LIBRARY
): MaterialSpec {
  const resolved = resolveMaterialAlias(id);
  return (
    customList.find((m) => m.id === id || m.id === resolved) ||
    MATERIALS_LIBRARY.find((m) => m.id === id || m.id === resolved) ||
    MATERIALS_LIBRARY[0]
  );
}
