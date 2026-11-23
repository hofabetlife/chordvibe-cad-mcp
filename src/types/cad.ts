export type RenderMode =
  | 'shaded'
  | 'flat_shading'
  | 'wireframe'
  | 'normals'
  | 'backfaces'
  | 'boundary_edges'
  | 'fea_heatmap';

export type ProjectionMode = 'perspective' | 'orthographic';

export type ViewportLayout = 'single' | 'quad';

export type QualityPreset = 'draft' | 'standard' | 'high' | 'fine';

export type EngineeringWorkbench =
  | 'modeling'
  | 'fea'
  | 'simulation'
  | 'optimizer';

export type DagNodeType =
  | 'box'
  | 'cylinder'
  | 'sphere'
  | 'cone'
  | 'polygon_prism'
  | 'spur_gear'
  | 'revolve'
  | 'wheel_rim'
  | 'helix'
  | 'cut_hole'
  | 'cut_helical_groove'
  | 'boolean_union'
  | 'boolean_difference'
  | 'boolean_intersection'
  | 'transform'
  | 'stl_import';

export interface DagNode {
  id: string;
  name: string;
  type: DagNodeType;
  params: Record<string, any>;
  enabled: boolean;
  children?: string[];
}

export type MaterialFailureMode = 'ductile' | 'brittle';

export interface MaterialSpec {
  id: string;
  name: string;
  category: 'Metal' | 'Polymer' | 'Ceramic' | 'Composite' | 'Custom';
  density: number; // kg/m^3
  youngsModulus: number; // Pa
  poissonsRatio: number;
  yieldStrength: number; // Pa
  /** Ultimate tensile strength σ_u in Pascals (Pa, σ_u >= σ_y > 0) */
  ultimateStrengthPa: number;
  /** Isotropic linear hardening modulus H in Pascals (Pa, H >= 0) */
  hardeningModulusPa: number;
  /** Critical equivalent plastic strain to fracture ε_f (dimensionless, ε_f > 0) */
  fractureStrain: number;
  /** Specific fracture energy G_f in J/m^2 (G_f > 0) */
  fractureEnergyJM2: number;
  /** Dry Coulomb friction coefficient μ (dimensionless, μ >= 0) */
  frictionCoeff: number;
  /** Constitutive failure mode ('ductile' | 'brittle') */
  failureMode: MaterialFailureMode;
  colorHex: string;
  roughness: number;
  metalness: number;
}

export interface DetectedPlanarFace {
  id: string;
  normal: [number, number, number];
  centroid: [number, number, number];
  area: number;
  triangleCount?: number;
}

export interface AngleHistogramBuckets {
  lt1Deg: number;
  deg1To5: number;
  deg5To15: number;
  deg15To30: number;
  deg30To45: number;
  deg45To60: number;
}

export interface MeshValidationReport {
  partId: string;
  status: 'PASS' | 'WARN' | 'FAIL';
  issues?: string[];
  warnings: string[];
  isValidSolid?: boolean;
  feaReady?: boolean;
  rawVerticesCount: number;
  weldedVerticesCount: number;
  trianglesCount: number;
  uniqueEdgesCount: number;
  boundaryEdgesCount: number;
  nonManifoldEdgesCount: number;
  inconsistentEdgesCount: number;
  degenerateTrianglesCount: number;
  zeroNormalsCount: number;
  selfIntersectionsCount: number;
  tJunctionsCount: number;
  connectedComponentsCount: number;
  eulerCharacteristic: number;
  eulerGenus: number;
  isWatertight: boolean;
  isManifold: boolean;
  isOutwardOriented: boolean;
  sliverTrianglesCount: number; // triangles with min_angle < 5 deg
  lt5DegCount: number;
  lt5DegRatio: number;
  lt1DegCount: number;
  lt1DegRatio: number;
  minAngleDeg: number;
  p5AngleDeg: number;
  medianAngleDeg: number;
  angleHistogram: AngleHistogramBuckets;
  minAspectRatio: number;
  p5AspectRatio: number;
  medianAspectRatio: number;
  maxAspectRatio: number;
  volume: number;
  surfaceArea: number;
  boundingBox: {
    min: [number, number, number];
    max: [number, number, number];
    dimensions: [number, number, number];
  };
  centerOfMass: [number, number, number];
  principalInertia: [number, number, number];
}

export interface MeshGeometryData {
  vertices: Float32Array; // x, y, z
  normals: Float32Array; // nx, ny, nz
  indices: Uint32Array;
  feaStressMPa?: Float32Array; // Per-vertex von Mises stress in MPa
  feaDisplacements?: Float32Array; // dx, dy, dz per vertex
  boundaryEdgePositions?: Float32Array; // Line segments (x1,y1,z1, x2,y2,z2) for open boundary edges
  triangleCount: number;
  vertexCount: number;
  weldedVertexCount: number;
  uniqueEdgeCount: number;
  volume: number; // m^3
  surfaceArea: number; // m^2
  mass: number; // kg
  boundingBox: {
    min: [number, number, number];
    max: [number, number, number];
  };
  centerOfMass: [number, number, number];
  inertiaTensor: [number, number, number]; // Principal moments Ixx, Iyy, Izz
  isWatertight: boolean;
  isManifold: boolean;
  isOutwardOriented: boolean;
  eulerCharacteristic: number;
  eulerGenus: number;
  boundaryEdgesCount: number;
  nonManifoldEdgesCount: number;
  inconsistentEdgesCount: number;
  degenerateTrianglesCount: number;
  zeroNormalsCount: number;
  sliverTrianglesCount?: number;
  lt1DegCount?: number;
  minAngleDeg?: number;
  p5AngleDeg?: number;
  medianAngleDeg?: number;
  maxAspectRatio?: number;
  detectedPlanarFaces: DetectedPlanarFace[];
}

export interface CadPart {
  id: string;
  name: string;
  materialId: string;
  dag: DagNode[];
  featureIdMap?: Record<string, string>;
  mesh: MeshGeometryData;
  visible: boolean;
  isFixed?: boolean;
  position: [number, number, number];
  rotation: [number, number, number]; // Euler radians
  scale: [number, number, number];
  nativeStlUrl?: string; // Optional reference to pre-compiled Windows Rust STL artifact
}

export interface KinematicJoint {
  id: string;
  name: string;
  type: 'revolute' | 'prismatic' | 'fixed' | 'spherical' | 'gear';
  partA: string;
  partB: string;
  anchor: [number, number, number];
  anchorB?: [number, number, number];
  axis: [number, number, number];
  limits?: [number, number];
  motorSpeedRpm: number;
  /** Signed linear motor speed in m/s for prismatic joints */
  motorLinearSpeedMs?: number;
  motorTorqueNm: number;
  motorEnabled: boolean;
  currentAngleRad: number;
  gearRatio?: number;
}

export interface FeaBoundaryCondition {
  region: 'min_x' | 'max_x' | 'min_y' | 'max_y' | 'min_z' | 'max_z' | 'point' | 'edge';
  type: 'fixed' | 'force';
  forceVector?: [number, number, number]; // N
  position?: [number, number, number]; // m (for concentrated point load / constraint)
}

export interface FeaTetraQualityMetrics {
  minJacobianDeterminant: number;
  minAspectRatio: number;
  medianAspectRatio: number;
  maxAspectRatio: number;
  minDihedralAngleDeg: number;
  degenerateElementCount?: number;
  qualityStatus?: 'PASS' | 'WARN' | 'REJECT';
}

export interface KirschAnalysisMetrics {
  plateWidthMm: number;
  plateHeightMm: number;
  plateThicknessMm: number;
  holeRadiusMm: number;
  holeDiameterMm: number;
  tensileForceKN: number;
  grossAreaM2: number;
  netAreaM2: number;
  grossNominalStressMPa: number;
  netNominalStressMPa: number;
  theoreticalKtInfinite: number;
  theoreticalKtGross: number;
  theoreticalKtNet: number;
  theoreticalPeakStressMPa: number;
  feaPeakStressMPa: number;
  holeEdgeMaxStressMPa?: number;
  globalMaxStressMPa?: number;
  feaKtGross: number;
  feaKtNet: number;
  howlandErrorPct: number;
}

export interface FeaSimulationResult {
  partId: string;
  nodeCount: number;
  tetrahedraCount: number;
  degreesOfFreedom?: number;
  solverIterations?: number;
  residualNorm?: number;
  meshDensity?: 'coarse' | 'medium' | 'fine';
  tetQuality?: FeaTetraQualityMetrics;
  discretizationWarning?: string;
  certificationDisclaimer?: string;
  maxDisplacementMm: number;
  maxVonMisesMPa: number;
  minVonMisesMPa: number;
  holeEdgeMaxStressMPa?: number;
  globalMaxStressMPa?: number;
  yieldStrengthMPa: number;
  safetyFactor: number;
  status: 'SAFE' | 'WARNING_HIGH_STRESS' | 'FAILURE_YIELD_EXCEEDED';
  solveTimeMs: number;
  conditions: FeaBoundaryCondition[];
  analyticalBenchmarkErrorPct?: number;
  warnings?: string[];
  singularityDetected?: boolean;
  elementOrder?: 'TET4' | 'TET10';
  kirschMetrics?: KirschAnalysisMetrics;
}

export interface PhysicsTrajectoryFrame {
  timeSec: number;
  partTransforms: Record<
    string,
    {
      position: [number, number, number];
      rotation: [number, number, number];
      angularVelocity: [number, number, number];
      linearVelocity: [number, number, number];
    }
  >;
}

export interface McpToolExecution {
  id: string;
  timestamp: string;
  toolName: string;
  params: Record<string, any>;
  response: Record<string, any>;
  durationMs: number;
  status: 'success' | 'error';
}

export type SimulationSubMode = 'kinematics' | 'destruction';

export type DestructionScenarioMode =
  | 'counter_clash'
  | 'torque_overload'
  | 'impact_crush';

export type DestructionFieldMode =
  | 'shaded'
  | 'von_mises'
  | 'plastic_strain'
  | 'damage';

export type DeformationScalePreset = 1 | 5 | 20;

export type SimulationSolverBackend = 'rust_fem_f64' | 'web_local_preview';

export interface EnergyBalanceSnapshot {
  kineticJ: number;
  elasticJ: number;
  plasticJ: number;
  fractureJ: number;
  totalJ: number;
}

export interface DestructionSimConfig {
  scenarioMode: DestructionScenarioMode;
  durationSec: number; // e.g., 2.5
  fps: number; // e.g., 60
  motorRpmLeft: number; // signed RPM (+ = CCW, - = CW)
  motorRpmRight: number; // signed RPM (+ = CCW -> head-on clash with left +RPM!)
  impactTorqueNm: number; // driving torque / impact load in N*m
  brittleness: number; // 0.2 (ductile steel) .. 1.0 (brittle cast iron / glass)
  shardCountPerPart: number; // target shards per destructible part (12..48)
  gravityY: number; // m/s^2, e.g. -9.81 or 0
  restitution: number; // bounce factor 0.1..0.75
  seed: number; // deterministic PRNG seed
}

export interface FractureShardGeometry {
  id: string;
  parentPartId: string;
  materialId: string;
  colorHex: string;
  roughness: number;
  metalness: number;
  /** Local center of mass in parent part's local space */
  localCom: [number, number, number];
  boundingRadius: number;
  mass: number;
  /** Distance from shard centroid to primary contact/impact point (in meters) */
  distToContact: number;
  /** Outer machined CAD surface triangles (centered at localCom) */
  vertices: Float32Array;
  normals: Float32Array;
  indices: Uint32Array;
  /** Newly exposed inner crystalline fracture faces (centered at localCom) */
  innerVertices: Float32Array;
  innerNormals: Float32Array;
  innerIndices: Uint32Array;
}

export interface DestructionTimelineEvent {
  frameIndex: number;
  timeSec: number;
  type:
    | 'contact'
    | 'yield_onset'
    | 'tooth_shear'
    | 'fragment_detachment'
    | 'body_fracture'
    | 'floor_bounce';
  label: string;
  partId?: string;
  stressMPa?: number;
}

export interface DestructionFrameMetrics {
  peakStressMPa: number;
  yieldLimitMPa: number;
  contactForceN: number;
  kineticEnergyJ: number;
  elasticEnergyJ: number;
  plasticEnergyJ: number;
  fractureEnergyJ: number;
  maxPlasticStrainPct: number;
  maxDamage: number;
  maxDisplacementMm: number;
  activeShardsCount: number;
  totalShardsCount: number;
  fracturedPartsCount: number;
  intactPartsCount: number;
  statusLabel: string;
}

/**
 * Active exterior triangle surface mesh of a deformable FEM body for a single simulation frame
 * (`P5-T15`), transmitted directly from Rust `query_simulation_frame` (`surface_positions_m`,
 * `surface_reference_positions_m`, `surface_indices`, `surface_node_indices`).
 * Coordinates are in world meters and already reflect eroded/detached element cavities.
 */
export interface CachedFemSurfaceMesh {
  /** Reference undeformed surface vertex positions in world meters [x, y, z, ...] */
  referenceVertices: Float32Array;
  /** Deformed surface vertex positions x(t) = X + u(t) in world meters [x, y, z, ...] */
  deformedVertices: Float32Array;
  /** Surface vertex displacement vectors u(t) = x(t) - X in meters [dx, dy, dz, ...] */
  displacements: Float32Array;
  /** Outward CCW triangle index buffer referencing compacted surface vertices */
  indices: Uint32Array;
  vertexCount: number;
  triangleCount: number;
  /** Per-surface-vertex von Mises stress σ_vM in MPa */
  nodalStressMPa?: Float32Array;
  /** Per-surface-vertex equivalent plastic strain ε_p (dimensionless) */
  nodalPlasticStrain?: Float32Array;
  /** Per-surface-vertex continuum damage D in [0, 1] */
  nodalDamage?: Float32Array;
}

export interface CachedPartFrameState {
  position: [number, number, number];
  rotation: [number, number, number];
  /** Optional exact orientation quaternion [qx, qy, qz, qw] (used by live Rust physics replays) */
  quaternion?: [number, number, number, number];
  /** 0..1+ normalized stress ratio against material yieldStrength */
  stressRatio: number;
  /** True while the whole intact parent mesh should be displayed */
  intact: boolean;
  /** True when only contact-zone shards have broken off, rest is still spinning */
  partiallyFractured: boolean;
  /** Per-vertex local displacement vectors [dx, dy, dz, ...] in meters (scaled by 1x/5x/20x in viewport) */
  deformedDisplacements?: Float32Array;
  /** Per-vertex von Mises stress σ_vM in MPa */
  nodalStressMPa?: Float32Array;
  /** Per-vertex equivalent plastic strain ε_p (dimensionless, e.g. 0.02 = 2%) */
  nodalPlasticStrain?: Float32Array;
  /** Per-vertex continuum damage scalar D in [0, 1] */
  nodalDamage?: Float32Array;
  /** Active exterior FEM triangle surface mesh in world meters (P5-T15) */
  femSurfaceMesh?: CachedFemSurfaceMesh;
  /** Peak displacement magnitude on this part at this frame (mm) */
  maxDisplacementMm?: number;
  /** Peak equivalent plastic strain on this part at this frame (%) */
  maxPlasticStrainPct?: number;
  /** Peak damage scalar on this part at this frame [0..1] */
  maxDamage?: number;
}

export interface DestructionCacheFrame {
  frameIndex: number;
  timeSec: number;
  partStates: Record<string, CachedPartFrameState>;
  /** Packed [x, y, z] per shard (length = shards.length * 3) */
  shardPos: Float32Array;
  /** Packed [qx, qy, qz, qw] per shard (length = shards.length * 4) */
  shardQuat: Float32Array;
  /** 0 = attached to intact parent, 1 = detached & simulating */
  shardActive: Uint8Array;
  /** 0..1 hot fracture glow intensity per shard */
  shardStress: Float32Array;
  /** Per-shard von Mises stress in MPa for field colormap rendering */
  shardStressMPa?: Float32Array;
  /** Per-shard equivalent plastic strain (dimensionless) for ε_p colormap rendering */
  shardPlasticStrain?: Float32Array;
  /** Per-shard continuum damage D in [0, 1] for damage colormap rendering */
  shardDamage?: Float32Array;
  /** Packed spark segments: [x, y, z, vx, vy, vz, life01] per spark */
  sparks: Float32Array;
  metrics: DestructionFrameMetrics;
}

export interface DestructionCache {
  id: string;
  createdAt: string;
  solverBackend: SimulationSolverBackend;
  solverLabel: string;
  rustSimulationId?: string;
  config: DestructionSimConfig;
  fps: number;
  totalFrames: number;
  durationSec: number;
  floorY: number;
  contactPoint: [number, number, number];
  bakeTimeMs: number;
  cacheBytes: number;
  shards: FractureShardGeometry[];
  frames: DestructionCacheFrame[];
  events: DestructionTimelineEvent[];
  maxStressOverallMPa: number;
  yieldStrengthRefMPa: number;
  maxPlasticStrainOverallPct: number;
  maxDamageOverall: number;
  maxEnergyOverallJ: number;
}


