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

export interface MaterialSpec {
  id: string;
  name: string;
  category: 'Metal' | 'Polymer' | 'Ceramic' | 'Composite' | 'Custom';
  density: number; // kg/m^3
  youngsModulus: number; // Pa
  poissonsRatio: number;
  yieldStrength: number; // Pa
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
  warnings: string[];
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
