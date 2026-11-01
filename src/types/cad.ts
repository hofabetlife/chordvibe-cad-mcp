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

export interface MeshValidationReport {
  partId: string;
  rawVerticesCount: number;
  weldedVerticesCount: number;
  trianglesCount: number;
  uniqueEdgesCount: number;
  boundaryEdgesCount: number;
  nonManifoldEdgesCount: number;
  inconsistentEdgesCount: number;
  degenerateTrianglesCount: number;
  zeroNormalsCount: number;
  eulerCharacteristic: number;
  eulerGenus: number;
  isWatertight: boolean;
  isManifold: boolean;
  isOutwardOriented: boolean;
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
  detectedPlanarFaces: DetectedPlanarFace[];
}

export interface CadPart {
  id: string;
  name: string;
  materialId: string;
  dag: DagNode[];
  mesh: MeshGeometryData;
  visible: boolean;
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
  axis: [number, number, number];
  motorSpeedRpm: number;
  motorTorqueNm: number;
  motorEnabled: boolean;
  currentAngleRad: number;
  gearRatio?: number;
}

export interface FeaBoundaryCondition {
  region: 'min_x' | 'max_x' | 'min_y' | 'max_y' | 'min_z' | 'max_z';
  type: 'fixed' | 'force';
  forceVector?: [number, number, number]; // N
}

export interface FeaSimulationResult {
  partId: string;
  nodeCount: number;
  tetrahedraCount: number;
  maxDisplacementMm: number;
  maxVonMisesMPa: number;
  minVonMisesMPa: number;
  yieldStrengthMPa: number;
  safetyFactor: number;
  status: 'SAFE' | 'WARNING_HIGH_STRESS' | 'FAILURE_YIELD_EXCEEDED';
  solveTimeMs: number;
  conditions: FeaBoundaryCondition[];
  analyticalBenchmarkErrorPct?: number;
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
