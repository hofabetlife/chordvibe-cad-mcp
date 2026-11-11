export interface PartSummary {
  id: string;
  name: string;
  material_name: string;
  volume: number;
  mass: number;
  triangle_count: number;
  bounding_box_dimensions: [number, number, number];
  is_fixed?: boolean;
}

export interface DagNode {
  id: string;
  name?: string | null;
  op: Record<string, unknown>;
}

export interface FeaResult {
  part_id?: string;
  max_von_mises_mpa: number;
  yield_strength_mpa: number;
  safety_factor: number;
  max_displacement_mm: number;
  status: 'PASS' | 'FAIL' | string;
  node_count: number;
  element_count: number;
  computation_time_ms: number;
  heatmap_base64?: string;
}

export interface BodyStateSample {
  time_s: number;
  position: [number, number, number];
  orientation_quat: [number, number, number, number];
  linear_velocity: [number, number, number];
  angular_velocity: [number, number, number];
}

export interface PhysicsSimulationResult {
  success: boolean;
  steps_simulated: number;
  trajectory_summary: Record<
    string,
    {
      part_id: string;
      samples: BodyStateSample[];
    }
  >;
  peak_joint_forces: Record<string, number>;
  collisions: Array<{
    time_s: number;
    body_a: string;
    body_b: string;
    event_type: string;
  }>;
}

export interface McpToolDefinition {
  name: string;
  description: string;
  inputSchema: {
    type: string;
    properties?: Record<string, unknown>;
    required?: string[];
  };
}

export interface ParsedStlGeometry {
  positions: Float32Array;
  normals: Float32Array;
  triangleCount: number;
  boundsMin: [number, number, number];
  boundsMax: [number, number, number];
  center: [number, number, number];
  maxDimension: number;
}

export interface RpcLogEntry {
  id: string;
  timestamp: string;
  toolName: string;
  args: Record<string, unknown>;
  durationMs: number;
  status: 'ok' | 'error';
  responseText: string;
  imageBase64?: string;
}
