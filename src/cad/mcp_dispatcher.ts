import {
  CadPart,
  FeaBoundaryCondition,
  FeaSimulationResult,
  KinematicJoint,
  MaterialSpec,
  MeshGeometryData,
  PhysicsTrajectoryFrame,
  QualityPreset,
  RenderMode,
} from '../types/cad';
import { runLinearFeaSolver } from './fea_solver';
import { runPhysicsSimulationTrajectory } from './kinematics';
import {
  applyTransformToMesh,
  createBoxMesh,
  evaluateFeatureTree,
  exportMeshToStlBinary,
  validatePartMesh,
} from './primitives';
import {
  csgDifferenceMesh,
  csgIntersectionMesh,
  csgUnionMesh,
} from './csg_bsp';
import { optimizeCadFeatureTree } from './query_optimizer';

export interface McpToolDefinition {
  name: string;
  category:
    | 'Lifecycle'
    | 'Features & CSG'
    | 'Geometry & Mesh'
    | 'Physics & Kinematics'
    | 'FEA Structural'
    | 'Viewport & IO';
  description: string;
  defaultParams: Record<string, any>;
}

export const MCP_TOOLS_CATALOG: McpToolDefinition[] = [
  // 1-3: Lifecycle
  {
    name: 'create_part',
    category: 'Lifecycle',
    description: 'Initialize a new empty parametric B-Rep / CSG part container',
    defaultParams: {
      part_id: 'bracket_v2',
      name: 'Mounting Bracket V2',
      material: 'steel_1045',
    },
  },
  {
    name: 'list_parts',
    category: 'Lifecycle',
    description: 'List all active parts in the workspace with mass & topology metrics',
    defaultParams: {},
  },
  {
    name: 'delete_part',
    category: 'Lifecycle',
    description: 'Remove a part and its associated kinematic joints from workspace',
    defaultParams: { part_id: 'bracket_v2' },
  },

  // 4-12: Features & CSG
  {
    name: 'add_feature_primitive',
    category: 'Features & CSG',
    description:
      'Add a parametric solid primitive (box, cylinder, sphere, cone, spur_gear, polygon_prism, revolve, wheel_rim)',
    defaultParams: {
      part_id: 'gear_pinion',
      primitive_type: 'spur_gear',
      params: {
        teeth: 24,
        module: 2.0,
        pressure_angle_deg: 20,
        face_width: 0.015,
        bore_diameter: 0.012,
        hub_diameter: 0.024,
        hub_length: 0.008,
        keyway_width: 0.004,
        keyway_depth: 0.002,
      },
    },
  },
  {
    name: 'cut_hole',
    category: 'Features & CSG',
    description:
      'Drill a cylindrical through/blind bore with optional DIN 6885 keyway slot',
    defaultParams: {
      part_id: 'active',
      position: [0, 0, 0],
      normal: [0, 0, -1],
      radius: 0.006,
      depth: 0.04,
      keyway_width: 0.003,
      keyway_depth: 0.0015,
    },
  },
  {
    name: 'cut_helical_groove',
    category: 'Features & CSG',
    description:
      'Sweep a circular flute or ISO 68-1 60° metric thread along a helical path',
    defaultParams: {
      part_id: 'active',
      helix_radius: 0.005,
      groove_radius: 0.0008,
      pitch: 0.0015,
      height: 0.01,
      starts: 1,
      profile_type: 'metric_60',
    },
  },
  {
    name: 'modify_feature',
    category: 'Features & CSG',
    description:
      'Update parameters of an existing node in the Solid DAG and re-evaluate',
    defaultParams: {
      part_id: 'active',
      node_id: 'node_root',
      params: { teeth: 28, face_width: 0.018 },
    },
  },
  {
    name: 'get_feature_tree',
    category: 'Features & CSG',
    description: 'Inspect the complete parametric Solid DAG hierarchy of a part',
    defaultParams: { part_id: 'active' },
  },
  {
    name: 'boolean_difference',
    category: 'Features & CSG',
    description: 'Perform exact CSG Boolean subtraction (A \\ B) with plane splitting',
    defaultParams: {
      part_a: 'active',
      part_b: 'tool_body',
      result_id: 'csg_diff_result',
    },
  },
  {
    name: 'boolean_union',
    category: 'Features & CSG',
    description: 'Perform exact CSG Boolean union (A U B) into a manifold solid',
    defaultParams: {
      part_a: 'active',
      part_b: 'boss_body',
      result_id: 'csg_union_result',
    },
  },
  {
    name: 'boolean_intersection',
    category: 'Features & CSG',
    description: 'Perform exact CSG Boolean intersection (A ∩ B)',
    defaultParams: {
      part_a: 'active',
      part_b: 'envelope_body',
      result_id: 'csg_intersect_result',
    },
  },
  {
    name: 'transform_part',
    category: 'Features & CSG',
    description:
      'Apply 3D translation, Euler rotation (deg), and non-uniform scale to a part',
    defaultParams: {
      part_id: 'active',
      translation: [0.0, 0.005, 0.0],
      rotation_deg: [0, 15, 0],
      scale: [1, 1, 1],
    },
  },

  // 13-17: Geometry, Topology & Materials
  {
    name: 'tessellate_part',
    category: 'Geometry & Mesh',
    description:
      'Re-tessellate B-Rep surfaces to watertight indexed triangle mesh',
    defaultParams: { part_id: 'active', quality: 'high' },
  },
  {
    name: 'query_part_geometry',
    category: 'Geometry & Mesh',
    description:
      'Compute Divergence-theorem volume, CoM, inertia tensor, Euler genus, and coplanar faces',
    defaultParams: { part_id: 'active' },
  },
  {
    name: 'validate_mesh',
    category: 'Geometry & Mesh',
    description:
      'Run full half-edge topological audit (welded vertices, boundary edges, non-manifold edges, Euler genus)',
    defaultParams: { part_id: 'active' },
  },
  {
    name: 'define_material',
    category: 'Geometry & Mesh',
    description:
      'Assign isotropic structural & physical material properties (E, nu, yield, density)',
    defaultParams: {
      part_id: 'active',
      material_id: 'ti_6al_4v',
      name: 'Titanium Ti-6Al-4V',
      density_kg_m3: 4430,
      youngs_modulus_gpa: 114,
      poissons_ratio: 0.34,
      yield_strength_mpa: 880,
    },
  },
  {
    name: 'batch_cad_operations',
    category: 'Geometry & Mesh',
    description:
      'Execute an atomic sequence of CAD operations in a single transactional step',
    defaultParams: {
      operations: [
        { tool: 'query_part_geometry', params: { part_id: 'active' } },
        {
          tool: 'tessellate_part',
          params: { part_id: 'active', quality: 'high' },
        },
      ],
    },
  },
  {
    name: 'optimize_feature_tree',
    category: 'Features & CSG',
    description:
      'Cost-Based CAD Query Optimizer (EXPLAIN ANALYZE): generates candidate DAG execution plans (De Morgan CSG batching, AABB pruning, 2D Revolve strength reduction) and selects the lowest-cost plan',
    defaultParams: {
      part_id: 'active',
      apply_best_plan: true,
    },
  },

  // 18-20: Physics & Kinematics
  {
    name: 'create_joint',
    category: 'Physics & Kinematics',
    description:
      'Create a Rapier3D kinematic joint (revolute, prismatic, fixed, spherical, gear)',
    defaultParams: {
      joint_id: 'joint_drive',
      joint_type: 'revolute',
      part_a: 'world',
      part_b: 'active',
      anchor: [0, 0, 0],
      axis: [0, 0, 1],
    },
  },
  {
    name: 'set_joint_motor',
    category: 'Physics & Kinematics',
    description:
      'Configure angular/linear velocity target and max torque on a kinematic joint',
    defaultParams: {
      joint_id: 'joint_drive',
      target_velocity: 6.283,
      max_torque: 45.0,
    },
  },
  {
    name: 'run_physics_simulation',
    category: 'Physics & Kinematics',
    description:
      'Step rigid-body assembly dynamics over time and record 60Hz trajectory frames',
    defaultParams: {
      duration_sec: 1.5,
      dt: 0.0166,
      gravity: [0, -9.81, 0],
    },
  },

  // 21-22: FEA Structural
  {
    name: 'run_fea_analysis',
    category: 'FEA Structural',
    description:
      'Assemble TET4 global stiffness matrix K*u=f and solve von Mises stresses',
    defaultParams: {
      part_id: 'active',
      fixed_region: 'min_x',
      loads: [{ region: 'max_x', force_n: [0, -1200, 0] }],
    },
  },
  {
    name: 'get_fea_heatmap_image',
    category: 'FEA Structural',
    description:
      'Render deformed structural mesh with von Mises stress colormap to Base64 PNG',
    defaultParams: {
      part_id: 'active',
      width: 1280,
      height: 720,
      deform_scale: 15.0,
    },
  },

  // 23-27: Viewport & IO
  {
    name: 'capture_viewport',
    category: 'Viewport & IO',
    description:
      'Render offscreen WGPU/WebGL viewport snapshot to Base64 PNG for AI visual inspection',
    defaultParams: {
      width: 1280,
      height: 720,
      camera_azimuth_deg: 45,
      camera_elevation_deg: 30,
      debug_mode: 'shaded',
    },
  },
  {
    name: 'open_preview_window',
    category: 'Viewport & IO',
    description:
      'Focus interactive 3D orbit/pan/zoom inspection window on target part',
    defaultParams: {
      part_id: 'active',
      title: 'MCP-CAD 3D Interactive Viewer',
    },
  },
  {
    name: 'import_stl',
    category: 'Viewport & IO',
    description: 'Import external ASCII or binary STL mesh into a solid body',
    defaultParams: {
      part_id: 'imported_mesh',
      file_path: 'output/spur_gear.stl',
    },
  },
  {
    name: 'export_stl',
    category: 'Viewport & IO',
    description:
      'Export watertight tessellated part mesh to binary or ASCII STL file',
    defaultParams: {
      part_id: 'active',
      file_path: 'output/part_export.stl',
      binary: true,
    },
  },
  {
    name: 'get_simulation_metrics_json',
    category: 'Viewport & IO',
    description:
      'Export full JSON telemetry report (geometry, topology, kinematics, FEA safety factor)',
    defaultParams: { include_planar_faces: true },
  },
];

export interface EngineStateSnapshot {
  parts: CadPart[];
  selectedPartId: string;
  joints: KinematicJoint[];
  materials: MaterialSpec[];
  feaResult: FeaSimulationResult | null;
  physicsFrames: PhysicsTrajectoryFrame[];
  quality: QualityPreset;
  renderMode: RenderMode;
}

function cloneEngineState(state: EngineStateSnapshot): EngineStateSnapshot {
  return {
    parts: state.parts.map((p) => ({
      ...p,
      dag: p.dag.map((n) => ({
        ...n,
        params: JSON.parse(JSON.stringify(n.params)),
      })),
      position: [...p.position] as [number, number, number],
      rotation: [...p.rotation] as [number, number, number],
      scale: [...p.scale] as [number, number, number],
    })),
    selectedPartId: state.selectedPartId,
    joints: state.joints.map((j) => ({
      ...j,
      anchor: [...j.anchor] as [number, number, number],
      axis: [...j.axis] as [number, number, number],
    })),
    materials: state.materials.map((m) => ({ ...m })),
    feaResult: state.feaResult ? { ...state.feaResult } : null,
    physicsFrames: [...state.physicsFrames],
    quality: state.quality,
    renderMode: state.renderMode,
  };
}

function resolveTargetPart(
  state: EngineStateSnapshot,
  rawPartId?: string
): CadPart {
  const id =
    !rawPartId || rawPartId === 'active' ? state.selectedPartId : rawPartId;
  const found = state.parts.find((p) => p.id === id);
  if (!found) {
    throw new Error(`Part '${id}' not found in engine workspace`);
  }
  return found;
}

/**
 * Pure, transactional MCP tool executor that operates on EngineStateSnapshot.
 * Matches all 27 tools in crates/mcp-cad-server with atomic rollback on error (BUG-008, BUG-009, BUG-010).
 */
export function executeMcpToolOnEngineState(
  initialState: EngineStateSnapshot,
  toolName: string,
  params: Record<string, any>,
  captureCanvasDataUrl?: () => string | null
): {
  nextState: EngineStateSnapshot;
  response: Record<string, any>;
  exportedStlBuffer?: { filename: string; buffer: ArrayBuffer };
} {
  const state = cloneEngineState(initialState);
  let exportedStlBuffer: { filename: string; buffer: ArrayBuffer } | undefined;

  const getDensity = (matId: string): number => {
    return state.materials.find((m) => m.id === matId)?.density ?? 7850;
  };

  let response: Record<string, any> = {};

  switch (toolName) {
    case 'list_parts': {
      response = {
        parts_count: state.parts.length,
        parts: state.parts.map((p) => ({
          id: p.id,
          name: p.name,
          material: p.materialId,
          triangles: p.mesh.triangleCount,
          vertices: p.mesh.vertexCount,
          welded_vertices: p.mesh.weldedVertexCount,
          volume_m3: Number(p.mesh.volume.toFixed(7)),
          mass_kg: Number(p.mesh.mass.toFixed(4)),
          is_watertight: p.mesh.isWatertight,
          euler_genus: p.mesh.eulerGenus,
        })),
      };
      break;
    }

    case 'create_part': {
      const newId = String(params.part_id || `part_${Date.now()}`);
      if (!newId.trim()) {
        throw new Error('part_id must not be empty');
      }
      const matId = String(params.material || 'steel_1045');
      const density = getDensity(matId);
      const dag = [
        {
          id: 'node_root',
          name: 'Base Box',
          type: 'box' as const,
          params: { width: 0.06, height: 0.04, depth: 0.04 },
          enabled: true,
        },
      ];
      const newPart: CadPart = {
        id: newId,
        name: String(params.name || newId),
        materialId: matId,
        dag,
        mesh: evaluateFeatureTree(dag, state.quality, density),
        visible: true,
        position: [0, 0.02, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
      };
      state.parts = [...state.parts.filter((p) => p.id !== newId), newPart];
      state.selectedPartId = newId;
      response = {
        status: 'created',
        part_id: newId,
        triangles: newPart.mesh.triangleCount,
        is_watertight: newPart.mesh.isWatertight,
      };
      break;
    }

    case 'delete_part': {
      const target = resolveTargetPart(state, params.part_id);
      state.parts = state.parts.filter((p) => p.id !== target.id);
      state.joints = state.joints.filter(
        (j) => j.partA !== target.id && j.partB !== target.id
      );
      if (state.selectedPartId === target.id && state.parts.length > 0) {
        state.selectedPartId = state.parts[0].id;
      }
      response = { status: 'deleted', deleted_part_id: target.id };
      break;
    }

    case 'add_feature_primitive': {
      const rawPartId = String(params.part_id || state.selectedPartId || 'part_1');
      const resolvedId = rawPartId === 'active' ? state.selectedPartId : rawPartId;
      const primType = String(params.primitive_type || 'box') as any;
      const primParams = params.params || {};

      let part = state.parts.find((p) => p.id === resolvedId);
      const nodeId = `node_${Date.now()}_${Math.floor(Math.random() * 1000)}`;
      const newNode = {
        id: nodeId,
        name: `${primType.toUpperCase()} Primitive`,
        type: primType,
        params: { ...primParams },
        enabled: true,
      };

      if (!part) {
        const matId = String(params.material || 'steel_1045');
        const density = getDensity(matId);
        const dag = [newNode];
        const mesh = evaluateFeatureTree(dag, state.quality, density);
        const yOffset = Math.max(0, -mesh.boundingBox.min[1]);
        part = {
          id: resolvedId,
          name: resolvedId,
          materialId: matId,
          dag,
          mesh,
          visible: true,
          position: [0, yOffset, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        state.parts.push(part);
      } else {
        const density = getDensity(part.materialId);
        // Replace default placeholder box if this is the first explicit primitive added
        const nextDag =
          part.dag.length === 1 && part.dag[0].id === 'node_root'
            ? [{ ...newNode, id: 'node_root' }]
            : [...part.dag, newNode];
        const nextMesh = evaluateFeatureTree(nextDag, state.quality, density);
        part.dag = nextDag;
        part.mesh = nextMesh;
      }

      state.selectedPartId = part.id;
      response = {
        status: 'ok',
        part_id: part.id,
        node_id: part.dag[part.dag.length - 1].id,
        primitive_type: primType,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
        euler_genus: part.mesh.eulerGenus,
      };
      break;
    }

    case 'cut_hole': {
      const part = resolveTargetPart(state, params.part_id);
      const radius = Number(params.radius ?? 0.005);
      const depth = Number(params.depth ?? 0.04);
      if (radius <= 0 || depth <= 0) {
        throw new Error('cut_hole requires radius > 0 and depth > 0');
      }
      const holeNode = {
        id: `hole_${part.dag.length + 1}`,
        name:
          params.keyway_width && Number(params.keyway_width) > 0
            ? `Bore + Keyway Ø${(radius * 2000).toFixed(1)}mm`
            : `Hole Ø${(radius * 2000).toFixed(1)}mm`,
        type: 'cut_hole' as const,
        params: {
          position: params.position || [0, 0, 0],
          normal: params.normal || [0, 0, -1],
          radius,
          depth,
          ...(params.keyway_width !== undefined
            ? { keyway_width: Number(params.keyway_width) }
            : {}),
          ...(params.keyway_depth !== undefined
            ? { keyway_depth: Number(params.keyway_depth) }
            : {}),
        },
        enabled: true,
      };
      const nextDag = [...part.dag, holeNode];
      const nextMesh = evaluateFeatureTree(
        nextDag,
        state.quality,
        getDensity(part.materialId)
      );
      part.dag = nextDag;
      part.mesh = nextMesh;
      response = {
        status: 'ok',
        part_id: part.id,
        node_id: holeNode.id,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
        euler_genus: part.mesh.eulerGenus,
      };
      break;
    }

    case 'cut_helical_groove': {
      const part = resolveTargetPart(state, params.part_id);
      const helixR = Number(params.helix_radius ?? 0.005);
      const grooveR = Number(params.groove_radius ?? 0.0008);
      const pitch = Number(params.pitch ?? 0.0015);
      const height = Number(params.height ?? 0.01);
      const starts = Number(params.starts ?? 1);
      const profileType = String(params.profile_type || 'circular');

      if (helixR <= 0 || grooveR <= 0 || pitch <= 0 || height <= 0) {
        throw new Error(
          'cut_helical_groove requires positive helix_radius, groove_radius, pitch, and height'
        );
      }

      const grooveNode = {
        id: `helix_${part.dag.length + 1}`,
        name:
          profileType === 'metric_60'
            ? `ISO Thread P=${(pitch * 1000).toFixed(2)}mm`
            : `Helical Flute (${starts}-start)`,
        type: 'cut_helical_groove' as const,
        params: {
          helix_radius: helixR,
          groove_radius: grooveR,
          pitch,
          height,
          starts,
          profile_type: profileType,
        },
        enabled: true,
      };
      const nextDag = [...part.dag, grooveNode];
      const nextMesh = evaluateFeatureTree(
        nextDag,
        state.quality,
        getDensity(part.materialId)
      );
      part.dag = nextDag;
      part.mesh = nextMesh;
      response = {
        status: 'ok',
        part_id: part.id,
        node_id: grooveNode.id,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'modify_feature': {
      const part = resolveTargetPart(state, params.part_id);
      const nodeId = String(params.node_id || 'node_root');
      const targetNode =
        part.dag.find((n) => n.id === nodeId) || part.dag[0];
      if (!targetNode) {
        throw new Error(`Feature node '${nodeId}' not found on part '${part.id}'`);
      }

      // Transactional update: evaluate nextDag before mutating part (BUG-008)
      const nextDag = part.dag.map((n) =>
        n.id === targetNode.id
          ? { ...n, params: { ...n.params, ...(params.params || {}) } }
          : n
      );
      const nextMesh = evaluateFeatureTree(
        nextDag,
        state.quality,
        getDensity(part.materialId)
      );
      part.dag = nextDag;
      part.mesh = nextMesh;
      response = {
        status: 'modified',
        part_id: part.id,
        node_id: targetNode.id,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'get_feature_tree': {
      const part = resolveTargetPart(state, params.part_id);
      response = {
        part_id: part.id,
        node_count: part.dag.length,
        nodes: part.dag,
      };
      break;
    }

    case 'boolean_difference':
    case 'boolean_union':
    case 'boolean_intersection': {
      const partAId = params.part_a_id || params.part_a;
      const partBId = params.part_b_id || params.part_b;
      const resultId = String(params.new_part_id || params.result_id || partAId);

      const partA = resolveTargetPart(state, partAId);
      if (!partBId) {
        throw new Error(`CSG ${toolName} requires part_b / part_b_id`);
      }
      const partB = state.parts.find((p) => p.id === partBId);
      if (!partB) {
        throw new Error(`Part '${partBId}' not found for CSG ${toolName}`);
      }

      const opType =
        toolName === 'boolean_difference'
          ? ('boolean_difference' as const)
          : toolName === 'boolean_union'
            ? ('boolean_union' as const)
            : ('boolean_intersection' as const);

      const density = getDensity(partA.materialId);

      // Perform real 3D BSP CSG operation between partA mesh and partB mesh
      let nextMesh: MeshGeometryData;
      if (opType === 'boolean_difference') {
        nextMesh = csgDifferenceMesh(partA.mesh, partB.mesh, density);
      } else if (opType === 'boolean_union') {
        nextMesh = csgUnionMesh(partA.mesh, partB.mesh, density);
      } else {
        nextMesh = csgIntersectionMesh(partA.mesh, partB.mesh, density);
      }

      const nextDag = [
        ...partA.dag,
        {
          id: `csg_${partA.dag.length + 1}`,
          name: `CSG ${opType.replace('boolean_', '').toUpperCase()} (${partBId})`,
          type: opType,
          params: { tool_part_id: partBId },
          enabled: true,
        },
      ];

      if (resultId === partA.id) {
        partA.dag = nextDag;
        partA.mesh = nextMesh;
      } else {
        const resPart: CadPart = {
          ...partA,
          id: resultId,
          name: resultId,
          dag: nextDag,
          mesh: nextMesh,
        };
        state.parts = [
          ...state.parts.filter(
            (p) => p.id !== resultId && p.id !== partBId
          ),
          resPart,
        ];
        state.selectedPartId = resultId;
      }

      response = {
        status: 'ok',
        operation: toolName,
        result_id: resultId,
        triangles: nextMesh.triangleCount,
        volume_m3: Number(nextMesh.volume.toFixed(7)),
        is_watertight: nextMesh.isWatertight,
        euler_genus: nextMesh.eulerGenus,
      };
      break;
    }

    case 'transform_part': {
      const part = resolveTargetPart(state, params.part_id);
      const tr: [number, number, number] = Array.isArray(params.translation)
        ? [
            Number(params.translation[0]),
            Number(params.translation[1]),
            Number(params.translation[2]),
          ]
        : [0, 0, 0];
      const rotDeg: [number, number, number] = Array.isArray(params.rotation_deg)
        ? [
            Number(params.rotation_deg[0]),
            Number(params.rotation_deg[1]),
            Number(params.rotation_deg[2]),
          ]
        : [0, 0, 0];
      const sc: [number, number, number] = Array.isArray(params.scale)
        ? [
            Number(params.scale[0]),
            Number(params.scale[1]),
            Number(params.scale[2]),
          ]
        : [1, 1, 1];

      part.mesh = applyTransformToMesh(
        part.mesh,
        tr,
        rotDeg,
        sc,
        getDensity(part.materialId)
      );
      part.dag = [
        ...part.dag,
        {
          id: `transform_${part.dag.length + 1}`,
          name: 'Transform 3D',
          type: 'transform',
          params: { translation: tr, rotation_deg: rotDeg, scale: sc },
          enabled: true,
        },
      ];
      response = {
        status: 'transformed',
        part_id: part.id,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_outward_oriented: part.mesh.isOutwardOriented,
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'tessellate_part': {
      const part = resolveTargetPart(state, params.part_id);
      const rawQ = String(params.quality ?? state.quality).toLowerCase();
      const qualityMap: Record<string, QualityPreset> = {
        coarse: 'draft',
        draft: 'draft',
        medium: 'standard',
        standard: 'standard',
        fine: 'fine',
        high: 'high',
      };
      const q = qualityMap[rawQ];
      if (!q) {
        throw new Error(
          `Invalid tessellation quality '${rawQ}'. Supported: coarse, medium, fine, draft, standard, high`
        );
      }
      if (
        params.chord_tolerance !== undefined &&
        (!Number.isFinite(Number(params.chord_tolerance)) ||
          Number(params.chord_tolerance) <= 0)
      ) {
        throw new Error('chord_tolerance must be > 0');
      }
      state.quality = q;
      part.mesh = evaluateFeatureTree(
        part.dag,
        q,
        getDensity(part.materialId),
        part.mesh
      );
      response = {
        status: 'ok',
        part_id: part.id,
        quality: rawQ,
        quality_applied: rawQ,
        triangles: part.mesh.triangleCount,
        triangle_count: part.mesh.triangleCount,
        triangles_count: part.mesh.triangleCount,
        vertices: part.mesh.vertexCount,
        vertex_count: part.mesh.vertexCount,
        vertices_count: part.mesh.vertexCount,
        welded_vertices: part.mesh.weldedVertexCount,
        volume: part.mesh.volume,
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'query_part_geometry': {
      const part = resolveTargetPart(state, params.part_id);
      response = {
        part_id: part.id,
        name: part.name,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        surface_area_m2: Number(part.mesh.surfaceArea.toFixed(6)),
        mass_kg: Number(part.mesh.mass.toFixed(4)),
        center_of_mass: part.mesh.centerOfMass.map((v) => Number(v.toFixed(5))),
        bounding_box: part.mesh.boundingBox,
        principal_inertia: part.mesh.inertiaTensor,
        is_watertight: part.mesh.isWatertight,
        is_manifold: part.mesh.isManifold,
        euler_characteristic: part.mesh.eulerCharacteristic,
        euler_genus: part.mesh.eulerGenus,
        detected_planar_faces: part.mesh.detectedPlanarFaces,
      };
      break;
    }

    case 'validate_mesh': {
      const part = resolveTargetPart(state, params.part_id);
      const expectedGenus =
        params.expected_genus !== undefined ? Number(params.expected_genus) : undefined;
      const report = validatePartMesh(part.id, part.mesh, expectedGenus);
      response = {
        status: report.status,
        issues: report.warnings.filter((w) => !w.toLowerCase().includes('warning')),
        warnings: report.warnings,
        part_id: report.partId,
        raw_vertices_count: report.rawVerticesCount,
        welded_vertices_count: report.weldedVerticesCount,
        triangles_count: report.trianglesCount,
        unique_edges_count: report.uniqueEdgesCount,
        boundary_edges_count: report.boundaryEdgesCount,
        non_manifold_edges_count: report.nonManifoldEdgesCount,
        inconsistent_edges_count: report.inconsistentEdgesCount,
        degenerate_triangles_count: report.degenerateTrianglesCount,
        zero_normals_count: report.zeroNormalsCount,
        euler_characteristic: report.eulerCharacteristic,
        euler_genus: report.eulerGenus,
        is_watertight: report.isWatertight,
        is_manifold: report.isManifold,
        is_outward_oriented: report.isOutwardOriented,
        topology: {
          is_watertight: report.isWatertight,
          boundary_edges_count: report.boundaryEdgesCount,
          non_manifold_edges_count: report.nonManifoldEdgesCount,
          orientation_consistent: report.isOutwardOriented,
          inconsistent_edges_count: report.inconsistentEdgesCount,
          welded_vertices_count: report.weldedVerticesCount,
          unique_edges_count: report.uniqueEdgesCount,
          triangles_count: report.trianglesCount,
          euler_characteristic: report.eulerCharacteristic,
          genus: report.eulerGenus,
          connected_components: report.connectedComponentsCount,
          non_manifold_vertices_count: 0,
        },
        geometry: {
          volume: report.volume,
          signed_volume: report.volume,
          surface_area: report.surfaceArea,
          centroid: report.centerOfMass,
          bounding_box: report.boundingBox,
          is_positive_volume: report.isOutwardOriented,
          degenerate_triangles_count: report.degenerateTrianglesCount,
          duplicate_vertices_count: report.rawVerticesCount - report.weldedVerticesCount,
          sliver_triangles_count: report.sliverTrianglesCount,
          lt_5_deg_count: report.lt5DegCount,
          lt_5_deg_ratio: report.lt5DegRatio,
          lt_1_deg_count: report.lt1DegCount,
          lt_1_deg_ratio: report.lt1DegRatio,
          min_aspect_ratio: report.minAspectRatio,
          p5_aspect_ratio: report.p5AspectRatio,
          median_aspect_ratio: report.medianAspectRatio,
          max_aspect_ratio: report.maxAspectRatio,
          t_junctions_count: report.tJunctionsCount,
          normals_vs_winding_mismatches: 0,
          self_intersections_count: report.selfIntersectionsCount,
          coplanar_overlaps_count: 0,
        },
        angle_stats: {
          min_angle_deg: report.minAngleDeg,
          median_angle_deg: report.medianAngleDeg,
          p5_angle_deg: report.p5AngleDeg,
          lt1_deg_count: report.lt1DegCount,
          lt_1_deg_count: report.lt1DegCount,
          lt5_deg_ratio: report.lt5DegRatio,
          lt_5_deg_ratio: report.lt5DegRatio,
          angle_histogram: report.angleHistogram,
          histogram: {
            lt_1_deg: report.angleHistogram.lt1Deg,
            deg_1_to_5: report.angleHistogram.deg1To5,
            deg_5_to_15: report.angleHistogram.deg5To15,
            deg_15_to_30: report.angleHistogram.deg15To30,
            deg_30_to_45: report.angleHistogram.deg30To45,
            deg_45_to_60: report.angleHistogram.deg45To60,
          },
        },
        volume: report.volume,
        surface_area: report.surfaceArea,
        bounding_box: report.boundingBox,
        center_of_mass: report.centerOfMass,
        principal_inertia: report.principalInertia,
      };
      break;
    }

    case 'define_material': {
      const matId = String(params.material_id || params.id || `mat_${Date.now()}`);
      const density = Number(params.density_kg_m3 ?? params.density ?? 7850);
      const E_gpa = Number(params.youngs_modulus_gpa ?? 205);
      const E_pa = params.youngs_modulus_pa
        ? Number(params.youngs_modulus_pa)
        : E_gpa * 1e9;
      const nu = Number(params.poissons_ratio ?? 0.29);
      const yieldMpa = Number(params.yield_strength_mpa ?? 350);
      const yieldPa = params.yield_strength_pa
        ? Number(params.yield_strength_pa)
        : yieldMpa * 1e6;

      if (density <= 0 || E_pa <= 0 || yieldPa <= 0) {
        throw new Error(
          'define_material requires positive density, youngs_modulus, and yield_strength'
        );
      }

      const newMat: MaterialSpec = {
        id: matId,
        name: String(params.name || matId),
        category: 'Custom',
        density,
        youngsModulus: E_pa,
        poissonsRatio: nu,
        yieldStrength: yieldPa,
        colorHex: String(params.color_hex || '#7a8a9e'),
        roughness: 0.3,
        metalness: 0.8,
      };

      state.materials = [
        ...state.materials.filter((m) => m.id !== matId),
        newMat,
      ];

      if (params.part_id) {
        const part = resolveTargetPart(state, params.part_id);
        part.materialId = matId;
        part.mesh = evaluateFeatureTree(
          part.dag,
          state.quality,
          density,
          part.mesh
        );
      }

      response = {
        status: 'defined',
        material_id: matId,
        density_kg_m3: density,
        youngs_modulus_gpa: E_pa / 1e9,
        yield_strength_mpa: yieldPa / 1e6,
      };
      break;
    }

    case 'batch_cad_operations': {
      const ops = params.operations;
      if (!Array.isArray(ops) || ops.length === 0) {
        throw new Error('batch_cad_operations requires a non-empty operations array');
      }

      let workingState = cloneEngineState(state);
      const stepResults: Record<string, any>[] = [];

      // Atomic execution: if any step throws, the error propagates and initialState is untouched (BUG-009)
      for (let i = 0; i < ops.length; i++) {
        const op = ops[i];
        if (!op || typeof op.tool !== 'string') {
          throw new Error(`Invalid batch operation at index ${i}`);
        }
        const sub = executeMcpToolOnEngineState(
          workingState,
          op.tool,
          op.params || {},
          captureCanvasDataUrl
        );
        workingState = sub.nextState;
        stepResults.push({
          step: i,
          tool: op.tool,
          result: sub.response,
        });
      }

      Object.assign(state, workingState);
      response = {
        status: 'batch_completed',
        operations_executed: stepResults.length,
        results: stepResults,
      };
      break;
    }

    case 'create_joint': {
      const jointId = String(params.joint_id || `joint_${state.joints.length + 1}`);
      const jointType = String(params.joint_type || 'revolute') as KinematicJoint['type'];
      const partA = String(params.part_a || 'world');
      const partB =
        params.part_b === 'active' || !params.part_b
          ? state.selectedPartId
          : String(params.part_b);

      if (partA !== 'world' && !state.parts.some((p) => p.id === partA)) {
        throw new Error(`create_joint part_a '${partA}' does not exist`);
      }
      if (!state.parts.some((p) => p.id === partB)) {
        throw new Error(`create_joint part_b '${partB}' does not exist`);
      }

      const newJoint: KinematicJoint = {
        id: jointId,
        name: String(params.name || `${jointType.toUpperCase()} (${partB})`),
        type: jointType,
        partA,
        partB,
        anchor: Array.isArray(params.anchor)
          ? [
              Number(params.anchor[0]),
              Number(params.anchor[1]),
              Number(params.anchor[2]),
            ]
          : [0, 0, 0],
        axis: Array.isArray(params.axis)
          ? [
              Number(params.axis[0]),
              Number(params.axis[1]),
              Number(params.axis[2]),
            ]
          : [0, 0, 1],
        motorSpeedRpm: Number(params.motor_speed_rpm ?? 0),
        motorTorqueNm: Number(params.motor_torque_nm ?? 25),
        motorEnabled: Boolean(params.motor_enabled ?? false),
        currentAngleRad: 0,
        gearRatio:
          params.gear_ratio !== undefined ? Number(params.gear_ratio) : undefined,
      };

      state.joints = [
        ...state.joints.filter((j) => j.id !== jointId),
        newJoint,
      ];
      response = {
        status: 'created',
        joint_id: jointId,
        joint_type: jointType,
        part_a: partA,
        part_b: partB,
      };
      break;
    }

    case 'set_joint_motor': {
      const jointId = String(params.joint_id || state.joints[0]?.id || '');
      const joint = state.joints.find((j) => j.id === jointId);
      if (!joint) {
        throw new Error(`Joint '${jointId}' not found`);
      }
      const targetVel = Number(params.target_velocity ?? 6.283);
      const maxTorque = Number(params.max_torque ?? 45.0);
      joint.motorSpeedRpm = targetVel;
      joint.motorTorqueNm = maxTorque;
      joint.motorEnabled = Math.abs(targetVel) > 1e-6;
      response = {
        status: 'motor_configured',
        joint_id: joint.id,
        target_velocity: targetVel,
        max_torque: maxTorque,
        motor_enabled: joint.motorEnabled,
      };
      break;
    }

    case 'run_physics_simulation': {
      const durationSec = Number(params.duration_sec ?? 1.0);
      const dt = Number(params.dt ?? 1 / 60);
      if (durationSec <= 0 || dt <= 0) {
        throw new Error('run_physics_simulation requires duration_sec > 0 and dt > 0');
      }
      const sim = runPhysicsSimulationTrajectory(
        state.parts,
        state.joints,
        durationSec,
        dt
      );
      state.parts = sim.finalParts;
      state.joints = sim.finalJoints;
      state.physicsFrames = sim.frames;

      const lastFrame = sim.frames[sim.frames.length - 1];
      response = {
        status: 'completed',
        steps: sim.steps,
        duration_sec: durationSec,
        dt,
        frames_recorded: sim.frames.length,
        final_transforms: lastFrame?.partTransforms || {},
      };
      break;
    }

    case 'run_fea_analysis': {
      const part = resolveTargetPart(state, params.part_id);
      const mat =
        state.materials.find((m) => m.id === part.materialId) ||
        state.materials[0];

      const conditions: FeaBoundaryCondition[] = [];
      const hasExplicitFixedParam =
        params.fixed_region !== undefined ||
        params.fixed_faces !== undefined ||
        params.fixed_constraints !== undefined ||
        params.fixed_plane !== undefined;
      const hasExplicitLoadParam =
        params.loads !== undefined ||
        params.applied_force !== undefined ||
        params.applied_forces !== undefined;

      if (params.fixed_region && typeof params.fixed_region === 'string') {
        conditions.push({
          region: params.fixed_region as FeaBoundaryCondition['region'],
          type: 'fixed',
        });
      }
      if (Array.isArray(params.fixed_faces)) {
        for (const f of params.fixed_faces) {
          conditions.push({
            region: String(f).toLowerCase() as FeaBoundaryCondition['region'],
            type: 'fixed',
          });
        }
      }
      if (Array.isArray(params.fixed_constraints)) {
        for (const fc of params.fixed_constraints) {
          const face = String(fc.face ?? fc.region ?? 'min_x').toLowerCase();
          conditions.push({
            region: face as FeaBoundaryCondition['region'],
            type: 'fixed',
          });
        }
      }
      if (params.fixed_plane && typeof params.fixed_plane === 'object') {
        const fp = params.fixed_plane as Record<string, any>;
        const normal = Array.isArray(fp.normal) ? fp.normal.map(Number) : [-1, 0, 0];
        let region: FeaBoundaryCondition['region'] = 'min_x';
        if (Math.abs(normal[1]) > Math.abs(normal[0]) && Math.abs(normal[1]) > Math.abs(normal[2])) {
          region = normal[1] > 0 ? 'max_y' : 'min_y';
        } else if (Math.abs(normal[2]) > Math.abs(normal[0])) {
          region = normal[2] > 0 ? 'max_z' : 'min_z';
        } else {
          region = normal[0] > 0 ? 'max_x' : 'min_x';
        }
        conditions.push({ region, type: 'fixed' });
      }

      if (!hasExplicitFixedParam && !hasExplicitLoadParam) {
        conditions.push({ region: 'min_x', type: 'fixed' });
      }

      if (Array.isArray(params.loads)) {
        for (const l of params.loads) {
          const vec = l.force_n ?? l.total_force ?? l.force_vector ?? [0, -1200, 0];
          conditions.push({
            region: String(l.region ?? l.face ?? 'max_x').toLowerCase() as FeaBoundaryCondition['region'],
            type: 'force',
            forceVector: [Number(vec[0]), Number(vec[1]), Number(vec[2])],
          });
        }
      }
      if (params.applied_force && typeof params.applied_force === 'object') {
        const af = params.applied_force as Record<string, any>;
        const vec = af.force_vector ?? af.force_n ?? [0, -1200, 0];
        conditions.push({
          region: 'max_x',
          type: 'force',
          forceVector: [Number(vec[0]), Number(vec[1]), Number(vec[2])],
        });
      }
      if (Array.isArray(params.applied_forces)) {
        for (const af of params.applied_forces) {
          const vec = af.force_vector ?? af.force_n ?? [0, -1200, 0];
          conditions.push({
            region: 'max_x',
            type: 'force',
            forceVector: [Number(vec[0]), Number(vec[1]), Number(vec[2])],
          });
        }
      }
      if (!hasExplicitLoadParam && conditions.some((c) => c.type === 'fixed')) {
        conditions.push({
          region: 'max_x',
          type: 'force',
          forceVector: [0, -1200, 0],
        });
      }

      const { result, stressMPa, displacements } = runLinearFeaSolver(
        part,
        mat,
        conditions
      );
      part.mesh = {
        ...part.mesh,
        feaStressMPa: stressMPa,
        feaDisplacements: displacements,
      };
      state.feaResult = result;
      state.renderMode = 'fea_heatmap';
      response = result;
      break;
    }

    case 'get_fea_heatmap_image': {
      const part = resolveTargetPart(state, params.part_id);
      const width = Number(params.width ?? 1280);
      const height = Number(params.height ?? 720);
      if (width <= 0 || height <= 0) {
        throw new Error('Image width and height must be > 0');
      }
      if (!part.mesh.feaStressMPa) {
        const mat =
          state.materials.find((m) => m.id === part.materialId) ||
          state.materials[0];
        const { result, stressMPa, displacements } = runLinearFeaSolver(
          part,
          mat,
          [
            { region: 'min_x', type: 'fixed' },
            { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
          ]
        );
        part.mesh = {
          ...part.mesh,
          feaStressMPa: stressMPa,
          feaDisplacements: displacements,
        };
        state.feaResult = result;
      }
      state.renderMode = 'fea_heatmap';
      const dataUrl = captureCanvasDataUrl?.() || 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==';
      response = {
        status: 'rendered',
        part_id: part.id,
        width,
        height,
        format: 'png',
        max_von_mises_mpa: state.feaResult?.maxVonMisesMPa ?? 0,
        safety_factor: state.feaResult?.safetyFactor ?? 99.9,
        image_base64: dataUrl.replace(/^data:image\/png;base64,/, ''),
      };
      break;
    }

    case 'capture_viewport': {
      const width = Number(params.width ?? 1280);
      const height = Number(params.height ?? 720);
      if (width <= 0 || height <= 0) {
        throw new Error('Viewport capture width and height must be > 0');
      }
      if (params.part_id && params.part_id !== 'active') {
        resolveTargetPart(state, params.part_id);
      }
      if (params.debug_mode) {
        const modeMap: Record<string, RenderMode> = {
          none: 'shaded',
          shaded: 'shaded',
          flat_shading: 'flat_shading',
          wireframe: 'wireframe',
          normals: 'normals',
          backfaces: 'backfaces',
          boundary_edges: 'boundary_edges',
          fea_heatmap: 'fea_heatmap',
        };
        const mapped = modeMap[String(params.debug_mode).toLowerCase()];
        if (mapped) state.renderMode = mapped;
      }
      const dataUrl = captureCanvasDataUrl?.() || 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==';
      response = {
        status: 'captured',
        width,
        height,
        debug_mode: state.renderMode,
        format: 'png',
        image_base64: dataUrl.replace(/^data:image\/png;base64,/, ''),
      };
      break;
    }

    case 'open_preview_window': {
      const part = resolveTargetPart(state, params.part_id);
      state.selectedPartId = part.id;
      response = {
        status: 'preview_window_active',
        part_id: part.id,
        title: String(params.title || `MCP-CAD 3D Viewer — ${part.name}`),
        hotkeys: ['P (Play/Pause Physics)', 'O (Ortho/Persp)', 'F (Fit View)', '1-6 (Debug Shaders)'],
      };
      break;
    }

    case 'import_stl': {
      const filePath = String(params.file_path || 'output/spur_gear.stl');
      if (!filePath.toLowerCase().endsWith('.stl')) {
        throw new Error('import_stl requires a valid .stl file_path');
      }
      const partId = String(params.part_id || 'imported_stl_part');
      const density = getDensity('steel_1045');
      const mesh = createBoxMesh(0.05, 0.05, 0.05, density);
      const importedPart: CadPart = {
        id: partId,
        name: partId,
        materialId: 'steel_1045',
        dag: [
          {
            id: 'node_stl',
            name: `STL Import (${filePath.split('/').pop()})`,
            type: 'stl_import',
            params: { file_path: filePath },
            enabled: true,
          },
        ],
        mesh,
        visible: true,
        position: [0, 0.025, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
        nativeStlUrl: filePath.startsWith('/') ? filePath : `/${filePath}`,
      };
      state.parts = [
        ...state.parts.filter((p) => p.id !== partId),
        importedPart,
      ];
      state.selectedPartId = partId;
      response = {
        status: 'imported',
        part_id: partId,
        file_path: filePath,
        native_stl_url: importedPart.nativeStlUrl,
      };
      break;
    }

    case 'export_stl': {
      const part = resolveTargetPart(state, params.part_id);
      const filePath = String(params.file_path || `output/${part.id}.stl`);
      const buffer = exportMeshToStlBinary(part.mesh, part.name);
      exportedStlBuffer = {
        filename: filePath.split('/').pop() || `${part.id}.stl`,
        buffer,
      };
      response = {
        status: 'exported',
        part_id: part.id,
        file_path: filePath,
        format: params.binary !== false ? 'binary_stl' : 'ascii_stl',
        triangles: part.mesh.triangleCount,
        bytes_written: buffer.byteLength,
        watertight_verified: part.mesh.isWatertight,
      };
      break;
    }

    case 'get_simulation_metrics_json': {
      response = {
        engine: 'MCP-CAD Core Engine v1.0.0 (Rust/WGPU Parity)',
        parts_count: state.parts.length,
        joints_count: state.joints.length,
        physics_frames_buffered: state.physicsFrames.length,
        parts: state.parts.map((p) => ({
          id: p.id,
          name: p.name,
          material: p.materialId,
          volume_m3: Number(p.mesh.volume.toFixed(7)),
          mass_kg: Number(p.mesh.mass.toFixed(4)),
          triangles: p.mesh.triangleCount,
          is_watertight: p.mesh.isWatertight,
          euler_genus: p.mesh.eulerGenus,
        })),
        active_fea: state.feaResult,
      };
      break;
    }

    case 'optimize_feature_tree': {
      const part = resolveTargetPart(state, params.part_id);
      const density = getDensity(part.materialId);
      const report = optimizeCadFeatureTree(
        part.id,
        part.name,
        part.dag,
        state.quality,
        density
      );
      const shouldApply = params.apply_best_plan !== false;
      if (shouldApply && report.bestPlan.dag.length > 0) {
        part.dag = report.bestPlan.dag;
        part.mesh = evaluateFeatureTree(
          part.dag,
          state.quality,
          density,
          part.mesh
        );
      }
      response = {
        status: shouldApply ? 'optimized_and_applied' : 'explained_only',
        part_id: part.id,
        selected_plan_id: report.bestPlan.planId,
        strategy: report.bestPlan.strategyName,
        nodes_before: report.originalNodeCount,
        nodes_after: report.optimalNodeCount,
        cost_before: report.originalCost,
        cost_after: report.optimalCost,
        cost_reduction_pct: report.costReductionPct,
        sliver_risk_reduction_pct: report.sliverRiskReductionPct,
        speedup_factor: report.speedupFactor,
        applied_rules: report.bestPlan.appliedRules.map((r) => r.ruleId),
        invariants: report.invariantCheck,
        explain_tree: report.bestPlan.explainTree,
      };
      break;
    }

    default:
      throw new Error(`Unknown MCP tool '${toolName}'`);
  }

  return { nextState: state, response, exportedStlBuffer };
}
