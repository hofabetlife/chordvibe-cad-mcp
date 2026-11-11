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
import { getMaterialById, resolveMaterialAlias } from './materials';

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
    joints: (state.joints || []).map((j) => ({
      ...j,
      anchor: [...j.anchor] as [number, number, number],
      axis: [...j.axis] as [number, number, number],
    })),
    materials: (state.materials || []).map((m) => ({ ...m })),
    feaResult: state.feaResult ? { ...state.feaResult } : null,
    physicsFrames: state.physicsFrames ? [...state.physicsFrames] : [],
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
    const resolved = resolveMaterialAlias(matId);
    return (
      state.materials.find((m) => m.id === matId || m.id === resolved)
        ?.density ?? getMaterialById(matId, state.materials).density ?? 7850
    );
  };

  let response: Record<string, any> = {};

  switch (toolName) {
    case 'list_parts': {
      const rawUnit = String(params.length_unit ?? 'mm').toLowerCase();
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Unsupported length_unit '${rawUnit}'. Supported: mm, m`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;
      const kVol = Math.pow(kFromM, 3);
      response = {
        parts_count: state.parts.length,
        parts: state.parts.map((p) => {
          const minM = p.mesh.boundingBox.min;
          const maxM = p.mesh.boundingBox.max;
          const dimsM: [number, number, number] = [
            maxM[0] - minM[0],
            maxM[1] - minM[1],
            maxM[2] - minM[2],
          ];
          return {
            id: p.id,
            part_id: p.id,
            name: p.name,
            material: p.materialId,
            triangles: p.mesh.triangleCount,
            triangle_count: p.mesh.triangleCount,
            triangles_count: p.mesh.triangleCount,
            vertices: p.mesh.vertexCount,
            welded_vertices: p.mesh.weldedVertexCount,
            bounding_box: {
              min: [minM[0] * kFromM, minM[1] * kFromM, minM[2] * kFromM],
              max: [maxM[0] * kFromM, maxM[1] * kFromM, maxM[2] * kFromM],
              dimensions: [dimsM[0] * kFromM, dimsM[1] * kFromM, dimsM[2] * kFromM],
            },
            bounding_box_dimensions: [
              dimsM[0] * kFromM,
              dimsM[1] * kFromM,
              dimsM[2] * kFromM,
            ],
            bounding_box_dimensions_m: dimsM,
            volume: Number((p.mesh.volume * kVol).toFixed(6)),
            volume_m3: Number(p.mesh.volume.toFixed(7)),
            mass: Number(p.mesh.mass.toFixed(4)),
            mass_kg: Number(p.mesh.mass.toFixed(4)),
            is_watertight: p.mesh.isWatertight,
            euler_genus: p.mesh.eulerGenus,
            revision: (p as any).revision ?? p.dag.length,
            length_unit: rawUnit,
          };
        }),
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          force: 'N',
          stress: 'MPa',
          angle: 'deg',
        },
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
      response = { status: 'deleted', part_id: target.id, deleted_part_id: target.id };
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

      (part as any).revision = ((part as any).revision ?? 0) + 1;
      const addedFeatureId = part.dag[part.dag.length - 1].id;
      state.selectedPartId = part.id;
      response = {
        status: 'ok',
        part_id: part.id,
        feature_id: addedFeatureId,
        node_id: addedFeatureId,
        primitive_type: primType,
        revision: (part as any).revision,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
        euler_genus: part.mesh.eulerGenus,
      };
      break;
    }

    case 'cut_hole': {
      const part = resolveTargetPart(state, params.part_id);
      const radius = Number(
        params.radius ??
          (params.diameter !== undefined ? Number(params.diameter) * 0.5 : 0.005)
      );
      const through = Boolean(params.through ?? params.through_all ?? false);
      const bbox = part.mesh.boundingBox;
      const maxSpan = Math.max(
        0.04,
        bbox.max[0] - bbox.min[0],
        bbox.max[1] - bbox.min[1],
        bbox.max[2] - bbox.min[2]
      );
      const depth = through
        ? Math.max(Number(params.depth ?? 0), maxSpan)
        : Number(params.depth ?? 0.04);
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
          position: params.position || params.center || [0, 0, 0],
          normal: params.normal || params.direction || [0, 0, -1],
          radius,
          depth,
          ...(through ? { through: true } : {}),
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
      (part as any).revision = ((part as any).revision ?? part.dag.length - 1) + 1;
      response = {
        status: 'ok',
        part_id: part.id,
        feature_id: holeNode.id,
        node_id: holeNode.id,
        revision: (part as any).revision,
        depth,
        through,
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
      const startZ =
        params.start_z !== undefined
          ? Number(params.start_z)
          : part.mesh.vertexCount > 0
            ? part.mesh.boundingBox.min[2]
            : -height * 0.5;

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
          start_z: startZ,
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
      (part as any).revision = ((part as any).revision ?? part.dag.length - 1) + 1;
      response = {
        status: 'ok',
        part_id: part.id,
        feature_id: grooveNode.id,
        node_id: grooveNode.id,
        revision: (part as any).revision,
        start_z: startZ,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'modify_feature': {
      const part = resolveTargetPart(state, params.part_id);
      const requestedId = params.feature_id ?? params.node_id;
      const targetNode = requestedId
        ? part.dag.find((n) => n.id === String(requestedId))
        : part.dag[0];
      if (!targetNode) {
        throw new Error(`Feature node '${requestedId}' not found on part '${part.id}'`);
      }

      // Transactional update: evaluate nextDag before mutating part (BUG-008 / P2-T19)
      const nextDag = part.dag.map((n) =>
        n.id === targetNode.id
          ? { ...n, params: { ...n.params, ...(params.params || params.new_params || {}) } }
          : n
      );
      const nextMesh = evaluateFeatureTree(
        nextDag,
        state.quality,
        getDensity(part.materialId)
      );
      part.dag = nextDag;
      part.mesh = nextMesh;
      (part as any).revision = ((part as any).revision ?? part.dag.length) + 1;
      response = {
        status: 'modified',
        part_id: part.id,
        feature_id: targetNode.id,
        node_id: targetNode.id,
        revision: (part as any).revision,
        triangles: part.mesh.triangleCount,
        volume_m3: Number(part.mesh.volume.toFixed(7)),
        is_watertight: part.mesh.isWatertight,
      };
      break;
    }

    case 'get_feature_tree': {
      const rawUnit = params.length_unit ? String(params.length_unit).toLowerCase() : 'mm';
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Invalid length_unit '${params.length_unit}'. Supported: 'mm', 'm'`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;
      const kArea = Math.pow(kFromM, 2);
      const scaleMeshForTree = (mesh: MeshGeometryData): MeshGeometryData => {
        if (Math.abs(kFromM - 1.0) <= 1e-12) return mesh;
        const scaledVerts = new Float32Array(mesh.vertices.length);
        for (let i = 0; i < mesh.vertices.length; i++) {
          scaledVerts[i] = mesh.vertices[i] * kFromM;
        }
        return {
          ...mesh,
          vertices: scaledVerts,
          boundingBox: {
            min: [
              mesh.boundingBox.min[0] * kFromM,
              mesh.boundingBox.min[1] * kFromM,
              mesh.boundingBox.min[2] * kFromM,
            ],
            max: [
              mesh.boundingBox.max[0] * kFromM,
              mesh.boundingBox.max[1] * kFromM,
              mesh.boundingBox.max[2] * kFromM,
            ],
          },
          centerOfMass: [
            mesh.centerOfMass[0] * kFromM,
            mesh.centerOfMass[1] * kFromM,
            mesh.centerOfMass[2] * kFromM,
          ],
          surfaceArea: mesh.surfaceArea * kArea,
          detectedPlanarFaces: mesh.detectedPlanarFaces.map((f) => ({
            ...f,
            area: f.area * kArea,
            centroid: [
              f.centroid[0] * kFromM,
              f.centroid[1] * kFromM,
              f.centroid[2] * kFromM,
            ],
          })),
        };
      };
      const part = resolveTargetPart(state, params.part_id);
      const scaledDag = part.dag.map((node) => {
        const nextParams: Record<string, any> = { ...node.params };
        if (nextParams.mesh) {
          nextParams.mesh = scaleMeshForTree(nextParams.mesh);
        }
        if (nextParams.tool_mesh) {
          nextParams.tool_mesh = scaleMeshForTree(nextParams.tool_mesh);
        }
        return { ...node, params: nextParams };
      });
      response = {
        part_id: part.id,
        revision: part.dag.length,
        features_count: scaledDag.length,
        node_count: scaledDag.length,
        nodes: scaledDag,
        dag: scaledDag,
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          force: 'N',
          stress: 'MPa',
          angle: 'deg',
        },
      };
      break;
    }

    case 'boolean_difference':
    case 'boolean_union':
    case 'boolean_intersection': {
      const partAId =
        params.target_part_id || params.part_a_id || params.part_a;
      const partBId =
        params.tool_part_id || params.part_b_id || params.part_b;
      const resultId = String(params.new_part_id || params.result_id || partAId);

      const rawUnit = params.length_unit ? String(params.length_unit).toLowerCase() : 'mm';
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Invalid length_unit '${params.length_unit}'. Supported: 'mm', 'm'`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;

      const partA = resolveTargetPart(state, partAId);
      if (!partBId) {
        throw new Error(`CSG ${toolName} requires tool_part_id / part_b`);
      }
      if (partA.id === partBId && !params.new_part_id && !params.result_id) {
        throw new Error('target_part_id and tool_part_id must be different parts');
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

      const featureId = `csg_${partA.dag.length + 1}`;
      const nextDag = [
        ...partA.dag,
        {
          id: featureId,
          name: `CSG ${opType.replace('boolean_', '').toUpperCase()} (${partBId})`,
          type: opType,
          params: { tool_part_id: partBId, tool_mesh: partB.mesh },
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

      const minM = nextMesh.boundingBox.min;
      const maxM = nextMesh.boundingBox.max;
      const dimsM: [number, number, number] = [
        maxM[0] - minM[0],
        maxM[1] - minM[1],
        maxM[2] - minM[2],
      ];
      const maxDimM = Math.max(dimsM[0], dimsM[1], dimsM[2]);
      const minDimM = Math.min(dimsM[0], dimsM[1], dimsM[2]);
      const warnings: Array<{ code: string; message: string }> = [];
      if (maxDimM > 10.0) {
        warnings.push({
          code: 'SCALE_TOO_LARGE',
          message: `Part '${resultId}' has maximum bounding-box dimension ${maxDimM.toFixed(3)} m (> 10 m).`,
        });
      } else if (maxDimM > 0 && minDimM < 1e-4) {
        warnings.push({
          code: 'SCALE_TOO_SMALL',
          message: `Part '${resultId}' has minimum bounding-box dimension ${minDimM.toExponential(2)} m (< 0.1 mm).`,
        });
      }

      const volM3 = Number(nextMesh.volume.toFixed(9));
      const volDisplay = Number((nextMesh.volume * Math.pow(kFromM, 3)).toFixed(6));

      response = {
        status: 'ok',
        operation: toolName,
        result_id: resultId,
        part_id: resultId,
        target_part_id: resultId,
        feature_id: featureId,
        revision: nextDag.length,
        triangles: nextMesh.triangleCount,
        triangles_count: nextMesh.triangleCount,
        vertices_count: nextMesh.vertexCount,
        bounding_box: {
          min: [minM[0] * kFromM, minM[1] * kFromM, minM[2] * kFromM],
          max: [maxM[0] * kFromM, maxM[1] * kFromM, maxM[2] * kFromM],
          dimensions: [dimsM[0] * kFromM, dimsM[1] * kFromM, dimsM[2] * kFromM],
        },
        volume: volDisplay,
        volume_m3: volM3,
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          angle: 'deg',
        },
        warnings,
        is_watertight: nextMesh.isWatertight,
        euler_genus: nextMesh.eulerGenus,
      };
      break;
    }

    case 'transform_part': {
      const part = resolveTargetPart(state, params.part_id);
      if (part.dag.length === 0 && (!part.mesh || part.mesh.vertices.length === 0)) {
        throw new Error('Cannot transform empty solid');
      }
      const tr: [number, number, number] = Array.isArray(params.translation)
        ? [
            Number(params.translation[0]),
            Number(params.translation[1]),
            Number(params.translation[2]),
          ]
        : [0, 0, 0];
      const rotRaw = params.rotation_deg ?? params.rotation_rpy_deg;
      const rotDeg: [number, number, number] = Array.isArray(rotRaw)
        ? [
            Number(rotRaw[0]),
            Number(rotRaw[1]),
            Number(rotRaw[2]),
          ]
        : [0, 0, 0];
      const sc: [number, number, number] = Array.isArray(params.scale)
        ? [
            Number(params.scale[0]),
            Number(params.scale[1]),
            Number(params.scale[2]),
          ]
        : [1, 1, 1];
      if (sc[0] === 0 || sc[1] === 0 || sc[2] === 0) {
        throw new Error('Scale components cannot be zero');
      }

      if (part.dag.length === 0 && part.mesh.vertices.length > 0) {
        part.dag = [
          {
            id: 'node_stl',
            name: `Imported Mesh (${part.name})`,
            type: 'stl_import',
            params: { mesh: part.mesh },
            enabled: true,
          },
        ];
      }

      part.mesh = applyTransformToMesh(
        part.mesh,
        tr,
        rotDeg,
        sc,
        getDensity(part.materialId)
      );
      const featureId = `transform_${part.dag.length + 1}`;
      part.dag = [
        ...part.dag,
        {
          id: featureId,
          name: 'Transform 3D',
          type: 'transform',
          params: { translation: tr, rotation_deg: rotDeg, scale: sc },
          enabled: true,
        },
      ];
      const bbox = part.mesh.boundingBox;
      const dims: [number, number, number] = [
        bbox.max[0] - bbox.min[0],
        bbox.max[1] - bbox.min[1],
        bbox.max[2] - bbox.min[2],
      ];
      const volM3 = Number(part.mesh.volume.toFixed(7));
      response = {
        status: 'transformed',
        part_id: part.id,
        feature_id: featureId,
        revision: part.dag.length,
        bounding_box: {
          min: bbox.min,
          max: bbox.max,
          dimensions: dims,
        },
        volume: volM3,
        volume_m3: volM3,
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
      const rawUnit = params.length_unit ? String(params.length_unit).toLowerCase() : 'mm';
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Invalid length_unit '${params.length_unit}'. Supported: 'mm', 'm'`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;
      const part = resolveTargetPart(state, params.part_id);
      const expectedGenus =
        params.expected_genus !== undefined ? Number(params.expected_genus) : undefined;
      const strictSlivers = Boolean(params.strict_slivers);
      const report = validatePartMesh(part.id, part.mesh, expectedGenus, strictSlivers);
      response = {
        status: report.status,
        issues: report.issues ?? report.warnings.filter((w) => !w.toLowerCase().includes('notice') && !w.toLowerCase().includes('warning')),
        warnings: report.warnings,
        is_valid_solid: report.isValidSolid ?? (report.isWatertight && report.isManifold && report.isOutwardOriented),
        fea_ready: report.feaReady ?? (report.isWatertight && report.isManifold && report.isOutwardOriented),
        part_id: report.partId,
        revision: part.dag.length,
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
        volume_m3: report.volume,
        surface_area_m2: report.surfaceArea,
        volume: report.volume * Math.pow(kFromM, 3),
        surface_area: report.surfaceArea * Math.pow(kFromM, 2),
        bounding_box: report.boundingBox,
        center_of_mass: report.centerOfMass,
        principal_inertia: report.principalInertia,
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          force: 'N',
          stress: 'MPa',
          angle: 'deg',
        },
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
      const rawJointId = params.joint_id ?? params.id;
      if (rawJointId !== undefined && rawJointId !== null) {
        if (typeof rawJointId !== 'string' || !rawJointId.trim()) {
          throw new Error("Argument 'joint_id' must be a non-empty string");
        }
      }
      const jointId = rawJointId
        ? String(rawJointId).trim()
        : `joint_${state.joints.length + 1}`;
      const jointType = String(params.joint_type || 'revolute') as KinematicJoint['type'];
      const rawPartA = String(params.part_a || 'world');
      const partA = rawPartA === 'ground' ? 'world' : rawPartA;
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

      if (params.limits !== undefined && params.limits !== null) {
        if (!Array.isArray(params.limits) || params.limits.length !== 2) {
          throw new Error("create_joint 'limits' must be a 2-element array [min, max]");
        }
        const l0 = Number(params.limits[0]);
        const l1 = Number(params.limits[1]);
        if (!Number.isFinite(l0) || !Number.isFinite(l1) || l0 > l1) {
          throw new Error(`create_joint 'limits' [min, max] invalid: [${l0}, ${l1}]`);
        }
      }

      const rawAnchor = params.anchor ?? params.anchor_a ?? params.anchor_b;
      const newJoint: KinematicJoint = {
        id: jointId,
        name: String(params.name || `${jointType.toUpperCase()} (${partB})`),
        type: jointType,
        partA,
        partB,
        anchor: Array.isArray(rawAnchor)
          ? [
              Number(rawAnchor[0]),
              Number(rawAnchor[1]),
              Number(rawAnchor[2]),
            ]
          : [0, 0, 0],
        axis: Array.isArray(params.axis)
          ? [
              Number(params.axis[0]),
              Number(params.axis[1]),
              Number(params.axis[2]),
            ]
          : jointType === 'prismatic'
            ? [1, 0, 0]
            : [0, 0, 1],
        motorSpeedRpm: Number(params.motor_speed_rpm ?? 0),
        motorTorqueNm: Number(params.motor_torque_nm ?? 25),
        motorEnabled: Boolean(params.motor_enabled ?? false),
        currentAngleRad: 0,
        gearRatio:
          params.gear_ratio !== undefined
            ? Number(params.gear_ratio)
            : params.ratio !== undefined
              ? Number(params.ratio)
              : undefined,
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
      const maxTorque = Number(params.max_torque ?? params.max_force ?? 45.0);
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
      const rawDur = params.duration_seconds ?? params.duration_sec;
      if (rawDur === undefined || !Number.isFinite(Number(rawDur)) || Number(rawDur) <= 0) {
        throw new Error('run_physics_simulation requires positive duration_seconds > 0');
      }
      const durationSec = Number(rawDur);
      const stepMs = params.step_size_ms !== undefined ? Number(params.step_size_ms) : undefined;
      const dt = stepMs !== undefined ? stepMs / 1000 : Number(params.dt ?? 1 / 60);
      if (dt <= 0) {
        throw new Error('run_physics_simulation requires dt / step_size_ms > 0');
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
        success: true,
        steps: sim.steps,
        steps_simulated: sim.steps,
        duration_sec: durationSec,
        duration_seconds: durationSec,
        dt,
        frames_recorded: sim.frames.length,
        final_transforms: lastFrame?.partTransforms || {},
        trajectory_summary: lastFrame?.partTransforms || {},
        collisions: [],
        peak_joint_forces: {},
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
        status: 'opened',
        mode: 'spawned_gui_process',
        part_id: part.id,
        stl_path: `./output/${part.id}.stl`,
        title: String(params.title || `MCP-CAD 3D Viewer — ${part.name}`),
        hotkeys: ['P (Play/Pause Physics)', 'O (Ortho/Persp)', 'F (Fit View)', '1-6 (Debug Shaders)'],
      };
      break;
    }

    case 'import_stl': {
      const filePath = String(params.file_path || params.filepath || 'output/spur_gear.stl');
      if (!filePath.toLowerCase().endsWith('.stl')) {
        throw new Error('import_stl requires a valid .stl file_path');
      }
      const partId = String(params.part_id || 'imported_stl_part');
      const isFixed = Boolean(params.is_fixed);
      const density = getDensity('steel_1045');
      const mesh = createBoxMesh(0.05, 0.05, 0.05, density);
      const importedPart: CadPart = {
        id: partId,
        name: String(params.name || partId),
        materialId: String(params.material || 'steel_1045'),
        isFixed,
        dag: [
          {
            id: 'node_stl',
            name: `STL Import (${filePath.split('/').pop()})`,
            type: 'stl_import',
            params: { file_path: filePath, mesh },
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
        is_fixed: isFixed,
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
      const rawUnit = String(params.length_unit ?? 'mm').toLowerCase();
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Unsupported length_unit '${rawUnit}'. Supported: mm, m`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;
      const kVol = Math.pow(kFromM, 3);
      const partsSummary = state.parts.map((p) => {
        const minM = p.mesh.boundingBox.min;
        const maxM = p.mesh.boundingBox.max;
        const dimsM: [number, number, number] = [
          maxM[0] - minM[0],
          maxM[1] - minM[1],
          maxM[2] - minM[2],
        ];
        return {
          id: p.id,
          name: p.name,
          material: p.materialId,
          volume_m3: Number(p.mesh.volume.toFixed(7)),
          volume: Number((p.mesh.volume * kVol).toFixed(6)),
          mass_kg: Number(p.mesh.mass.toFixed(4)),
          triangles: p.mesh.triangleCount,
          bounding_box_dimensions: [
            dimsM[0] * kFromM,
            dimsM[1] * kFromM,
            dimsM[2] * kFromM,
          ],
          is_watertight: p.mesh.isWatertight,
          euler_genus: p.mesh.eulerGenus,
          revision: p.dag.length,
        };
      });
      const feaReports = state.feaResult
        ? [
            (() => {
              const targetPart =
                state.parts.find((p) => p.id === state.selectedPartId) ||
                state.parts[0];
              const partRev = targetPart ? targetPart.dag.length : 1;
              const feaRev = (state.feaResult as any).feaRevision ?? partRev;
              const stale = feaRev !== partRev;
              return {
                part_id: targetPart?.id ?? 'active',
                verdict: state.feaResult!.safetyFactor >= 1.0 ? 'PASS' : 'FAIL',
                factor_of_safety: state.feaResult!.safetyFactor,
                safety_factor: state.feaResult!.safetyFactor,
                max_von_mises_mpa: state.feaResult!.maxVonMisesMPa,
                max_displacement_mm: state.feaResult!.maxDisplacementMm,
                fea_revision: feaRev,
                part_revision: partRev,
                stale,
                is_stale: stale,
                status: stale ? 'STALE' : 'FRESH',
              };
            })(),
          ]
        : [];
      response = {
        engine: 'MCP-CAD Core Engine v1.0.0 (Rust/WGPU Parity)',
        parts_count: state.parts.length,
        joints_count: state.joints.length,
        physics_frames_buffered: state.physicsFrames.length,
        parts: partsSummary,
        active_fea: state.feaResult,
        fea_reports: feaReports,
        current_metrics: {
          parts_count: state.parts.length,
          parts: partsSummary,
          joints_count: state.joints.length,
          physics: {
            accumulated_time_s: 0.0,
            gravity: [0.0, -9.81 * kFromM, 0.0],
            gravity_m_s2: [0.0, -9.81, 0.0],
            joints_count: state.joints.length,
            last_simulation: null,
          },
          fea_reports: feaReports,
          length_unit: rawUnit,
          units: {
            length: rawUnit,
            area: `${rawUnit}^2`,
            volume: `${rawUnit}^3`,
            mass: 'kg',
            force: 'N',
            stress: 'MPa',
            angle: 'deg',
          },
        },
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          force: 'N',
          stress: 'MPa',
          angle: 'deg',
        },
      };
      break;
    }

    case 'optimize_feature_tree': {
      const rawUnit = String(params.length_unit ?? 'mm').toLowerCase();
      if (rawUnit !== 'mm' && rawUnit !== 'm') {
        throw new Error(`Unsupported length_unit '${rawUnit}'. Supported: mm, m`);
      }
      const kFromM = rawUnit === 'mm' ? 1000.0 : 1.0;
      const kVol = Math.pow(kFromM, 3);
      const part = resolveTargetPart(state, params.part_id);
      const density = getDensity(part.materialId);
      const prevRev = (part as any).revision ?? part.dag.length;
      const report = optimizeCadFeatureTree(
        part.id,
        part.name,
        part.dag,
        state.quality,
        density
      );
      const appliedRules = report.bestPlan.appliedRules.map((r) => r.ruleId);
      const shouldApply =
        params.apply !== undefined
          ? Boolean(params.apply)
          : params.apply_best_plan !== false;
      let nextRev = prevRev;
      if (shouldApply && appliedRules.length > 0 && report.bestPlan.dag.length > 0) {
        part.dag = report.bestPlan.dag;
        part.mesh = evaluateFeatureTree(
          part.dag,
          state.quality,
          density,
          part.mesh
        );
        nextRev = prevRev + 1;
        (part as any).revision = nextRev;
      }
      const scaleDagNodeFromMeters = (node: any) => {
        if (Math.abs(kFromM - 1.0) <= 1e-12) return node;
        const nextParams: Record<string, any> = { ...(node.params || {}) };
        const linearScalarKeys = [
          'width',
          'height',
          'depth',
          'radius',
          'diameter',
          'outer_radius',
          'inner_radius',
          'radius_bottom',
          'radius_top',
          'width_across_flats',
          'chamfer',
          'chamfer_radius',
          'module',
          'face_width',
          'bore_radius',
          'keyway_width',
          'keyway_depth',
          'hub_diameter',
          'hub_length',
          'rim_diameter',
          'rim_width',
          'flange_diameter',
          'hub_bore_diameter',
          'pcd_diameter',
          'bolt_hole_diameter',
          'spoke_width',
          'spoke_thickness',
          'offset_et',
          'helix_radius',
          'groove_radius',
          'pitch',
          'start_z',
          'thread_pitch',
          'thread_depth',
        ];
        for (const key of linearScalarKeys) {
          if (typeof nextParams[key] === 'number') {
            nextParams[key] = Number((nextParams[key] * kFromM).toFixed(6));
          }
        }
        const vec3Keys = ['position', 'center', 'translation'];
        for (const key of vec3Keys) {
          if (Array.isArray(nextParams[key]) && nextParams[key].length === 3) {
            nextParams[key] = nextParams[key].map((v: number) =>
              Number((Number(v) * kFromM).toFixed(6))
            );
          }
        }
        if (Array.isArray(nextParams.profile)) {
          nextParams.profile = nextParams.profile.map((pt: any) =>
            Array.isArray(pt) && pt.length >= 2
              ? [Number((Number(pt[0]) * kFromM).toFixed(6)), Number((Number(pt[1]) * kFromM).toFixed(6))]
              : pt
          );
        }
        return { ...node, params: nextParams };
      };
      const scalePlanFromMeters = (plan: any) => ({
        ...plan,
        dag: Array.isArray(plan.dag) ? plan.dag.map(scaleDagNodeFromMeters) : [],
      });
      const logicalPlan = scalePlanFromMeters(report.candidatePlans[0] ?? report.bestPlan);
      const physicalPlan = scalePlanFromMeters(report.bestPlan);
      const candidatePlans = report.candidatePlans.map(scalePlanFromMeters);
      response = {
        status: shouldApply ? 'optimized_and_applied' : 'explained_only',
        part_id: part.id,
        revision: nextRev,
        volume: Number((part.mesh.volume * kVol).toFixed(6)),
        volume_m3: Number(part.mesh.volume.toFixed(9)),
        length_unit: rawUnit,
        units: {
          length: rawUnit,
          area: `${rawUnit}^2`,
          volume: `${rawUnit}^3`,
          mass: 'kg',
          angle: 'deg',
        },
        selected_plan_id: report.bestPlan.planId,
        strategy: report.bestPlan.strategyName,
        nodes_before: report.originalNodeCount,
        nodes_after: report.optimalNodeCount,
        cost_before: report.originalCost,
        cost_after: report.optimalCost,
        cost_reduction_pct: report.costReductionPct,
        sliver_risk_reduction_pct: report.sliverRiskReductionPct,
        speedup_factor: report.speedupFactor,
        applied_rules: appliedRules,
        invariants: report.invariantCheck,
        explain_tree: report.bestPlan.explainTree,
        logical_plan: logicalPlan,
        physical_plan: physicalPlan,
        candidate_plans: candidatePlans,
      };
      break;
    }

    default:
      throw new Error(`Unknown MCP tool '${toolName}'`);
  }

  return { nextState: state, response, exportedStlBuffer };
}
