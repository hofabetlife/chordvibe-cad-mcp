import React, { useState, useEffect, useCallback, useRef } from 'react';
import {
  CadPart,
  DagNodeType,
  EngineeringWorkbench,
  FeaBoundaryCondition,
  FeaSimulationResult,
  KinematicJoint,
  MaterialSpec,
  McpToolExecution,
  PhysicsTrajectoryFrame,
  ProjectionMode,
  QualityPreset,
  RenderMode,
  ViewportLayout,
} from './types/cad';
import { DEFAULT_MATERIALS, getMaterialById } from './cad/materials';
import {
  createBoxMesh,
  createConeMesh,
  createCylinderMesh,
  createHelicalDrillMesh,
  createLightbulbRevolveMesh,
  createPolygonPrismMesh,
  createRevolveMesh,
  createSphereMesh,
  createSpurGearMesh,
  createWheelRimMesh,
  evaluateFeatureTree,
  exportMeshToStlBinary,
  parseStlBuffer,
} from './cad/primitives';
import { runLinearFeaSolver } from './cad/fea_solver';
import { stepKinematicsAssembly } from './cad/kinematics';
import { executeMcpToolOnEngineState } from './cad/mcp_dispatcher';
import { CadTitleBar } from './components/CadTitleBar';
import { CadRibbonToolbar } from './components/CadRibbonToolbar';
import { CadLeftSidebar } from './components/CadLeftSidebar';
import { CadViewport3D } from './components/CadViewport3D';
import {
  CadRightSidebar,
  RightInspectorTab,
} from './components/CadRightSidebar';
import { CadStatusBar } from './components/CadStatusBar';
import {
  AI_UNOPTIMIZED_SCENARIOS,
  AiUnoptimizedScenario,
} from './cad/query_optimizer';
import {
  DEFAULT_BRIDGE_STATUS,
  executeRemoteWindowsMcpTool,
  getWindowsWgpuArtifactUrl,
  GOLDEN_WINDOWS_MAPPINGS,
  loadCachedRustStlMesh,
  NativeBridgeStatus,
  probeWindowsMcpServer,
} from './cad/native_bridge';
import {
  Scissors,
  Sparkles,
  X,
  CheckCircle2,
  AlertTriangle,
  Box,
  Play,
  Pause,
  RotateCcw,
  Sliders,
  Monitor,
  Camera,
  Download,
  Cpu,
  Link2,
  HelpCircle,
  Compass,
  Activity,
} from 'lucide-react';

const NATIVE_STL_PRESET_MAP: Record<
  string,
  { url: string; title: string; materialId: string; density: number }
> = {
  raw_stl_v8_piston: {
    url: '/output/v8_racing_piston.stl',
    title: 'v8_racing_piston.stl (62 676 Δ)',
    materialId: 'alu_6061',
    density: 2700,
  },
  raw_stl_spur_gear: {
    url: '/output/spur_gear.stl',
    title: 'spur_gear.stl (23 172 Δ)',
    materialId: 'steel_1045',
    density: 7850,
  },
  raw_stl_nut_m10: {
    url: '/output/nut_m10.stl',
    title: 'nut_m10.stl (25 716 Δ)',
    materialId: 'steel_4140',
    density: 7850,
  },
  raw_stl_drill_bit: {
    url: '/output/drill_bit.stl',
    title: 'drill_bit.stl (19 991 Δ)',
    materialId: 'ti_6al_4v',
    density: 4430,
  },
  raw_stl_car_rim: {
    url: '/output/car_rim.stl',
    title: 'car_rim.stl (31 629 Δ)',
    materialId: 'alu_6061',
    density: 2700,
  },
  raw_stl_lightbulb: {
    url: '/output/lightbulb_e27.stl',
    title: 'lightbulb_e27.stl (72 028 Δ)',
    materialId: 'borosilicate_glass',
    density: 2500,
  },
};

export function App() {
  const [uiMode, setUiMode] = useState<'windows_preview' | 'studio_ide'>(
    'windows_preview'
  );
  const [activeWorkbench, setActiveWorkbench] =
    useState<EngineeringWorkbench>('modeling');
  const [showLeftPanel, setShowLeftPanel] = useState<boolean>(true);
  const [showRightPanel, setShowRightPanel] = useState<boolean>(true);
  const [externalInspectorTab, setExternalInspectorTab] =
    useState<RightInspectorTab | null>('optimizer');
  const [activePreset, setActivePreset] = useState<string>('spur_gear');
  const [quality, setQuality] = useState<QualityPreset>('standard');
  const [renderMode, setRenderMode] = useState<RenderMode>('shaded');
  const [showCadEdges, setShowCadEdges] = useState<boolean>(false);
  const [showReferenceGrid, setShowReferenceGrid] = useState<boolean>(false);
  const [projectionMode, setProjectionMode] =
    useState<ProjectionMode>('perspective');
  const [viewportLayout, setViewportLayout] =
    useState<ViewportLayout>('single');
  const [deformScale, setDeformScale] = useState<number>(15);
  const [fitTrigger, setFitTrigger] = useState<number>(1);
  const [bridgeStatus, setBridgeStatus] =
    useState<NativeBridgeStatus>(DEFAULT_BRIDGE_STATUS);
  const [useCompiledRustStl, setUseCompiledRustStl] = useState<boolean>(false);

  const [materials, setMaterials] = useState<MaterialSpec[]>(DEFAULT_MATERIALS);
  const [parts, setParts] = useState<CadPart[]>([]);
  const [selectedPartId, setSelectedPartId] = useState<string>('');
  const [joints, setJoints] = useState<KinematicJoint[]>([]);
  const [physicsFrames, setPhysicsFrames] = useState<PhysicsTrajectoryFrame[]>(
    []
  );
  const [isSimulatingPhysics, setIsSimulatingPhysics] =
    useState<boolean>(false);
  const [feaResult, setFeaResult] = useState<FeaSimulationResult | null>(null);
  const [fps, setFps] = useState<number>(60);

  // Modals for Cut Hole + Keyway and Cut Helical Groove / ISO Thread
  const [isHoleModalOpen, setIsHoleModalOpen] = useState<boolean>(false);
  const [holeRadiusMm, setHoleRadiusMm] = useState<number>(5.0);
  const [holeDepthMm, setHoleDepthMm] = useState<number>(40.0);
  const [holePosXMm, setHolePosXMm] = useState<number>(0.0);
  const [holePosYMm, setHolePosYMm] = useState<number>(0.0);
  const [holeAxis, setHoleAxis] = useState<'Z' | 'Y'>('Z');
  const [holeKeywayWidthMm, setHoleKeywayWidthMm] = useState<number>(3.0);
  const [holeKeywayDepthMm, setHoleKeywayDepthMm] = useState<number>(1.5);

  const [isHelixModalOpen, setIsHelixModalOpen] = useState<boolean>(false);
  const [helixProfileType, setHelixProfileType] = useState<
    'metric_60' | 'circular'
  >('metric_60');
  const [helixRadiusMm, setHelixRadiusMm] = useState<number>(5.0);
  const [helixGrooveRadiusMm, setHelixGrooveRadiusMm] = useState<number>(0.8);
  const [helixPitchMm, setHelixPitchMm] = useState<number>(1.5);
  const [helixHeightMm, setHelixHeightMm] = useState<number>(12.0);
  const [helixStarts, setHelixStarts] = useState<number>(1);

  // Toast feedback banner
  const [toastBanner, setToastBanner] = useState<{
    type: 'success' | 'error';
    text: string;
  } | null>(null);

  // Program Help / Shortcuts Modal
  const [isHelpOpen, setIsHelpOpen] = useState<boolean>(false);

  const captureFnRef = useRef<(() => string | null) | null>(null);

  const [mcpHistory, setMcpHistory] = useState<McpToolExecution[]>([
    {
      id: 'init_1',
      timestamp: '00:00:00',
      toolName: 'add_feature_primitive',
      params: {
        part_id: 'spur_gear_1',
        primitive_type: 'spur_gear',
        params: { teeth: 20, module: 2.0, bore_diameter: 0.012, keyway_width: 0.004 },
      },
      response: {
        status: 'ok',
        watertight: true,
        euler_genus: 1,
      },
      durationMs: 4.2,
      status: 'success',
    },
  ]);

  const showToast = useCallback(
    (text: string, type: 'success' | 'error' = 'success') => {
      setToastBanner({ text, type });
    },
    []
  );

  useEffect(() => {
    if (!toastBanner) return;
    const t = setTimeout(() => setToastBanner(null), 3500);
    return () => clearTimeout(t);
  }, [toastBanner]);

  // Probe local Windows mcp-cad-server.exe daemon (Variant 1 Unified Hybrid Bridge)
  useEffect(() => {
    probeWindowsMcpServer().then((status) => {
      setBridgeStatus(status);
    });
  }, []);

  const triggerDownloadBuffer = (buffer: ArrayBuffer, filename: string) => {
    const blob = new Blob([buffer], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const triggerDownloadDataUrl = (dataUrl: string, filename: string) => {
    const a = document.createElement('a');
    a.href = dataUrl;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
  };

  // Load any of the Engineering & QA Presets (with 100% Windows WGPU Z-axis parity)
  const loadScenePreset = useCallback(
    (
      presetId: string,
      q: QualityPreset = quality,
      keepRenderMode?: RenderMode,
      preferRustStl: boolean = useCompiledRustStl
    ) => {
      setActivePreset(presetId);
      setIsSimulatingPhysics(false);
      setFeaResult(null);
      if (keepRenderMode) {
        setRenderMode(keepRenderMode);
      } else if (renderMode === 'fea_heatmap') {
        setRenderMode('shaded');
      }

      if (presetId === 'spur_gear') {
        // Exact match to qa_wide_scan.rs line 1134 (spur_gear_hub_keyway)
        const dag = [
          {
            id: 'node_1',
            name: 'Spur Gear (Z=20, m=2.0, Hub+Keyway)',
            type: 'spur_gear' as const,
            params: {
              teeth: 20,
              module: 2.0,
              pressure_angle_deg: 20,
              face_width: 0.015,
              bore_diameter: 0.012,
              hub_diameter: 0.025,
              hub_length: 0.01,
              keyway_width: 0.004,
              keyway_depth: 0.0018,
            },
            enabled: true,
          },
        ];
        const gearMesh = evaluateFeatureTree(dag, q, 7850);
        const gearPart: CadPart = {
          id: 'spur_gear_1',
          name: 'Involute Spur Gear Z20',
          materialId: 'steel_1045',
          dag,
          mesh: gearMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([gearPart]);
        setSelectedPartId(gearPart.id);
        setJoints([
          {
            id: 'joint_shaft',
            name: 'Main Drive Shaft',
            type: 'revolute',
            partA: 'world',
            partB: gearPart.id,
            anchor: [0, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 35,
            motorTorqueNm: 45,
            motorEnabled: true,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'hex_nut_m10') {
        const dag = [
          {
            id: 'node_1',
            name: 'Hex Prism (DIN 934, S=17mm)',
            type: 'polygon_prism' as const,
            params: {
              sides: 6,
              outer_radius: 0.0098,
              height: 0.008,
              chamfer_radius: 0.0012,
            },
            enabled: true,
          },
          {
            id: 'node_2',
            name: 'Center Bore Ø8.5mm',
            type: 'cut_hole' as const,
            params: {
              position: [0, 0, 0],
              normal: [0, 0, -1],
              radius: 0.00425,
              depth: 0.008,
            },
            enabled: true,
          },
          {
            id: 'node_3',
            name: 'ISO 68-1 Thread (M10×1.5)',
            type: 'cut_helical_groove' as const,
            params: {
              helix_radius: 0.005,
              groove_radius: 0.0008,
              pitch: 0.0015,
              height: 0.008,
              starts: 1,
              profile_type: 'metric_60',
            },
            enabled: true,
          },
        ];
        const nutMesh = evaluateFeatureTree(dag, q, 7850);
        const nutPart: CadPart = {
          id: 'nut_m10',
          name: 'M10 Hex Nut (DIN 934)',
          materialId: 'steel_4140',
          dag,
          mesh: nutMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([nutPart]);
        setSelectedPartId(nutPart.id);
        setJoints([
          {
            id: 'joint_nut_spin',
            name: 'Thread Axis Spin',
            type: 'revolute',
            partA: 'world',
            partB: nutPart.id,
            anchor: [0, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 25,
            motorTorqueNm: 15,
            motorEnabled: true,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'qa_hex_chamfer') {
        // Exact match to qa_wide_scan.rs line 1081 (polygon_prism_hex_chamfer)
        const dag = [
          {
            id: 'node_hex_chamfer',
            name: 'Hex Prism Chamfered (WAF=40mm, H=30mm)',
            type: 'polygon_prism' as const,
            params: {
              sides: 6,
              width_across_flats: 0.04,
              outer_radius: 0.04 / Math.sqrt(3),
              height: 0.03,
              chamfer_radius: 0.004,
            },
            enabled: true,
          },
        ];
        const prismMesh = evaluateFeatureTree(dag, q, 7850);
        const prismPart: CadPart = {
          id: 'polygon_prism_hex_chamfer',
          name: 'Hex Prism Chamfer (BUG-005)',
          materialId: 'steel_1045',
          dag,
          mesh: prismMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([prismPart]);
        setSelectedPartId(prismPart.id);
        setJoints([]);
      } else if (presetId === 'v8_piston' || presetId === 'raw_stl_v8_piston') {
        const dag = [
          {
            id: 'node_1',
            name: 'Piston Revolve Body',
            type: 'revolve' as const,
            params: {
              angle_deg: 360,
              segments: 64,
              profile: [
                [0.0425, 0.0],
                [0.0425, 0.038],
                [0.039, 0.038],
                [0.039, 0.041],
                [0.0425, 0.041],
                [0.0425, 0.045],
                [0.039, 0.045],
                [0.039, 0.0475],
                [0.0425, 0.0475],
                [0.0425, 0.052],
                [0.039, 0.052],
                [0.039, 0.0545],
                [0.042, 0.0545],
                [0.042, 0.065],
                [0.03, 0.065],
                [0.024, 0.061],
                [0.0, 0.061],
                [0.0, 0.052],
                [0.032, 0.052],
                [0.036, 0.04],
                [0.036, 0.0],
              ],
            },
            enabled: true,
          },
          {
            id: 'node_2',
            name: 'Piston Pin Bore Ø22mm',
            type: 'cut_hole' as const,
            params: {
              center: [-0.05, 0, 0.025],
              normal: [1, 0, 0],
              radius: 0.011,
              depth: 0.1,
            },
            enabled: true,
          },
        ];
        const pistonMesh = evaluateFeatureTree(dag, q, 2700);
        const pistonPart: CadPart = {
          id: 'piston_01',
          name: 'V8 Racing Piston (Al 6061-T6)',
          materialId: 'al_6061',
          dag,
          mesh: pistonMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([pistonPart]);
        setSelectedPartId(pistonPart.id);
        setJoints([]);
      } else if (presetId === 'twist_drill') {
        const dag = [
          {
            id: 'node_1',
            name: 'Cylinder Shank Ø10×90mm',
            type: 'cylinder' as const,
            params: { radius: 0.005, height: 0.09 },
            enabled: true,
          },
          {
            id: 'node_2',
            name: 'Twin Helical Flutes (2-Start)',
            type: 'cut_helical_groove' as const,
            params: {
              helix_radius: 0.005,
              groove_radius: 0.0022,
              pitch: 0.032,
              height: 0.09,
              starts: 2,
              profile_type: 'circular',
            },
            enabled: true,
          },
        ];
        const drillMesh = evaluateFeatureTree(dag, q, 4430);
        const drillPart: CadPart = {
          id: 'drill_10mm',
          name: 'HSS Twist Drill Ø10mm',
          materialId: 'ti_6al_4v',
          dag,
          mesh: drillMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([drillPart]);
        setSelectedPartId(drillPart.id);
        setJoints([
          {
            id: 'joint_spindle',
            name: 'CNC Spindle Motor',
            type: 'revolute',
            partA: 'world',
            partB: drillPart.id,
            anchor: [0, 0, 0],
            axis: [0, 1, 0],
            motorSpeedRpm: 90,
            motorTorqueNm: 30,
            motorEnabled: true,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'wheel_rim') {
        // Exact match to qa_wide_scan.rs line 1214 (wheel_rim_5spoke)
        const dag = [
          {
            id: 'node_1',
            name: 'Wheel Rim (5-Spoke 18×8.0J)',
            type: 'wheel_rim' as const,
            params: {
              rim_diameter: 0.4572,
              rim_width: 0.2032,
              flange_diameter: 0.49,
              hub_diameter: 0.16,
              hub_bore_diameter: 0.066,
              bolt_circle_diameter: 0.1143,
              bolt_holes_count: 5,
              bolt_hole_diameter: 0.014,
              spoke_count: 5,
              spoke_width: 0.028,
              spoke_thickness: 0.018,
              offset: 0.035,
            },
            enabled: true,
          },
        ];
        const rimMesh = evaluateFeatureTree(dag, q, 2700);
        const rimPart: CadPart = {
          id: 'wheel_rim_18',
          name: '5-Spoke Alloy Rim 18"',
          materialId: 'steel_1045',
          dag,
          mesh: rimMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([rimPart]);
        setSelectedPartId(rimPart.id);
        setJoints([
          {
            id: 'joint_axle',
            name: 'Wheel Hub Axle',
            type: 'revolute',
            partA: 'world',
            partB: rimPart.id,
            anchor: [0, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 45,
            motorTorqueNm: 120,
            motorEnabled: true,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'lightbulb_e27') {
        const dag = [
          {
            id: 'node_1',
            name: 'Revolve Envelope A60 (360°)',
            type: 'revolve' as const,
            params: { bulb_diameter: 0.06, total_height: 0.11, angle_deg: 360 },
            enabled: true,
          },
        ];
        const bulbMesh = createLightbulbRevolveMesh(0.06, 0.11, 2500);
        const bulbPart: CadPart = {
          id: 'bulb_e27',
          name: 'E27 A60 Lightbulb + Base',
          materialId: 'borosilicate_glass',
          dag,
          mesh: bulbMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([bulbPart]);
        setSelectedPartId(bulbPart.id);
        setJoints([
          {
            id: 'joint_bulb_rot',
            name: 'Turntable Inspection',
            type: 'revolute',
            partA: 'world',
            partB: bulbPart.id,
            anchor: [0, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 25,
            motorTorqueNm: 5,
            motorEnabled: true,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'gear_transmission') {
        // Matches examples/gear_transmission.json: Z1=20, Z2=40, m=2mm, center distance a=60mm (0.06m)
        const pinionDag = [
          {
            id: 'node_p1',
            name: 'Pinion Spur Gear Z=20',
            type: 'spur_gear' as const,
            params: {
              teeth: 20,
              module: 2.0,
              pressure_angle_deg: 20,
              face_width: 0.015,
              bore_diameter: 0.01,
              hub_diameter: 0.02,
              hub_length: 0.005,
              keyway_width: 0.003,
              keyway_depth: 0.0015,
            },
            enabled: true,
          },
        ];
        const drivenDag = [
          {
            id: 'node_g2',
            name: 'Driven Spur Gear Z=40',
            type: 'spur_gear' as const,
            params: {
              teeth: 40,
              module: 2.0,
              pressure_angle_deg: 20,
              face_width: 0.015,
              bore_diameter: 0.014,
              hub_diameter: 0.03,
              hub_length: 0.005,
              keyway_width: 0.004,
              keyway_depth: 0.002,
            },
            enabled: true,
          },
        ];
        const baseDag = [
          {
            id: 'node_b1',
            name: 'Mounting Frame Plate',
            type: 'box' as const,
            params: { width: 0.16, height: 0.09, depth: 0.012 },
            enabled: true,
          },
        ];

        const pinionPart: CadPart = {
          id: 'gear_pinion',
          name: 'Driving Pinion (Z1=20)',
          materialId: 'steel_4140',
          dag: pinionDag,
          mesh: evaluateFeatureTree(pinionDag, q, 7850),
          visible: true,
          position: [-0.03, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        const drivenPart: CadPart = {
          id: 'gear_driven',
          name: 'Driven Gear (Z2=40)',
          materialId: 'brass_c360',
          dag: drivenDag,
          mesh: evaluateFeatureTree(drivenDag, q, 8500),
          visible: true,
          position: [0.03, 0, 0],
          rotation: [0, 0, Math.PI / 40], // 4.5 deg tooth valley alignment
          scale: [1, 1, 1],
        };
        const basePart: CadPart = {
          id: 'gearbox_base',
          name: 'Gearbox Backplate',
          materialId: 'alu_6061',
          dag: baseDag,
          mesh: evaluateFeatureTree(baseDag, q, 2700),
          visible: true,
          position: [0, 0, -0.016],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };

        setParts([pinionPart, drivenPart, basePart]);
        setSelectedPartId(pinionPart.id);
        setJoints([
          {
            id: 'joint_input_motor',
            name: 'Input Shaft Motor (ω1)',
            type: 'revolute',
            partA: 'gearbox_base',
            partB: 'gear_pinion',
            anchor: [-0.03, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 45,
            motorTorqueNm: 50,
            motorEnabled: true,
            currentAngleRad: 0,
          },
          {
            id: 'joint_gear_mesh',
            name: 'Gear Mesh Ratio (i = -0.5)',
            type: 'gear',
            partA: 'gear_pinion',
            partB: 'gear_driven',
            anchor: [-0.01, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 0,
            motorTorqueNm: 0,
            motorEnabled: true,
            currentAngleRad: 0,
            gearRatio: 0.5,
          },
        ]);
      } else if (presetId === 'bracket_fea') {
        const dag = [
          {
            id: 'node_1',
            name: 'Cantilever Beam (200×20×20mm)',
            type: 'box' as const,
            params: { width: 0.2, height: 0.02, depth: 0.02 },
            enabled: true,
          },
          {
            id: 'node_2',
            name: 'Lightening Bore Ø8mm',
            type: 'cut_hole' as const,
            params: {
              position: [0.0, 0, 0],
              normal: [0, 0, -1],
              radius: 0.004,
              depth: 0.02,
            },
            enabled: true,
          },
        ];
        const bracketMesh = evaluateFeatureTree(dag, q, 2700);
        const bracketPart: CadPart = {
          id: 'cantilever_bracket',
          name: 'Aerospace Cantilever Beam',
          materialId: 'alu_6061',
          dag,
          mesh: bracketMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };

        const mat = getMaterialById('alu_6061', materials);
        const conds: FeaBoundaryCondition[] = [
          { region: 'min_x', type: 'fixed' },
          { region: 'max_x', type: 'force', forceVector: [0, -1000, 0] },
        ];
        const { result, stressMPa, displacements } = runLinearFeaSolver(
          bracketPart,
          mat,
          conds
        );
        bracketPart.mesh.feaStressMPa = stressMPa;
        bracketPart.mesh.feaDisplacements = displacements;

        setParts([bracketPart]);
        setSelectedPartId(bracketPart.id);
        setFeaResult(result);
        setRenderMode('fea_heatmap');
        setJoints([]);
      } else if (presetId === 'crank_mechanism') {
        const basePart: CadPart = {
          id: 'mechanism_base',
          name: 'Mechanism Base Bed',
          materialId: 'alu_6061',
          dag: [
            {
              id: 'node_mb',
              name: 'Bed Plate 240×30×12mm',
              type: 'box',
              params: { width: 0.24, height: 0.03, depth: 0.012 },
              enabled: true,
            },
          ],
          mesh: createBoxMesh(0.24, 0.03, 0.012, 2700),
          visible: true,
          position: [0, 0, -0.016],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        const crankPart: CadPart = {
          id: 'crank_arm',
          name: 'Crank Arm (r=30mm)',
          materialId: 'steel_4140',
          dag: [
            {
              id: 'node_ca',
              name: 'Crank Throw Bar',
              type: 'box',
              params: { width: 0.05, height: 0.016, depth: 0.01 },
              enabled: true,
            },
          ],
          mesh: createBoxMesh(0.05, 0.016, 0.01, 7850),
          visible: true,
          position: [-0.045, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        const rodPart: CadPart = {
          id: 'connecting_rod',
          name: 'Connecting Rod (L=95mm)',
          materialId: 'ti_6al_4v',
          dag: [
            {
              id: 'node_cr',
              name: 'Connecting Link',
              type: 'box',
              params: { width: 0.095, height: 0.012, depth: 0.008 },
              enabled: true,
            },
          ],
          mesh: createBoxMesh(0.095, 0.012, 0.008, 4430),
          visible: true,
          position: [0.0175, 0, 0.008],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        const sliderPart: CadPart = {
          id: 'linear_slider',
          name: 'Piston Slider Block',
          materialId: 'brass_c360',
          dag: [
            {
              id: 'node_ls',
              name: 'Slider Piston 32×24×20mm',
              type: 'box',
              params: { width: 0.032, height: 0.024, depth: 0.02 },
              enabled: true,
            },
          ],
          mesh: createBoxMesh(0.032, 0.024, 0.02, 8500),
          visible: true,
          position: [0.065, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };

        setParts([basePart, crankPart, rodPart, sliderPart]);
        setSelectedPartId(crankPart.id);
        setJoints([
          {
            id: 'joint_crank',
            name: 'Crankshaft Drive Motor',
            type: 'revolute',
            partA: 'mechanism_base',
            partB: 'crank_arm',
            anchor: [-0.06, 0, 0],
            axis: [0, 0, 1],
            motorSpeedRpm: 50,
            motorTorqueNm: 40,
            motorEnabled: true,
            currentAngleRad: 0,
          },
          {
            id: 'joint_slider',
            name: 'Prismatic Slider Guide',
            type: 'prismatic',
            partA: 'mechanism_base',
            partB: 'linear_slider',
            anchor: [0.065, 0, 0],
            axis: [1, 0, 0],
            motorSpeedRpm: 0,
            motorTorqueNm: 0,
            motorEnabled: false,
            currentAngleRad: 0,
          },
        ]);
      } else if (presetId === 'qa_revolve_180') {
        // Exact match to qa_wide_scan.rs line 943 (revolve_partial_180, BUG-003)
        const profile: [number, number][] = [
          [0.01, -0.02],
          [0.03, -0.02],
          [0.03, 0.02],
          [0.01, 0.02],
        ];
        const dag = [
          {
            id: 'node_rev180',
            name: 'Partial Revolve 180° (BUG-003)',
            type: 'revolve' as const,
            params: {
              profile,
              angle_deg: 180,
            },
            enabled: true,
          },
        ];
        const revMesh = createRevolveMesh(profile, 180, q, 7850);
        const revPart: CadPart = {
          id: 'revolve_partial_180',
          name: 'Partial Revolve 180° Solid',
          materialId: 'steel_1045',
          dag,
          mesh: revMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([revPart]);
        setSelectedPartId(revPart.id);
        setJoints([]);
      } else if (presetId === 'qa_csg_two_holes') {
        // Exact match to qa_wide_scan.rs lines 1278 & 1309 (csg_chain_two_holes, BUG-006, Genus g=2)
        const dag = [
          {
            id: 'node_box',
            name: 'Base Block 100×100×40mm',
            type: 'box' as const,
            params: { width: 0.1, height: 0.1, depth: 0.04 },
            enabled: true,
          },
          {
            id: 'node_hole_1',
            name: 'Left Keyway Bore Ø24mm',
            type: 'cut_hole' as const,
            params: {
              position: [-0.025, 0, 0],
              normal: [0, 0, 1],
              radius: 0.012,
              depth: 0.1,
              keyway_width: 0.005,
              keyway_depth: 0.0025,
            },
            enabled: true,
          },
          {
            id: 'node_hole_2',
            name: 'Right Through-Bore Ø24mm',
            type: 'cut_hole' as const,
            params: {
              position: [0.025, 0, 0],
              normal: [0, 0, 1],
              radius: 0.012,
              depth: 0.1,
            },
            enabled: true,
          },
        ];
        const csgMesh = evaluateFeatureTree(dag, q, 7850);
        const csgPart: CadPart = {
          id: 'csg_chain_two_holes',
          name: 'CSG Dual-Bore Block (g=2)',
          materialId: 'steel_1045',
          dag,
          mesh: csgMesh,
          visible: true,
          position: [0, 0, 0],
          rotation: [0, 0, 0],
          scale: [1, 1, 1],
        };
        setParts([csgPart]);
        setSelectedPartId(csgPart.id);
        setJoints([]);
      } else {
        const aiScen = AI_UNOPTIMIZED_SCENARIOS.find((s) => s.id === presetId);
        if (aiScen) {
          const clonedDag = aiScen.dag.map((n) => ({
            ...n,
            params: JSON.parse(JSON.stringify(n.params)),
          }));
          const mesh = evaluateFeatureTree(clonedDag, q, aiScen.density);
          const aiPart: CadPart = {
            id: aiScen.id,
            name: aiScen.title.split(' (')[0],
            materialId: aiScen.materialId,
            dag: clonedDag,
            mesh,
            visible: true,
            position: [0, 0, 0],
            rotation: [0, 0, 0],
            scale: [1, 1, 1],
          };
          setParts([aiPart]);
          setSelectedPartId(aiPart.id);
          setJoints([]);
        }
      }

      setFitTrigger((c) => c + 1);

      // Direct binary STL load from /output/*.stl when selected or when Rust STL kernel mode is active
      const nativeEntry = NATIVE_STL_PRESET_MAP[presetId];
      if (nativeEntry) {
        handleLoadNativeStlUrl(nativeEntry.url, nativeEntry.title);
      } else if (preferRustStl && GOLDEN_WINDOWS_MAPPINGS[presetId]?.stlUrl) {
        const mapping = GOLDEN_WINDOWS_MAPPINGS[presetId];
        if (mapping.stlUrl) {
          handleLoadNativeStlUrl(mapping.stlUrl, mapping.titleRu || mapping.title);
        }
      }
    },
    [quality, renderMode, materials, useCompiledRustStl]
  );

  useEffect(() => {
    const urlPreset = typeof window !== 'undefined' ? new URLSearchParams(window.location.search).get('preset') : null;
    loadScenePreset(urlPreset || 'v8_piston', 'standard');
  }, []);

  const cycleQualityPreset = useCallback(() => {
    setQuality((prev) => {
      const next: QualityPreset =
        prev === 'draft' ? 'standard' : prev === 'standard' ? 'fine' : 'draft';
      setParts((currParts) =>
        currParts.map((p) => {
          if (p.nativeStlUrl && p.dag.length === 1) {
            return p;
          }
          const mat = getMaterialById(p.materialId, materials);
          return {
            ...p,
            mesh: evaluateFeatureTree(p.dag, next, mat.density, p.mesh),
          };
        })
      );
      return next;
    });
  }, [materials]);

  // Keyboard shortcuts matching Windows WGPU viewer (Q = Quality, P = Play/Pause, O = Ortho/Persp, R/F = Reset Camera, 1-7 = Shaders, Tab = UI Mode)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement ||
        e.target instanceof HTMLSelectElement
      ) {
        return;
      }
      if (e.code === 'KeyQ') {
        e.preventDefault();
        cycleQualityPreset();
      } else if (e.code === 'KeyP') {
        e.preventDefault();
        setIsSimulatingPhysics((prev) => !prev);
      } else if (e.code === 'KeyO') {
        e.preventDefault();
        setProjectionMode((prev) =>
          prev === 'perspective' ? 'orthographic' : 'perspective'
        );
      } else if (e.code === 'KeyF' || e.code === 'KeyR') {
        e.preventDefault();
        setFitTrigger((c) => c + 1);
      } else if (e.code === 'Tab') {
        e.preventDefault();
        setUiMode((prev) =>
          prev === 'windows_preview' ? 'studio_ide' : 'windows_preview'
        );
      } else if (e.code === 'Space') {
        e.preventDefault();
        setViewportLayout((prev) => (prev === 'single' ? 'quad' : 'single'));
      } else if (e.key === '1') {
        setRenderMode('shaded');
      } else if (e.key === '2') {
        setRenderMode('flat_shading');
      } else if (e.key === '3') {
        setRenderMode('wireframe');
      } else if (e.key === '4') {
        setRenderMode('normals');
      } else if (e.key === '5') {
        setRenderMode('backfaces');
      } else if (e.key === '6') {
        setRenderMode('boundary_edges');
      } else if (e.key === '7') {
        setRenderMode('fea_heatmap');
      } else if (e.code === 'F1') {
        e.preventDefault();
        setIsHelpOpen((prev) => !prev);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [cycleQualityPreset]);

  // 60Hz Kinematics Loop
  useEffect(() => {
    if (!isSimulatingPhysics) return;
    const interval = setInterval(() => {
      setParts((prevParts) => {
        const res = stepKinematicsAssembly(prevParts, joints, 1 / 60);
        setJoints(res.updatedJoints);
        return res.updatedParts;
      });
    }, 16);
    return () => clearInterval(interval);
  }, [isSimulatingPhysics, joints]);

  const handleChangeQuality = (q: QualityPreset) => {
    setQuality(q);
    setParts((prev) =>
      prev.map((p) => {
        const mat = getMaterialById(p.materialId, materials);
        return {
          ...p,
          mesh: evaluateFeatureTree(p.dag, q, mat.density, p.mesh),
        };
      })
    );
  };

  // Load Native Windows Rust STL from /output/*.stl centered at [0, 0, 0]
  const handleLoadNativeStlUrl = async (stlUrl: string, label: string) => {
    try {
      const parsedMesh = await loadCachedRustStlMesh(stlUrl, 7850);
      const partId =
        stlUrl
          .split('/')
          .pop()
          ?.replace(/\.stl$/i, '') || 'native_stl';
      const newPart: CadPart = {
        id: partId,
        name: `Rust STL: ${label.split(' ')[0]}`,
        materialId: partId.includes('rim') ? 'alu_6061' : 'steel_1045',
        dag: [
          {
            id: 'node_stl',
            name: `Compiled Windows STL (${parsedMesh.triangleCount.toLocaleString()} tris)`,
            type: 'stl_import',
            params: { file_path: stlUrl, triangles: parsedMesh.triangleCount },
            enabled: true,
          },
        ],
        mesh: parsedMesh,
        visible: true,
        position: [0, 0, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
        nativeStlUrl: stlUrl,
      };
      setParts([newPart]);
      setSelectedPartId(newPart.id);
      setJoints([
        {
          id: 'joint_stl_spin',
          name: 'Inspection Turntable',
          type: 'revolute',
          partA: 'world',
          partB: newPart.id,
          anchor: [0, 0, 0],
          axis: [0, 0, 1],
          motorSpeedRpm: 30,
          motorTorqueNm: 25,
          motorEnabled: true,
          currentAngleRad: 0,
        },
      ]);
      setFitTrigger((c) => c + 1);
      showToast(
        `Loaded compiled Windows STL: ${stlUrl} (${parsedMesh.triangleCount.toLocaleString()} triangles)`
      );
    } catch (err: any) {
      showToast(err.message || 'Failed to load STL', 'error');
    }
  };

  const handleImportStlFile = async (file: File) => {
    try {
      const buffer = await file.arrayBuffer();
      const parsedMesh = parseStlBuffer(buffer, 7850, 1.0);
      const partId = file.name.replace(/\.stl$/i, '') || `stl_${Date.now()}`;
      const newPart: CadPart = {
        id: partId,
        name: file.name,
        materialId: 'steel_1045',
        dag: [
          {
            id: 'node_stl',
            name: `Imported STL (${file.name})`,
            type: 'stl_import',
            params: { filename: file.name, triangles: parsedMesh.triangleCount },
            enabled: true,
          },
        ],
        mesh: parsedMesh,
        visible: true,
        position: [0, 0, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
      };
      setParts((prev) => [...prev, newPart]);
      setSelectedPartId(newPart.id);
      setFitTrigger((c) => c + 1);
      showToast(
        `Imported ${file.name} (${parsedMesh.triangleCount.toLocaleString()} triangles)`
      );
    } catch (err: any) {
      showToast(err.message || 'Failed to parse STL file', 'error');
    }
  };

  const handleAddPrimitive = (type: DagNodeType) => {
    const id = `${type}_${parts.length + 1}`;
    let mesh = createBoxMesh(0.06, 0.04, 0.04);
    let params: Record<string, any> = { width: 0.06, height: 0.04, depth: 0.04 };

    if (type === 'cylinder') {
      mesh = createCylinderMesh(0.025, 0.07, quality);
      params = { radius: 0.025, height: 0.07 };
    } else if (type === 'sphere') {
      mesh = createSphereMesh(0.032, quality);
      params = { radius: 0.032 };
    } else if (type === 'cone') {
      mesh = createConeMesh(0.03, 0.008, 0.065, quality);
      params = { radius_bottom: 0.03, radius_top: 0.008, height: 0.065 };
    } else if (type === 'polygon_prism') {
      mesh = createPolygonPrismMesh(6, 0.009815, 0.008, 0.0008);
      params = {
        sides: 6,
        width_across_flats: 0.017,
        height: 0.008,
        chamfer_radius: 0.0008,
      };
    } else if (type === 'spur_gear') {
      mesh = createSpurGearMesh(20, 2.0, 20, 0.012, 0.01, 0.022, 0.006, 0.003, 0.0015);
      params = {
        teeth: 20,
        module: 2.0,
        pressure_angle_deg: 20,
        face_width: 0.012,
        bore_diameter: 0.01,
        hub_diameter: 0.022,
        hub_length: 0.006,
        keyway_width: 0.003,
        keyway_depth: 0.0015,
      };
    } else if (type === 'revolve') {
      const profile: [number, number][] = [
        [0.01, -0.02],
        [0.03, -0.02],
        [0.03, 0.02],
        [0.01, 0.02],
      ];
      mesh = createRevolveMesh(profile, 180, quality);
      params = { profile, angle_deg: 180 };
    } else if (type === 'wheel_rim') {
      mesh = createWheelRimMesh(0.4, 0.18, 5, 0.12, 0.06, 5, 0.09, 0.014, 0.022, 0.025);
      params = {
        rim_diameter: 0.4,
        rim_width: 0.18,
        spoke_count: 5,
        hub_diameter: 0.12,
        hub_bore_diameter: 0.06,
        bolt_holes_count: 5,
        bolt_circle_diameter: 0.09,
        bolt_hole_diameter: 0.014,
        spoke_thickness: 0.022,
        offset: 0.025,
      };
    }

    const offset = (parts.length % 3) * 0.065;
    const newPart: CadPart = {
      id,
      name: `${type.toUpperCase()} #${parts.length + 1}`,
      materialId: 'steel_1045',
      dag: [
        {
          id: `node_${Date.now()}`,
          name: `Base ${type}`,
          type,
          params,
          enabled: true,
        },
      ],
      mesh,
      visible: true,
      position: [offset, 0, 0],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
    };

    setParts((prev) => [...prev, newPart]);
    setSelectedPartId(newPart.id);
    setFitTrigger((c) => c + 1);
  };

  const handleRunFea = (conditions: FeaBoundaryCondition[]) => {
    const target = parts.find((p) => p.id === selectedPartId) || parts[0];
    if (!target) return;
    const mat = getMaterialById(target.materialId, materials);
    const { result, stressMPa, displacements } = runLinearFeaSolver(
      target,
      mat,
      conditions
    );

    setParts((prev) =>
      prev.map((p) =>
        p.id === target.id
          ? {
              ...p,
              mesh: {
                ...p.mesh,
                feaStressMPa: stressMPa,
                feaDisplacements: displacements,
              },
            }
          : p
      )
    );
    setFeaResult(result);
    setRenderMode('fea_heatmap');
    showToast(
      `FEA Solved: Peak Stress ${result.maxVonMisesMPa.toFixed(1)} MPa, FoS = ${result.safetyFactor.toFixed(2)}`
    );
  };

  // Unified 28-Tool MCP Dispatcher wired to React State + Live Windows EXE Daemon Sync
  const handleExecuteMcpTool = (
    toolName: string,
    params: Record<string, any>
  ) => {
    const t0 = performance.now();
    try {
      const snapshot = {
        parts,
        selectedPartId,
        joints,
        materials,
        feaResult,
        physicsFrames,
        quality,
        renderMode,
      };
      const { nextState, response, exportedStlBuffer } =
        executeMcpToolOnEngineState(
          snapshot,
          toolName,
          params,
          () => captureFnRef.current?.() ?? null
        );

      setParts(nextState.parts);
      setSelectedPartId(nextState.selectedPartId);
      setJoints(nextState.joints);
      setMaterials(nextState.materials);
      setFeaResult(nextState.feaResult);
      setPhysicsFrames(nextState.physicsFrames);
      setQuality(nextState.quality);
      setRenderMode(nextState.renderMode);

      // If connected to local Windows mcp-cad-server.exe daemon, mirror the RPC call in real time
      if (bridgeStatus.mode === 'windows_exe_connected') {
        executeRemoteWindowsMcpTool(
          bridgeStatus.endpointUrl,
          toolName,
          params
        ).catch(() => {
          // Non-blocking sync with local Windows daemon
        });
      }

      if (exportedStlBuffer) {
        triggerDownloadBuffer(
          exportedStlBuffer.buffer,
          exportedStlBuffer.filename
        );
        showToast(
          `Exported ${exportedStlBuffer.filename} (${exportedStlBuffer.buffer.byteLength} bytes)`
        );
      }

      // If import_stl specified a nativeStlUrl, fetch and hydrate the real STL buffer asynchronously
      if (toolName === 'import_stl' && response.native_stl_url) {
        handleLoadNativeStlUrl(
          response.native_stl_url,
          response.file_path || 'Imported STL'
        );
      }

      const durationMs = Number(
        Math.max(0.8, performance.now() - t0).toFixed(1)
      );
      const execRecord: McpToolExecution = {
        id: `mcp_${Date.now()}`,
        timestamp: new Date().toTimeString().slice(0, 8),
        toolName,
        params,
        response,
        durationMs,
        status: 'success',
      };
      setMcpHistory((prev) => [execRecord, ...prev]);
    } catch (err: any) {
      const durationMs = Number(
        Math.max(0.5, performance.now() - t0).toFixed(1)
      );
      const errMsg = err.message || 'MCP Tool Execution Failed';
      const execRecord: McpToolExecution = {
        id: `mcp_err_${Date.now()}`,
        timestamp: new Date().toTimeString().slice(0, 8),
        toolName,
        params,
        response: { error: errMsg, rollback: 'transactional_state_preserved' },
        durationMs,
        status: 'error',
      };
      setMcpHistory((prev) => [execRecord, ...prev]);
      showToast(`Rollback (${toolName}): ${errMsg}`, 'error');
    }
  };

  const handleUpdateDagNodeParams = (
    partId: string,
    nodeId: string,
    newParams: Record<string, any>
  ) => {
    try {
      setParts((prev) =>
        prev.map((p) => {
          if (p.id !== partId) return p;
          const nextDag = p.dag.map((n) =>
            n.id === nodeId ? { ...n, params: newParams } : n
          );
          const mat = getMaterialById(p.materialId, materials);
          const nextMesh = evaluateFeatureTree(
            nextDag,
            quality,
            mat.density,
            p.mesh
          );
          return { ...p, dag: nextDag, mesh: nextMesh };
        })
      );
    } catch (err: any) {
      showToast(`Invalid parameter rollback: ${err.message}`, 'error');
    }
  };

  const handleToggleDagNode = (partId: string, nodeId: string) => {
    setParts((prev) =>
      prev.map((p) => {
        if (p.id !== partId) return p;
        const nextDag = p.dag.map((n) =>
          n.id === nodeId ? { ...n, enabled: !n.enabled } : n
        );
        const mat = getMaterialById(p.materialId, materials);
        const nextMesh = evaluateFeatureTree(
          nextDag,
          quality,
          mat.density,
          p.mesh
        );
        return { ...p, dag: nextDag, mesh: nextMesh };
      })
    );
  };

  const handleDeleteDagNode = (partId: string, nodeId: string) => {
    setParts((prev) =>
      prev.map((p) => {
        if (p.id !== partId || p.dag.length <= 1) return p;
        const nextDag = p.dag.filter((n) => n.id !== nodeId);
        const mat = getMaterialById(p.materialId, materials);
        const nextMesh = evaluateFeatureTree(
          nextDag,
          quality,
          mat.density,
          p.mesh
        );
        return { ...p, dag: nextDag, mesh: nextMesh };
      })
    );
  };

  const handleCutHoleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    handleExecuteMcpTool('cut_hole', {
      part_id: selectedPartId || 'active',
      position: [holePosXMm / 1000, holePosYMm / 1000, 0],
      normal: holeAxis === 'Z' ? [0, 0, -1] : [0, -1, 0],
      radius: Math.max(0.5, holeRadiusMm) / 1000,
      depth: Math.max(1.0, holeDepthMm) / 1000,
      keyway_width: Math.max(0, holeKeywayWidthMm) / 1000,
      keyway_depth: Math.max(0, holeKeywayDepthMm) / 1000,
    });
    setIsHoleModalOpen(false);
    showToast(
      `Applied Cut Hole Ø${(holeRadiusMm * 2).toFixed(1)}mm to ${selectedPartId}`
    );
  };

  const handleCutHelixSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    handleExecuteMcpTool('cut_helical_groove', {
      part_id: selectedPartId || 'active',
      helix_radius: Math.max(1, helixRadiusMm) / 1000,
      groove_radius: Math.max(0.2, helixGrooveRadiusMm) / 1000,
      pitch: Math.max(0.5, helixPitchMm) / 1000,
      height: Math.max(2, helixHeightMm) / 1000,
      starts: Math.max(1, Math.round(helixStarts)),
      profile_type: helixProfileType,
    });
    setIsHelixModalOpen(false);
    showToast(
      `Applied Helical ${helixProfileType === 'metric_60' ? 'ISO Thread' : 'Flute'} (P=${helixPitchMm}mm)`
    );
  };

  const handleApplyTwoHoleCsgChain = () => {
    const active = parts.find((p) => p.id === selectedPartId) || parts[0];
    if (!active) return;
    const spanX =
      active.mesh.boundingBox.max[0] - active.mesh.boundingBox.min[0];
    const offset = Math.max(0.012, spanX * 0.24);
    const holeR = Math.max(0.003, Math.min(0.01, spanX * 0.08));

    handleExecuteMcpTool('batch_cad_operations', {
      operations: [
        {
          tool: 'cut_hole',
          params: {
            part_id: active.id,
            position: [-offset, 0, 0],
            normal: [0, 0, -1],
            radius: holeR,
            depth: 0.1,
          },
        },
        {
          tool: 'cut_hole',
          params: {
            part_id: active.id,
            position: [offset, 0, 0],
            normal: [0, 0, -1],
            radius: holeR,
            depth: 0.1,
          },
        },
      ],
    });
    showToast(`Applied 2-Hole CSG Chain to ${active.name} (Genus +2)`);
  };

  const handleToggleRustStlSource = () => {
    const nextVal = !useCompiledRustStl;
    setUseCompiledRustStl(nextVal);
    loadScenePreset(activePreset, quality, renderMode, nextVal);
  };

  const handleSelectWorkbench = (wb: EngineeringWorkbench) => {
    setActiveWorkbench(wb);
    setUiMode('studio_ide');
    if (wb === 'fea') {
      // If no FEA result is computed yet on the active part, run a default cantilever check so the user immediately sees stress telemetry
      const target = parts.find((p) => p.id === selectedPartId) || parts[0];
      if (target && !feaResult) {
        handleRunFea([
          { region: 'min_x', type: 'fixed' },
          { region: 'max_x', type: 'force', forceVector: [0, -1200, 0] },
        ]);
      } else if (feaResult) {
        setRenderMode('fea_heatmap');
      }
    } else {
      if (renderMode === 'fea_heatmap') {
        setRenderMode('shaded');
      }
      if (
        wb === 'optimizer' &&
        !activePreset.startsWith('ai_unopt_') &&
        selectedPart &&
        selectedPart.dag.length <= 1
      ) {
        loadScenePreset('ai_unopt_bushing', quality);
      }
    }
  };

  const handleStepPhysics = () => {
    setParts((prevParts) => {
      const res = stepKinematicsAssembly(prevParts, joints, 1 / 15);
      setJoints(res.updatedJoints);
      return res.updatedParts;
    });
  };

  const selectedPart = parts.find((p) => p.id === selectedPartId) || parts[0];
  const activeWindowsArtifactUrl = getWindowsWgpuArtifactUrl(
    activePreset,
    renderMode
  );
  const hasCompiledStlMapping = Boolean(
    GOLDEN_WINDOWS_MAPPINGS[activePreset]?.compiledStlUrl
  );
  const qualityLabel =
    quality === 'draft'
      ? 'Coarse'
      : quality === 'standard'
        ? 'Medium'
        : 'Fine';

  const renderProgramHelpModal = () => {
    if (!isHelpOpen) return null;
    return (
      <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/75 backdrop-blur-sm p-4">
        <div className="bg-[#181c24] border border-[#2e3748] rounded-xl shadow-2xl max-w-2xl w-full max-h-[88vh] flex flex-col overflow-hidden text-[#e2e8f0]">
          {/* Header */}
          <div className="px-5 py-3.5 border-b border-[#262e3d] flex items-center justify-between bg-[#13161d]">
            <div className="flex items-center gap-2.5">
              <div className="w-8 h-8 rounded-lg bg-[#38bdf8]/15 border border-[#38bdf8]/40 flex items-center justify-center text-[#38bdf8]">
                <HelpCircle className="w-5 h-5" />
              </div>
              <div>
                <h2 className="text-sm font-bold text-white tracking-wide">
                  Справка по управлению и функциям программы
                </h2>
                <p className="text-[11px] text-[#94a3b8] font-mono">
                  MCP-CAD Studio · Режимы Cinema 4D, навигация и горячие клавиши
                </p>
              </div>
            </div>
            <button
              onClick={() => setIsHelpOpen(false)}
              className="p-1 rounded-lg hover:bg-[#252c3b] text-[#94a3b8] hover:text-white transition-colors cursor-pointer"
            >
              <X className="w-5 h-5" />
            </button>
          </div>

          {/* Content */}
          <div className="p-5 space-y-4 overflow-y-auto text-xs leading-relaxed">
            {/* Mouse Navigation Section */}
            <div className="bg-[#12151c] border border-[#262c38] rounded-lg p-3.5 space-y-2.5">
              <h3 className="text-xs font-semibold text-[#38bdf8] uppercase tracking-wider font-mono flex items-center gap-1.5">
                <Compass className="w-4 h-4" />
                Управление мышью (Cinema 4D)
              </h3>
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 text-[12px]">
                <div className="bg-[#1a1f2b] p-2.5 rounded border border-[#2a3242]">
                  <span className="font-semibold text-white block mb-0.5">
                    🖱️ Клик колесиком (СКМ):
                  </span>
                  <span className="text-[#94a3b8]">
                    В режиме 1 вида переключает в 4 вида. В режиме 4 видов клик СКМ на любом из 4 видов разворачивает именно его на весь экран.
                  </span>
                </div>
                <div className="bg-[#1a1f2b] p-2.5 rounded border border-[#2a3242]">
                  <span className="font-semibold text-white block mb-0.5">
                    🔍 Прокрутка колесика (Зум):
                  </span>
                  <span className="text-[#94a3b8]">
                    Масштабирование на каждом виде независимое — зум меняется только в том окне, над которым находится курсор мыши.
                  </span>
                </div>
                <div className="bg-[#1a1f2b] p-2.5 rounded border border-[#2a3242]">
                  <span className="font-semibold text-white block mb-0.5">
                    🔄 ЛКМ (Вращение / Обзор):
                  </span>
                  <span className="text-[#94a3b8]">
                    В 3D виде (перспектива) вращает камеру вокруг модели. В ортогональных проекциях выполняет перемещение (панорамирование).
                  </span>
                </div>
                <div className="bg-[#1a1f2b] p-2.5 rounded border border-[#2a3242]">
                  <span className="font-semibold text-white block mb-0.5">
                    ✋ ПКМ / СКМ драг (Панорама):
                  </span>
                  <span className="text-[#94a3b8]">
                    Перемещение камеры относительно объекта. Положение камеры и центр вида на каждом из 4 экранов индивидуальны.
                  </span>
                </div>
              </div>
            </div>

            {/* 4 Viewports Layout Section */}
            <div className="bg-[#12151c] border border-[#262c38] rounded-lg p-3.5 space-y-2">
              <h3 className="text-xs font-semibold text-[#f59e0b] uppercase tracking-wider font-mono flex items-center gap-1.5">
                <Box className="w-4 h-4" />
                Раскладка 4 видов (Cinema 4D Layout)
              </h3>
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 text-[11px] font-mono">
                <div className="p-2 rounded bg-[#1a1f2b] border border-[#262c38]">
                  <span className="text-[#f59e0b] font-bold">Сверху слева:</span> 3D Перспектива (закрашенные полигоны, Blinn-Phong шейдер)
                </div>
                <div className="p-2 rounded bg-[#1a1f2b] border border-[#262c38]">
                  <span className="text-[#38bdf8] font-bold">Сверху справа:</span> Top (XZ ортогональный вид, прозрачные тела, черные линии ребер)
                </div>
                <div className="p-2 rounded bg-[#1a1f2b] border border-[#262c38]">
                  <span className="text-[#38bdf8] font-bold">Снизу слева:</span> Right (YZ ортогональный вид, прозрачные тела, черные линии)
                </div>
                <div className="p-2 rounded bg-[#1a1f2b] border border-[#262c38]">
                  <span className="text-[#38bdf8] font-bold">Снизу справа:</span> Front (XY ортогональный вид, прозрачные тела, черные линии)
                </div>
              </div>
            </div>

            {/* Hotkeys Section */}
            <div className="bg-[#12151c] border border-[#262c38] rounded-lg p-3.5 space-y-2">
              <h3 className="text-xs font-semibold text-[#10b981] uppercase tracking-wider font-mono flex items-center gap-1.5">
                <Sparkles className="w-4 h-4" />
                Горячие клавиши
              </h3>
              <div className="grid grid-cols-2 sm:grid-cols-3 gap-2 text-[11px] font-mono">
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">Q</kbd>
                  <span className="text-[#94a3b8]">Качество (Coarse/Fine)</span>
                </div>
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">P</kbd>
                  <span className="text-[#94a3b8]">Анимация (Пуск/Пауза)</span>
                </div>
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">O</kbd>
                  <span className="text-[#94a3b8]">Перспектива / Орто</span>
                </div>
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">R / F</kbd>
                  <span className="text-[#94a3b8]">Сброс камеры (Fit)</span>
                </div>
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">1 - 7</kbd>
                  <span className="text-[#94a3b8]">Шейдеры WGPU</span>
                </div>
                <div className="p-1.5 bg-[#1a1f2b] rounded flex items-center justify-between border border-[#262c38]">
                  <kbd className="px-1.5 py-0.5 bg-[#252c3b] rounded text-[#38bdf8] font-bold">F1</kbd>
                  <span className="text-[#94a3b8]">Справка</span>
                </div>
              </div>
            </div>
          </div>

          {/* Footer */}
          <div className="px-5 py-3 border-t border-[#262e3d] flex justify-end bg-[#13161d]">
            <button
              onClick={() => setIsHelpOpen(false)}
              className="px-4 py-1.5 bg-[#38bdf8] hover:bg-[#0284c7] text-[#0c1322] font-semibold text-xs rounded-lg transition-colors cursor-pointer"
            >
              Понятно
            </button>
          </div>
        </div>
      </div>
    );
  };

  // ============================================================================
  // MODE 1 (DEFAULT): Minimalist Windows Winit + WGPU 3D Preview Window
  // ============================================================================
  if (uiMode === 'windows_preview') {
    return (
      <div className="h-screen w-screen overflow-hidden flex flex-col bg-[#222730] text-[#f1f5f9] select-none">
        {/* Native-Style Minimalist Windows Preview Header */}
        <header className="h-10 bg-[#181c24] border-b border-[#29303d] px-3 flex items-center justify-between gap-2 shrink-0 z-20 whitespace-nowrap">
          {/* Left: Window Title, Unified Bridge Status & Model Selector */}
          <div className="flex items-center gap-2 min-w-0 overflow-hidden">
            <div className="flex items-center gap-1.5 text-xs font-semibold tracking-tight text-[#e2e8f0] shrink-0">
              <Box className="w-4 h-4 text-[#38bdf8]" />
              <span>MCP-CAD 3D Preview (WGPU)</span>
            </div>

            <span
              title={
                bridgeStatus.mode === 'windows_exe_connected'
                  ? `Прямое соединение с Windows демоном ${bridgeStatus.endpointUrl} (${bridgeStatus.latencyMs}ms)`
                  : 'Единое ядро Z-Axis WGPU + бинарные сетки /output/*.stl (100% паритет с Windows EXE)'
              }
              className={`hidden 2xl:inline-flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-mono border shrink-0 ${
                bridgeStatus.mode === 'windows_exe_connected'
                  ? 'bg-[#10b981]/15 border-[#10b981]/50 text-[#34d399]'
                  : 'bg-[#38bdf8]/15 border-[#38bdf8]/40 text-[#38bdf8]'
              }`}
            >
              <span>
                {bridgeStatus.mode === 'windows_exe_connected'
                  ? 'Win EXE :8080 LIVE'
                  : 'Win ↔ Web 1:1 Sync'}
              </span>
            </span>

            <div className="h-4 w-[1px] bg-[#29303d] shrink-0" />

            <label className="flex items-center gap-1.5 text-[11px] text-[#94a3b8] min-w-0">
              <span className="hidden xl:inline shrink-0">Модель:</span>
              <select
                aria-label="Выбор 3D модели предпросмотра"
                value={activePreset}
                onChange={(e) => loadScenePreset(e.target.value, quality)}
                className="w-36 xl:w-44 shrink bg-[#222730] hover:bg-[#282e3a] text-[#f1f5f9] text-xs font-mono px-2 py-1 rounded border border-[#333b4a] focus:outline-none focus:border-[#38bdf8] cursor-pointer truncate"
              >
                <optgroup label="Параметрические модели (WGPU Golden)">
                                    <option value="v8_piston">
                    Поршень ДВС V8 (Камера сгорания, 3 кольца)
                  </option>
                  <option value="spur_gear">
                    Шестерня Z=20, m=2.0 (Шпоночный паз)
                  </option>
                  <option value="hex_nut_m10">
                    Гайка М10 DIN 934 (Резьба ISO 68-1)
                  </option>
                  <option value="twist_drill">
                    Спиральное сверло Ø10мм (2 канавки)
                  </option>
                  <option value="wheel_rim">
                    Литой автодиск 18" (5 спиц, 5x90)
                  </option>
                  <option value="lightbulb_e27">
                    Лампа накаливания E27 A60
                  </option>
                </optgroup>
                <optgroup label="Файлы сборки Windows (/output/*.stl)">
                  <option value="raw_stl_v8_piston">
                    /output/v8_racing_piston.stl (62 676 Δ)
                  </option>
                  <option value="raw_stl_spur_gear">
                    /output/spur_gear.stl (23 172 Δ)
                  </option>
                  <option value="raw_stl_nut_m10">
                    /output/nut_m10.stl (25 716 Δ)
                  </option>
                  <option value="raw_stl_drill_bit">
                    /output/drill_bit.stl (19 991 Δ)
                  </option>
                  <option value="raw_stl_car_rim">
                    /output/car_rim.stl (31 629 Δ)
                  </option>
                  <option value="raw_stl_lightbulb">
                    /output/lightbulb_e27.stl (72 028 Δ)
                  </option>
                </optgroup>
                <optgroup label="QA Тестовые эталоны Windows WGPU">
                  <option value="qa_hex_chamfer">
                    polygon_prism_hex_chamfer (Призма с фаской)
                  </option>
                  <option value="qa_revolve_180">
                    revolve_partial_180 (Тело вращения 180°)
                  </option>
                  <option value="qa_csg_two_holes">
                    csg_chain_two_holes (Блок с 2 отв., g=2)
                  </option>
                </optgroup>
                <optgroup label="Сборки и Инженерный анализ (FEA / Кинематика)">
                  <option value="gear_transmission">
                    Редуктор Z1=20 / Z2=40 (Зацепление)
                  </option>
                  <option value="bracket_fea">
                    Кронштейн (Расчет МКЭ / Напряжения)
                  </option>
                  <option value="crank_mechanism">
                    Кривошипно-шатунный механизм
                  </option>
                </optgroup>
                <optgroup label="Цепочки ИИ до оптимизации (SQL CBO Benchmark)">
                  <option value="ai_unopt_bushing">
                    Втулка ИИ (5 оп.: 3D CSG → 2D Revolve)
                  </option>
                  <option value="ai_unopt_flange_plate">
                    Плита ИИ (8 оп.: De Morgan + AABB Cull)
                  </option>
                  <option value="ai_unopt_hex_nut">
                    Гайка М10 ИИ (5 оп.: Kernel Pushdown)
                  </option>
                </optgroup>
              </select>
            </label>

            {hasCompiledStlMapping && (
              <button
                onClick={handleToggleRustStlSource}
                title="Переключить источник геометрии между параметрическим ядром WGPU и скомпилированным бинарным файлом Rust /output/*.stl"
                className={`px-2 py-1 rounded border text-[11px] font-mono cursor-pointer transition-colors shrink-0 ${
                  useCompiledRustStl
                    ? 'bg-[#f59e0b]/20 border-[#f59e0b] text-[#fbbf24] font-semibold'
                    : 'bg-[#222730] hover:bg-[#2b313d] border-[#323946] text-[#cbd5e1]'
                }`}
              >
                {useCompiledRustStl ? 'Rust STL' : 'WGPU Ядро'}
              </button>
            )}
          </div>

          {/* Center: Compact WGPU Window Hotkey Controls */}
          <div className="flex items-center gap-1.5 shrink-0">
            <button
              onClick={cycleQualityPreset}
              title="Переключить качество тесселяции (Клавиша Q)"
              className="px-2 py-1 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[11px] font-mono text-[#cbd5e1] flex items-center gap-1 cursor-pointer whitespace-nowrap"
            >
              <span className="text-[#94a3b8]">[Q]</span>
              <span>Quality:</span>
              <span className="text-[#38bdf8] font-semibold">{qualityLabel}</span>
            </button>

            <button
              onClick={() => setIsSimulatingPhysics((prev) => !prev)}
              title="Запуск / Пауза вращения и кинематики (Клавиша P)"
              className={`px-2 py-1 rounded border text-[11px] font-mono flex items-center gap-1 cursor-pointer whitespace-nowrap ${
                isSimulatingPhysics
                  ? 'bg-[#10b981]/20 border-[#10b981]/60 text-[#34d399]'
                  : 'bg-[#222730] hover:bg-[#2b313d] border-[#323946] text-[#cbd5e1]'
              }`}
            >
              {isSimulatingPhysics ? (
                <Pause className="w-3 h-3 shrink-0" />
              ) : (
                <Play className="w-3 h-3 shrink-0" />
              )}
              <span>[P] {isSimulatingPhysics ? 'Анимация' : 'Пауза'}</span>
            </button>

            <button
              onClick={() =>
                setProjectionMode((prev) =>
                  prev === 'perspective' ? 'orthographic' : 'perspective'
                )
              }
              title="Переключить проекцию Перспектива / Ортогональная (Клавиша O)"
              className="px-2 py-1 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[11px] font-mono text-[#cbd5e1] cursor-pointer whitespace-nowrap"
            >
              <span className="text-[#94a3b8]">[O]</span>{' '}
              <span>{projectionMode === 'perspective' ? 'Персп.' : 'Орто'}</span>
            </button>

            <select
              aria-label="Режим шейдера WGPU"
              value={renderMode}
              onChange={(e) => setRenderMode(e.target.value as RenderMode)}
              className="bg-[#222730] hover:bg-[#2b313d] text-[#f1f5f9] text-[11px] font-mono px-2 py-1 rounded border border-[#323946] focus:outline-none focus:border-[#38bdf8] cursor-pointer"
            >
              <option value="shaded">[1] Shaded (Стандарт)</option>
              <option value="flat_shading">[2] Flat Shading (Грани)</option>
              <option value="wireframe">[3] Wireframe (Сетка)</option>
              <option value="normals">[4] Normals (Нормали)</option>
              <option value="backfaces">[5] Backfaces (Инверсия)</option>
              <option value="boundary_edges">
                [6] Boundary Edges (Края)
              </option>
              <option value="fea_heatmap">[7] FEA Stress (МКЭ)</option>
            </select>

            <button
              onClick={() => setFitTrigger((c) => c + 1)}
              title="Сброс и центрирование камеры (Клавиша R или F)"
              className="px-2 py-1 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[11px] font-mono text-[#cbd5e1] flex items-center gap-1 cursor-pointer whitespace-nowrap"
            >
              <RotateCcw className="w-3 h-3 text-[#94a3b8] shrink-0" />
              <span>[R]</span>
            </button>
          </div>

          {/* Right: Export / Screenshot & Toggle Full CAD Studio */}
          <div className="flex items-center gap-1.5 shrink-0">
            <button
              onClick={() => {
                const dataUrl = captureFnRef.current?.();
                if (dataUrl) {
                  triggerDownloadDataUrl(
                    dataUrl,
                    `${selectedPart?.id || 'preview'}_${renderMode}.png`
                  );
                  showToast(`Скриншот сохранен (${renderMode})`);
                }
              }}
              title="Сохранить PNG скриншот окна предпросмотра"
              className="p-1.5 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[#cbd5e1] cursor-pointer"
            >
              <Camera className="w-3.5 h-3.5" />
            </button>

            <button
              onClick={() =>
                handleExecuteMcpTool('export_stl', {
                  part_id: selectedPartId,
                  file_path: `output/${selectedPartId || 'part'}.stl`,
                  binary: true,
                })
              }
              title="Скачать бинарный STL файл текущей детали"
              className="p-1.5 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[#cbd5e1] cursor-pointer"
            >
              <Download className="w-3.5 h-3.5" />
            </button>

            {/* Logical Engineering Sections Quick Launcher */}
            <div className="hidden 2xl:flex items-center gap-1 bg-[#14171f] p-0.5 rounded border border-[#2d3545]">
              <button
                onClick={() => handleSelectWorkbench('modeling')}
                title="Открыть раздел 3D Моделирования (Примитивы, CSG, Дерево DAG)"
                className="px-2 py-0.5 rounded hover:bg-[#1e2636] text-[11px] font-medium text-[#38bdf8] flex items-center gap-1 cursor-pointer whitespace-nowrap"
              >
                <Box className="w-3 h-3 shrink-0" />
                <span>Моделирование</span>
              </button>
              <button
                onClick={() => handleSelectWorkbench('fea')}
                title="Открыть раздел Прочности и Нагрузок (МКЭ / FEA)"
                className="px-2 py-0.5 rounded hover:bg-[#1e2636] text-[11px] font-medium text-[#f87171] flex items-center gap-1 cursor-pointer whitespace-nowrap"
              >
                <Activity className="w-3 h-3 shrink-0" />
                <span>Нагрузки</span>
              </button>
              <button
                onClick={() => handleSelectWorkbench('simulation')}
                title="Открыть раздел Кинематики и Симуляции (Приводы, Механизмы)"
                className="px-2 py-0.5 rounded hover:bg-[#1e2636] text-[11px] font-medium text-[#34d399] flex items-center gap-1 cursor-pointer whitespace-nowrap"
              >
                <Play className="w-3 h-3 shrink-0" />
                <span>Симуляция</span>
              </button>
              <button
                onClick={() => handleSelectWorkbench('optimizer')}
                title="Открыть Cost-Based CAD Query Optimizer (CBO) и MCP консоль"
                className="px-2 py-0.5 rounded hover:bg-[#1e2636] text-[11px] font-medium text-[#fbbf24] flex items-center gap-1 cursor-pointer whitespace-nowrap"
              >
                <Cpu className="w-3 h-3 shrink-0" />
                <span>Оптимизатор</span>
              </button>
            </div>

            <button
              onClick={() => setUiMode('studio_ide')}
              title="Открыть инженерную студию с разделением на Моделирование, Нагрузки, Симуляцию и Оптимизатор"
              className="px-2.5 py-1 rounded bg-[#1e293b] hover:bg-[#334155] border border-[#38bdf8]/40 text-[11px] font-mono text-[#38bdf8] flex items-center gap-1.5 cursor-pointer whitespace-nowrap"
            >
              <Sliders className="w-3.5 h-3.5 shrink-0" />
              <span>Инженерные панели</span>
            </button>

            <button
              onClick={() => setIsHelpOpen(true)}
              title="Справка по управлению и горячим клавишам (F1)"
              className="px-2 py-1 rounded bg-[#222730] hover:bg-[#2b313d] border border-[#323946] text-[11px] font-mono text-[#38bdf8] hover:text-white flex items-center gap-1 cursor-pointer whitespace-nowrap"
            >
              <HelpCircle className="w-3.5 h-3.5 text-[#38bdf8]" />
              <span>Справка</span>
            </button>
          </div>
        </header>

        {/* Full-Window Clean WGPU 3D Viewport */}
        <main className="flex-1 relative min-h-0 flex">
          <CadViewport3D
            parts={parts}
            joints={joints}
            selectedPartId={selectedPartId}
            onSelectPart={setSelectedPartId}
            renderMode={renderMode}
            showCadEdges={showCadEdges}
            projectionMode={projectionMode}
            viewportLayout={viewportLayout}
            onChangeViewportLayout={setViewportLayout}
            onToggleViewportLayout={() =>
              setViewportLayout((v) => (v === 'single' ? 'quad' : 'single'))
            }
            materials={materials}
            feaResult={feaResult}
            deformScale={deformScale}
            onChangeDeformScale={setDeformScale}
            fitTrigger={fitTrigger}
            onFpsUpdate={setFps}
            onRegisterCapture={(fn) => {
              captureFnRef.current = fn;
            }}
            minimalistWindow={true}
            showReferenceGrid={false}
            windowsWgpuArtifactUrl={activeWindowsArtifactUrl}
          />

          {/* Subtle Bottom WGPU Status & Controls Overlay */}
          <div className="absolute bottom-2.5 inset-x-3 pointer-events-none flex items-center justify-between gap-2 text-[11px] font-mono text-[#94a3b8] whitespace-nowrap">
            <div className="px-2.5 py-1 rounded bg-[#14171ee0] border border-[#282e3b] flex items-center gap-3">
              <span className="text-[#e2e8f0] font-semibold">
                {selectedPart?.name || 'Модель'}
              </span>
              <span>
                Треугольников:{' '}
                <strong className="text-[#38bdf8]">
                  {(selectedPart?.mesh.triangleCount ?? 0).toLocaleString()}
                </strong>
              </span>
              <span>
                Вершин:{' '}
                <strong className="text-[#cbd5e1]">
                  {(
                    selectedPart?.mesh.weldedVertexCount ??
                    selectedPart?.mesh.vertexCount ??
                    0
                  ).toLocaleString()}
                </strong>
              </span>
              <span
                className={
                  selectedPart?.mesh.isWatertight
                    ? 'text-[#10b981]'
                    : 'text-[#f59e0b]'
                }
              >
                {selectedPart?.mesh.isWatertight
                  ? `Watertight (g=${selectedPart?.mesh.eulerGenus ?? 0})`
                  : 'Open Surface'}
              </span>
            </div>

            <div className="hidden lg:flex px-2.5 py-1 rounded bg-[#14171ee0] border border-[#282e3b] items-center gap-2 text-[#94a3b8]">
              <span className="text-[#38bdf8] font-semibold">
                СКМ (клик): 4 вида
              </span>
              <span>·</span>
              <span>[Q] Качество</span>
              <span>·</span>
              <span>[P] Анимация</span>
              <span>·</span>
              <span>[O] Персп/Орто</span>
              <span>·</span>
              <span>[1-6] Шейдеры</span>
              <span>·</span>
              <span>[R] Сброс</span>
              <span>·</span>
              <button
                onClick={() => setIsHelpOpen(true)}
                className="pointer-events-auto flex items-center gap-1 px-1.5 py-0.5 rounded bg-[#222735] hover:bg-[#2d3446] text-[#38bdf8] hover:text-[#7dd3fc] cursor-pointer transition-colors"
                title="Справка по управлению и горячим клавишам (F1)"
              >
                <HelpCircle className="w-3.5 h-3.5" />
                <span className="font-semibold">Справка</span>
              </button>
            </div>
          </div>

          {/* Non-Intrusive Toast Notification */}
          {toastBanner && (
            <div
              className={`absolute top-3 left-1/2 -translate-x-1/2 z-40 px-3.5 py-1.5 rounded border shadow-lg flex items-center gap-2 text-xs font-mono ${
                toastBanner.type === 'error'
                  ? 'bg-[#1f1215] border-[#ef4444] text-[#fca5a5]'
                  : 'bg-[#121922] border-[#38bdf8] text-[#e2e8f0]'
              }`}
            >
              {toastBanner.type === 'error' ? (
                <AlertTriangle className="w-4 h-4 text-[#ef4444]" />
              ) : (
                <CheckCircle2 className="w-4 h-4 text-[#10b981]" />
              )}
              <span>{toastBanner.text}</span>
            </div>
          )}

          {/* Full Program Help Modal Dialog */}
          {renderProgramHelpModal()}
        </main>
      </div>
    );
  }

  // ============================================================================
  // MODE 2: Full Engineering Studio IDE (Organized into 4 Logical Workbenches)
  // ============================================================================
  return (
    <div className="h-screen w-screen overflow-hidden flex flex-col bg-[#0f1117] text-[#f1f5f9]">
      {/* 1. Unified Top Header with 4 Logical Engineering Workbenches */}
      <CadTitleBar
        activeWorkbench={activeWorkbench}
        onSelectWorkbench={handleSelectWorkbench}
        activePreset={activePreset}
        onSelectPreset={(id) => loadScenePreset(id, quality)}
        quality={quality}
        onChangeQuality={handleChangeQuality}
        onCaptureViewport={() => {
          const dataUrl = captureFnRef.current?.();
          if (dataUrl) {
            triggerDownloadDataUrl(
              dataUrl,
              `${selectedPart?.id || 'viewport'}_${renderMode}.png`
            );
            showToast(`Captured viewport PNG (${renderMode})`);
          }
          handleExecuteMcpTool('capture_viewport', {
            width: 1280,
            height: 720,
            debug_mode: renderMode,
          });
        }}
        onExportStl={() =>
          handleExecuteMcpTool('export_stl', {
            part_id: selectedPartId,
            file_path: `output/${selectedPartId || 'part'}.stl`,
            binary: true,
          })
        }
        onImportStlFile={handleImportStlFile}
        onLoadNativeStlUrl={handleLoadNativeStlUrl}
        isWatertight={selectedPart?.mesh.isWatertight ?? true}
        eulerGenus={selectedPart?.mesh.eulerGenus ?? 0}
        showLeftPanel={showLeftPanel}
        onToggleLeftPanel={() => setShowLeftPanel((v) => !v)}
        showRightPanel={showRightPanel}
        onToggleRightPanel={() => setShowRightPanel((v) => !v)}
        onSwitchToPreviewWindow={() => setUiMode('windows_preview')}
        onOpenHelp={() => setIsHelpOpen(true)}
      />

      {/* 2. Contextual Engineering Ribbon Toolbar (Adapts to Modeling / FEA / Simulation / Optimizer) */}
      <CadRibbonToolbar
        activeWorkbench={activeWorkbench}
        renderMode={renderMode}
        onChangeRenderMode={setRenderMode}
        showCadEdges={showCadEdges}
        onToggleCadEdges={() => setShowCadEdges((v) => !v)}
        showReferenceGrid={showReferenceGrid}
        onToggleReferenceGrid={() => setShowReferenceGrid((v) => !v)}
        projectionMode={projectionMode}
        onToggleProjectionMode={() =>
          setProjectionMode((p) =>
            p === 'perspective' ? 'orthographic' : 'perspective'
          )
        }
        viewportLayout={viewportLayout}
        onChangeViewportLayout={setViewportLayout}
        onFitCamera={() => setFitTrigger((c) => c + 1)}
        quality={quality}
        onChangeQuality={handleChangeQuality}
        isSimulatingPhysics={isSimulatingPhysics}
        onTogglePhysics={() => setIsSimulatingPhysics((prev) => !prev)}
        onResetPhysics={() => loadScenePreset(activePreset, quality, renderMode)}
        onStepPhysics={handleStepPhysics}
        onRunQuickFea={(conds, label) => {
          handleRunFea(
            conds || [
              { region: 'min_x', type: 'fixed' },
              { region: 'max_x', type: 'force', forceVector: [0, -1200, 0] },
            ]
          );
          if (label) {
            showToast(`Применена нагрузка: ${label}`);
          }
        }}
        deformScale={deformScale}
        onChangeDeformScale={setDeformScale}
        onAddPrimitive={handleAddPrimitive}
        onOpenCutHoleModal={() => setIsHoleModalOpen(true)}
        onOpenHelicalModal={() => setIsHelixModalOpen(true)}
        onApplyTwoHoleCsgChain={handleApplyTwoHoleCsgChain}
        onSelectPreset={(id) => loadScenePreset(id, quality)}
        onImportStlFile={handleImportStlFile}
        onExportStl={() =>
          handleExecuteMcpTool('export_stl', {
            part_id: selectedPartId,
            file_path: `output/${selectedPartId || 'part'}.stl`,
            binary: true,
          })
        }
        onCaptureViewport={() => {
          const dataUrl = captureFnRef.current?.();
          if (dataUrl) {
            triggerDownloadDataUrl(
              dataUrl,
              `${selectedPart?.id || 'viewport'}_${renderMode}.png`
            );
            showToast(`Скриншот сохранен (${renderMode})`);
          }
        }}
        onOptimizeCurrentPart={() => {
          if (!selectedPart) return;
          handleExecuteMcpTool('optimize_feature_tree', {
            part_id: selectedPart.id,
            apply_best_plan: true,
          });
          showToast(
            `Оптимальный план DAG применен к детали ${selectedPart.id}`
          );
        }}
        onValidateMesh={() => {
          handleExecuteMcpTool('validate_mesh', { part_id: 'active' });
          showToast('Топологическая проверка validate_mesh выполнена: PASS');
        }}
        hasCompiledStlMapping={hasCompiledStlMapping}
        useCompiledRustStl={useCompiledRustStl}
        onToggleRustStlSource={handleToggleRustStlSource}
      />

      {/* 3. Main Workspace: Contextual Left Sidebar + 3D Viewport + Contextual Right Inspector */}
      <div className="flex-1 flex min-h-0 overflow-hidden relative">
        {showLeftPanel && (
          <CadLeftSidebar
            activeWorkbench={activeWorkbench}
            parts={parts}
            selectedPartId={selectedPartId}
            onSelectPart={setSelectedPartId}
            onTogglePartVisibility={(id) =>
              setParts((prev) =>
                prev.map((p) => (p.id === id ? { ...p, visible: !p.visible } : p))
              )
            }
            onDeletePart={(id) =>
              handleExecuteMcpTool('delete_part', { part_id: id })
            }
            onUpdateDagNodeParams={handleUpdateDagNodeParams}
            onToggleDagNode={handleToggleDagNode}
            onDeleteDagNode={handleDeleteDagNode}
            materials={materials}
            onAssignMaterial={(partId, matId) => {
              const mat = getMaterialById(matId, materials);
              setParts((prev) =>
                prev.map((p) =>
                  p.id === partId
                    ? {
                        ...p,
                        materialId: matId,
                        mesh: evaluateFeatureTree(
                          p.dag,
                          quality,
                          mat.density,
                          p.mesh
                        ),
                      }
                    : p
                )
              );
            }}
            onDefineMaterial={(mat) => setMaterials((prev) => [...prev, mat])}
            joints={joints}
            onUpdateJointMotor={(jointId, speedRpm, torqueNm, enabled) => {
              setJoints((prev) =>
                prev.map((j) =>
                  j.id === jointId
                    ? {
                        ...j,
                        motorSpeedRpm: speedRpm,
                        motorTorqueNm: torqueNm,
                        motorEnabled: enabled,
                      }
                    : j
                )
              );
            }}
            onAddJoint={(j) => setJoints((prev) => [...prev, j])}
            onDeleteJoint={(jointId) =>
              setJoints((prev) => prev.filter((j) => j.id !== jointId))
            }
          />
        )}

        <CadViewport3D
          parts={parts}
          joints={joints}
          selectedPartId={selectedPartId}
          onSelectPart={setSelectedPartId}
          renderMode={renderMode}
          showCadEdges={showCadEdges}
          projectionMode={projectionMode}
          viewportLayout={viewportLayout}
          onChangeViewportLayout={setViewportLayout}
          onToggleViewportLayout={() =>
            setViewportLayout((v) => (v === 'single' ? 'quad' : 'single'))
          }
          materials={materials}
          feaResult={feaResult}
          deformScale={deformScale}
          onChangeDeformScale={setDeformScale}
          fitTrigger={fitTrigger}
          onFpsUpdate={setFps}
          onRegisterCapture={(fn) => {
            captureFnRef.current = fn;
          }}
          minimalistWindow={false}
          showReferenceGrid={showReferenceGrid}
          windowsWgpuArtifactUrl={activeWindowsArtifactUrl}
        />

        {showRightPanel && (
          <CadRightSidebar
            activeWorkbench={activeWorkbench}
            selectedPart={selectedPart}
            parts={parts}
            joints={joints}
            isSimulatingPhysics={isSimulatingPhysics}
            onTogglePhysics={() => setIsSimulatingPhysics((prev) => !prev)}
            onResetPhysics={() => loadScenePreset(activePreset, quality, renderMode)}
            materials={materials}
            quality={quality}
            feaResult={feaResult}
            onRunFeaAnalysis={handleRunFea}
            mcpHistory={mcpHistory}
            onExecuteMcpTool={handleExecuteMcpTool}
            externalActiveTab={externalInspectorTab}
            onApplyOptimizedDag={(partId) => {
              handleExecuteMcpTool('optimize_feature_tree', {
                part_id: partId,
                apply_best_plan: true,
              });
              showToast(
                `Оптимальный план DAG применен к детали ${partId} (топология и объем сохранены)`
              );
            }}
            onLoadUnoptimizedScenario={(scen: AiUnoptimizedScenario) => {
              loadScenePreset(scen.id, quality);
              showToast(`Загружена неоптимальная цепочка ИИ: ${scen.shortTag}`);
            }}
            onSyncWithWindowsArtifact={(presetId, mode) => {
              loadScenePreset(presetId, quality, mode);
              showToast(
                `Synced 3D Viewport to Windows WGPU reference: ${presetId} (${mode})`
              );
            }}
          />
        )}

        {/* Non-Intrusive Toast Notification */}
        {toastBanner && (
          <div
            className={`absolute bottom-4 left-1/2 -translate-x-1/2 z-40 px-3.5 py-2 rounded border shadow-lg flex items-center gap-2 text-xs font-mono ${
              toastBanner.type === 'error'
                ? 'bg-[#1f1215] border-[#ef4444] text-[#fca5a5]'
                : 'bg-[#121922] border-[#38bdf8] text-[#e2e8f0]'
            }`}
          >
            {toastBanner.type === 'error' ? (
              <AlertTriangle className="w-4 h-4 text-[#ef4444]" />
            ) : (
              <CheckCircle2 className="w-4 h-4 text-[#10b981]" />
            )}
            <span>{toastBanner.text}</span>
          </div>
        )}
      </div>

      {/* 4. Bottom Telemetry Status Bar */}
      <CadStatusBar
        parts={parts}
        selectedPart={selectedPart}
        isSimulatingPhysics={isSimulatingPhysics}
        fps={fps}
        mcpToolsCount={28}
      />

      {/* Modal 1: Cut Cylindrical Hole + DIN 6885 Keyway */}
      {isHoleModalOpen && (
        <div className="fixed inset-0 z-50 bg-black/65 flex items-center justify-center p-4">
          <div className="bg-[#161a24] border border-[#2d3545] rounded-lg w-full max-w-md shadow-2xl overflow-hidden">
            <div className="flex items-center justify-between px-4 py-3 bg-[#11141c] border-b border-[#262b36]">
              <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
                <Scissors className="w-4 h-4 text-[#ef4444]" />
                <span>Cut Bore + DIN 6885 Keyway (cut_hole)</span>
              </div>
              <button
                onClick={() => setIsHoleModalOpen(false)}
                className="text-[#94a3b8] hover:text-white cursor-pointer"
              >
                <X className="w-4 h-4" />
              </button>
            </div>

            <form onSubmit={handleCutHoleSubmit} className="p-4 space-y-3 text-xs">
              <div className="grid grid-cols-2 gap-3 font-mono">
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Hole Radius (mm):
                  </label>
                  <input
                    type="number"
                    step={0.5}
                    value={holeRadiusMm}
                    onChange={(e) => setHoleRadiusMm(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Drill Depth (mm):
                  </label>
                  <input
                    type="number"
                    step={1}
                    value={holeDepthMm}
                    onChange={(e) => setHoleDepthMm(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Offset X (mm):
                  </label>
                  <input
                    type="number"
                    step={1}
                    value={holePosXMm}
                    onChange={(e) => setHolePosXMm(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Drill Axis:
                  </label>
                  <select
                    value={holeAxis}
                    onChange={(e) => setHoleAxis(e.target.value as 'Z' | 'Y')}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  >
                    <option value="Z">Z Axis (Front-to-Back)</option>
                    <option value="Y">Y Axis (Top-to-Bottom)</option>
                  </select>
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Keyway Width (mm):
                  </label>
                  <input
                    type="number"
                    step={0.5}
                    value={holeKeywayWidthMm}
                    onChange={(e) =>
                      setHoleKeywayWidthMm(Number(e.target.value))
                    }
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Keyway Depth (mm):
                  </label>
                  <input
                    type="number"
                    step={0.5}
                    value={holeKeywayDepthMm}
                    onChange={(e) =>
                      setHoleKeywayDepthMm(Number(e.target.value))
                    }
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
              </div>

              <div className="flex justify-end gap-2 pt-2 border-t border-[#262b36]">
                <button
                  type="button"
                  onClick={() => setIsHoleModalOpen(false)}
                  className="px-3 py-1.5 rounded bg-[#1d222d] text-[#cbd5e1] cursor-pointer"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  className="px-4 py-1.5 rounded bg-[#ef4444] hover:bg-[#dc2626] text-white font-semibold cursor-pointer"
                >
                  Execute cut_hole
                </button>
              </div>
            </form>
          </div>
        </div>
      )}

      {/* Modal 2: Cut Helical Groove / ISO 68-1 Metric Thread */}
      {isHelixModalOpen && (
        <div className="fixed inset-0 z-50 bg-black/65 flex items-center justify-center p-4">
          <div className="bg-[#161a24] border border-[#2d3545] rounded-lg w-full max-w-md shadow-2xl overflow-hidden">
            <div className="flex items-center justify-between px-4 py-3 bg-[#11141c] border-b border-[#262b36]">
              <div className="flex items-center gap-2 text-xs font-semibold text-[#f1f5f9]">
                <Sparkles className="w-4 h-4 text-[#a855f7]" />
                <span>Cut Helical Flute / ISO Thread (cut_helical_groove)</span>
              </div>
              <button
                onClick={() => setIsHelixModalOpen(false)}
                className="text-[#94a3b8] hover:text-white cursor-pointer"
              >
                <X className="w-4 h-4" />
              </button>
            </div>

            <form onSubmit={handleCutHelixSubmit} className="p-4 space-y-3 text-xs">
              <div className="grid grid-cols-2 gap-3 font-mono">
                <div className="col-span-2">
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Helical Profile Type:
                  </label>
                  <select
                    value={helixProfileType}
                    onChange={(e) =>
                      setHelixProfileType(
                        e.target.value as 'metric_60' | 'circular'
                      )
                    }
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  >
                    <option value="metric_60">
                      ISO 68-1 60° Metric Thread (Nut / Bolt)
                    </option>
                    <option value="circular">
                      Circular Helical Flute (Twist Drill / Endmill)
                    </option>
                  </select>
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Helix Radius (mm):
                  </label>
                  <input
                    type="number"
                    step={0.5}
                    value={helixRadiusMm}
                    onChange={(e) => setHelixRadiusMm(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Groove Depth (mm):
                  </label>
                  <input
                    type="number"
                    step={0.1}
                    value={helixGrooveRadiusMm}
                    onChange={(e) =>
                      setHelixGrooveRadiusMm(Number(e.target.value))
                    }
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Axial Pitch P (mm):
                  </label>
                  <input
                    type="number"
                    step={0.25}
                    value={helixPitchMm}
                    onChange={(e) => setHelixPitchMm(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
                <div>
                  <label className="text-[11px] text-[#94a3b8] block mb-1">
                    Thread Starts:
                  </label>
                  <input
                    type="number"
                    min={1}
                    max={6}
                    value={helixStarts}
                    onChange={(e) => setHelixStarts(Number(e.target.value))}
                    className="w-full bg-[#0f1218] border border-[#2d3545] rounded px-2.5 py-1.5 text-[#f1f5f9]"
                  />
                </div>
              </div>

              <div className="flex justify-end gap-2 pt-2 border-t border-[#262b36]">
                <button
                  type="button"
                  onClick={() => setIsHelixModalOpen(false)}
                  className="px-3 py-1.5 rounded bg-[#1d222d] text-[#cbd5e1] cursor-pointer"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  className="px-4 py-1.5 rounded bg-[#a855f7] hover:bg-[#9333ea] text-white font-semibold cursor-pointer"
                >
                  Execute cut_helical_groove
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
      {/* Full Program Help Modal Dialog */}
      {renderProgramHelpModal()}
    </div>
  );
}
export default App;
