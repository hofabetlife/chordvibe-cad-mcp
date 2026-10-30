import React, { useCallback, useEffect, useRef, useState } from 'react';
import { CadRibbonToolbar } from './components/CadRibbonToolbar';
import { CadStatusBar } from './components/CadStatusBar';
import { CadTitleBar } from './components/CadTitleBar';
import { CadViewport3D } from './components/CadViewport3D';
import { EngineeringInspector } from './components/EngineeringInspector';
import { FeatureBuilderPanel, type ScenarioPresetId } from './components/FeatureBuilderPanel';
import { McpAndRustOpsModal } from './components/McpAndRustOpsModal';
import type {
  DagNode,
  FeaResult,
  McpToolDefinition,
  ParsedStlGeometry,
  PartSummary,
  PhysicsSimulationResult,
  RpcLogEntry,
} from './types/cad';
import { parseBinaryStl, unwrapMcpToolResponse } from './utils/stlParser';

export function App() {
  const [activeWorkspace, setActiveWorkspace] = useState<'studio' | 'mcp_ops' | 'git_push'>('studio');
  const [cadTheme, setCadTheme] = useState<'dark' | 'light' | 'high_contrast'>('dark');

  // CAD State
  const [parts, setParts] = useState<PartSummary[]>([]);
  const [selectedPartId, setSelectedPartId] = useState<string | null>(null);
  const [partGeometries, setPartGeometries] = useState<Record<string, ParsedStlGeometry>>({});
  const [dagNodes, setDagNodes] = useState<DagNode[]>([]);
  const [outputStls, setOutputStls] = useState<
    Array<{ name: string; filepath: string; sizeBytes: number }>
  >([]);

  // Engineering & Render State
  const [feaResult, setFeaResult] = useState<FeaResult | null>(null);
  const [feaHeatmapBase64, setFeaHeatmapBase64] = useState<string | null>(null);
  const [physicsResult, setPhysicsResult] = useState<PhysicsSimulationResult | null>(null);
  const [wgpuSnapshotBase64, setWgpuSnapshotBase64] = useState<string | null>(null);
  const [geometryQueryResult, setGeometryQueryResult] = useState<Record<string, unknown> | null>(
    null
  );
  const [meshValidationResult, setMeshValidationResult] = useState<Record<string, unknown> | null>(
    null
  );

  // MCP Inspector State
  const [mcpTools, setMcpTools] = useState<McpToolDefinition[]>([]);
  const [rpcLogs, setRpcLogs] = useState<RpcLogEntry[]>([]);
  const [isBusy, setIsBusy] = useState(false);
  const [isCapturingWgpu, setIsCapturingWgpu] = useState(false);
  const [statusMessage, setStatusMessage] = useState<string>('Инициализация ядра Rust MCP-CAD...');

  const rpcIdCounter = useRef(1);
  const initialBootstrapDone = useRef(false);

  // Core JSON-RPC 2.0 helper talking to /api/mcp
  const callMcpTool = useCallback(
    async <T = Record<string, unknown>>(
      toolName: string,
      args: Record<string, unknown> = {}
    ): Promise<T & { _imageBase64?: string }> => {
      const id = rpcIdCounter.current++;
      const t0 = performance.now();
      const response = await fetch('/api/mcp', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          jsonrpc: '2.0',
          id,
          method: 'tools/call',
          params: {
            name: toolName,
            arguments: args,
          },
        }),
      });
      const rawJson = (await response.json()) as Record<string, unknown>;
      const durationMs = Math.round(performance.now() - t0);
      const unwrapped = unwrapMcpToolResponse(rawJson);

      // Avoid storing huge base64 strings inside log view
      const sanitizedData = { ...unwrapped.data };
      if (typeof sanitizedData.image_base64 === 'string') {
        sanitizedData.image_base64 = `<base64 PNG: ${(sanitizedData.image_base64 as string).length} chars>`;
      }

      const logEntry: RpcLogEntry = {
        id: `${id}-${Date.now()}`,
        timestamp: new Date().toLocaleTimeString(),
        toolName,
        args,
        durationMs,
        status: unwrapped.isError ? 'error' : 'ok',
        responseText: JSON.stringify(sanitizedData, null, 2),
        imageBase64: unwrapped.imageBase64,
      };
      setRpcLogs((prev) => [logEntry, ...prev.slice(0, 49)]);

      if (unwrapped.isError) {
        throw new Error(unwrapped.rawText || `MCP tool ${toolName} failed`);
      }

      return {
        ...(unwrapped.data as T),
        _imageBase64: unwrapped.imageBase64 || (unwrapped.data.image_base64 as string | undefined),
      };
    },
    []
  );

  // Fetch binary STL for a part and parse into Float32Arrays for Three.js
  const loadPartGeometry = useCallback(async (partId: string) => {
    try {
      const res = await fetch(`/api/stl/${encodeURIComponent(partId)}`);
      if (!res.ok) return;
      const buf = await res.arrayBuffer();
      const parsed = parseBinaryStl(buf);
      setPartGeometries((prev) => ({
        ...prev,
        [partId]: parsed,
      }));
    } catch {
      // Ignore transient STL fetch error
    }
  }, []);

  // Refresh DAG feature tree for selected part
  const refreshFeatureTree = useCallback(
    async (partId: string | null) => {
      if (!partId) {
        setDagNodes([]);
        return;
      }
      try {
        const res = await callMcpTool<{ features?: DagNode[]; dag?: DagNode[] }>(
          'get_feature_tree',
          {
            part_id: partId,
          }
        );
        setDagNodes(res.features || res.dag || []);
      } catch {
        setDagNodes([]);
      }
    },
    [callMcpTool]
  );

  // Refresh list of parts and their STL meshes
  const refreshScene = useCallback(
    async (preferredPartId?: string | null) => {
      const listRes = await callMcpTool<{ parts?: PartSummary[] }>('list_parts', {});
      const nextParts = listRes.parts || [];
      setParts(nextParts);

      // Load geometries in parallel for all parts with triangle_count > 0
      await Promise.all(
        nextParts
          .filter((p) => (p.triangle_count ?? 0) > 0)
          .map((p) => loadPartGeometry(p.id))
      );

      const targetSelection =
        preferredPartId && nextParts.some((p) => p.id === preferredPartId)
          ? preferredPartId
          : nextParts[nextParts.length - 1]?.id || null;

      setSelectedPartId(targetSelection);
      if (targetSelection) {
        await refreshFeatureTree(targetSelection);
      } else {
        setDagNodes([]);
      }
      return nextParts;
    },
    [callMcpTool, loadPartGeometry, refreshFeatureTree]
  );

  // Capture headless WGPU snapshot from Rust server
  const handleCaptureWgpuSnapshot = useCallback(
    async (preset = 'iso') => {
      setIsCapturingWgpu(true);
      try {
        const res = await callMcpTool<{ image_base64?: string }>('capture_viewport', {
          camera_preset: preset,
          width: 1024,
          height: 768,
        });
        const img = res._imageBase64 || res.image_base64;
        if (img) {
          setWgpuSnapshotBase64(img);
        }
      } catch (err: unknown) {
        setStatusMessage(
          `Ошибка снимка WGPU: ${err instanceof Error ? err.message : String(err)}`
        );
      } finally {
        setIsCapturingWgpu(false);
      }
    },
    [callMcpTool]
  );

  // E2E Engineering Scenarios matching Rust tools/mod.rs schemas (SI units in meters)
  const handleRunScenario = useCallback(
    async (preset: ScenarioPresetId) => {
      setIsBusy(true);
      try {
        // Clear existing parts first for a clean scene
        const existing = await callMcpTool<{ parts?: PartSummary[] }>('list_parts', {});
        for (const p of existing.parts || []) {
          await callMcpTool('delete_part', { part_id: p.id });
        }
        setPartGeometries({});
        setFeaResult(null);
        setFeaHeatmapBase64(null);
        setPhysicsResult(null);

        if (preset === 'gear_transmission') {
          setStatusMessage('Построение зубчатой передачи (Z1=20, Z2=40, M=2 мм + GearJoint)...');
          const g1 = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Ведущая шестерня Z20 (M=2мм)',
            material: 'steel_high_strength_4140',
          });
          await callMcpTool('add_feature_primitive', {
            part_id: g1.part_id,
            primitive_type: 'spur_gear',
            params: {
              module: 0.002,
              teeth: 20,
              face_width: 0.012,
              pressure_angle_deg: 20.0,
              bore_diameter: 0.008,
              keyway_width: 0.003,
              keyway_depth: 0.0015,
            },
          });

          const g2 = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Ведомое колесо Z40 (M=2мм)',
            material: 'brass_c360',
          });
          // Center distance = m * (Z1 + Z2) / 2 = 0.002 * (20 + 40) / 2 = 0.060 m
          await callMcpTool('add_feature_primitive', {
            part_id: g2.part_id,
            primitive_type: 'spur_gear',
            params: {
              module: 0.002,
              teeth: 40,
              face_width: 0.012,
              pressure_angle_deg: 20.0,
              bore_diameter: 0.012,
              position: [0.06, 0.0, 0.0],
            },
          });

          // Render the 2 spur gears immediately in the 3D viewport
          await refreshScene(g1.part_id);

          // Create Revolute joints anchored to 'ground' + GearJoint with ratio -2.0
          const j1 = await callMcpTool<{ joint_id: string }>('create_joint', {
            joint_type: 'revolute',
            part_a: 'ground',
            part_b: g1.part_id,
            anchor_a: [0, 0, 0],
            anchor_b: [0, 0, 0],
            axis: [0, 0, 1],
          });
          await callMcpTool('create_joint', {
            joint_type: 'revolute',
            part_a: 'ground',
            part_b: g2.part_id,
            anchor_a: [0.06, 0, 0],
            anchor_b: [0.06, 0, 0],
            axis: [0, 0, 1],
          });
          await callMcpTool('create_joint', {
            joint_type: 'gear',
            part_a: g1.part_id,
            part_b: g2.part_id,
            anchor_a: [0, 0, 0],
            anchor_b: [0.06, 0, 0],
            axis: [0, 0, 1],
            ratio: -2.0,
          });
          await callMcpTool('set_joint_motor', {
            joint_id: j1.joint_id,
            target_velocity: 6.28,
            max_force: 50.0,
          });

          const sim = await callMcpTool<PhysicsSimulationResult>('run_physics_simulation', {
            duration_seconds: 1.5,
            step_size_ms: 16.67,
          });
          setPhysicsResult(sim);
          await refreshScene(g1.part_id);
          setStatusMessage('Зубчатая передача Z20:Z40 собрана и рассчитана в Rapier3D');
        } else if (preset === 'aerospace_bracket') {
          setStatusMessage('Создание силового кронштейна Al 6061-T6 и расчет МКЭ (TET4)...');
          const b = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Силовой кронштейн Al 6061-T6',
            material: 'aluminum_6061_t6',
          });
          await callMcpTool('add_feature_primitive', {
            part_id: b.part_id,
            primitive_type: 'box',
            params: { width: 0.12, height: 0.015, depth: 0.06 },
          });
          await callMcpTool('cut_hole', {
            part_id: b.part_id,
            position: [-0.03, 0.0, 0.0],
            normal: [0, 1, 0],
            radius: 0.006,
            depth: 0.03,
          });
          await callMcpTool('cut_hole', {
            part_id: b.part_id,
            position: [0.03, 0.0, 0.0],
            normal: [0, 1, 0],
            radius: 0.006,
            depth: 0.03,
          });

          const fea = await callMcpTool<FeaResult>('run_fea_analysis', {
            part_id: b.part_id,
            mesh_density: 'medium',
            fixed_faces: ['min_x'],
            applied_force: {
              position: [0.055, 0.0, 0.0],
              force_vector: [0, -800, 0],
            },
          });
          setFeaResult(fea);

          const heatmap = await callMcpTool<{ image_base64?: string }>('get_fea_heatmap_image', {
            part_id: b.part_id,
            camera_preset: 'iso',
            width: 900,
            height: 675,
          });
          const img = heatmap._imageBase64 || heatmap.image_base64;
          if (img) {
            setFeaHeatmapBase64(img);
          }

          await refreshScene(b.part_id);
          setStatusMessage(
            `Кронштейн готов • Max Von Mises: ${(fea.max_von_mises_mpa ?? 0).toFixed(2)} МПа (SF=${(fea.safety_factor ?? 0).toFixed(2)})`
          );
        } else if (preset === 'crank_mechanism') {
          setStatusMessage('Сборка кривошипно-шатунного механизма с анкеровкой и мотором...');
          const base = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Станина (is_fixed=true)',
            material: 'steel_structural_aisi1020',
            is_fixed: true,
          });
          await callMcpTool('add_feature_primitive', {
            part_id: base.part_id,
            primitive_type: 'box',
            params: { width: 0.06, height: 0.02, depth: 0.06, position: [0, -0.015, 0] },
          });

          const arm = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Кривошип Al 6061-T6',
            material: 'aluminum_6061_t6',
          });
          await callMcpTool('add_feature_primitive', {
            part_id: arm.part_id,
            primitive_type: 'box',
            params: { width: 0.1, height: 0.012, depth: 0.024, position: [0.035, 0.005, 0] },
          });
          await callMcpTool('cut_hole', {
            part_id: arm.part_id,
            position: [0.07, 0.005, 0],
            normal: [0, 1, 0],
            radius: 0.005,
            depth: 0.025,
          });

          const j = await callMcpTool<{ joint_id: string }>('create_joint', {
            joint_type: 'revolute',
            part_a: base.part_id,
            part_b: arm.part_id,
            anchor_a: [0, 0, 0],
            anchor_b: [0, 0, 0],
            axis: [0, 1, 0],
          });
          await callMcpTool('set_joint_motor', {
            joint_id: j.joint_id,
            target_velocity: 6.28,
            max_force: 100.0,
          });

          const sim = await callMcpTool<PhysicsSimulationResult>('run_physics_simulation', {
            duration_seconds: 1.5,
            step_size_ms: 16.67,
          });
          setPhysicsResult(sim);
          await refreshScene(arm.part_id);
          setStatusMessage('Кривошипный механизм собран и промоделирован');
        } else if (preset === 'din934_hex_nut') {
          setStatusMessage('Построение шестигранной гайки M10 DIN 934 с резьбой 60°...');
          const nut = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Гайка M10 DIN 934',
            material: 'steel_high_strength_4140',
          });
          await callMcpTool('add_feature_primitive', {
            part_id: nut.part_id,
            primitive_type: 'polygon_prism',
            params: {
              sides: 6,
              width_across_flats: 0.017,
              height: 0.008,
              chamfer: 0.0008,
            },
          });
          await callMcpTool('cut_hole', {
            part_id: nut.part_id,
            position: [0, 0, 0],
            normal: [0, 0, 1],
            radius: 0.00425,
            depth: 0.02,
          });
          await callMcpTool('cut_helical_groove', {
            part_id: nut.part_id,
            helix_radius: 0.0045,
            groove_radius: 0.00075,
            pitch: 0.0015,
            height: 0.01,
            start_z: -0.005,
            starts: 1,
            profile_type: 'metric_60',
          });
          await refreshScene(nut.part_id);
          setStatusMessage('Гайка M10 DIN 934 построена с метрической резьбой ISO 68-1');
        } else if (preset === 'forged_wheel_rim') {
          setStatusMessage('Параметрическое построение кованого диска 18" (wheel_rim)...');
          const rim = await callMcpTool<{ part_id: string }>('create_part', {
            name: 'Кованый диск 18" Ti-6Al-4V',
            material: 'titanium_grade5_ti6al4v',
          });
          await callMcpTool('add_feature_primitive', {
            part_id: rim.part_id,
            primitive_type: 'wheel_rim',
            params: {
              rim_diameter: 0.4572,
              rim_width: 0.215,
              hub_diameter: 0.15,
              hub_bore_diameter: 0.0661,
              pcd_diameter: 0.1143,
              bolt_holes_count: 5,
              bolt_hole_diameter: 0.014,
              spokes_count: 5,
              spoke_width: 0.032,
              spoke_thickness: 0.022,
              offset_et: 0.035,
            },
          });
          await refreshScene(rim.part_id);
          setStatusMessage('Кованый колесный диск 18" построен и загружен в 3D сцену');
        }
      } catch (err: unknown) {
        setStatusMessage(`Ошибка сценария: ${err instanceof Error ? err.message : String(err)}`);
      } finally {
        setIsBusy(false);
      }
    },
    [callMcpTool, refreshScene]
  );

  // Initial bootstrap: load MCP tools list, output STLs, and populate default scene if empty
  useEffect(() => {
    if (initialBootstrapDone.current) return;
    initialBootstrapDone.current = true;

    (async () => {
      try {
        // 1. Load output STLs
        const stlRes = await fetch('/api/output-stls');
        if (stlRes.ok) {
          const stlJson = await stlRes.json();
          setOutputStls(stlJson.files || []);
        }

        // 2. Load MCP tools list
        const toolsRes = await fetch('/api/mcp', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            jsonrpc: '2.0',
            id: 999,
            method: 'tools/list',
            params: {},
          }),
        });
        const toolsJson = await toolsRes.json();
        const registeredTools: McpToolDefinition[] = toolsJson?.result?.tools || [];
        setMcpTools(registeredTools);

        // 3. Check if server already has parts; if not, run gear_transmission so viewport is populated
        const existingParts = await refreshScene();
        if (existingParts.length === 0) {
          await handleRunScenario('gear_transmission');
        } else {
          setStatusMessage(`Подключено к mcp-cad-server • ${registeredTools.length} инструментов активно`);
        }
      } catch (err: unknown) {
        setStatusMessage(
          `Ожидание запуска Rust-сервера: ${err instanceof Error ? err.message : String(err)}`
        );
      }
    })();
  }, [handleRunScenario, refreshScene]);

  // Handlers for FeatureBuilderPanel & EngineeringInspector & Ribbon
  const handleSelectPart = async (partId: string) => {
    setSelectedPartId(partId);
    setGeometryQueryResult(null);
    setMeshValidationResult(null);
    await refreshFeatureTree(partId);
  };

  const handleDeletePart = async (partId: string) => {
    setIsBusy(true);
    try {
      await callMcpTool('delete_part', { part_id: partId });
      setPartGeometries((prev) => {
        const copy = { ...prev };
        delete copy[partId];
        return copy;
      });
      await refreshScene();
      setStatusMessage('Деталь удалена');
    } finally {
      setIsBusy(false);
    }
  };

  const handleClearAllParts = async () => {
    setIsBusy(true);
    try {
      for (const p of parts) {
        await callMcpTool('delete_part', { part_id: p.id });
      }
      setPartGeometries({});
      setFeaResult(null);
      setFeaHeatmapBase64(null);
      setPhysicsResult(null);
      await refreshScene();
      setStatusMessage('Сцена очищена');
    } finally {
      setIsBusy(false);
    }
  };

  const handleImportStl = async (filepath: string, material: string) => {
    setIsBusy(true);
    try {
      const filename = filepath.split('/').pop() || 'imported.stl';
      const res = await callMcpTool<{ part_id: string }>('import_stl', {
        filepath,
        name: filename.replace('.stl', ''),
        material,
      });
      await refreshScene(res.part_id);
      setStatusMessage(`Импортирован файл ${filename}`);
    } catch (err: unknown) {
      setStatusMessage(`Ошибка импорта STL: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsBusy(false);
    }
  };

  const handleGenericBuilderTool = async (toolName: string, args: Record<string, unknown>) => {
    setIsBusy(true);
    try {
      const res = await callMcpTool<{ part_id?: string }>(toolName, args);
      const targetId = res.part_id || (args.part_id as string) || selectedPartId;
      await refreshScene(targetId);
      setStatusMessage(`Операция ${toolName} успешно выполнена`);
    } catch (err: unknown) {
      setStatusMessage(`Ошибка ${toolName}: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsBusy(false);
    }
  };

  // Quick Ribbon actions
  const handleRibbonAddPrimitive = async (
    type: 'spur_gear' | 'box' | 'cylinder' | 'polygon_prism' | 'wheel_rim' | 'cone' | 'sphere'
  ) => {
    let targetPartId = selectedPartId;
    if (!targetPartId) {
      const created = await callMcpTool<{ part_id: string }>('create_part', {
        name: `part_${type}`,
        material: 'steel_high_strength_4140',
      });
      targetPartId = created.part_id;
    }

    let params: Record<string, unknown> = {};
    if (type === 'spur_gear') {
      params = {
        module: 0.0025,
        teeth: 24,
        pressure_angle_deg: 20.0,
        face_width: 0.015,
        bore_diameter: 0.012,
        keyway_width: 0.003,
        keyway_depth: 0.0015,
      };
    } else if (type === 'polygon_prism') {
      params = { sides: 6, width_across_flats: 0.017, height: 0.008, chamfer: 0.0008 };
    } else if (type === 'box') {
      params = { width: 0.1, height: 0.02, depth: 0.05 };
    } else if (type === 'cylinder') {
      params = { radius: 0.02, height: 0.06, segments: 48 };
    } else if (type === 'wheel_rim') {
      params = {
        rim_diameter: 0.4572,
        rim_width: 0.2032,
        hub_diameter: 0.15,
        hub_bore_diameter: 0.066,
        pcd_diameter: 0.1143,
        bolt_holes_count: 5,
        bolt_hole_diameter: 0.014,
        spokes_count: 5,
        spoke_width: 0.028,
        spoke_thickness: 0.018,
        offset_et: 0.035,
      };
    }

    await handleGenericBuilderTool('add_feature_primitive', {
      part_id: targetPartId,
      primitive_type: type,
      params,
    });
  };

  const handleRibbonCutHole = async () => {
    if (!selectedPartId) return;
    await handleGenericBuilderTool('cut_hole', {
      part_id: selectedPartId,
      position: [0, 0, 0],
      normal: [0, 1, 0],
      radius: 0.006,
      depth: 0.04,
      keyway_width: 0.002,
      keyway_depth: 0.001,
    });
  };

  const handleRibbonCutHelicalThread = async () => {
    if (!selectedPartId) return;
    await handleGenericBuilderTool('cut_helical_groove', {
      part_id: selectedPartId,
      helix_radius: 0.006,
      groove_radius: 0.0008,
      pitch: 0.0015,
      height: 0.02,
      starts: 1,
      profile_type: 'metric_60',
    });
  };

  const handleRunFea = async (params: {
    part_id: string;
    mesh_density: 'coarse' | 'medium' | 'fine';
    fixed_faces: string[];
    force_vector: [number, number, number];
  }) => {
    setIsBusy(true);
    try {
      setStatusMessage('Запуск МКЭ решателя TET4 и генерация тепловой карты Von Mises...');
      const fea = await callMcpTool<FeaResult>('run_fea_analysis', {
        part_id: params.part_id,
        mesh_density: params.mesh_density,
        fixed_faces: params.fixed_faces,
        applied_force: {
          position: [0.02, 0.0, 0.0],
          force_vector: params.force_vector,
        },
      });
      setFeaResult(fea);

      const heatmap = await callMcpTool<{ image_base64?: string }>('get_fea_heatmap_image', {
        part_id: params.part_id,
        camera_preset: 'iso',
        width: 900,
        height: 675,
      });
      const img = heatmap._imageBase64 || heatmap.image_base64;
      if (img) {
        setFeaHeatmapBase64(img);
      }
      setStatusMessage(
        `МКЭ завершен: Max Von Mises = ${(fea.max_von_mises_mpa ?? 0).toFixed(2)} МПа, запас прочности = ${(fea.safety_factor ?? 0).toFixed(2)}`
      );
    } catch (err: unknown) {
      setStatusMessage(`Ошибка МКЭ: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsBusy(false);
    }
  };

  const handleRunPhysics = async (durationSeconds: number, stepSizeMs: number) => {
    setIsBusy(true);
    try {
      setStatusMessage('Запуск физической симуляции Rapier3D...');
      const sim = await callMcpTool<PhysicsSimulationResult>('run_physics_simulation', {
        duration_seconds: durationSeconds,
        step_size_ms: stepSizeMs,
      });
      setPhysicsResult(sim);
      setStatusMessage(`Физическая симуляция Rapier3D завершена (${sim.steps_simulated} шагов)`);
    } catch (err: unknown) {
      setStatusMessage(`Ошибка физики: ${err instanceof Error ? err.message : String(err)}`);
    } finally {
      setIsBusy(false);
    }
  };

  const handleQueryGeometry = async (partId: string) => {
    setIsBusy(true);
    try {
      const res = await callMcpTool<Record<string, unknown>>('query_part_geometry', {
        part_id: partId,
      });
      setGeometryQueryResult(res);
    } finally {
      setIsBusy(false);
    }
  };

  const handleValidateMesh = async (partId: string) => {
    setIsBusy(true);
    try {
      const res = await callMcpTool<Record<string, unknown>>('validate_mesh', {
        part_id: partId,
      });
      setMeshValidationResult(res);
    } finally {
      setIsBusy(false);
    }
  };

  const handleTessellatePart = async (partId: string, quality: 'coarse' | 'medium' | 'fine') => {
    setIsBusy(true);
    try {
      await callMcpTool('tessellate_part', { part_id: partId, quality });
      await loadPartGeometry(partId);
      await refreshScene(partId);
      setStatusMessage(`Перестроена сетка детали (качество: ${quality})`);
    } finally {
      setIsBusy(false);
    }
  };

  const selectedPart = parts.find((p) => p.id === selectedPartId) || parts[0] || null;

  return (
    <div
      className={`h-screen w-screen overflow-hidden flex flex-col font-sans select-none ${
        cadTheme === 'light'
          ? 'bg-slate-100 text-slate-900'
          : cadTheme === 'high_contrast'
            ? 'bg-black text-amber-400'
            : 'bg-[#090D16] text-slate-100'
      }`}
    >
      {/* 1. Native Windows Style Title Bar */}
      <CadTitleBar
        activePartName={selectedPart ? selectedPart.name : null}
        mcpToolCount={mcpTools.length || 26}
        statusMessage={statusMessage}
        theme={cadTheme}
        onThemeChange={setCadTheme}
        onOpenMcpOps={() => setActiveWorkspace('mcp_ops')}
        onCaptureWgpu={() => handleCaptureWgpuSnapshot('iso')}
        onResetScene={handleClearAllParts}
        isBusy={isBusy}
      />

      {/* 2. CAD Command Ribbon Toolbar */}
      <CadRibbonToolbar
        activePartId={selectedPartId}
        activePartName={selectedPart ? selectedPart.name : null}
        activeMaterial={selectedPart ? selectedPart.material_name : 'aluminum_6061_t6'}
        isBusy={isBusy}
        onRunScenario={handleRunScenario}
        onAddPrimitive={handleRibbonAddPrimitive}
        onCutHole={handleRibbonCutHole}
        onCutHelicalThread={handleRibbonCutHelicalThread}
        onRunFeaQuick={() => {
          if (selectedPartId) {
            handleRunFea({
              part_id: selectedPartId,
              mesh_density: 'medium',
              fixed_faces: ['min_x'],
              force_vector: [0, -800, 0],
            });
          }
        }}
        onRunPhysicsQuick={() => handleRunPhysics(1.5, 16.67)}
        onImportStlQuick={(path) => handleImportStl(path, 'aluminum_6061_t6')}
        onTessellateQuick={(q) => selectedPartId && handleTessellatePart(selectedPartId, q)}
        onQueryGeometryQuick={() => selectedPartId && handleQueryGeometry(selectedPartId)}
      />

      {/* 3. Main CAD Workspace (Left Panel, Viewport, Right Inspector) */}
      {activeWorkspace === 'studio' ? (
        <main className="flex-1 flex overflow-hidden relative">
          {/* Left Panel: Feature Builder, Scenarios, Parts List & DAG Tree */}
          <FeatureBuilderPanel
            parts={parts}
            selectedPartId={selectedPartId}
            dagNodes={dagNodes}
            outputStls={outputStls}
            isBusy={isBusy}
            onSelectPart={handleSelectPart}
            onDeletePart={handleDeletePart}
            onClearAllParts={handleClearAllParts}
            onRunScenario={handleRunScenario}
            onImportStl={handleImportStl}
            onCallTool={handleGenericBuilderTool}
          />

          {/* Center 3D Viewport Canvas with View Cube */}
          <CadViewport3D
            parts={parts}
            selectedPartId={selectedPartId}
            partGeometries={partGeometries}
            wgpuSnapshotBase64={wgpuSnapshotBase64}
            feaHeatmapBase64={feaHeatmapBase64}
            onSelectPart={handleSelectPart}
            onCaptureWgpuSnapshot={handleCaptureWgpuSnapshot}
            isCapturingWgpu={isCapturingWgpu}
          />

          {/* Right Panel: FEA TET4 Solver, Rapier3D Kinematics, Mesh QA */}
          <EngineeringInspector
            parts={parts}
            selectedPartId={selectedPartId}
            feaResult={feaResult}
            physicsResult={physicsResult}
            geometryQueryResult={geometryQueryResult}
            meshValidationResult={meshValidationResult}
            isBusy={isBusy}
            onRunFea={handleRunFea}
            onRunPhysics={handleRunPhysics}
            onQueryGeometry={handleQueryGeometry}
            onValidateMesh={handleValidateMesh}
            onTessellatePart={handleTessellatePart}
          />
        </main>
      ) : (
        <div className="flex-1 overflow-hidden relative bg-[#090D16]">
          <McpAndRustOpsModal
            tools={mcpTools}
            rpcLogs={rpcLogs}
            initialSubTab={activeWorkspace === 'git_push' ? 'github_removal' : 'mcp_rpc'}
            onExecuteRawTool={async (name, args) => {
              const res = await callMcpTool(name, args);
              await refreshScene(selectedPartId);
              return res;
            }}
            onClearLogs={() => setRpcLogs([])}
          />
          <button
            type="button"
            onClick={() => setActiveWorkspace('studio')}
            className="absolute top-3 right-4 z-50 px-3 py-1.5 text-xs font-semibold bg-amber-500 text-slate-950 rounded-lg shadow-xl hover:bg-amber-400 cursor-pointer transition-colors"
          >
            ← Вернуться в 3D CAD Вьюпорт
          </button>
        </div>
      )}

      {/* 4. Bottom CAD Status Bar */}
      <CadStatusBar
        selectedPart={selectedPart}
        feaResult={feaResult}
        statusMessage={statusMessage}
        mcpToolsCount={mcpTools.length || 26}
        onOpenMcpLogs={() => setActiveWorkspace('mcp_ops')}
      />
    </div>
  );
}

export default App;
