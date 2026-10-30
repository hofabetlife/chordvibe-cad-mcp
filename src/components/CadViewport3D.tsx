import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import type { ParsedStlGeometry, PartSummary } from '../types/cad';
import { getMaterialPbrStyle } from '../utils/stlParser';
import {
  Camera,
  ChevronDown,
  ChevronUp,
  Download,
  Eye,
  Grid,
  Layers,
  Maximize2,
} from 'lucide-react';

interface CadViewport3DProps {
  parts: PartSummary[];
  selectedPartId: string | null;
  partGeometries: Record<string, ParsedStlGeometry>;
  wgpuSnapshotBase64: string | null;
  feaHeatmapBase64: string | null;
  onSelectPart: (partId: string) => void;
  onCaptureWgpuSnapshot: (preset: string) => void;
  isCapturingWgpu: boolean;
}

export const CadViewport3D: React.FC<CadViewport3DProps> = ({
  parts,
  selectedPartId,
  partGeometries,
  wgpuSnapshotBase64,
  feaHeatmapBase64,
  onSelectPart,
  onCaptureWgpuSnapshot,
  isCapturingWgpu,
}) => {
  const canvasMountRef = useRef<HTMLDivElement | null>(null);
  const requestRenderRef = useRef<(() => void) | null>(null);
  const setCameraPresetRef = useRef<((preset: 'iso' | 'front' | 'top' | 'right' | 'fit') => void) | null>(null);
  const sceneGroupRef = useRef<THREE.Group | null>(null);
  const gridHelperRef = useRef<THREE.GridHelper | null>(null);

  const [wireframe, setWireframe] = useState(false);
  const [showGrid, setShowGrid] = useState(true);
  const [dockCollapsed, setDockCollapsed] = useState(false);
  const [activeCameraPreset, setActiveCameraPreset] = useState<'iso' | 'front' | 'top' | 'right'>('iso');
  const [viewMode, setViewMode] = useState<'webgl' | 'wgpu_render' | 'fea_heatmap'>('webgl');

  // Auto-switch to FEA heatmap tab when a new heatmap arrives
  useEffect(() => {
    if (feaHeatmapBase64) {
      setViewMode('fea_heatmap');
    }
  }, [feaHeatmapBase64]);

  // Initialize Three.js WebGLRenderer once
  useEffect(() => {
    const container = canvasMountRef.current;
    if (!container) return;

    const isSoftwareGl = (() => {
      const probeCanvas = document.createElement('canvas');
      const gl = probeCanvas.getContext('webgl2') || probeCanvas.getContext('webgl');
      if (!gl) return true;
      const ext = gl.getExtension('WEBGL_debug_renderer_info');
      const param = ext ? ext.UNMASKED_RENDERER_WEBGL : gl.RENDERER;
      const gpu = String(gl.getParameter(param));
      gl.getExtension('WEBGL_lose_context')?.loseContext();
      return /SwiftShader|llvmpipe|softpipe|Software|Basic Render/i.test(gpu);
    })();

    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f17);
    scene.fog = new THREE.FogExp2(0x0b0f17, 0.35);

    const camera = new THREE.PerspectiveCamera(42, width / height, 0.001, 100);
    camera.position.set(0.28, 0.22, 0.28);

    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({
        antialias: !isSoftwareGl,
        powerPreference: 'high-performance',
        preserveDrawingBuffer: true,
      });
    } catch {
      return;
    }

    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.12;
    renderer.setPixelRatio(isSoftwareGl ? 1 : Math.min(window.devicePixelRatio, 2));
    renderer.setSize(width, height);
    renderer.shadowMap.enabled = !isSoftwareGl;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    container.innerHTML = '';
    container.appendChild(renderer.domElement);

    // Studio reflection environment
    const pmremGenerator = new THREE.PMREMGenerator(renderer);
    scene.environment = pmremGenerator.fromScene(new RoomEnvironment(), 0.04).texture;
    pmremGenerator.dispose();

    // Warm/Cool 3-point engineering lighting rig
    const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.85);
    scene.add(hemiLight);

    const keyLight = new THREE.DirectionalLight(0xfffbeb, 1.8);
    keyLight.position.set(1.5, 2.5, 1.8);
    scene.add(keyLight);

    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.75);
    rimLight.position.set(-1.5, -1.0, -1.2);
    scene.add(rimLight);

    // Engineering Reference Grid & Axes
    const grid = new THREE.GridHelper(1.0, 20, 0x334155, 0x1e293b);
    grid.position.y = -0.0005;
    scene.add(grid);
    gridHelperRef.current = grid;

    const axes = new THREE.AxesHelper(0.08);
    axes.position.set(0, 0.0002, 0);
    scene.add(axes);

    // Group holding CAD part meshes
    const cadGroup = new THREE.Group();
    scene.add(cadGroup);
    sceneGroupRef.current = cadGroup;

    // OrbitControls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = !isSoftwareGl;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0, 0);

    let frameId = 0;
    let renderBusy = false;
    let renderDirty = false;

    function afterDraw(cb: () => void) {
      const gl = renderer.getContext() as WebGL2RenderingContext;
      if (typeof gl.fenceSync === 'function') {
        const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
        gl.flush();
        const poll = () => {
          const state = sync && gl.getSyncParameter(sync, gl.SYNC_STATUS);
          if (state === gl.UNSIGNALED) {
            frameId = window.setTimeout(poll, 5);
          } else {
            if (sync) gl.deleteSync(sync);
            frameId = window.setTimeout(cb, 80);
          }
        };
        frameId = window.setTimeout(poll, 5);
      } else {
        frameId = window.setTimeout(cb, 80);
      }
    }

    function requestRender() {
      if (renderBusy) {
        renderDirty = true;
        return;
      }
      renderBusy = true;
      frameId = requestAnimationFrame(() => {
        renderDirty = false;
        controls.update();
        renderer.render(scene, camera);
        const done = () => {
          renderBusy = false;
          if (renderDirty) requestRender();
        };
        if (isSoftwareGl) afterDraw(done);
        else done();
      });
    }

    requestRenderRef.current = requestRender;

    setCameraPresetRef.current = (preset) => {
      // Compute combined bounding sphere of cadGroup
      const box = new THREE.Box3().setFromObject(cadGroup);
      const center = new THREE.Vector3(0, 0, 0);
      let maxDim = 0.2;
      if (!box.isEmpty()) {
        box.getCenter(center);
        const size = new THREE.Vector3();
        box.getSize(size);
        maxDim = Math.max(size.x, size.y, size.z, 0.04);
      }
      const dist = maxDim * 2.1;

      controls.target.copy(center);
      if (preset === 'front') {
        camera.position.set(center.x, center.y, center.z + dist);
      } else if (preset === 'top') {
        camera.position.set(center.x, center.y + dist, center.z + 0.001);
      } else if (preset === 'right') {
        camera.position.set(center.x + dist, center.y, center.z);
      } else {
        camera.position.set(center.x + dist * 0.72, center.y + dist * 0.58, center.z + dist * 0.72);
      }
      camera.near = Math.max(maxDim * 0.005, 0.0001);
      camera.far = Math.max(maxDim * 50, 20);
      camera.updateProjectionMatrix();
      controls.update();
      requestRender();
    };

    controls.addEventListener('change', requestRender);

    const handleResize = () => {
      if (!container) return;
      const w = container.clientWidth || 800;
      const h = container.clientHeight || 600;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
      requestRender();
    };

    const resizeObserver = new ResizeObserver(handleResize);
    resizeObserver.observe(container);

    requestRender();

    return () => {
      resizeObserver.disconnect();
      controls.dispose();
      cancelAnimationFrame(frameId);
      clearTimeout(frameId);
      renderer.dispose();
    };
  }, []);

  // Rebuild Three.js meshes whenever parts, geometries, selection, or wireframe state updates
  useEffect(() => {
    const group = sceneGroupRef.current;
    if (!group) return;

    // Dispose existing children
    while (group.children.length > 0) {
      const child = group.children[0];
      group.remove(child);
      if (child instanceof THREE.Mesh || child instanceof THREE.LineSegments) {
        child.geometry.dispose();
        if (Array.isArray(child.material)) {
          child.material.forEach((m) => m.dispose());
        } else {
          child.material.dispose();
        }
      }
    }

    let hasAnyGeometry = false;

    for (const part of parts) {
      const parsed = partGeometries[part.id];
      if (!parsed || parsed.triangleCount === 0) continue;

      hasAnyGeometry = true;
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(parsed.positions, 3));
      geometry.setAttribute('normal', new THREE.BufferAttribute(parsed.normals, 3));
      geometry.computeBoundingSphere();

      const style = getMaterialPbrStyle(part.material_name);
      const isSelected = part.id === selectedPartId;

      const material = new THREE.MeshStandardMaterial({
        color: style.colorHex,
        metalness: style.metalness,
        roughness: style.roughness,
        emissive: isSelected ? new THREE.Color(0xf59e0b) : new THREE.Color(0x000000),
        emissiveIntensity: isSelected ? 0.12 : 0,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
      });

      const mesh = new THREE.Mesh(geometry, material);
      mesh.userData = { partId: part.id };
      group.add(mesh);

      if (wireframe || isSelected) {
        const edgesGeo = new THREE.EdgesGeometry(geometry, 22);
        const edgesMat = new THREE.LineBasicMaterial({
          color: isSelected ? 0xf59e0b : 0x38bdf8,
          transparent: true,
          opacity: wireframe ? 0.55 : 0.32,
        });
        const lines = new THREE.LineSegments(edgesGeo, edgesMat);
        group.add(lines);
      }
    }

    if (gridHelperRef.current) {
      gridHelperRef.current.visible = showGrid;
    }

    if (hasAnyGeometry && setCameraPresetRef.current) {
      setCameraPresetRef.current(activeCameraPreset);
    } else if (requestRenderRef.current) {
      requestRenderRef.current();
    }
  }, [parts, partGeometries, selectedPartId, wireframe, showGrid, activeCameraPreset]);

  const selectedPart = parts.find((p) => p.id === selectedPartId) || parts[0] || null;

  const handleCameraPreset = (preset: 'iso' | 'front' | 'top' | 'right') => {
    setActiveCameraPreset(preset);
    setViewMode('webgl');
    setCameraPresetRef.current?.(preset);
  };

  return (
    <div className="relative flex-1 h-full bg-[#0B0F17] overflow-hidden select-none flex flex-col">
      {/* Non-overlapping Top Viewport Bar */}
      <div className="absolute top-3 inset-x-3 z-10 flex items-center justify-between gap-3 pointer-events-none">
        {/* Minimal Top-Left Viewport Telemetry Overlay */}
        <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-[#111827]/90 border border-slate-800 text-xs text-slate-300 pointer-events-auto min-w-0 max-w-[46%] overflow-hidden">
          <span className="font-semibold text-white truncate">
            {selectedPart ? selectedPart.name : 'Сцена пуста'}
          </span>
          {selectedPart && (
            <>
              <span aria-hidden="true" className="text-slate-600 shrink-0">·</span>
              <span className="font-mono text-amber-400 shrink-0">
                {selectedPart.triangle_count.toLocaleString()} tris
              </span>
              <span aria-hidden="true" className="text-slate-600 shrink-0 hidden xl:inline">·</span>
              <span className="font-mono text-emerald-400 shrink-0 hidden xl:inline">
                {selectedPart.mass >= 1
                  ? `${selectedPart.mass.toFixed(3)} кг`
                  : `${(selectedPart.mass * 1000).toFixed(1)} г`}
              </span>
            </>
          )}
        </div>

        {/* View Mode Switcher Top-Right */}
        <div className="flex items-center gap-1 p-1 rounded-lg bg-[#111827]/90 border border-slate-800 pointer-events-auto shrink-0">
          <button
            type="button"
            onClick={() => setViewMode('webgl')}
            className={`px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
              viewMode === 'webgl'
                ? 'bg-amber-500 text-slate-950 font-semibold'
                : 'text-slate-300 hover:text-white'
            }`}
          >
            3D WebGL (STL)
          </button>
          <button
            type="button"
            onClick={() => {
              setViewMode('wgpu_render');
              if (!wgpuSnapshotBase64) {
                onCaptureWgpuSnapshot(activeCameraPreset === 'iso' ? 'isometric' : activeCameraPreset);
              }
            }}
            className={`px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
              viewMode === 'wgpu_render'
                ? 'bg-amber-500 text-slate-950 font-semibold'
                : 'text-slate-300 hover:text-white'
            }`}
          >
            WGPU Снимок (Rust)
          </button>
          {feaHeatmapBase64 && (
            <button
              type="button"
              onClick={() => setViewMode('fea_heatmap')}
              className={`px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
                viewMode === 'fea_heatmap'
                  ? 'bg-amber-500 text-slate-950 font-semibold'
                  : 'text-slate-300 hover:text-white'
              }`}
            >
              Тепловая карта МКЭ
            </button>
          )}
        </div>
      </div>

      {/* Dedicated Three.js WebGL Mount Container */}
      <div
        ref={canvasMountRef}
        data-testid="cad-webgl-viewport"
        className={`w-full h-full ${viewMode === 'webgl' ? 'block' : 'hidden'}`}
      />

      {/* WGPU Offscreen Render View */}
      {viewMode === 'wgpu_render' && (
        <div className="w-full h-full flex flex-col items-center justify-center pt-14 pb-28 px-6 bg-[#0B0F17]">
          {wgpuSnapshotBase64 ? (
            <div className="relative max-w-full max-h-full flex flex-col items-center">
              <img
                src={`data:image/png;base64,${wgpuSnapshotBase64}`}
                alt="WGPU Offscreen CAD Snapshot"
                referrerPolicy="no-referrer"
                className="max-h-[62vh] w-auto rounded-lg border border-slate-800 shadow-2xl object-contain"
              />
              <p className="mt-2 text-xs text-slate-400 font-mono">
                Оффскрин-рендер ядра Rust `mcp-cad-viewer` (WGPU / Vulkan RGBA8)
              </p>
            </div>
          ) : (
            <div className="text-center space-y-3">
              <p className="text-sm text-slate-400">
                {isCapturingWgpu
                  ? 'Рендеринг кадра через Rust WGPU...'
                  : 'Нажмите «Сделать WGPU-снимок» для генерации кадра на стороне Rust-сервера'}
              </p>
              <button
                type="button"
                onClick={() =>
                  onCaptureWgpuSnapshot(activeCameraPreset === 'iso' ? 'isometric' : activeCameraPreset)
                }
                disabled={isCapturingWgpu}
                className="px-4 py-2 text-xs font-semibold bg-amber-500 text-slate-950 rounded-lg hover:bg-amber-400 transition-colors"
              >
                Сделать WGPU-снимок
              </button>
            </div>
          )}
        </div>
      )}

      {/* WGPU FEA Stress Heatmap View */}
      {viewMode === 'fea_heatmap' && feaHeatmapBase64 && (
        <div className="w-full h-full flex flex-col items-center justify-center pt-14 pb-28 px-6 bg-[#0B0F17]">
          <img
            src={`data:image/png;base64,${feaHeatmapBase64}`}
            alt="FEA von Mises Stress Heatmap"
            referrerPolicy="no-referrer"
            className="max-h-[60vh] w-auto rounded-lg border border-slate-800 shadow-2xl object-contain"
          />
          <div className="mt-3 flex items-center gap-3 text-xs font-mono text-slate-300 bg-[#111827] px-4 py-1.5 rounded-lg border border-slate-800">
            <span>Шкала Мизеса:</span>
            <span className="text-blue-400">0.0 (Мин)</span>
            <span>→</span>
            <span className="text-cyan-400">Циан</span>
            <span>→</span>
            <span className="text-emerald-400">Зелёный (Норма)</span>
            <span>→</span>
            <span className="text-yellow-400">Жёлтый</span>
            <span>→</span>
            <span className="text-red-400">Красный (Пик)</span>
          </div>
        </div>
      )}

      {/* Empty Scene Overlay */}
      {parts.length === 0 && viewMode === 'webgl' && (
        <div className="absolute inset-0 pointer-events-none flex items-center justify-center">
          <div className="max-w-md text-center p-6 rounded-xl bg-[#111827]/90 border border-slate-800">
            <Layers className="w-8 h-8 text-amber-400 mx-auto mb-2" />
            <h3 className="text-base font-semibold text-white font-display">
              3D-сцена MCP-CAD готова к работе
            </h3>
            <p className="mt-1 text-xs text-slate-400 leading-relaxed">
              Выберите один из готовых инженерных сценариев слева (Зубчатая передача, Силовой кронштейн МКЭ, Кривошипно-ползунный механизм) или создайте деталь вручную.
            </p>
          </div>
        </div>
      )}

      {/* Collapsible Bottom Control Dock (Zero Viewport Occlusion) */}
      <div className="absolute bottom-4 left-1/2 -translate-x-1/2 z-20 flex flex-col items-center max-w-[94vw]">
        <button
          type="button"
          onClick={() => setDockCollapsed((v) => !v)}
          className="flex items-center gap-1.5 px-3 py-1 text-[11px] font-medium text-slate-300 bg-[#111827] border border-b-0 border-slate-800 rounded-t-lg hover:text-white transition-colors"
        >
          {dockCollapsed ? (
            <>
              <ChevronUp className="w-3.5 h-3.5" />
              <span>Показать панель вида</span>
            </>
          ) : (
            <>
              <ChevronDown className="w-3.5 h-3.5" />
              <span>Скрыть панель</span>
            </>
          )}
        </button>

        {!dockCollapsed && (
          <div className="flex flex-wrap items-center gap-2 px-4 py-2.5 rounded-xl bg-[#111827] border border-slate-800 shadow-2xl">
            {/* Part Switcher if multiple parts exist */}
            {parts.length > 1 && (
              <div className="flex items-center gap-1.5 pr-2 border-r border-slate-800">
                <span className="text-[11px] text-slate-400">Деталь:</span>
                <select
                  aria-label="Выбор активной детали"
                  value={selectedPartId || ''}
                  onChange={(e) => onSelectPart(e.target.value)}
                  className="bg-slate-900 text-xs text-white border border-slate-700 rounded-md px-2 py-1 focus:outline-none focus:border-amber-500"
                >
                  {parts.map((p) => (
                    <option key={p.id} value={p.id}>
                      {p.name}
                    </option>
                  ))}
                </select>
              </div>
            )}

            {/* Camera Presets */}
            <div className="flex items-center gap-1 pr-2 border-r border-slate-800">
              {(['iso', 'front', 'top', 'right'] as const).map((preset) => (
                <button
                  key={preset}
                  type="button"
                  onClick={() => handleCameraPreset(preset)}
                  className={`px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
                    activeCameraPreset === preset
                      ? 'bg-slate-800 text-amber-400 border border-slate-700'
                      : 'text-slate-400 hover:text-white'
                  }`}
                >
                  {preset === 'iso'
                    ? 'Изометрия'
                    : preset === 'front'
                      ? 'Спереди'
                      : preset === 'top'
                        ? 'Сверху'
                        : 'Справа'}
                </button>
              ))}
              <button
                type="button"
                onClick={() => setCameraPresetRef.current?.('fit')}
                title="Вписать в экран"
                className="p-1.5 text-xs text-slate-400 hover:text-white rounded-md hover:bg-slate-800 transition-colors"
              >
                <Maximize2 className="w-3.5 h-3.5" />
              </button>
            </div>

            {/* Wireframe & Grid Toggles */}
            <div className="flex items-center gap-1.5 pr-2 border-r border-slate-800">
              <button
                type="button"
                onClick={() => setWireframe((w) => !w)}
                className={`flex items-center gap-1 px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
                  wireframe
                    ? 'bg-amber-500/20 text-amber-300 border border-amber-500/40'
                    : 'text-slate-400 hover:text-white'
                }`}
              >
                <Eye className="w-3.5 h-3.5" />
                <span>Каркас</span>
              </button>
              <button
                type="button"
                onClick={() => setShowGrid((g) => !g)}
                className={`flex items-center gap-1 px-2.5 py-1 text-xs font-medium rounded-md transition-colors whitespace-nowrap shrink-0 ${
                  showGrid
                    ? 'bg-slate-800 text-slate-200'
                    : 'text-slate-500 hover:text-slate-300'
                }`}
              >
                <Grid className="w-3.5 h-3.5" />
                <span>Сетка</span>
              </button>
            </div>

            {/* WGPU Snapshot & STL Download */}
            <div className="flex items-center gap-1.5">
              <button
                type="button"
                onClick={() => {
                  setViewMode('wgpu_render');
                  onCaptureWgpuSnapshot(activeCameraPreset === 'iso' ? 'isometric' : activeCameraPreset);
                }}
                disabled={isCapturingWgpu}
                className="flex items-center gap-1.5 px-3 py-1 text-xs font-medium text-slate-200 bg-slate-800 hover:bg-slate-700 rounded-md transition-colors whitespace-nowrap shrink-0"
              >
                <Camera className="w-3.5 h-3.5 text-amber-400" />
                <span>{isCapturingWgpu ? 'Рендер...' : 'Снимок WGPU'}</span>
              </button>

              {selectedPart && (
                <a
                  href={`/api/stl/${encodeURIComponent(selectedPart.id)}`}
                  download={`${selectedPart.name}.stl`}
                  className="flex items-center gap-1.5 px-3 py-1 text-xs font-semibold text-slate-950 bg-amber-500 hover:bg-amber-400 rounded-md transition-colors whitespace-nowrap shrink-0"
                >
                  <Download className="w-3.5 h-3.5" />
                  <span>Скачать STL</span>
                </a>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};
