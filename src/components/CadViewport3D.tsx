import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import {
  CadPart,
  FeaSimulationResult,
  KinematicJoint,
  MaterialSpec,
  ProjectionMode,
  RenderMode,
  ViewportLayout,
} from '../types/cad';
import { getMaterialById } from '../cad/materials';
import { Compass, Eye, Layers, Maximize2, ShieldCheck } from 'lucide-react';

const WGPU_ISO_THETA = Math.PI / 4; // 45 deg
const WGPU_ISO_PHI = Math.acos(1 / Math.sqrt(3)); // ~54.7356 deg (eye at [+d, +d, +d])

interface CadViewport3DProps {
  parts: CadPart[];
  joints: KinematicJoint[];
  selectedPartId: string;
  onSelectPart: (id: string) => void;
  renderMode: RenderMode;
  showCadEdges: boolean;
  projectionMode: ProjectionMode;
  viewportLayout: ViewportLayout;
  onChangeViewportLayout?: (layout: ViewportLayout) => void;
  onToggleViewportLayout?: () => void;
  materials: MaterialSpec[];
  feaResult: FeaSimulationResult | null;
  deformScale: number;
  onChangeDeformScale: (scale: number) => void;
  fitTrigger: number;
  onFpsUpdate: (fps: number) => void;
  onRegisterCapture?: (fn: () => string | null) => void;
  minimalistWindow?: boolean;
  showReferenceGrid?: boolean;
  windowsWgpuArtifactUrl?: string | null;
}

function createWorldSpaceNormalMaterial(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    vertexShader: `
      varying vec3 vWorldNormal;
      void main() {
        vWorldNormal = normalize(mat3(modelMatrix) * normal);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      varying vec3 vWorldNormal;
      void main() {
        vec3 n = normalize(vWorldNormal);
        gl_FragColor = vec4(n * 0.5 + 0.5, 1.0);
      }
    `,
  });
}

function vonMisesToRgb(t: number): [number, number, number] {
  const s = Math.max(0, Math.min(1, t));
  if (s < 0.25) {
    const u = s / 0.25;
    return [0.05, u * 0.85, 0.95];
  } else if (s < 0.5) {
    const u = (s - 0.25) / 0.25;
    return [0.05, 0.85 + 0.15 * u, 0.95 * (1 - u)];
  } else if (s < 0.75) {
    const u = (s - 0.5) / 0.25;
    return [u, 1.0, 0.0];
  } else {
    const u = (s - 0.75) / 0.25;
    return [1.0, 1.0 - u * 0.9, 0.05];
  }
}

export const CadViewport3D: React.FC<CadViewport3DProps> = ({
  parts,
  joints,
  selectedPartId,
  onSelectPart,
  renderMode,
  showCadEdges,
  projectionMode,
  viewportLayout,
  onChangeViewportLayout,
  onToggleViewportLayout,
  materials,
  feaResult,
  deformScale,
  onChangeDeformScale,
  fitTrigger,
  onFpsUpdate,
  onRegisterCapture,
  minimalistWindow = false,
  showReferenceGrid = false,
  windowsWgpuArtifactUrl = null,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const middleDownRef = useRef<{ x: number; y: number; time: number } | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const perspCameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const orthoCameraRef = useRef<THREE.OrthographicCamera | null>(null);
  const partGroupRef = useRef<THREE.Group | null>(null);
  const jointGroupRef = useRef<THREE.Group | null>(null);
  const gridHelperRef = useRef<THREE.GridHelper | null>(null);
  const axesHelperRef = useRef<THREE.AxesHelper | null>(null);

  type CadViewportId = 'perspective' | 'top' | 'right' | 'front';

  interface ViewCameraState {
    radius: number;
    target: THREE.Vector3;
    theta?: number;
    phi?: number;
  }

  const orbitRef = useRef({
    theta: WGPU_ISO_THETA,
    phi: WGPU_ISO_PHI,
    radius: 0.12,
    target: new THREE.Vector3(0, 0, 0),
    isDragging: false,
    isPanning: false,
    lastX: 0,
    lastY: 0,
  });

  const viewsCameraRef = useRef<Record<CadViewportId, ViewCameraState>>({
    perspective: {
      theta: WGPU_ISO_THETA,
      phi: WGPU_ISO_PHI,
      radius: 0.12,
      target: new THREE.Vector3(0, 0, 0),
    },
    top: {
      radius: 0.12,
      target: new THREE.Vector3(0, 0, 0),
    },
    right: {
      radius: 0.12,
      target: new THREE.Vector3(0, 0, 0),
    },
    front: {
      radius: 0.12,
      target: new THREE.Vector3(0, 0, 0),
    },
  });

  const activeInteractionViewportRef = useRef<CadViewportId>('perspective');

  const [cameraHud, setCameraHud] = useState({
    azDeg: 45,
    elDeg: 35,
    distMm: 120,
  });

  const [compareOverlayActive, setCompareOverlayActive] = useState<boolean>(false);
  const [compareOpacity, setCompareOpacity] = useState<number>(50);

  const layoutRef = useRef<ViewportLayout>(viewportLayout);
  const projRef = useRef<ProjectionMode>(projectionMode);
  const [activeView, setActiveView] = useState<CadViewportId>('perspective');
  const activeViewRef = useRef<CadViewportId>('perspective');

  useEffect(() => {
    layoutRef.current = viewportLayout;
  }, [viewportLayout]);
  useEffect(() => {
    projRef.current = projectionMode;
  }, [projectionMode]);
  useEffect(() => {
    activeViewRef.current = activeView;
  }, [activeView]);

  const syncCamerasFromOrbit = () => {
    const v = viewsCameraRef.current.perspective;
    const theta = v.theta ?? orbitRef.current.theta;
    const phi = v.phi ?? orbitRef.current.phi;
    const radius = v.radius;
    const target = v.target;

    const x = target.x + radius * Math.sin(phi) * Math.cos(theta);
    const y = target.y + radius * Math.cos(phi);
    const z = target.z + radius * Math.sin(phi) * Math.sin(theta);

    if (perspCameraRef.current) {
      perspCameraRef.current.position.set(x, y, z);
      perspCameraRef.current.lookAt(target);
    }

    if (orthoCameraRef.current && containerRef.current) {
      const aspect =
        Math.max(1, containerRef.current.clientWidth) /
        Math.max(1, containerRef.current.clientHeight);
      const frustumHalfH = radius * 0.55;
      orthoCameraRef.current.left = -frustumHalfH * aspect;
      orthoCameraRef.current.right = frustumHalfH * aspect;
      orthoCameraRef.current.top = frustumHalfH;
      orthoCameraRef.current.bottom = -frustumHalfH;
      orthoCameraRef.current.position.set(x, y, z);
      orthoCameraRef.current.lookAt(target);
      orthoCameraRef.current.updateProjectionMatrix();
    }

    setCameraHud({
      azDeg: Math.round(((theta * 180) / Math.PI) % 360),
      elDeg: Math.round(90 - (phi * 180) / Math.PI),
      distMm: Math.round(radius * 1000),
    });
  };

  const fitCameraToParts = (targetParts: CadPart[]) => {
    const visible = targetParts.filter((p) => p.visible);
    if (visible.length === 0) return;

    let minX = Infinity,
      minY = Infinity,
      minZ = Infinity;
    let maxX = -Infinity,
      maxY = -Infinity,
      maxZ = -Infinity;

    for (const p of visible) {
      const b = p.mesh.boundingBox;
      const sx = Math.abs(p.scale[0]);
      const sy = Math.abs(p.scale[1]);
      const sz = Math.abs(p.scale[2]);
      const px = p.position[0];
      const py = p.position[1];
      const pz = p.position[2];

      minX = Math.min(minX, px + b.min[0] * sx);
      minY = Math.min(minY, py + b.min[1] * sy);
      minZ = Math.min(minZ, pz + b.min[2] * sz);
      maxX = Math.max(maxX, px + b.max[0] * sx);
      maxY = Math.max(maxY, py + b.max[1] * sy);
      maxZ = Math.max(maxZ, pz + b.max[2] * sz);
    }

    const cx = 0.5 * (minX + maxX);
    const cy = 0.5 * (minY + maxY);
    const cz = 0.5 * (minZ + maxZ);
    const dx = maxX - minX;
    const dy = maxY - minY;
    const dz = maxZ - minZ;
    const diag = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const r = Math.max(0.035, Math.min(4.0, diag * 1.18));

    // Reset / fit all 4 independent cameras to object center and fitting radius
    viewsCameraRef.current.perspective.target.set(cx, cy, cz);
    viewsCameraRef.current.perspective.theta = WGPU_ISO_THETA;
    viewsCameraRef.current.perspective.phi = WGPU_ISO_PHI;
    viewsCameraRef.current.perspective.radius = r;

    viewsCameraRef.current.top.target.set(cx, cy, cz);
    viewsCameraRef.current.top.radius = r;

    viewsCameraRef.current.right.target.set(cx, cy, cz);
    viewsCameraRef.current.right.radius = r;

    viewsCameraRef.current.front.target.set(cx, cy, cz);
    viewsCameraRef.current.front.radius = r;

    orbitRef.current.target.set(cx, cy, cz);
    orbitRef.current.theta = WGPU_ISO_THETA;
    orbitRef.current.phi = WGPU_ISO_PHI;
    orbitRef.current.radius = r;

    syncCamerasFromOrbit();
  };

  // Initialize Three.js WebGLRenderer & WGPU-Matched Studio Lighting
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const width = Math.max(1, container.clientWidth);
    const height = Math.max(1, container.clientHeight);

    const scene = new THREE.Scene();
    // Matches exact Windows WGPU offscreen render slate background (#262b33 = RGB(38, 43, 51))
    scene.background = new THREE.Color(0x262b33);
    sceneRef.current = scene;

    // Matches Windows WGPU PerspectiveCamera FOV = 45 deg
    const perspCam = new THREE.PerspectiveCamera(45, width / height, 0.001, 50);
    perspCameraRef.current = perspCam;

    const orthoCam = new THREE.OrthographicCamera(
      -0.2,
      0.2,
      0.15,
      -0.15,
      0.001,
      50
    );
    orthoCameraRef.current = orthoCam;

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
      preserveDrawingBuffer: true,
    });

    const gl = renderer.getContext();
    const debugInfo = gl.getExtension('WEBGL_debug_renderer_info');
    const rendererStr = debugInfo
      ? gl.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL)
      : '';
    const isSoftwareGl = /SwiftShader|llvmpipe|Software|Microsoft Basic Render/i.test(
      rendererStr
    );

    renderer.setSize(width, height);
    renderer.setPixelRatio(
      isSoftwareGl ? 1 : Math.min(window.devicePixelRatio, 2)
    );
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    renderer.shadowMap.enabled = !isSoftwareGl;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    container.innerHTML = '';
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    const handleNativeMiddleDown = (e: MouseEvent) => {
      if (e.button === 1) {
        e.preventDefault();
      }
    };
    container.addEventListener('mousedown', handleNativeMiddleDown, { passive: false });
    container.addEventListener('auxclick', handleNativeMiddleDown, { passive: false });

    const pmremGenerator = new THREE.PMREMGenerator(renderer);
    scene.environment = pmremGenerator.fromScene(
      new RoomEnvironment(),
      0.04
    ).texture;

    // Precision CAD Reference Grid & Coordinate Axes (hidden in Windows WGPU preview mode)
    const gridHelper = new THREE.GridHelper(1.2, 24, 0x38bdf8, 0x2e3543);
    gridHelper.position.y = -0.0002;
    gridHelper.visible = showReferenceGrid;
    scene.add(gridHelper);
    gridHelperRef.current = gridHelper;

    const axesHelper = new THREE.AxesHelper(0.08);
    axesHelper.position.set(0, 0.0004, 0);
    axesHelper.visible = showReferenceGrid;
    scene.add(axesHelper);
    axesHelperRef.current = axesHelper;

    // WGPU Blinn-Phong / Studio Key + Fill + Rim Lights
    const ambientLight = new THREE.AmbientLight(0xdbeafe, 0.55);
    scene.add(ambientLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.85);
    keyLight.position.set(0.5, 0.85, 0.6);
    if (!isSoftwareGl) {
      keyLight.castShadow = true;
      keyLight.shadow.mapSize.width = 1024;
      keyLight.shadow.mapSize.height = 1024;
    }
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x93c5fd, 0.85);
    fillLight.position.set(-0.6, 0.35, -0.45);
    scene.add(fillLight);

    const rimLight = new THREE.DirectionalLight(0xfde68a, 0.55);
    rimLight.position.set(0.1, -0.6, -0.6);
    scene.add(rimLight);

    const partGroup = new THREE.Group();
    scene.add(partGroup);
    partGroupRef.current = partGroup;

    const jointGroup = new THREE.Group();
    scene.add(jointGroup);
    jointGroupRef.current = jointGroup;

    syncCamerasFromOrbit();

    if (onRegisterCapture) {
      onRegisterCapture(() => {
        if (!rendererRef.current) return null;
        return rendererRef.current.domElement.toDataURL('image/png');
      });
    }

    const orthoViews = [
      {
        pos: new THREE.Vector3(0, 0.4, 0.0001),
        up: new THREE.Vector3(0, 0, -1),
      },
      {
        pos: new THREE.Vector3(0, 0.03, 0.4),
        up: new THREE.Vector3(0, 1, 0),
      },
      {
        pos: new THREE.Vector3(0.4, 0.03, 0),
        up: new THREE.Vector3(0, 1, 0),
      },
    ];
    const quadCam = new THREE.OrthographicCamera(
      -0.15,
      0.15,
      0.15,
      -0.15,
      0.001,
      50
    );

    let frameCount = 0;
    let lastFpsTime = performance.now();
    let animId = 0;

    const animate = () => {
      animId = requestAnimationFrame(animate);
      frameCount++;
      const now = performance.now();
      if (now - lastFpsTime >= 500) {
        onFpsUpdate(Math.round((frameCount * 1000) / (now - lastFpsTime)));
        frameCount = 0;
        lastFpsTime = now;
      }

      const w = Math.max(1, container.clientWidth);
      const h = Math.max(1, container.clientHeight);
      const activeCam =
        projRef.current === 'orthographic' ? orthoCam : perspCam;

      const setC4dWireframeOnly = (wireframeOnly: boolean) => {
        if (!partGroupRef.current) return;
        partGroupRef.current.traverse((child) => {
          if (child instanceof THREE.Mesh && child.userData?.partId) {
            child.material.visible = !wireframeOnly;
          }
          if (child.userData?.isC4dLines) {
            child.visible = wireframeOnly || (renderMode === 'wireframe');
          }
          if (
            child.userData?.isWfOverlay ||
            child.userData?.isBackfacesOverlay ||
            child.userData?.isBoundaryOverlay ||
            child.userData?.isCadEdgesOverlay
          ) {
            child.visible = !wireframeOnly;
          }
        });
      };

      if (layoutRef.current === 'single') {
        renderer.setScissorTest(false);
        renderer.setViewport(0, 0, w, h);
        const aspect = w / h;

        if (activeViewRef.current === 'perspective') {
          // Full screen 3D Perspective View (shaded colored polygons)
          const v = viewsCameraRef.current.perspective;
          const theta = v.theta ?? WGPU_ISO_THETA;
          const phi = v.phi ?? WGPU_ISO_PHI;
          const px = v.target.x + v.radius * Math.sin(phi) * Math.cos(theta);
          const py = v.target.y + v.radius * Math.cos(phi);
          const pz = v.target.z + v.radius * Math.sin(phi) * Math.sin(theta);
          perspCam.position.set(px, py, pz);
          perspCam.lookAt(v.target);
          perspCam.aspect = aspect;
          perspCam.updateProjectionMatrix();

          setC4dWireframeOnly(renderMode === 'wireframe');
          renderer.render(scene, activeCam);
        } else {
          // Full screen Orthographic View (Top, Right, Front)
          // ONLY black lines on vertices, polygons transparent!
          setC4dWireframeOnly(true);

          let viewIdx = 0; // 0: TOP, 1: FRONT, 2: RIGHT
          if (activeViewRef.current === 'top') viewIdx = 0;
          else if (activeViewRef.current === 'front') viewIdx = 1;
          else if (activeViewRef.current === 'right') viewIdx = 2;

          const v = viewsCameraRef.current[activeViewRef.current];
          const frustumSize = v.radius * 1.1;

          quadCam.left = (-frustumSize * aspect) / 2;
          quadCam.right = (frustumSize * aspect) / 2;
          quadCam.top = frustumSize / 2;
          quadCam.bottom = -frustumSize / 2;
          quadCam.position
            .copy(orthoViews[viewIdx].pos)
            .setLength(v.radius * 1.5)
            .add(v.target);
          quadCam.up.copy(orthoViews[viewIdx].up);
          quadCam.lookAt(v.target);
          quadCam.updateProjectionMatrix();

          renderer.render(scene, quadCam);
          setC4dWireframeOnly(renderMode === 'wireframe');
        }
      } else {
        // Quad Split Mode (Cinema 4D style)
        renderer.setScissorTest(true);
        const halfW = Math.floor(w / 2);
        const halfH = Math.floor(h / 2);
        const aspect = Math.max(1, halfW) / Math.max(1, halfH);

        // 1. Top-Left (WebGL: 0, halfH): 3D Perspective View (Cinema 4D style - shaded)
        renderer.setViewport(0, halfH, halfW, halfH);
        renderer.setScissor(0, halfH, halfW, halfH);
        const vPersp = viewsCameraRef.current.perspective;
        const theta = vPersp.theta ?? WGPU_ISO_THETA;
        const phi = vPersp.phi ?? WGPU_ISO_PHI;
        const px = vPersp.target.x + vPersp.radius * Math.sin(phi) * Math.cos(theta);
        const py = vPersp.target.y + vPersp.radius * Math.cos(phi);
        const pz = vPersp.target.z + vPersp.radius * Math.sin(phi) * Math.sin(theta);
        perspCam.position.set(px, py, pz);
        perspCam.lookAt(vPersp.target);
        perspCam.aspect = aspect;
        perspCam.updateProjectionMatrix();
        setC4dWireframeOnly(renderMode === 'wireframe');
        renderer.render(scene, perspCam);

        // 2. The other 3 views: Isometric/Orthographic with black lines only (polygons transparent)
        // Top-Right (WebGL: halfW, halfH): TOP View (XZ Ortho)
        // Bottom-Left (WebGL: 0, 0): RIGHT View (YZ Ortho)
        // Bottom-Right (WebGL: halfW, 0): FRONT View (XY Ortho)
        const orthoQuads: { x: number; y: number; idx: number; id: 'top' | 'right' | 'front' }[] = [
          { x: halfW, y: halfH, idx: 0, id: 'top' }, // Top-Right: TOP
          { x: 0, y: 0, idx: 2, id: 'right' }, // Bottom-Left: RIGHT
          { x: halfW, y: 0, idx: 1, id: 'front' }, // Bottom-Right: FRONT
        ];

        setC4dWireframeOnly(true);
        for (const q of orthoQuads) {
          renderer.setViewport(q.x, q.y, halfW, halfH);
          renderer.setScissor(q.x, q.y, halfW, halfH);
          const v = viewsCameraRef.current[q.id];
          const frustumSize = v.radius * 1.1;
          quadCam.left = (-frustumSize * aspect) / 2;
          quadCam.right = (frustumSize * aspect) / 2;
          quadCam.top = frustumSize / 2;
          quadCam.bottom = -frustumSize / 2;
          quadCam.position
            .copy(orthoViews[q.idx].pos)
            .setLength(v.radius * 1.5)
            .add(v.target);
          quadCam.up.copy(orthoViews[q.idx].up);
          quadCam.lookAt(v.target);
          quadCam.updateProjectionMatrix();
          renderer.render(scene, quadCam);
        }
        setC4dWireframeOnly(renderMode === 'wireframe');
      }
    };

    animate();

    const resizeObserver = new ResizeObserver(() => {
      if (!container || !rendererRef.current) return;
      const nw = Math.max(1, container.clientWidth);
      const nh = Math.max(1, container.clientHeight);
      rendererRef.current.setSize(nw, nh);
      syncCamerasFromOrbit();
    });
    resizeObserver.observe(container);

    return () => {
      cancelAnimationFrame(animId);
      resizeObserver.disconnect();
      container.removeEventListener('mousedown', handleNativeMiddleDown);
      container.removeEventListener('auxclick', handleNativeMiddleDown);
      pmremGenerator.dispose();
      renderer.dispose();
    };
  }, []);

  // Auto-fit camera when preset/fitTrigger changes
  useEffect(() => {
    fitCameraToParts(parts);
  }, [fitTrigger]);

  // Rebuild Three.js meshes & WGPU debug overlays when parts, joints, or renderMode change
  useEffect(() => {
    const group = partGroupRef.current;
    const jGroup = jointGroupRef.current;
    if (!group || !jGroup) return;

    while (group.children.length > 0) {
      const child = group.children[0] as THREE.Mesh;
      if (child.geometry) child.geometry.dispose();
      group.remove(child);
    }
    while (jGroup.children.length > 0) {
      jGroup.remove(jGroup.children[0]);
    }

    for (const part of parts) {
      if (!part.visible) continue;

      const geom = new THREE.BufferGeometry();
      const verts = new Float32Array(part.mesh.vertices);

      if (
        renderMode === 'fea_heatmap' &&
        part.mesh.feaDisplacements &&
        part.mesh.feaDisplacements.length === verts.length
      ) {
        const disp = part.mesh.feaDisplacements;
        for (let i = 0; i < verts.length; i++) {
          verts[i] += disp[i] * deformScale;
        }
      }

      geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
      geom.setAttribute(
        'normal',
        new THREE.BufferAttribute(part.mesh.normals, 3)
      );
      geom.setIndex(new THREE.BufferAttribute(part.mesh.indices, 1));

      const matSpec = getMaterialById(part.materialId, materials);
      const isSelected = part.id === selectedPartId;

      let material: THREE.Material;

      if (renderMode === 'normals') {
        // Matches WGPU World-Space Normals shader (rgb = normalize(worldNormal) * 0.5 + 0.5)
        material = createWorldSpaceNormalMaterial();
      } else if (renderMode === 'backfaces') {
        // Matches WGPU Backfaces shader: normal metallic shaded front-faces, bright #ff007f magenta on back-faces
        material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(matSpec.colorHex),
          roughness: matSpec.roughness,
          metalness: matSpec.metalness,
          side: THREE.FrontSide,
        });
      } else if (renderMode === 'boundary_edges') {
        // Matches WGPU Boundary Edges mode: slate body so #ffd700 yellow open boundary edges stand out
        material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(matSpec.colorHex),
          roughness: matSpec.roughness,
          metalness: matSpec.metalness,
        });
      } else if (renderMode === 'wireframe') {
        // WGPU Wireframe mode: shaded steel solid + crisp dark triangulation lines
        material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(matSpec.colorHex),
          roughness: matSpec.roughness,
          metalness: matSpec.metalness,
          polygonOffset: true,
          polygonOffsetFactor: 1,
          polygonOffsetUnits: 1,
        });
      } else if (renderMode === 'fea_heatmap' && part.mesh.feaStressMPa) {
        const stresses = part.mesh.feaStressMPa;
        const maxStress = Math.max(feaResult?.maxVonMisesMPa || 100, 1);
        const colors = new Float32Array(part.mesh.vertexCount * 3);

        for (let i = 0; i < part.mesh.vertexCount; i++) {
          const [r, g, b] = vonMisesToRgb(stresses[i] / maxStress);
          colors[i * 3] = r;
          colors[i * 3 + 1] = g;
          colors[i * 3 + 2] = b;
        }
        geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        material = new THREE.MeshStandardMaterial({
          vertexColors: true,
          roughness: 0.35,
          metalness: 0.15,
        });
      } else {
        material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(matSpec.colorHex),
          roughness: matSpec.roughness,
          metalness: matSpec.metalness,
          flatShading: renderMode === 'flat_shading',
          emissive: isSelected
            ? new THREE.Color(0x1e293b)
            : new THREE.Color(0x000000),
          emissiveIntensity: isSelected ? 0.22 : 0,
          polygonOffset: true,
          polygonOffsetFactor: 1,
          polygonOffsetUnits: 1,
        });
      }

      const meshObj = new THREE.Mesh(geom, material);
      meshObj.position.set(...part.position);
      meshObj.rotation.set(...part.rotation);
      meshObj.scale.set(...part.scale);
      meshObj.castShadow = true;
      meshObj.receiveShadow = true;
      meshObj.userData = { partId: part.id };

      // Cinema 4D Wireframe Overlay for Orthographic Viewports:
      // Crisp pure black polygon lines connecting vertices (Cinema 4D "Lines / Wireframe" mode)
      const c4dWfGeom = new THREE.WireframeGeometry(geom);
      const c4dLinesMat = new THREE.LineBasicMaterial({
        color: 0x000000,
        linewidth: 1,
        transparent: false,
        depthTest: true,
      });
      const c4dLines = new THREE.LineSegments(c4dWfGeom, c4dLinesMat);
      c4dLines.renderOrder = 2;
      c4dLines.userData = { isC4dLines: true };
      c4dLines.visible = renderMode === 'wireframe';
      meshObj.add(c4dLines);

      // 1. WGPU Wireframe Mode Overlay: full dark triangle wireframe matching `spur_gear_hub_keyway_wireframe.png`
      if (renderMode === 'wireframe') {
        const wfGeom = new THREE.WireframeGeometry(geom);
        const wfMat = new THREE.LineBasicMaterial({
          color: 0x141820,
          transparent: true,
          opacity: 0.82,
        });
        const wfMesh = new THREE.LineSegments(wfGeom, wfMat);
        wfMesh.userData = { isWfOverlay: true };
        meshObj.add(wfMesh);
      }

      // 2. WGPU Backfaces Mode Overlay: magenta #ff007f on all back-facing triangles (!gl_FrontFacing)
      if (renderMode === 'backfaces') {
        const bfMat = new THREE.MeshBasicMaterial({
          color: 0xff007f,
          side: THREE.BackSide,
          polygonOffset: true,
          polygonOffsetFactor: 1,
          polygonOffsetUnits: 1,
        });
        const bfMesh = new THREE.Mesh(geom, bfMat);
        bfMesh.userData = { isBackfacesOverlay: true };
        meshObj.add(bfMesh);
      }

      // 3. WGPU Boundary Edges Mode Overlay: bright #ffd700 yellow lines ONLY on open boundary edges
      if (
        renderMode === 'boundary_edges' &&
        part.mesh.boundaryEdgePositions &&
        part.mesh.boundaryEdgePositions.length > 0
      ) {
        const bGeom = new THREE.BufferGeometry();
        bGeom.setAttribute(
          'position',
          new THREE.BufferAttribute(part.mesh.boundaryEdgePositions, 3)
        );
        const bMat = new THREE.LineBasicMaterial({
          color: 0xffd700,
          linewidth: 2,
        });
        const bMesh = new THREE.LineSegments(bGeom, bMat);
        bMesh.userData = { isBoundaryOverlay: true };
        meshObj.add(bMesh);
      }

      // 4. Crisp CAD Feature Edges in Shaded / Flat / FEA modes
      if (
        showCadEdges &&
        (renderMode === 'shaded' ||
          renderMode === 'flat_shading' ||
          renderMode === 'fea_heatmap')
      ) {
        const edgesGeom = new THREE.EdgesGeometry(geom, 22);
        const edgesMat = new THREE.LineBasicMaterial({
          color: isSelected ? 0x38bdf8 : 0x141820,
          transparent: true,
          opacity: isSelected ? 0.65 : 0.42,
        });
        const edgesMesh = new THREE.LineSegments(edgesGeom, edgesMat);
        edgesMesh.userData = { isCadEdgesOverlay: true };
        meshObj.add(edgesMesh);
      }

      group.add(meshObj);
    }

    if (gridHelperRef.current) {
      gridHelperRef.current.visible = showReferenceGrid;
    }
    if (axesHelperRef.current) {
      axesHelperRef.current.visible = showReferenceGrid;
    }

    // Visualize Kinematic Joint Axes & Anchors in 3D (only when reference helpers are enabled)
    if (showReferenceGrid) {
      for (const j of joints) {
        const anchorVec = new THREE.Vector3(...j.anchor);
        const axisVec = new THREE.Vector3(...j.axis).normalize();
        const arrowColor =
          j.type === 'gear'
            ? 0xf59e0b
            : j.motorEnabled
              ? 0x10b981
              : 0x38bdf8;
        const arrow = new THREE.ArrowHelper(
          axisVec,
          anchorVec,
          0.035,
          arrowColor,
          0.008,
          0.005
        );
        jGroup.add(arrow);
      }
    }
  }, [parts, joints, selectedPartId, renderMode, showCadEdges, showReferenceGrid, materials, feaResult, deformScale]);

  const getViewportAtCoords = (clientX: number, clientY: number): CadViewportId => {
    if (layoutRef.current === 'single') {
      return activeViewRef.current;
    }
    if (!containerRef.current) return 'perspective';
    const rect = containerRef.current.getBoundingClientRect();
    const relX = clientX - rect.left;
    const relY = clientY - rect.top;
    const halfW = rect.width / 2;
    const halfH = rect.height / 2;

    if (relX < halfW && relY < halfH) {
      // Top-Left: 3D Perspective View (Cinema 4D)
      return 'perspective';
    } else if (relX >= halfW && relY < halfH) {
      // Top-Right: TOP View (XZ Ortho)
      return 'top';
    } else if (relX < halfW && relY >= halfH) {
      // Bottom-Left: RIGHT View (YZ Ortho)
      return 'right';
    } else {
      // Bottom-Right: FRONT View (XY Ortho)
      return 'front';
    }
  };

  const handleMiddleClickToggle = (clientX: number, clientY: number) => {
    if (!containerRef.current) {
      if (onChangeViewportLayout) {
        onChangeViewportLayout(layoutRef.current === 'single' ? 'quad' : 'single');
      } else {
        onToggleViewportLayout?.();
      }
      return;
    }

    if (layoutRef.current === 'single') {
      // Switch from single view to 4-view quad split
      if (onChangeViewportLayout) {
        onChangeViewportLayout('quad');
      } else {
        onToggleViewportLayout?.();
      }
    } else {
      // In 4-view quad split:
      // Clicking mouse wheel in a quadrant maximizes that viewport!
      const clickedVp = getViewportAtCoords(clientX, clientY);
      setActiveView(clickedVp);

      if (onChangeViewportLayout) {
        onChangeViewportLayout('single');
      } else {
        onToggleViewportLayout?.();
      }
    }
  };

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    const vp = getViewportAtCoords(e.clientX, e.clientY);
    activeInteractionViewportRef.current = vp;

    if (e.button === 1) {
      e.preventDefault();
      middleDownRef.current = {
        x: e.clientX,
        y: e.clientY,
        time: performance.now(),
      };
    }
    orbitRef.current.isDragging = true;
    orbitRef.current.isPanning = e.button === 2 || e.button === 1 || e.shiftKey;
    orbitRef.current.lastX = e.clientX;
    orbitRef.current.lastY = e.clientY;
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    if (!orbitRef.current.isDragging) return;
    const dx = e.clientX - orbitRef.current.lastX;
    const dy = e.clientY - orbitRef.current.lastY;
    orbitRef.current.lastX = e.clientX;
    orbitRef.current.lastY = e.clientY;

    const vp = activeInteractionViewportRef.current;
    const cam = viewsCameraRef.current[vp];
    const isPan = orbitRef.current.isPanning || vp !== 'perspective';

    if (vp === 'perspective') {
      if (isPan) {
        const panSpeed = cam.radius * 0.0015;
        cam.target.x -= dx * panSpeed;
        cam.target.y += dy * panSpeed;
        orbitRef.current.target.copy(cam.target);
      } else {
        cam.theta = (cam.theta ?? WGPU_ISO_THETA) + dx * 0.008;
        cam.phi = Math.max(
          0.12,
          Math.min(Math.PI - 0.12, (cam.phi ?? WGPU_ISO_PHI) - dy * 0.008)
        );
        orbitRef.current.theta = cam.theta;
        orbitRef.current.phi = cam.phi;
      }
      syncCamerasFromOrbit();
    } else if (vp === 'top') {
      // Top ortho view: looking down Y axis onto XZ plane. Up vector is -Z (0, 0, -1).
      // Screen X moves target.X, Screen Y moves target.Z (down screen = +Z in world)
      const panSpeed = cam.radius * 0.0018;
      cam.target.x -= dx * panSpeed;
      cam.target.z -= dy * panSpeed;
    } else if (vp === 'right') {
      // Right ortho view: looking down X axis onto ZY plane. Up vector is +Y (0, 1, 0).
      // Screen X moves target.Z, Screen Y moves target.Y
      const panSpeed = cam.radius * 0.0018;
      cam.target.z += dx * panSpeed;
      cam.target.y += dy * panSpeed;
    } else if (vp === 'front') {
      // Front ortho view: looking along Z axis onto XY plane. Up vector is +Y (0, 1, 0).
      // Screen X moves target.X, Screen Y moves target.Y
      const panSpeed = cam.radius * 0.0018;
      cam.target.x -= dx * panSpeed;
      cam.target.y += dy * panSpeed;
    }
  };

  const handleMouseUp = (e: React.MouseEvent<HTMLDivElement>) => {
    if (e.button === 1) {
      e.preventDefault();
      if (middleDownRef.current) {
        const dx = e.clientX - middleDownRef.current.x;
        const dy = e.clientY - middleDownRef.current.y;
        const dist = Math.hypot(dx, dy);
        // If distance < 8 pixels, it's a click on the mouse wheel (not a drag)
        if (dist < 8) {
          handleMiddleClickToggle(e.clientX, e.clientY);
        }
        middleDownRef.current = null;
      }
    }
    orbitRef.current.isDragging = false;
  };

  const handleMouseLeave = () => {
    orbitRef.current.isDragging = false;
    middleDownRef.current = null;
  };

  const handleAuxClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (e.button === 1) {
      e.preventDefault();
    }
  };

  const handleWheel = (e: React.WheelEvent<HTMLDivElement>) => {
    // Zoom only the specific hovered quadrant
    const vp = getViewportAtCoords(e.clientX, e.clientY);
    const factor = e.deltaY > 0 ? 1.1 : 0.9;
    const cam = viewsCameraRef.current[vp];
    cam.radius = Math.max(0.015, Math.min(8.0, cam.radius * factor));

    if (vp === 'perspective') {
      orbitRef.current.radius = cam.radius;
      syncCamerasFromOrbit();
    }
  };

  const snapCameraView = (theta: number, phi: number) => {
    viewsCameraRef.current.perspective.theta = theta;
    viewsCameraRef.current.perspective.phi = phi;
    orbitRef.current.theta = theta;
    orbitRef.current.phi = phi;
    syncCamerasFromOrbit();
  };

  const activePart = parts.find((p) => p.id === selectedPartId) || parts[0];

  return (
    <div
      className="relative flex-1 h-full bg-[#262b33] overflow-hidden select-none"
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseLeave}
      onAuxClick={handleAuxClick}
      onWheel={handleWheel}
      onContextMenu={(e) => e.preventDefault()}
    >
      {/* WebGL Canvas Container */}
      <div ref={containerRef} className="w-full h-full cursor-grab active:cursor-grabbing" />

      {/* Windows WGPU Golden PNG Reference Overlay (1:1 Parity Verification) */}
      {compareOverlayActive && windowsWgpuArtifactUrl && (
        <div
          className="absolute inset-0 pointer-events-none flex items-center justify-center"
          style={{ opacity: compareOpacity / 100 }}
        >
          <img
            src={windowsWgpuArtifactUrl}
            alt="Windows WGPU Golden Render"
            className="w-full h-full object-contain"
          />
          <div className="absolute top-14 left-1/2 -translate-x-1/2 px-3 py-1 rounded bg-[#0f172ae6] border border-[#f59e0b]/70 text-[11px] font-mono text-[#f59e0b] shadow-lg">
            Эталон Windows WGPU: {windowsWgpuArtifactUrl} ({compareOpacity}% наложение)
          </div>
        </div>
      )}

      {/* Top-Left HUD: WGPU Shader & Camera Telemetry */}
      <div className="absolute top-3 left-3 pointer-events-none flex flex-col gap-1.5">
        {!minimalistWindow && (
          <>
            <div className="px-2.5 py-1.5 rounded bg-[#12151ce6] border border-[#262b36] flex items-center gap-2">
              <Compass className="w-3.5 h-3.5 text-[#38bdf8]" />
              <span className="text-[11px] font-mono text-[#e2e8f0] uppercase">
                {viewportLayout === 'quad'
                  ? 'QUAD SPLIT (3D PERSPECTIVE / TOP / RIGHT / FRONT)'
                  : activeView === 'perspective'
                    ? `${projectionMode.toUpperCase()} VIEWPORT`
                    : `${activeView.toUpperCase()} ORTHO VIEW (WIREFRAME)`}
              </span>
              <span className="text-[10px] font-mono text-[#f59e0b] px-1.5 py-0.5 rounded bg-[#1b202b]">
                {viewportLayout === 'quad'
                  ? '4-VIEW C4D'
                  : activeView === 'perspective'
                    ? renderMode.toUpperCase()
                    : 'WIREFRAME'}
              </span>
            </div>

            <div className="px-2.5 py-1 rounded bg-[#12151ce6] border border-[#262b36] text-[10px] font-mono text-[#94a3b8] flex items-center gap-3">
              <span>AZ: {cameraHud.azDeg}°</span>
              <span>EL: {cameraHud.elDeg}°</span>
              <span>DIST: {cameraHud.distMm}mm</span>
            </div>
          </>
        )}

        {/* Diagnostic Shader Banner when in Backfaces or Boundary Edges mode */}
        {(renderMode === 'boundary_edges' || renderMode === 'backfaces') &&
          activePart && (
            <div className="px-2.5 py-1.5 rounded bg-[#12151ce6] border border-[#10b981]/50 text-[10px] font-mono text-[#10b981] flex items-center gap-1.5">
              <ShieldCheck className="w-3.5 h-3.5" />
              {renderMode === 'boundary_edges' ? (
                <span>
                  OPEN BOUNDARY EDGES: {activePart.mesh.boundaryEdgesCount} (
                  {activePart.mesh.isWatertight ? 'WATERTIGHT SOLID' : 'OPEN'})
                </span>
              ) : (
                <span>
                  INVERTED BACKFACES:{' '}
                  {activePart.mesh.isOutwardOriented ? 0 : activePart.mesh.triangleCount}{' '}
                  (OUTWARD ORIENTED)
                </span>
              )}
            </div>
          )}
      </div>

      {/* Top-Right HUD: Windows WGPU 1:1 Overlay Comparison & Camera Presets */}
      <div className="absolute top-3 right-3 flex items-center gap-1.5">
        {windowsWgpuArtifactUrl && (
          <div className="flex items-center gap-1.5 bg-[#12151ce6] px-2 py-1 rounded border border-[#2a3242]">
            <button
              onClick={() => {
                const next = !compareOverlayActive;
                setCompareOverlayActive(next);
                if (next) {
                  fitCameraToParts(parts);
                }
              }}
              title="Наложить эталонный рендер Windows WGPU (/qa/artifacts/*.png) поверх WebGL для проверки 100% совпадения"
              className={`px-2 py-0.5 rounded text-[10px] font-mono flex items-center gap-1 cursor-pointer transition-colors ${
                compareOverlayActive
                  ? 'bg-[#f59e0b] text-[#0c0e12] font-semibold'
                  : 'text-[#cbd5e1] hover:text-white hover:bg-[#1e2532]'
              }`}
            >
              <Layers className="w-3 h-3" />
              <span>Эталон Win WGPU {compareOverlayActive ? 'ВКЛ' : '1:1'}</span>
            </button>
            {compareOverlayActive && (
              <div className="flex items-center gap-1.5 pl-1 border-l border-[#2a3242]">
                <Eye className="w-3 h-3 text-[#f59e0b]" />
                <input
                  type="range"
                  aria-label="Прозрачность наложения эталона Windows WGPU"
                  min={0}
                  max={100}
                  step={5}
                  value={compareOpacity}
                  onChange={(e) => setCompareOpacity(Number(e.target.value))}
                  className="w-16 accent-[#f59e0b] h-1 bg-[#1e2430] rounded cursor-pointer"
                />
                <span className="text-[10px] font-mono text-[#f59e0b] w-7">
                  {compareOpacity}%
                </span>
              </div>
            )}
          </div>
        )}

        {!minimalistWindow && (
          <div className="flex items-center gap-1 bg-[#12151ce6] p-1 rounded border border-[#262b36]">
            <button
              onClick={() => {
                setActiveView('perspective');
                snapCameraView(WGPU_ISO_THETA, WGPU_ISO_PHI);
              }}
              className={`px-2 py-1 text-[10px] font-mono rounded cursor-pointer font-semibold ${
                activeView === 'perspective'
                  ? 'bg-[#1e2532] text-[#38bdf8]'
                  : 'text-[#cbd5e1] hover:bg-[#1e2532]'
              }`}
            >
              ISO
            </button>
            <button
              onClick={() => {
                setActiveView('front');
              }}
              className={`px-2 py-1 text-[10px] font-mono rounded cursor-pointer ${
                activeView === 'front'
                  ? 'bg-[#1e2532] text-[#38bdf8] font-semibold'
                  : 'text-[#cbd5e1] hover:bg-[#1e2532]'
              }`}
            >
              FRONT
            </button>
            <button
              onClick={() => {
                setActiveView('right');
              }}
              className={`px-2 py-1 text-[10px] font-mono rounded cursor-pointer ${
                activeView === 'right'
                  ? 'bg-[#1e2532] text-[#38bdf8] font-semibold'
                  : 'text-[#cbd5e1] hover:bg-[#1e2532]'
              }`}
            >
              RIGHT
            </button>
            <button
              onClick={() => {
                setActiveView('top');
              }}
              className={`px-2 py-1 text-[10px] font-mono rounded cursor-pointer ${
                activeView === 'top'
                  ? 'bg-[#1e2532] text-[#38bdf8] font-semibold'
                  : 'text-[#cbd5e1] hover:bg-[#1e2532]'
              }`}
            >
              TOP
            </button>
            <button
              onClick={() => fitCameraToParts(parts)}
              title="Fit Camera to Active Parts (F)"
              className="p-1 text-[#94a3b8] hover:text-[#f1f5f9] cursor-pointer"
            >
              <Maximize2 className="w-3 h-3" />
            </button>
          </div>
        )}
      </div>

      {/* Quad Viewport Labels Overlay (Cinema 4D layout: Top-Left Perspective 3D, others Ortho with lines) */}
      {viewportLayout === 'quad' && (
        <div className="absolute inset-0 pointer-events-none">
          <div className="absolute inset-x-0 top-1/2 h-[1px] bg-[#38bdf8]/35" />
          <div className="absolute inset-y-0 left-1/2 w-[1px] bg-[#38bdf8]/35" />
          {/* Top-Left: 3D Perspective */}
          <span className="absolute top-14 left-3 px-1.5 py-0.5 bg-[#0f1117]/85 text-[10px] font-mono text-[#f59e0b] rounded border border-[#262b36]">
            3D PERSPECTIVE (ORBIT)
          </span>
          {/* Top-Right: Top View */}
          <span className="absolute top-14 right-3 px-1.5 py-0.5 bg-[#0f1117]/85 text-[10px] font-mono text-[#38bdf8] rounded border border-[#262b36]">
            TOP (XZ ORTHO · LINES)
          </span>
          {/* Bottom-Left: Right View */}
          <span className="absolute bottom-3 left-3 px-1.5 py-0.5 bg-[#0f1117]/85 text-[10px] font-mono text-[#38bdf8] rounded border border-[#262b36]">
            RIGHT (YZ ORTHO · LINES)
          </span>
          {/* Bottom-Right: Front View */}
          <span className="absolute bottom-3 right-3 px-1.5 py-0.5 bg-[#0f1117]/85 text-[10px] font-mono text-[#38bdf8] rounded border border-[#262b36]">
            FRONT (XY ORTHO · LINES)
          </span>
          <div className="absolute top-14 left-1/2 -translate-x-1/2 px-2.5 py-1 bg-[#0f1117]/90 text-[10px] font-mono text-[#38bdf8] rounded border border-[#38bdf8]/40 shadow-lg pointer-events-none">
            Клик колесиком мыши (СКМ): развернуть выбранный вид
          </div>
        </div>
      )}

      {/* Maximized Single Orthographic View Banner */}
      {viewportLayout === 'single' && activeView !== 'perspective' && (
        <div className="absolute top-14 left-1/2 -translate-x-1/2 px-3 py-1 bg-[#0f1117]/90 text-[11px] font-mono text-[#38bdf8] rounded border border-[#38bdf8]/40 shadow-lg pointer-events-none flex items-center gap-2">
          <span className="font-semibold uppercase">{activeView} VIEW (WIREFRAME)</span>
          <span className="text-[#94a3b8]">· Клик колесиком мыши (СКМ): вернуться к 4 видам</span>
        </div>
      )}

      {/* FEA Von Mises Stress Colormap Legend & Deformation Scale Overlay */}
      {renderMode === 'fea_heatmap' && feaResult && (
        <div className="absolute bottom-4 right-4 bg-[#12151ce6] border border-[#2a3140] rounded-md p-3 w-56 shadow-xl">
          <div className="text-[10px] font-mono uppercase tracking-wider text-[#cbd5e1] mb-1.5 flex items-center justify-between">
            <span>Von Mises Stress</span>
            <span className="text-[#f59e0b]">MPa</span>
          </div>
          <div
            className="h-3 w-full rounded-sm mb-1.5"
            style={{
              background:
                'linear-gradient(to right, #0d25f2, #0dd9f2, #10b981, #eab308, #ef4444)',
            }}
          />
          <div className="flex justify-between text-[10px] font-mono text-[#94a3b8]">
            <span>{feaResult.minVonMisesMPa.toFixed(1)}</span>
            <span>{(feaResult.maxVonMisesMPa * 0.5).toFixed(1)}</span>
            <span className="text-[#f87171] font-semibold">
              {feaResult.maxVonMisesMPa.toFixed(1)}
            </span>
          </div>

          <div className="mt-2 pt-2 border-t border-[#262b36]">
            <div className="flex justify-between text-[10px] font-mono text-[#94a3b8] mb-1">
              <span>Deform Exaggeration:</span>
              <span className="text-[#38bdf8]">{deformScale.toFixed(0)}x</span>
            </div>
            <input
              type="range"
              min={1}
              max={50}
              step={1}
              value={deformScale}
              onChange={(e) => onChangeDeformScale(Number(e.target.value))}
              className="w-full accent-[#f59e0b] h-1 bg-[#1e2430] rounded cursor-pointer"
            />
          </div>

          <div className="mt-2 pt-2 border-t border-[#262b36] flex flex-col gap-1 text-[10px] font-mono">
            <div className="flex justify-between">
              <span className="text-[#64748b]">Max Deflection:</span>
              <span className="text-[#38bdf8]">
                {feaResult.maxDisplacementMm.toFixed(3)} mm
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-[#64748b]">Yield Strength:</span>
              <span className="text-[#cbd5e1]">
                {feaResult.yieldStrengthMPa.toFixed(0)} MPa
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-[#64748b]">Safety Factor:</span>
              <span
                className={
                  feaResult.safetyFactor >= 1.5
                    ? 'text-[#10b981] font-semibold'
                    : feaResult.safetyFactor >= 1.0
                      ? 'text-[#f59e0b] font-semibold'
                      : 'text-[#ef4444] font-semibold'
                }
              >
                {feaResult.safetyFactor.toFixed(2)} ({feaResult.status})
              </span>
            </div>
          </div>
        </div>
      )}

      {/* Bottom-Left Part Switcher Bar (for Multi-Part Assemblies) */}
      {parts.length > 1 && (
        <div className="absolute bottom-3 left-3 flex items-center gap-1.5 bg-[#12151ce6] p-1.5 rounded border border-[#262b36]">
          <span className="text-[10px] font-mono text-[#64748b] px-1">
            Assembly Parts:
          </span>
          {parts.map((p) => (
            <button
              key={p.id}
              onClick={() => onSelectPart(p.id)}
              className={`px-2 py-1 rounded text-[11px] font-mono transition-colors cursor-pointer ${
                p.id === selectedPartId
                  ? 'bg-[#38bdf8] text-[#0c0e12] font-semibold'
                  : 'bg-[#1a1f29] text-[#cbd5e1] hover:bg-[#252c3a]'
              }`}
            >
              {p.name}
            </button>
          ))}
        </div>
      )}
    </div>
  );
};
