import { spawn, execSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';

interface BenchmarkStep {
  tool: string;
  params: Record<string, any>;
}

interface BenchmarkTask {
  id: string;
  category: string;
  prompt_ru: string;
  target_part_id: string;
  expected_dimensions_m: [number, number, number];
  expected_genus: number;
  expected_fea?: {
    verdict: string;
    stress_mpa: number;
    displacement_mm?: number;
    tolerance_pct?: number;
  };
  expected_kinematics?: {
    type: 'gear_pair' | 'slider_crank';
    driver_id?: string;
    driven_id?: string;
    expected_velocity_ratio?: number;
    max_drift_m?: number;
    slider_id?: string;
    r_m?: number;
    l_m?: number;
    omega_rad_s?: number;
    max_position_error_pct?: number;
  };
  steps: BenchmarkStep[];
}

export interface RustBenchmarkTaskResult {
  id: string;
  category: string;
  passed: boolean;
  calls_succeeded: boolean;
  is_watertight: boolean;
  actual_genus: number | null;
  expected_genus: number;
  actual_dims_m: [number, number, number] | null;
  expected_dims_m: [number, number, number];
  max_dimension_error_pct: number;
  fact_vs_reference: string;
  failure_reason: string;
  duration_ms: number;
}

type ExecutionMode =
  | 'strict_as_is'
  | 'precreate_parts_verbatim_steps'
  | 'schema_bridged_diagnostic';

function resolveServerBinary(): string {
  const rootDir = process.cwd();
  const releaseBin = path.join(rootDir, 'target', 'release', 'mcp-cad-server');
  const debugBin = path.join(rootDir, 'target', 'debug', 'mcp-cad-server');

  if (fs.existsSync(releaseBin)) {
    return releaseBin;
  }
  if (fs.existsSync(debugBin)) {
    return debugBin;
  }

  const envWithCargo = {
    ...process.env,
    PATH: `/root/.cargo/bin:${process.env.PATH || ''}`,
  };
  execSync('cargo build -p mcp-cad-server --bin mcp-cad-server', {
    cwd: rootDir,
    env: envWithCargo,
    stdio: 'inherit',
  });
  return debugBin;
}

function mapTsMaterialToRust(tsMat?: string): string {
  switch (tsMat || 'steel_1045') {
    case 'steel_1045':
    case 'steel_structural_aisi1020':
      return 'steel_structural_aisi1020';
    case 'steel_4140':
    case 'steel_high_strength_4140':
      return 'steel_high_strength_4140';
    case 'alu_6061':
    case 'aluminum_6061_t6':
      return 'aluminum_6061_t6';
    case 'ti_6al_4v':
    case 'titanium_grade5_ti6al4v':
      return 'titanium_grade5_ti6al4v';
    case 'brass_c360':
      return 'brass_c360';
    case 'nylon_pa12':
      return 'nylon_pa12';
    case 'pla_3d':
    case 'pla_filament_3dprint':
      return 'pla_filament_3dprint';
    case 'petg_3d':
    case 'petg_filament_3dprint':
      return 'petg_filament_3dprint';
    default:
      return 'steel_structural_aisi1020';
  }
}

class StdioMcpClient {
  private proc;
  private rl;
  private nextId = 1;
  private pending = new Map<number, (resp: any) => void>();

  constructor(binPath: string) {
    this.proc = spawn(binPath, ['--headless'], {
      stdio: ['pipe', 'pipe', 'ignore'],
    });
    this.rl = readline.createInterface({
      input: this.proc.stdout,
      crlfDelay: Infinity,
    });
    this.rl.on('line', (line) => {
      const trimmed = line.trim();
      if (!trimmed) return;
      try {
        const parsed = JSON.parse(trimmed);
        if (parsed && typeof parsed.id === 'number') {
          const cb = this.pending.get(parsed.id);
          if (cb) {
            this.pending.delete(parsed.id);
            cb(parsed);
          }
        }
      } catch {
        // Ignore non-JSON lines if any
      }
    });
  }

  async callTool(name: string, args: Record<string, any>): Promise<any> {
    const id = this.nextId++;
    const req = {
      jsonrpc: '2.0',
      id,
      method: 'tools/call',
      params: {
        name,
        arguments: args,
      },
    };
    return new Promise((resolve) => {
      this.pending.set(id, resolve);
      this.proc.stdin.write(JSON.stringify(req) + '\n');
    });
  }

  close(): void {
    this.rl.close();
    this.proc.stdin.end();
    this.proc.kill();
  }
}

function extractToolJson(resp: any): { ok: true; value: any } | { ok: false; error: string } {
  if (resp?.error) {
    return { ok: false, error: `JSON-RPC error: ${JSON.stringify(resp.error)}` };
  }
  const isError = Boolean(resp?.result?.isError);
  const text = resp?.result?.content?.[0]?.text ?? '';
  if (isError) {
    return { ok: false, error: String(text) };
  }
  try {
    return { ok: true, value: JSON.parse(text) };
  } catch (e) {
    return { ok: false, error: `Invalid JSON output: ${text}` };
  }
}

async function runSingleTaskOnRust(
  client: StdioMcpClient,
  task: BenchmarkTask,
  mode: ExecutionMode
): Promise<RustBenchmarkTaskResult> {
  const t0 = performance.now();

  let callsSucceeded = true;
  let failedStepInfo: string | null = null;
  let lastFeaJson: any = null;
  let lastSimJson: any = null;
  const jointIdMap = new Map<string, string>();
  const partZSpan = new Map<string, number>();

  // Clean up any existing parts in the shared server session before starting the task
  const listResp = await client.callTool('list_parts', {});
  const listParsed = extractToolJson(listResp);
  if (listParsed.ok && Array.isArray(listParsed.value?.parts)) {
    for (const p of listParsed.value.parts) {
      const pid = p.part_id || p.id;
      if (typeof pid === 'string') {
        await client.callTool('delete_part', { part_id: pid });
      }
    }
  }

  if (mode !== 'strict_as_is') {
    const createdParts = new Set<string>();
    for (const step of task.steps) {
      if (step.tool === 'add_feature_primitive') {
        const partId = step.params?.part_id;
        if (typeof partId === 'string' && !createdParts.has(partId)) {
          createdParts.add(partId);
          const mat = mapTsMaterialToRust(step.params?.material);
          const isFixed =
            mode === 'schema_bridged_diagnostic' &&
            (partId.includes('housing') || partId.includes('base'));
          const resp = await client.callTool('create_part', {
            part_id: partId,
            name: partId,
            material: mat,
            is_fixed: isFixed,
          });
          const parsed = extractToolJson(resp);
          if (!parsed.ok) {
            callsSucceeded = false;
            failedStepInfo = `pre-create_part(${partId}): ${parsed.error}`;
            break;
          }
        }
      }
    }
  }

  if (callsSucceeded) {
    for (let idx = 0; idx < task.steps.length; idx++) {
      const step = task.steps[idx];
      const args = JSON.parse(JSON.stringify(step.params));

      if (mode === 'schema_bridged_diagnostic') {
        if (step.tool === 'add_feature_primitive') {
          const partId = String(args.part_id || '');
          const primType = String(args.primitive_type || '');
          if (primType === 'box' && typeof args.params?.depth === 'number') {
            partZSpan.set(partId, args.params.depth);
          } else if (primType === 'revolve' && args.params) {
            const prof = args.params.profile;
            const innerR = args.params.inner_radius;
            if (Array.isArray(prof) && prof.length === 2 && typeof innerR === 'number') {
              const rOut = Number(prof[0][0]);
              const zMin = Number(prof[0][1]);
              const zMax = Number(prof[1][1]);
              args.params.profile = [
                [innerR, zMin],
                [rOut, zMin],
                [rOut, zMax],
                [innerR, zMax],
              ];
            }
          } else if (primType === 'polygon_prism' && args.params) {
            if (typeof args.params.height === 'number') {
              partZSpan.set(partId, args.params.height);
            }
            if (args.params.outer_radius !== undefined && args.params.radius === undefined) {
              args.params.radius = args.params.outer_radius;
            }
            if (args.params.chamfer_radius !== undefined && args.params.chamfer === undefined) {
              args.params.chamfer = args.params.chamfer_radius;
            }
          }
        } else if (step.tool === 'cut_hole') {
          const partId = String(args.part_id || '');
          if (args.through && args.depth === undefined) {
            const zThickness = partZSpan.get(partId) ?? 0.04;
            args.depth = zThickness;
            if (
              Array.isArray(args.position) &&
              Array.isArray(args.normal) &&
              Math.abs(Number(args.position[2])) < 1e-9 &&
              Math.abs(Number(args.normal[2]) + 1.0) < 1e-6
            ) {
              args.position = [args.position[0], args.position[1], zThickness * 0.5];
            }
          }
        } else if (step.tool === 'cut_helical_groove') {
          const partId = String(args.part_id || '');
          if (args.start_z === undefined && partZSpan.has(partId)) {
            args.start_z = -(partZSpan.get(partId) as number) * 0.5;
          }
          if (args.starts === undefined) {
            args.starts = 1;
          }
        } else if (step.tool === 'set_joint_motor') {
          const origId = String(args.joint_id || '');
          if (jointIdMap.has(origId)) {
            args.joint_id = jointIdMap.get(origId);
          }
        } else if (step.tool === 'run_physics_simulation') {
          if (args.step_size_ms === undefined && typeof args.dt === 'number') {
            args.step_size_ms = args.dt * 1000.0;
          }
        }
      }

      const resp = await client.callTool(step.tool, args);
      const parsed = extractToolJson(resp);
      if (!parsed.ok) {
        callsSucceeded = false;
        failedStepInfo = `step #${idx + 1} \`${step.tool}\`: ${parsed.error}`;
        break;
      }
      if (step.tool === 'create_joint') {
        const reqJid = step.params?.joint_id;
        const actJid = parsed.value?.joint_id;
        if (typeof reqJid === 'string' && typeof actJid === 'string') {
          jointIdMap.set(reqJid, actJid);
        }
      } else if (step.tool === 'run_fea_analysis') {
        lastFeaJson = parsed.value;
      } else if (step.tool === 'run_physics_simulation') {
        lastSimJson = parsed.value;
      }
    }
  }

  // Query target part geometry & validation from the Rust server
  let isWatertight = false;
  let actualGenus: number | null = null;
  let genusMatched = false;
  let actualDims: [number, number, number] | null = null;
  let maxDimErrPct = 100.0;
  let dimsWithin1Pct = false;

  const geomResp = await client.callTool('query_part_geometry', {
    part_id: task.target_part_id,
  });
  const geomParsed = extractToolJson(geomResp);
  if (geomParsed.ok && geomParsed.value?.faces_count > 0) {
    const mv = geomParsed.value.mesh_validation;
    isWatertight = Boolean(mv?.topology?.is_watertight);
    const gFloat = Number(mv?.topology?.genus ?? NaN);
    if (Number.isFinite(gFloat)) {
      actualGenus = Math.round(gFloat);
      genusMatched = Math.abs(gFloat - task.expected_genus) < 1e-6;
    }
    const dims = geomParsed.value?.bounding_box?.dimensions;
    if (Array.isArray(dims) && dims.length === 3) {
      actualDims = [Number(dims[0]), Number(dims[1]), Number(dims[2])];
      maxDimErrPct = Math.max(
        ...task.expected_dimensions_m.map(
          (exp, i) =>
            (Math.abs((actualDims as [number, number, number])[i] - exp) /
              Math.max(1e-9, exp)) *
            100.0
        )
      );
      dimsWithin1Pct = maxDimErrPct <= 1.0;
    }
  }

  let domainMatched = callsSucceeded && isWatertight && genusMatched;
  let factVsRef = actualDims
    ? `dims=[${actualDims.map((d) => d.toFixed(4)).join(',')}] vs [${task.expected_dimensions_m
        .map((d) => d.toFixed(4))
        .join(',')}] m, g=${actualGenus} (exp ${task.expected_genus})`
    : 'no target part mesh';

  if (task.expected_fea) {
    if (!lastFeaJson) {
      domainMatched = false;
      factVsRef = `missing FEA result (exp σ=${task.expected_fea.stress_mpa.toFixed(2)} MPa, verdict=${task.expected_fea.verdict})`;
    } else {
      const statusStr = String(lastFeaJson.status || 'UNKNOWN');
      const verdictSemanticMatch =
        (task.expected_fea.verdict === 'SAFE' && statusStr === 'PASS') ||
        statusStr === task.expected_fea.verdict;
      const actStress = Number(lastFeaJson.max_von_mises_mpa);
      const actDisp = Number(lastFeaJson.max_displacement_mm);
      const stressErrPct =
        (Math.abs(actStress - task.expected_fea.stress_mpa) /
          task.expected_fea.stress_mpa) *
        100.0;
      const tol = Number(task.expected_fea.tolerance_pct ?? 5.0);
      const dispErrPct =
        task.expected_fea.displacement_mm !== undefined
          ? (Math.abs(actDisp - task.expected_fea.displacement_mm) /
              task.expected_fea.displacement_mm) *
            100.0
          : null;
      const dispOk = dispErrPct === null || dispErrPct <= tol;
      domainMatched = verdictSemanticMatch && stressErrPct <= tol && dispOk;
      factVsRef =
        dispErrPct !== null
          ? `status=${statusStr} (exp ${task.expected_fea.verdict}), σ=${actStress.toFixed(2)} vs ${task.expected_fea.stress_mpa.toFixed(2)} MPa (err ${stressErrPct.toFixed(2)}%), δ=${actDisp.toFixed(4)} vs ${task.expected_fea.displacement_mm!.toFixed(4)} mm (err ${dispErrPct.toFixed(2)}%), tol=${tol.toFixed(1)}%`
          : `status=${statusStr} (exp ${task.expected_fea.verdict}), σ=${actStress.toFixed(2)} vs ${task.expected_fea.stress_mpa.toFixed(2)} MPa (err ${stressErrPct.toFixed(2)}%), δ=${actDisp.toFixed(4)} mm, tol=${tol.toFixed(1)}%`;
    }
  } else if (task.expected_kinematics) {
    if (!lastSimJson) {
      domainMatched = false;
      factVsRef = `missing simulation result (${task.expected_kinematics.type})`;
    } else {
      const traj = lastSimJson.trajectory_summary || {};
      if (task.expected_kinematics.type === 'gear_pair') {
        const driverId = task.expected_kinematics.driver_id || 't14_gear_z20';
        const drivenId = task.expected_kinematics.driven_id || 't14_gear_z40';
        const expRatio = task.expected_kinematics.expected_velocity_ratio ?? -0.5;
        const maxDriftLimit = task.expected_kinematics.max_drift_m ?? 0.001;
        const s1 = traj[driverId]?.samples;
        const s2 = traj[drivenId]?.samples;
        if (Array.isArray(s1) && Array.isArray(s2) && s1.length > 0 && s2.length > 0) {
          const w1 = Number(s1[s1.length - 1]?.angular_velocity?.[2] ?? 0);
          const w2 = Number(s2[s2.length - 1]?.angular_velocity?.[2] ?? 0);
          const ratio = Math.abs(w1) > 1e-6 ? w2 / w1 : 0;
          const p0 = s2[0]?.position || [0, 0, 0];
          const pLast = s2[s2.length - 1]?.position || [0, 0, 0];
          const drift = Math.hypot(
            Number(pLast[0]) - Number(p0[0]),
            Number(pLast[1]) - Number(p0[1]),
            Number(pLast[2]) - Number(p0[2])
          );
          domainMatched = Math.abs(ratio - expRatio) < 1e-3 && drift <= maxDriftLimit;
          factVsRef = `ω1=${w1.toFixed(3)}, ω2=${w2.toFixed(3)} rad/s, ω2/ω1=${ratio.toFixed(4)} vs ${expRatio.toFixed(4)}, drift=${drift.toFixed(4)} m (max ${maxDriftLimit.toFixed(4)} m)`;
        } else {
          domainMatched = false;
          factVsRef = 'missing gear trajectory samples';
        }
      } else if (task.expected_kinematics.type === 'slider_crank') {
        const sliderId = task.expected_kinematics.slider_id || 't15_piston_slider';
        const r = Number(task.expected_kinematics.r_m ?? 0.05);
        const l = Number(task.expected_kinematics.l_m ?? 0.2);
        const omega = Number(task.expected_kinematics.omega_rad_s ?? 2 * Math.PI);
        const tolPct = Number(task.expected_kinematics.max_position_error_pct ?? 2.0);
        const samples = traj[sliderId]?.samples;
        if (Array.isArray(samples) && samples.length > 0) {
          let maxErrPct = 0;
          let lastX = 0;
          let lastExpX = 0;
          for (const s of samples) {
            const t = Number(s.time_s ?? 0);
            const theta = omega * t;
            const expX =
              r * Math.cos(theta) +
              Math.sqrt(l * l - r * r * Math.pow(Math.sin(theta), 2));
            const actX = Number(s.position?.[0] ?? 0);
            const errPct = (Math.abs(actX - expX) / Math.max(1e-9, Math.abs(expX))) * 100.0;
            if (errPct > maxErrPct) maxErrPct = errPct;
            lastX = actX;
            lastExpX = expX;
          }
          domainMatched = samples.length > 10 && maxErrPct <= tolPct;
          factVsRef = `slider x_end=${lastX.toFixed(4)} vs ${lastExpX.toFixed(4)} m, max_x_err=${maxErrPct.toFixed(2)}% (tol ${tolPct.toFixed(1)}%)`;
        } else {
          domainMatched = false;
          factVsRef = 'missing slider trajectory samples';
        }
      }
    }
  }

  const passed =
    callsSucceeded && isWatertight && genusMatched && dimsWithin1Pct && domainMatched;

  const reasons: string[] = [];
  if (failedStepInfo) reasons.push(failedStepInfo);
  if (callsSucceeded && !isWatertight) reasons.push('mesh not watertight');
  if (callsSucceeded && !genusMatched) {
    reasons.push(`genus ${actualGenus} != expected ${task.expected_genus}`);
  }
  if (callsSucceeded && !dimsWithin1Pct) {
    reasons.push(`dim error ${maxDimErrPct.toFixed(2)}% > 1%`);
  }
  if (callsSucceeded && !domainMatched) {
    reasons.push(`domain mismatch (${factVsRef})`);
  }

  return {
    id: task.id,
    category: task.category,
    passed,
    calls_succeeded: callsSucceeded,
    is_watertight: isWatertight,
    actual_genus: actualGenus,
    expected_genus: task.expected_genus,
    actual_dims_m: actualDims,
    expected_dims_m: task.expected_dimensions_m,
    max_dimension_error_pct: Number(maxDimErrPct.toFixed(4)),
    fact_vs_reference: factVsRef,
    failure_reason: passed ? 'OK' : reasons.join('; '),
    duration_ms: Number((performance.now() - t0).toFixed(2)),
  };
}

function printTable(title: string, results: RustBenchmarkTaskResult[]): void {
  const passedCount = results.filter((r) => r.passed).length;
  console.log('\n' + '='.repeat(120));
  console.log(title);
  console.log(
    `Summary: ${passedCount}/${results.length} PASSED (${((passedCount / results.length) * 100).toFixed(1)}%)`
  );
  console.log('='.repeat(120));
  console.log(
    `${'Task ID'.padEnd(32)} | ${'Status'.padEnd(6)} | ${'Calls'.padEnd(5)} | ${'Watertight'.padEnd(10)} | ${'MaxDimErr(%)'.padEnd(12)} | Fact vs Reference / Failure Reason`
  );
  console.log('-'.repeat(120));
  for (const r of results) {
    console.log(
      `${r.id.padEnd(32)} | ${(r.passed ? 'PASS' : 'FAIL').padEnd(6)} | ${(r.calls_succeeded ? 'OK' : 'ERR').padEnd(5)} | ${String(r.is_watertight).padEnd(10)} | ${r.max_dimension_error_pct.toFixed(4).padStart(11)}% | ${r.fact_vs_reference} [${r.failure_reason}]`
    );
  }
  console.log('='.repeat(120) + '\n');
}

export async function runRustBenchmarkSuite(): Promise<{
  strict: RustBenchmarkTaskResult[];
  precreate: RustBenchmarkTaskResult[];
  bridged: RustBenchmarkTaskResult[];
}> {
  const binPath = resolveServerBinary();
  const tasksPath = path.resolve(process.cwd(), 'bench/tasks.json');
  const spec = JSON.parse(fs.readFileSync(tasksPath, 'utf-8')) as {
    tasks: BenchmarkTask[];
  };

  const strict: RustBenchmarkTaskResult[] = [];
  const precreate: RustBenchmarkTaskResult[] = [];
  const bridged: RustBenchmarkTaskResult[] = [];

  const clientStrict = new StdioMcpClient(binPath);
  try {
    for (const task of spec.tasks) {
      strict.push(await runSingleTaskOnRust(clientStrict, task, 'strict_as_is'));
    }
  } finally {
    clientStrict.close();
  }

  const clientPrecreate = new StdioMcpClient(binPath);
  try {
    for (const task of spec.tasks) {
      precreate.push(
        await runSingleTaskOnRust(
          clientPrecreate,
          task,
          'precreate_parts_verbatim_steps'
        )
      );
    }
  } finally {
    clientPrecreate.close();
  }

  const clientBridged = new StdioMcpClient(binPath);
  try {
    for (const task of spec.tasks) {
      bridged.push(
        await runSingleTaskOnRust(
          clientBridged,
          task,
          'schema_bridged_diagnostic'
        )
      );
    }
  } finally {
    clientBridged.close();
  }

  return { strict, precreate, bridged };
}

if (
  typeof process !== 'undefined' &&
  process.argv[1] &&
  process.argv[1].endsWith('run_benchmark_rust.ts')
) {
  runRustBenchmarkSuite()
    .then(({ strict, precreate, bridged }) => {
      printTable(
        'MODE 1: STRICT AS-IS (`bench/tasks.json` steps sent verbatim to Rust `mcp-cad-server` via stdio JSON-RPC)',
        strict
      );
      printTable(
        'MODE 2: WITH `create_part` PRE-INITIALIZATION (`task.steps` and parameters verbatim from `bench/tasks.json`)',
        precreate
      );
      printTable(
        'MODE 3: SCHEMA-BRIDGED DIAGNOSTIC PROBE (schema differences adapted, all numbers/units from `bench/tasks.json` kept as-is)',
        bridged
      );
    })
    .catch((err) => {
      console.error('Rust benchmark execution failed:', err);
      process.exit(1);
    });
}
