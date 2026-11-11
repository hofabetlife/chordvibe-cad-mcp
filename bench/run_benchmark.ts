import fs from 'node:fs';
import path from 'node:path';
import { DEFAULT_MATERIALS } from '../src/cad/materials';
import {
  EngineStateSnapshot,
  executeMcpToolOnEngineState,
} from '../src/cad/mcp_dispatcher';

export interface BenchmarkTaskResult {
  id: string;
  category: string;
  prompt_ru: string;
  calls_succeeded: boolean;
  is_watertight: boolean;
  dimensions_within_1pct: boolean;
  max_dimension_error_pct: number;
  domain_verdict_matched: boolean;
  domain_summary: string;
  passed_first_attempt: boolean;
  duration_ms: number;
}

export function runAiNativeBenchmarkSuite(): {
  total: number;
  passed: number;
  pass_ratio: number;
  results: BenchmarkTaskResult[];
} {
  const tasksPath = path.resolve(process.cwd(), 'bench/tasks.json');
  const spec = JSON.parse(fs.readFileSync(tasksPath, 'utf-8'));
  const results: BenchmarkTaskResult[] = [];

  for (const task of spec.tasks) {
    const t0 = performance.now();
    let state: EngineStateSnapshot = {
      parts: [],
      selectedPartId: '',
      joints: [],
      materials: [...DEFAULT_MATERIALS],
      feaResult: null,
      physicsFrames: [],
      quality: 'standard',
      renderMode: 'shaded',
    };

    let callsSucceeded = true;
    let lastSimResponse: Record<string, any> | null = null;
    let lastFeaResponse: Record<string, any> | null = null;

    try {
      for (const step of task.steps) {
        const res = executeMcpToolOnEngineState(state, step.tool, step.params);
        state = res.nextState;
        if (step.tool === 'run_physics_simulation') {
          lastSimResponse = res.response;
        } else if (step.tool === 'run_fea_analysis') {
          lastFeaResponse = res.response;
        }
      }
    } catch (err) {
      callsSucceeded = false;
    }

    const targetPart = state.parts.find((p) => p.id === task.target_part_id);
    const isWatertight = Boolean(targetPart?.mesh?.isWatertight);

    let maxDimErrPct = 100;
    let dimsWithin1Pct = false;
    if (targetPart) {
      const bb = targetPart.mesh.boundingBox;
      const actualDims = [
        bb.max[0] - bb.min[0],
        bb.max[1] - bb.min[1],
        bb.max[2] - bb.min[2],
      ];
      const expectedDims: number[] = task.expected_dimensions_m;
      maxDimErrPct = Math.max(
        ...expectedDims.map(
          (exp, i) => (Math.abs(actualDims[i] - exp) / Math.max(1e-9, exp)) * 100
        )
      );
      dimsWithin1Pct = maxDimErrPct <= 1.0;
    }

    let domainVerdictMatched = true;
    let domainSummary = 'Geometry & topology verified';

    if (task.expected_fea) {
      if (!lastFeaResponse) {
        domainVerdictMatched = false;
        domainSummary = 'Missing FEA result';
      } else {
        const verdictOk =
          lastFeaResponse.verdict === task.expected_fea.verdict ||
          lastFeaResponse.safety_status === task.expected_fea.verdict;
        const actualStress = Number(lastFeaResponse.max_von_mises_stress_mpa);
        const expStress = Number(task.expected_fea.stress_mpa);
        const stressErrPct =
          (Math.abs(actualStress - expStress) / expStress) * 100;
        const tolPct = Number(task.expected_fea.tolerance_pct ?? 5.0);
        domainVerdictMatched = verdictOk && stressErrPct <= tolPct;
        domainSummary = `FEA ${lastFeaResponse.verdict}, σ=${actualStress.toFixed(2)} MPa (err=${stressErrPct.toFixed(2)}%)`;
      }
    } else if (task.expected_kinematics) {
      if (!lastSimResponse) {
        domainVerdictMatched = false;
        domainSummary = 'Missing physics simulation result';
      } else if (task.expected_kinematics.type === 'gear_pair') {
        const w1 =
          lastSimResponse.final_transforms[task.expected_kinematics.driver_id]
            ?.angularVelocity?.[2] ?? 0;
        const w2 =
          lastSimResponse.final_transforms[task.expected_kinematics.driven_id]
            ?.angularVelocity?.[2] ?? 0;
        const ratio = w1 !== 0 ? w2 / w1 : 0;
        const drift = Number(lastSimResponse.max_joint_drift_m ?? 0);
        domainVerdictMatched =
          Math.abs(ratio - task.expected_kinematics.expected_velocity_ratio) <
            1e-4 && drift < task.expected_kinematics.max_drift_m;
        domainSummary = `Gear ω2/ω1=${ratio.toFixed(3)}, drift=${drift.toExponential(2)} m`;
      } else if (task.expected_kinematics.type === 'slider_crank') {
        const frames = lastSimResponse.frames || [];
        const r = Number(task.expected_kinematics.r_m);
        const l = Number(task.expected_kinematics.l_m);
        const omega = Number(task.expected_kinematics.omega_rad_s);
        let maxErrPct = 0;
        for (const f of frames) {
          const theta = omega * f.timeSec;
          const expX =
            r * Math.cos(theta) +
            Math.sqrt(l * l - r * r * Math.pow(Math.sin(theta), 2));
          const actX =
            f.partTransforms[task.expected_kinematics.slider_id]?.position?.[0] ??
            0;
          const errPct = (Math.abs(actX - expX) / expX) * 100;
          if (errPct > maxErrPct) maxErrPct = errPct;
        }
        domainVerdictMatched =
          frames.length > 10 &&
          maxErrPct <= task.expected_kinematics.max_position_error_pct;
        domainSummary = `Slider-crank max x(θ) err=${maxErrPct.toFixed(4)}%`;
      }
    }

    const passedFirstAttempt =
      callsSucceeded &&
      isWatertight &&
      dimsWithin1Pct &&
      domainVerdictMatched;

    results.push({
      id: task.id,
      category: task.category,
      prompt_ru: task.prompt_ru,
      calls_succeeded: callsSucceeded,
      is_watertight: isWatertight,
      dimensions_within_1pct: dimsWithin1Pct,
      max_dimension_error_pct: Number(maxDimErrPct.toFixed(4)),
      domain_verdict_matched: domainVerdictMatched,
      domain_summary: domainSummary,
      passed_first_attempt: passedFirstAttempt,
      duration_ms: Number((performance.now() - t0).toFixed(2)),
    });
  }

  const passed = results.filter((r) => r.passed_first_attempt).length;
  return {
    total: results.length,
    passed,
    pass_ratio: Number((passed / results.length).toFixed(4)),
    results,
  };
}

if (
  typeof process !== 'undefined' &&
  process.argv[1] &&
  process.argv[1].endsWith('run_benchmark.ts')
) {
  const summary = runAiNativeBenchmarkSuite();
  console.log(
    `AI-Native Benchmark Suite: ${summary.passed}/${summary.total} passed on first attempt (ratio=${summary.pass_ratio})`
  );
  for (const r of summary.results) {
    console.log(
      `[${r.passed_first_attempt ? 'PASS' : 'FAIL'}] ${r.id} (${r.category}) | watertight=${r.is_watertight} | dim_err=${r.max_dimension_error_pct}% | ${r.domain_summary} | ${r.duration_ms}ms`
    );
  }
  if (summary.passed !== summary.total) {
    process.exit(1);
  }
}

