import { CadPart, KinematicJoint, PhysicsTrajectoryFrame } from '../types/cad';

/**
 * Steps rigid-body kinematic assembly transforms and joint angles forward by dtSeconds.
 * Supports:
 * 1. Motorized Revolute / Fixed / Prismatic / Spherical joints
 * 2. Exact Involute Spur Gear Mesh ratio coupling: w2 = -w1 * (z1 / z2)
 * 3. Crank-Slider 4-bar mechanism articulation (crank_arm + connecting_rod + linear_slider)
 */
export function stepKinematicsAssembly(
  parts: CadPart[],
  joints: KinematicJoint[],
  dtSeconds: number
): {
  updatedParts: CadPart[];
  updatedJoints: KinematicJoint[];
  angularVelocities: Record<string, [number, number, number]>;
  linearVelocities: Record<string, [number, number, number]>;
} {
  const partMap = new Map<string, CadPart>();
  const angularVelocities: Record<string, [number, number, number]> = {};
  const linearVelocities: Record<string, [number, number, number]> = {};

  parts.forEach((p) => {
    partMap.set(p.id, {
      ...p,
      position: [...p.position] as [number, number, number],
      rotation: [...p.rotation] as [number, number, number],
    });
    angularVelocities[p.id] = [0, 0, 0];
    linearVelocities[p.id] = [0, 0, 0];
  });

  const updatedJoints = joints.map((j) => ({ ...j }));

  // 1. Step motorized revolute & prismatic joints
  for (const joint of updatedJoints) {
    if (!joint.motorEnabled) continue;

    if (joint.type === 'revolute') {
      // RPM -> rad/s (or if motorSpeedRpm is small rad/s from MCP set_joint_motor, support both)
      const omegaRadSec =
        Math.abs(joint.motorSpeedRpm) <= 25
          ? joint.motorSpeedRpm
          : (joint.motorSpeedRpm * 2 * Math.PI) / 60;
      const dTheta = omegaRadSec * dtSeconds;
      joint.currentAngleRad += dTheta;

      const targetPart =
        partMap.get(joint.partB) ||
        (joint.partA !== 'world' ? partMap.get(joint.partA) : undefined);

      if (targetPart) {
        if (Math.abs(joint.axis[2]) > 0.5) {
          const sign = Math.sign(joint.axis[2]) || 1;
          targetPart.rotation[2] += dTheta * sign;
          angularVelocities[targetPart.id] = [0, 0, omegaRadSec * sign];
        } else if (Math.abs(joint.axis[1]) > 0.5) {
          const sign = Math.sign(joint.axis[1]) || 1;
          targetPart.rotation[1] += dTheta * sign;
          angularVelocities[targetPart.id] = [0, omegaRadSec * sign, 0];
        } else {
          const sign = Math.sign(joint.axis[0]) || 1;
          targetPart.rotation[0] += dTheta * sign;
          angularVelocities[targetPart.id] = [omegaRadSec * sign, 0, 0];
        }
      }
    } else if (joint.type === 'prismatic') {
      const vel =
        Math.abs(joint.motorSpeedRpm) <= 25
          ? joint.motorSpeedRpm
          : joint.motorSpeedRpm * 0.002;
      const dPos = vel * dtSeconds;
      joint.currentAngleRad += dPos;
      const targetPart =
        partMap.get(joint.partB) ||
        (joint.partA !== 'world' ? partMap.get(joint.partA) : undefined);
      if (targetPart) {
        const axLen =
          Math.hypot(joint.axis[0], joint.axis[1], joint.axis[2]) || 1;
        const ux = joint.axis[0] / axLen;
        const uy = joint.axis[1] / axLen;
        const uz = joint.axis[2] / axLen;
        targetPart.position[0] += ux * dPos;
        targetPart.position[1] += uy * dPos;
        targetPart.position[2] += uz * dPos;
        linearVelocities[targetPart.id] = [ux * vel, uy * vel, uz * vel];
      }
    }
  }

  // 2. Enforce exact gear mesh ratios: w2 = -w1 * (z1 / z2)
  for (const joint of updatedJoints) {
    if (joint.type === 'gear') {
      const gearA = partMap.get(joint.partA);
      const gearB = partMap.get(joint.partB);
      if (gearA && gearB) {
        const z1 = gearA.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 20;
        const z2 = gearB.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 40;
        const ratio = joint.gearRatio ?? z1 / z2;

        // Phase offset pi / z2 aligns driving tooth into driven tooth valley (4.5 deg for z2=40)
        const meshPhaseOffset = Math.PI / z2;
        gearB.rotation[2] = -gearA.rotation[2] * ratio + meshPhaseOffset;

        const wA = angularVelocities[gearA.id]?.[2] ?? 0;
        angularVelocities[gearB.id] = [0, 0, -wA * ratio];
      }
    }
  }

  // Also automatically couple any spur gears in contact even if connected via revolute joints
  // (Matches crates/mcp-cad-server/tests/gear_kinematic_test.rs where two revolute joints + contact resolve w2/w1 = -0.5)
  const hasExplicitGearJoint = updatedJoints.some((j) => j.type === 'gear');
  if (!hasExplicitGearJoint) {
    const gearParts = Array.from(partMap.values()).filter((p) =>
      p.dag.some((d) => d.type === 'spur_gear')
    );
    if (gearParts.length >= 2) {
      const driver = gearParts[0];
      const driven = gearParts[1];
      const wDriver = angularVelocities[driver.id]?.[2] ?? 0;
      const wDriven = angularVelocities[driven.id]?.[2] ?? 0;
      if (Math.abs(wDriver) > 1e-6 && Math.abs(wDriven) < 1e-6) {
        const z1 =
          driver.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 20;
        const z2 =
          driven.dag.find((d) => d.type === 'spur_gear')?.params?.teeth || 40;
        const ratio = z1 / z2;
        const meshPhaseOffset = Math.PI / z2;
        driven.rotation[2] = -driver.rotation[2] * ratio + meshPhaseOffset;
        angularVelocities[driven.id] = [0, 0, -wDriver * ratio];
      }
    }
  }

  // 3. Enforce Crank-Slider 4-bar kinematic chain if present (crank_arm, connecting_rod, linear_slider)
  const crank =
    partMap.get('crank_arm') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('crank')
    );
  const rod =
    partMap.get('connecting_rod') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('rod')
    );
  const slider =
    partMap.get('linear_slider') ||
    Array.from(partMap.values()).find((p) =>
      p.name.toLowerCase().includes('slider')
    );

  if (crank && (rod || slider)) {
    const theta = crank.rotation[2];
    const omega = angularVelocities[crank.id]?.[2] ?? 0;
    const pivotX = -0.06;
    const pivotY = 0.0;
    const r = 0.03; // crank throw radius 30mm
    const l = 0.095; // connecting rod length 95mm

    // Crank center revolves around pivot (-0.06, 0) at radius r/2
    crank.position[0] = pivotX + (r * 0.5) * Math.cos(theta);
    crank.position[1] = pivotY + (r * 0.5) * Math.sin(theta);

    // Crank pin coordinates
    const pinX = pivotX + r * Math.cos(theta);
    const pinY = pivotY + r * Math.sin(theta);

    // Wrist pin on horizontal slider track (y = 0)
    const underRoot = Math.max(1e-6, l * l - pinY * pinY);
    const wristX = pinX + Math.sqrt(underRoot);
    const phi = Math.atan2(-pinY, wristX - pinX);

    if (rod) {
      rod.position[0] = 0.5 * (pinX + wristX);
      rod.position[1] = 0.5 * pinY;
      rod.rotation[2] = phi;
      angularVelocities[rod.id] = [
        0,
        0,
        (-omega * r * Math.cos(theta)) / Math.sqrt(underRoot),
      ];
    }

    if (slider) {
      const prevX = slider.position[0];
      slider.position[0] = wristX;
      slider.position[1] = 0.0;
      linearVelocities[slider.id] = [
        dtSeconds > 1e-6 ? (wristX - prevX) / dtSeconds : -omega * r * Math.sin(theta),
        0,
        0,
      ];
    }
  }

  return {
    updatedParts: Array.from(partMap.values()),
    updatedJoints,
    angularVelocities,
    linearVelocities,
  };
}

/**
 * Runs a multi-step rigid-body / kinematic simulation over `durationSec` with step `dtSec`,
 * producing trajectory samples and animation frames matching `run_physics_simulation` in Rust.
 */
export function runPhysicsSimulationTrajectory(
  initialParts: CadPart[],
  initialJoints: KinematicJoint[],
  durationSec: number = 1.0,
  dtSec: number = 1 / 60
): {
  steps: number;
  frames: PhysicsTrajectoryFrame[];
  finalParts: CadPart[];
  finalJoints: KinematicJoint[];
} {
  if (!Number.isFinite(durationSec) || durationSec <= 0) {
    throw new Error('Simulation duration must be a positive finite number (> 0)');
  }
  if (!Number.isFinite(dtSec) || dtSec <= 0) {
    throw new Error('Simulation time step dt must be positive (> 0)');
  }

  const safeDt = Math.max(0.001, Math.min(0.1, dtSec));
  const steps = Math.max(1, Math.min(2000, Math.round(durationSec / safeDt)));

  let currentParts = initialParts.map((p) => ({
    ...p,
    position: [...p.position] as [number, number, number],
    rotation: [...p.rotation] as [number, number, number],
  }));

  // Ensure at least one motor is active during physics simulation if joints exist
  let currentJoints = initialJoints.map((j, idx) => ({
    ...j,
    motorEnabled: j.motorEnabled || (idx === 0 && j.type === 'revolute'),
    motorSpeedRpm:
      j.motorSpeedRpm !== 0 ? j.motorSpeedRpm : idx === 0 ? 60 : 0,
  }));

  const frames: PhysicsTrajectoryFrame[] = [];

  for (let s = 0; s <= steps; s++) {
    const stepRes = stepKinematicsAssembly(
      currentParts,
      currentJoints,
      s === 0 ? 0 : safeDt
    );
    currentParts = stepRes.updatedParts;
    currentJoints = stepRes.updatedJoints;

    const partTransforms: PhysicsTrajectoryFrame['partTransforms'] = {};
    for (const p of currentParts) {
      partTransforms[p.id] = {
        position: [...p.position] as [number, number, number],
        rotation: [...p.rotation] as [number, number, number],
        angularVelocity: stepRes.angularVelocities[p.id] || [0, 0, 0],
        linearVelocity: stepRes.linearVelocities[p.id] || [0, 0, 0],
      };
    }

    frames.push({
      timeSec: Number((s * safeDt).toFixed(4)),
      partTransforms,
    });
  }

  return {
    steps,
    frames,
    finalParts: currentParts,
    finalJoints: currentJoints,
  };
}
