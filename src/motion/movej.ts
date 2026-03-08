import { JOINT_VELOCITY_LIMITS } from "../data/ur5e";
import { forwardKinematics } from "../kinematics/ur5eKinematics";
import {
  clamp,
  interpolateJointVectors,
  normalizeAngle,
  rowMajorToPose,
  smoothstep01,
} from "../lib/math";
import type { JointVector, MotionTrajectory, TrajectorySample } from "../types";

export function buildMoveJTrajectory(
  points: JointVector[],
  speedScale: number,
  fps = 60,
): MotionTrajectory {
  if (points.length < 2) {
    return { motionType: "MoveJ", duration: 0, samples: [] };
  }

  const safeSpeedScale = clamp(speedScale, 0.05, 1.5);
  const samples: TrajectorySample[] = [];
  let currentTime = 0;

  for (let segmentIndex = 0; segmentIndex < points.length - 1; segmentIndex += 1) {
    const start = points[segmentIndex]!;
    const end = points[segmentIndex + 1]!;
    const duration = start.reduce((longest, value, jointIndex) => {
      const index = jointIndex as 0 | 1 | 2 | 3 | 4 | 5;
      const delta = Math.abs(normalizeAngle(end[index] - value));
      const jointDuration = delta / (JOINT_VELOCITY_LIMITS[index] * safeSpeedScale);
      return Math.max(longest, jointDuration);
    }, 0.25);

    const frameCount = Math.max(2, Math.ceil(duration * fps));
    for (let frame = 0; frame <= frameCount; frame += 1) {
      if (segmentIndex > 0 && frame === 0) {
        continue;
      }
      const u = frame / frameCount;
      const joints = interpolateJointVectors(start, end, smoothstep01(u));
      samples.push({
        time: currentTime + u * duration,
        joints,
        tcpPose: rowMajorToPose(forwardKinematics(joints)),
      });
    }
    currentTime += duration;
  }

  return {
    motionType: "MoveJ",
    duration: currentTime,
    samples,
  };
}
