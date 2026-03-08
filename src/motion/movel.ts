import { Euler, Quaternion } from "three";
import { forwardKinematics, inverseKinematics } from "../kinematics/ur5eKinematics";
import {
  clamp,
  distanceJointVectors,
  distance3,
  poseToRowMajor,
  radToDeg,
  rotationDistanceRadians,
  rowMajorToPose,
  smoothstep01,
} from "../lib/math";
import type {
  EulerPose,
  IKSolution,
  JointVector,
  MotionTrajectory,
  TrajectorySample,
} from "../types";

const TCP_LINEAR_SPEED = 0.25;
const TCP_ANGULAR_SPEED = Math.PI;

function poseQuaternion(pose: EulerPose): Quaternion {
  return new Quaternion().setFromEuler(
    new Euler(pose.rpy[0], pose.rpy[1], pose.rpy[2], "XYZ"),
  );
}

function quaternionToRpy(quaternion: Quaternion): EulerPose["rpy"] {
  const euler = new Euler().setFromQuaternion(quaternion, "XYZ");
  return [euler.x, euler.y, euler.z];
}

function interpolatePoseLinear(
  startPose: EulerPose,
  endPose: EulerPose,
  startQuaternion: Quaternion,
  endQuaternion: Quaternion,
  t: number,
): EulerPose {
  const position: EulerPose["position"] = [
    startPose.position[0] + (endPose.position[0] - startPose.position[0]) * t,
    startPose.position[1] + (endPose.position[1] - startPose.position[1]) * t,
    startPose.position[2] + (endPose.position[2] - startPose.position[2]) * t,
  ];
  const rotation = startQuaternion.clone().slerp(endQuaternion, t);
  return {
    position,
    rpy: quaternionToRpy(rotation),
  };
}

function branchSignature(solution: IKSolution): string {
  return `${solution.branch.shoulder}-${solution.branch.wrist}-${solution.branch.elbow}`;
}

function pickLockedBranchSolution(
  pose: EulerPose,
  referenceJoints: JointVector,
  lockedBranch: string | null,
): IKSolution | null {
  const solutions = inverseKinematics(pose, referenceJoints).filter(
    (candidate) => candidate.valid,
  );

  if (!solutions.length) {
    return null;
  }

  if (!lockedBranch) {
    return (
      solutions.reduce<IKSolution | null>((best, candidate) => {
        if (!best) {
          return candidate;
        }
        return distanceJointVectors(candidate.joints, referenceJoints) <
          distanceJointVectors(best.joints, referenceJoints)
          ? candidate
          : best;
      }, null) ?? null
    );
  }

  return (
    solutions.find((candidate) => branchSignature(candidate) === lockedBranch) ?? null
  );
}

export function buildMoveLTrajectory(
  startJoints: JointVector,
  targets: EulerPose[],
  speedScale: number,
  fps = 60,
): MotionTrajectory {
  if (!targets.length) {
    return { motionType: "MoveL", duration: 0, samples: [] };
  }

  const safeSpeedScale = clamp(speedScale, 0.05, 1.5);
  const samples: TrajectorySample[] = [];
  let currentTime = 0;
  let previousJoints = [...startJoints] as JointVector;
  let segmentStartPose = rowMajorToPose(forwardKinematics(startJoints));
  const initialSolution = pickLockedBranchSolution(segmentStartPose, startJoints, null);

  if (!initialSolution) {
    return {
      motionType: "MoveL",
      duration: 0,
      samples: [],
      error: "MoveL failed to identify a valid analytical IK branch for the current start pose.",
    };
  }

  const lockedBranch = branchSignature(initialSolution);
  previousJoints = initialSolution.joints;

  for (let segmentIndex = 0; segmentIndex < targets.length; segmentIndex += 1) {
    const segmentEndPose = targets[segmentIndex]!;
    const translationDistance = distance3(
      segmentStartPose.position,
      segmentEndPose.position,
    );
    const rotationDistance = rotationDistanceRadians(
      poseToRowMajor(segmentStartPose),
      poseToRowMajor(segmentEndPose),
    );
    const duration = Math.max(
      translationDistance / (TCP_LINEAR_SPEED * safeSpeedScale),
      rotationDistance / (TCP_ANGULAR_SPEED * safeSpeedScale),
      0.25,
    );
    const frameCount = Math.max(2, Math.ceil(duration * fps));
    const startQuaternion = poseQuaternion(segmentStartPose);
    const endQuaternion = poseQuaternion(segmentEndPose);

    for (let frame = 0; frame <= frameCount; frame += 1) {
      if (segmentIndex > 0 && frame === 0) {
        continue;
      }

      const u = frame / frameCount;
      const blend = smoothstep01(u);
      const pose = interpolatePoseLinear(
        segmentStartPose,
        segmentEndPose,
        startQuaternion,
        endQuaternion,
        blend,
      );
      const solution = pickLockedBranchSolution(pose, previousJoints, lockedBranch);

      if (!solution) {
        return {
          motionType: "MoveL",
          duration: 0,
          samples: [],
          error: `MoveL failed on segment ${segmentIndex + 1}: analytical branch ${lockedBranch} is not available along the straight-line path.`,
        };
      }

      previousJoints = solution.joints;
      samples.push({
        time: currentTime + u * duration,
        joints: solution.joints,
        tcpPose: pose,
      });
    }

    currentTime += duration;
    segmentStartPose = segmentEndPose;
  }

  if (!samples.length) {
    return {
      motionType: "MoveL",
      duration: 0,
      samples: [],
      error: "MoveL did not generate any trajectory samples.",
    };
  }

  const lastSample = samples.at(-1);
  if (!lastSample) {
    return {
      motionType: "MoveL",
      duration: 0,
      samples: [],
      error: "MoveL did not generate a valid end sample.",
    };
  }

  const endPose = targets.at(-1);
  if (endPose) {
    const positionErrorMm =
      distance3(lastSample.tcpPose.position, endPose.position) * 1000;
    const orientationErrorDeg = radToDeg(
      rotationDistanceRadians(
        poseToRowMajor(lastSample.tcpPose),
        poseToRowMajor(endPose),
      ),
    );
    if (positionErrorMm > 0.1 || orientationErrorDeg > 0.1) {
      return {
        motionType: "MoveL",
        duration: 0,
        samples: [],
        error: `MoveL endpoint drift exceeded tolerance (${positionErrorMm.toFixed(3)} mm / ${orientationErrorDeg.toFixed(3)} deg).`,
      };
    }
  }

  return {
    motionType: "MoveL",
    duration: currentTime,
    samples,
  };
}
