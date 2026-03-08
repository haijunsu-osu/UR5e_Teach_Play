import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import { JOINT_INDICES, JOINT_LIMITS } from "../data/ur5e";
import type {
  EulerPose,
  JointIndex,
  JointVector,
  Matrix4RowMajor,
  Vec3,
} from "../types";

export const TAU = Math.PI * 2;

export function degToRad(value: number): number {
  return (value * Math.PI) / 180;
}

export function radToDeg(value: number): number {
  return (value * 180) / Math.PI;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function normalizeAngle(value: number): number {
  let wrapped = (value + Math.PI) % TAU;
  if (wrapped < 0) {
    wrapped += TAU;
  }
  return wrapped - Math.PI;
}

export function formatRadians(value: number): string {
  return `${radToDeg(value).toFixed(1)}°`;
}

export function rowMajorFromMatrix4(matrix: Matrix4): Matrix4RowMajor {
  const e = matrix.elements;
  return [
    e[0],
    e[4],
    e[8],
    e[12],
    e[1],
    e[5],
    e[9],
    e[13],
    e[2],
    e[6],
    e[10],
    e[14],
    e[3],
    e[7],
    e[11],
    e[15],
  ];
}

export function matrix4FromRowMajor(elements: Matrix4RowMajor): Matrix4 {
  return new Matrix4().set(
    elements[0],
    elements[1],
    elements[2],
    elements[3],
    elements[4],
    elements[5],
    elements[6],
    elements[7],
    elements[8],
    elements[9],
    elements[10],
    elements[11],
    elements[12],
    elements[13],
    elements[14],
    elements[15],
  );
}

export function poseToRowMajor(pose: EulerPose): Matrix4RowMajor {
  const matrix = new Matrix4();
  const quaternion = new Quaternion().setFromEuler(
    new Euler(pose.rpy[0], pose.rpy[1], pose.rpy[2], "XYZ"),
  );
  matrix.compose(
    new Vector3(pose.position[0], pose.position[1], pose.position[2]),
    quaternion,
    new Vector3(1, 1, 1),
  );
  return rowMajorFromMatrix4(matrix);
}

export function rowMajorToPose(elements: Matrix4RowMajor): EulerPose {
  const matrix = matrix4FromRowMajor(elements);
  const position = new Vector3();
  const quaternion = new Quaternion();
  const scale = new Vector3();
  matrix.decompose(position, quaternion, scale);
  const euler = new Euler().setFromQuaternion(quaternion, "XYZ");
  return {
    position: [position.x, position.y, position.z],
    rpy: [euler.x, euler.y, euler.z],
  };
}

export function translationFromRowMajor(elements: Matrix4RowMajor): Vec3 {
  return [elements[3], elements[7], elements[11]];
}

export function rotationDistanceRadians(
  a: Matrix4RowMajor,
  b: Matrix4RowMajor,
): number {
  const qa = new Quaternion().setFromRotationMatrix(matrix4FromRowMajor(a));
  const qb = new Quaternion().setFromRotationMatrix(matrix4FromRowMajor(b));
  const dot = clamp(Math.abs(qa.dot(qb)), -1, 1);
  return 2 * Math.acos(dot);
}

export function distance3(a: Vec3, b: Vec3): number {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  const dz = a[2] - b[2];
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

export function distanceJointVectors(a: JointVector, b: JointVector): number {
  let sum = 0;
  for (const index of JOINT_INDICES) {
    const delta = a[index] - b[index];
    sum += delta * delta;
  }
  return Math.sqrt(sum);
}

export function nearestEquivalentAngle(
  reference: number,
  candidate: number,
  limits: readonly [number, number],
): number | null {
  let best: number | null = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (let turns = -2; turns <= 2; turns += 1) {
    const value = candidate + turns * TAU;
    if (value < limits[0] - 1e-8 || value > limits[1] + 1e-8) {
      continue;
    }
    const distance = Math.abs(value - reference);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = value;
    }
  }
  return best;
}

export function wrapJointVectorNearReference(
  reference: JointVector,
  candidate: JointVector,
): JointVector | null {
  const wrapped: number[] = [];
  for (const index of JOINT_INDICES) {
    const value = nearestEquivalentAngle(
      reference[index],
      candidate[index],
      JOINT_LIMITS[index]!,
    );
    if (value === null) {
      return null;
    }
    wrapped.push(value);
  }
  return wrapped as JointVector;
}

export function isWithinJointLimits(joints: JointVector): boolean {
  return JOINT_INDICES.every((index) => {
    const [lower, upper] = JOINT_LIMITS[index]!;
    const value = joints[index];
    return value >= lower - 1e-8 && value <= upper + 1e-8;
  });
}

export function interpolateJointVectors(
  start: JointVector,
  end: JointVector,
  t: number,
): JointVector {
  const result: number[] = [];
  for (const index of JOINT_INDICES) {
    const value = start[index];
    const delta = normalizeAngle(end[index] - value);
    result.push(value + delta * t);
  }
  return result as JointVector;
}

export function smoothstep01(t: number): number {
  return t * t * (3 - 2 * t);
}
