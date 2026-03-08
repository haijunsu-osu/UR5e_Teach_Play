import { DEFAULT_JOINTS, UR5E_PARAMS } from "../data/ur5e";
import {
  clamp,
  distance3,
  distanceJointVectors,
  isWithinJointLimits,
  normalizeAngle,
  poseToRowMajor,
  radToDeg,
  rotationDistanceRadians,
  translationFromRowMajor,
  wrapJointVectorNearReference,
} from "../lib/math";
import type { EulerPose, IKSolution, JointVector, Matrix4RowMajor } from "../types";

const { a2, a3, d1, d4, d5, d6 } = UR5E_PARAMS;
const PI = Math.PI;
const ZERO_THRESH = 1e-8;

function sign(value: number): number {
  return (value > 0 ? 1 : 0) - (value < 0 ? 1 : 0);
}

function cleanAngle(value: number): number {
  if (Math.abs(value) < ZERO_THRESH) {
    return 0;
  }
  return value;
}

function normalizePositive(value: number): number {
  let result = cleanAngle(value);
  if (result < 0) {
    result += Math.PI * 2;
  }
  return result;
}

export function forwardKinematics(joints: JointVector): Matrix4RowMajor {
  const [q1, q2, q3, q4, q5, q6] = joints;
  const s1 = Math.sin(q1);
  const c1 = Math.cos(q1);
  const s2 = Math.sin(q2);
  const c2 = Math.cos(q2);
  const q23 = q2 + q3;
  const q234 = q2 + q3 + q4;
  const s3 = Math.sin(q3);
  const c3 = Math.cos(q3);
  const s5 = Math.sin(q5);
  const c5 = Math.cos(q5);
  const s6 = Math.sin(q6);
  const c6 = Math.cos(q6);
  const s23 = Math.sin(q23);
  const c23 = Math.cos(q23);
  const s234 = Math.sin(q234);
  const c234 = Math.cos(q234);

  return [
    c234 * c1 * s5 - c5 * s1,
    c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6,
    -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6,
    d6 * c234 * c1 * s5 -
      a3 * c23 * c1 -
      a2 * c1 * c2 -
      d6 * c5 * s1 -
      d5 * s234 * c1 -
      d4 * s1,
    c1 * c5 + c234 * s1 * s5,
    -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6,
    s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1,
    d6 * (c1 * c5 + c234 * s1 * s5) +
      d4 * c1 -
      a3 * c23 * s1 -
      a2 * c2 * s1 -
      d5 * s234 * s1,
    -s234 * s5,
    -c234 * s6 - s234 * c5 * c6,
    s234 * c5 * s6 - c234 * c6,
    d1 + a3 * s23 + a2 * s2 - d5 * c234 - d6 * s234 * s5,
    0,
    0,
    0,
    1,
  ];
}

function uniqueSolutions(
  candidates: IKSolution[],
  reference: JointVector,
): IKSolution[] {
  const unique: IKSolution[] = [];
  for (const solution of candidates) {
    const exists = unique.some(
      (existing) => distanceJointVectors(existing.joints, solution.joints) < 1e-4,
    );
    if (!exists) {
      unique.push(solution);
    }
  }
  unique.sort((a, b) => {
    if (a.valid !== b.valid) {
      return a.valid ? -1 : 1;
    }
    return (
      distanceJointVectors(a.joints, reference) -
      distanceJointVectors(b.joints, reference)
    );
  });
  return unique;
}

export function inverseKinematics(
  pose: EulerPose,
  reference: JointVector = DEFAULT_JOINTS,
): IKSolution[] {
  const matrix = poseToRowMajor(pose);

  const T02 = -matrix[0];
  const T00 = matrix[1];
  const T01 = matrix[2];
  const T03 = -matrix[3];
  const T12 = -matrix[4];
  const T10 = matrix[5];
  const T11 = matrix[6];
  const T13 = -matrix[7];
  const T22 = matrix[8];
  const T20 = -matrix[9];
  const T21 = -matrix[10];
  const T23 = matrix[11];

  const q1: [number, number] = [0, 0];
  {
    const A = d6 * T12 - T13;
    const B = d6 * T02 - T03;
    const R = A * A + B * B;

    if (Math.abs(A) < ZERO_THRESH) {
      const div =
        Math.abs(Math.abs(d4) - Math.abs(B)) < ZERO_THRESH
          ? -sign(d4) * sign(B)
          : clamp(-d4 / B, -1, 1);
      const arcsin = Math.asin(div);
      q1[0] = normalizePositive(arcsin);
      q1[1] = normalizePositive(PI - arcsin);
    } else if (Math.abs(B) < ZERO_THRESH) {
      const div =
        Math.abs(Math.abs(d4) - Math.abs(A)) < ZERO_THRESH
          ? sign(d4) * sign(A)
          : clamp(d4 / A, -1, 1);
      const arccos = Math.acos(div);
      q1[0] = arccos;
      q1[1] = Math.PI * 2 - arccos;
    } else if (d4 * d4 > R) {
      return [];
    } else {
      const arccos = Math.acos(clamp(d4 / Math.sqrt(R), -1, 1));
      const arctan = Math.atan2(-B, A);
      q1[0] = normalizePositive(arccos + arctan);
      q1[1] = normalizePositive(-arccos + arctan);
    }
  }

  const solutions: IKSolution[] = [];

  for (const shoulder of [0, 1] as const) {
    const q1Value = q1[shoulder];
    const q5Values: [number, number] = [0, 0];
    const numer = T03 * Math.sin(q1Value) - T13 * Math.cos(q1Value) - d4;
    const div =
      Math.abs(Math.abs(numer) - Math.abs(d6)) < ZERO_THRESH
        ? sign(numer) * sign(d6)
        : clamp(numer / d6, -1, 1);
    const arccosQ5 = Math.acos(div);
    q5Values[0] = arccosQ5;
    q5Values[1] = Math.PI * 2 - arccosQ5;

    for (const wrist of [0, 1] as const) {
      const q5Value = q5Values[wrist];
      const c1 = Math.cos(q1Value);
      const s1 = Math.sin(q1Value);
      const c5 = Math.cos(q5Value);
      const s5 = Math.sin(q5Value);

      let q6Value = reference[5];
      if (Math.abs(s5) >= ZERO_THRESH) {
        q6Value = Math.atan2(
          sign(s5) * -(T01 * s1 - T11 * c1),
          sign(s5) * (T00 * s1 - T10 * c1),
        );
        q6Value = normalizePositive(q6Value);
      }

      const c6 = Math.cos(q6Value);
      const s6 = Math.sin(q6Value);
      const x04x =
        -s5 * (T02 * c1 + T12 * s1) -
        c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
      const x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
      const p13x =
        d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) -
        d6 * (T02 * c1 + T12 * s1) +
        T03 * c1 +
        T13 * s1;
      const p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

      let c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2 * a2 * a3);
      if (Math.abs(Math.abs(c3) - 1) < ZERO_THRESH) {
        c3 = sign(c3);
      } else if (Math.abs(c3) > 1) {
        continue;
      }

      const arccosQ3 = Math.acos(c3);
      const q3Values: [number, number] = [arccosQ3, Math.PI * 2 - arccosQ3];
      const denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
      const s3 = Math.sin(arccosQ3);
      const A = a2 + a3 * c3;
      const B = a3 * s3;
      const q2Values: [number, number] = [
        Math.atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom),
        Math.atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom),
      ];

      for (const elbow of [0, 1] as const) {
        const q2Value = q2Values[elbow];
        const q3Value = q3Values[elbow];
        const c23 = Math.cos(q2Value + q3Value);
        const s23 = Math.sin(q2Value + q3Value);
        const q4Value = Math.atan2(
          c23 * x04y - s23 * x04x,
          x04x * c23 + x04y * s23,
        );

        const positiveCandidate: JointVector = [
          q1Value,
          normalizePositive(q2Value),
          normalizePositive(q3Value),
          normalizePositive(q4Value),
          q5Value,
          q6Value,
        ];

        const normalizedCandidate = positiveCandidate.map((value) =>
          normalizeAngle(value),
        ) as JointVector;
        const wrappedCandidate = wrapJointVectorNearReference(
          reference,
          normalizedCandidate,
        );

        if (!wrappedCandidate) {
          continue;
        }

        const candidateMatrix = forwardKinematics(wrappedCandidate);
        const targetPosition = translationFromRowMajor(matrix);
        const candidatePosition = translationFromRowMajor(candidateMatrix);

        solutions.push({
          id: `s${shoulder}-w${wrist}-e${elbow}`,
          label: `Branch ${solutions.length + 1} · S${shoulder + 1} W${wrist + 1} E${elbow + 1}`,
          joints: wrappedCandidate,
          valid: isWithinJointLimits(wrappedCandidate),
          positionErrorMm: distance3(candidatePosition, targetPosition) * 1000,
          orientationErrorDeg: radToDeg(
            rotationDistanceRadians(candidateMatrix, matrix),
          ),
          branch: {
            shoulder,
            wrist,
            elbow,
          },
        });
      }
    }
  }

  return uniqueSolutions(solutions, reference);
}
