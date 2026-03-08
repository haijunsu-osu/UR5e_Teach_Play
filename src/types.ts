export type Vec3 = [number, number, number];

export type JointVector = [
  number,
  number,
  number,
  number,
  number,
  number,
];

export type JointIndex = 0 | 1 | 2 | 3 | 4 | 5;

export type Matrix4RowMajor = [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
];

export interface EulerPose {
  position: Vec3;
  rpy: Vec3;
}

export interface IKSolution {
  id: string;
  label: string;
  joints: JointVector;
  valid: boolean;
  positionErrorMm: number;
  orientationErrorDeg: number;
  branch: {
    shoulder: number;
    wrist: number;
    elbow: number;
  };
}

export interface Waypoint {
  id: string;
  name: string;
  joints: JointVector;
  pose: EulerPose;
  source: "current" | "ik";
}

export interface MoveJSample {
  time: number;
  joints: JointVector;
  tcpPose: EulerPose;
}
