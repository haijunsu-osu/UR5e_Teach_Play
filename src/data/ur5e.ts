import type { JointVector, Vec3 } from "../types";

export const JOINT_NAMES = [
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
] as const;

export const JOINT_LABELS = [
  "Base",
  "Shoulder",
  "Elbow",
  "Wrist 1",
  "Wrist 2",
  "Wrist 3",
] as const;

export const JOINT_INDICES = [0, 1, 2, 3, 4, 5] as const;

export const JOINT_DISPLAY_OFFSETS: JointVector = [
  0,
  -Math.PI / 2,
  0,
  -Math.PI / 2,
  0,
  0,
];

export const JOINT_LIMITS: ReadonlyArray<readonly [number, number]> = [
  [-2 * Math.PI, 2 * Math.PI],
  [-2 * Math.PI, 2 * Math.PI],
  [-Math.PI, Math.PI],
  [-2 * Math.PI, 2 * Math.PI],
  [-2 * Math.PI, 2 * Math.PI],
  [-2 * Math.PI, 2 * Math.PI],
];

export const JOINT_VELOCITY_LIMITS = [
  Math.PI,
  Math.PI,
  Math.PI,
  Math.PI,
  Math.PI,
  Math.PI,
] as const;

export const DEFAULT_JOINTS: JointVector = [0, -1.57, 0, -1.57, 0, 0];

export const DEFAULT_TARGET_POSE = {
  position: [-0.55, -0.15, 0.35] as Vec3,
  rpy: [Math.PI, 0, -Math.PI / 2] as Vec3,
};

export const UR5E_PARAMS = {
  d1: 0.1625,
  a2: -0.425,
  a3: -0.3922,
  d4: 0.1333,
  d5: 0.0997,
  d6: 0.0996,
} as const;

export const BASE_INERTIA_RPY: Vec3 = [0, 0, Math.PI];

export const JOINT_ORIGINS = [
  { position: [0, 0, 0.1625] as Vec3, rpy: [0, 0, 0] as Vec3 },
  { position: [0, 0, 0] as Vec3, rpy: [Math.PI / 2, 0, 0] as Vec3 },
  { position: [-0.425, 0, 0] as Vec3, rpy: [0, 0, 0] as Vec3 },
  { position: [-0.3922, 0, 0.1333] as Vec3, rpy: [0, 0, 0] as Vec3 },
  { position: [0, -0.0997, 0] as Vec3, rpy: [Math.PI / 2, 0, 0] as Vec3 },
  { position: [0, 0.0996, 0] as Vec3, rpy: [Math.PI / 2, Math.PI, Math.PI] as Vec3 },
] as const;

export const WRIST3_TO_FLANGE = {
  position: [0, 0, 0] as Vec3,
  rpy: [0, -Math.PI / 2, -Math.PI / 2] as Vec3,
};

export const FLANGE_TO_TOOL0 = {
  position: [0, 0, 0] as Vec3,
  rpy: [Math.PI / 2, 0, Math.PI / 2] as Vec3,
};

export const VISUAL_LINKS = [
  {
    name: "base",
    path: "/assets/ur5e/visual/base.dae",
    offset: { position: [0, 0, 0] as Vec3, rpy: [0, 0, Math.PI] as Vec3 },
  },
  {
    name: "shoulder",
    path: "/assets/ur5e/visual/shoulder.dae",
    offset: { position: [0, 0, 0] as Vec3, rpy: [0, 0, Math.PI] as Vec3 },
  },
  {
    name: "upper_arm",
    path: "/assets/ur5e/visual/upperarm.dae",
    offset: {
      position: [0, 0, 0.138] as Vec3,
      rpy: [Math.PI / 2, 0, -Math.PI / 2] as Vec3,
    },
  },
  {
    name: "forearm",
    path: "/assets/ur5e/visual/forearm.dae",
    offset: {
      position: [0, 0, 0.007] as Vec3,
      rpy: [Math.PI / 2, 0, -Math.PI / 2] as Vec3,
    },
  },
  {
    name: "wrist_1",
    path: "/assets/ur5e/visual/wrist1.dae",
    offset: {
      position: [0, 0, -0.127] as Vec3,
      rpy: [Math.PI / 2, 0, 0] as Vec3,
    },
  },
  {
    name: "wrist_2",
    path: "/assets/ur5e/visual/wrist2.dae",
    offset: {
      position: [0, 0, -0.0997] as Vec3,
      rpy: [0, 0, 0] as Vec3,
    },
  },
  {
    name: "wrist_3",
    path: "/assets/ur5e/visual/wrist3.dae",
    offset: {
      position: [0, 0, -0.0989] as Vec3,
      rpy: [Math.PI / 2, 0, 0] as Vec3,
    },
  },
] as const;
