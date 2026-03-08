import { AxesHelper, Group, LoadingManager, Mesh, Object3D } from "three";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import {
  BASE_INERTIA_RPY,
  FLANGE_TO_TOOL0,
  JOINT_LIMITS,
  JOINT_NAMES,
  JOINT_ORIGINS,
  VISUAL_LINKS,
  WRIST3_TO_FLANGE,
} from "../data/ur5e";
import type { JointVector, Vec3 } from "../types";

const LINK_NAMES = [
  "shoulder_link",
  "upper_arm_link",
  "forearm_link",
  "wrist_1_link",
  "wrist_2_link",
  "wrist_3_link",
] as const;

const VISUAL_LINK_NAME_MAP = {
  base: "base_link_inertia",
  shoulder: "shoulder_link",
  upper_arm: "upper_arm_link",
  forearm: "forearm_link",
  wrist_1: "wrist_1_link",
  wrist_2: "wrist_2_link",
  wrist_3: "wrist_3_link",
} as const;

function formatTuple(values: readonly number[]): string {
  return values.map((value) => `${value}`).join(" ");
}

function buildVisualXml(
  linkName: string,
  visualName: keyof typeof VISUAL_LINK_NAME_MAP,
): string {
  const visual = VISUAL_LINKS.find((entry) => entry.name === visualName);
  if (!visual) {
    throw new Error(`Missing visual definition for ${visualName}.`);
  }

  return `
    <link name="${linkName}">
      <visual>
        <origin
          xyz="${formatTuple(visual.offset.position)}"
          rpy="${formatTuple(visual.offset.rpy)}" />
        <geometry>
          <mesh filename="${visual.path}" />
        </geometry>
      </visual>
    </link>
  `;
}

function buildRevoluteJointXml(
  jointName: string,
  parent: string,
  child: string,
  origin: { position: Vec3; rpy: Vec3 },
  limits: readonly [number, number],
): string {
  return `
    <joint name="${jointName}" type="revolute">
      <parent link="${parent}" />
      <child link="${child}" />
      <origin
        xyz="${formatTuple(origin.position)}"
        rpy="${formatTuple(origin.rpy)}" />
      <axis xyz="0 0 1" />
      <limit
        lower="${limits[0]}"
        upper="${limits[1]}"
        effort="150"
        velocity="3.141592653589793" />
    </joint>
  `;
}

function buildFixedJointXml(
  jointName: string,
  parent: string,
  child: string,
  origin: { position: Vec3; rpy: Vec3 },
): string {
  return `
    <joint name="${jointName}" type="fixed">
      <parent link="${parent}" />
      <child link="${child}" />
      <origin
        xyz="${formatTuple(origin.position)}"
        rpy="${formatTuple(origin.rpy)}" />
    </joint>
  `;
}

function buildUr5eUrdf(): string {
  const visualSections = [
    buildVisualXml("base_link_inertia", "base"),
    buildVisualXml("shoulder_link", "shoulder"),
    buildVisualXml("upper_arm_link", "upper_arm"),
    buildVisualXml("forearm_link", "forearm"),
    buildVisualXml("wrist_1_link", "wrist_1"),
    buildVisualXml("wrist_2_link", "wrist_2"),
    buildVisualXml("wrist_3_link", "wrist_3"),
  ].join("\n");

  const jointSections = JOINT_NAMES.map((jointName, index) => {
    const parent =
      index === 0 ? "base_link_inertia" : LINK_NAMES[index - 1] ?? "base_link_inertia";
    const child = LINK_NAMES[index]!;
    return buildRevoluteJointXml(
      jointName,
      parent,
      child,
      JOINT_ORIGINS[index]!,
      JOINT_LIMITS[index]!,
    );
  }).join("\n");

  return `
    <robot name="ur5e">
      <link name="base_link" />
      ${visualSections}
      <link name="flange" />
      <link name="tool0" />

      ${buildFixedJointXml("base_link-base_link_inertia", "base_link", "base_link_inertia", {
        position: [0, 0, 0],
        rpy: BASE_INERTIA_RPY,
      })}
      ${jointSections}
      ${buildFixedJointXml("wrist_3-flange", "wrist_3_link", "flange", WRIST3_TO_FLANGE)}
      ${buildFixedJointXml("flange-tool0", "flange", "tool0", FLANGE_TO_TOOL0)}
    </robot>
  `;
}

export class Ur5eRobotRig {
  readonly root = new Group();
  private robot: URDFRobot | null = null;

  async loadMeshes(): Promise<void> {
    const manager = new LoadingManager();
    const ready = new Promise<void>((resolve, reject) => {
      manager.onLoad = () => resolve();
      manager.onError = (url) => reject(new Error(`Failed to load UR5e asset: ${url}`));
    });

    const loader = new URDFLoader(manager);
    const robot = loader.parse(buildUr5eUrdf());
    robot.traverse((node: Object3D) => {
      if (!(node instanceof Mesh)) {
        return;
      }
      node.castShadow = true;
      node.receiveShadow = true;
    });

    const tool0 = robot.links.tool0;
    if (tool0) {
      const tcpAxes = new AxesHelper(0.12);
      tcpAxes.renderOrder = 10;
      tool0.add(tcpAxes);
    }

    this.robot = robot;
    this.root.add(robot);
    await ready;
  }

  setJoints(joints: JointVector): void {
    if (!this.robot) {
      return;
    }

    this.robot.setJointValues(
      Object.fromEntries(JOINT_NAMES.map((name, index) => [name, joints[index]!])) as Record<
        (typeof JOINT_NAMES)[number],
        number
      >,
    );
  }
}
