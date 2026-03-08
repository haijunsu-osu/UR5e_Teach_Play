import {
  AmbientLight,
  AxesHelper,
  BufferGeometry,
  CatmullRomCurve3,
  Color,
  DirectionalLight,
  GridHelper,
  Group,
  HemisphereLight,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  PlaneGeometry,
  Scene,
  SphereGeometry,
  Vector3,
  WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { EulerPose, JointVector, Vec3, Waypoint } from "../types";
import { Ur5eRobotRig } from "./Ur5eRobotRig";

function applyPose(target: Group, pose: EulerPose): void {
  target.position.set(pose.position[0], pose.position[1], pose.position[2]);
  target.rotation.set(pose.rpy[0], pose.rpy[1], pose.rpy[2], "XYZ");
}

function makeWaypointFrame(index: number, waypoint: Waypoint): Group {
  const frame = new Group();
  applyPose(frame, waypoint.pose);
  const axes = new AxesHelper(0.08);
  frame.add(axes);

  const marker = new Mesh(
    new SphereGeometry(0.011, 16, 16),
    new MeshStandardMaterial({
      color: index % 2 === 0 ? 0xc45d2e : 0x2c7a7b,
      metalness: 0.1,
      roughness: 0.6,
    }),
  );
  frame.add(marker);
  return frame;
}

export class Ur5eScene {
  readonly ready: Promise<void>;

  private readonly scene = new Scene();
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly robot = new Ur5eRobotRig();
  private readonly targetFrame = new Group();
  private readonly waypointGroup = new Group();
  private readonly resizeObserver: ResizeObserver;
  private trailLine: Line | null = null;

  constructor(private readonly container: HTMLElement) {
    this.scene.background = new Color("#161f4a");
    this.scene.up.set(0, 0, 1);

    this.camera = new PerspectiveCamera(42, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.92, -1.84, 1.38);

    this.renderer = new WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight, false);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.autoUpdate = true;
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(-0.2, 0, 0.35);
    this.controls.enableDamping = true;
    this.controls.maxDistance = 5;
    this.controls.minDistance = 0.6;

    this.scene.add(this.robot.root);
    this.scene.add(this.targetFrame);
    this.scene.add(this.waypointGroup);
    this.targetFrame.add(new AxesHelper(0.14));

    const ambient = new AmbientLight(0xffffff, 0.34);
    this.scene.add(ambient);

    const hemisphere = new HemisphereLight(0x5577d2, 0x12192d, 0.9);
    hemisphere.position.set(0, 0, 2);
    this.scene.add(hemisphere);

    const keyLight = new DirectionalLight(0xf2f6ff, 2.2);
    keyLight.position.set(2.8, -1.9, 3.6);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.set(2048, 2048);
    keyLight.shadow.camera.near = 0.1;
    keyLight.shadow.camera.far = 10;
    this.scene.add(keyLight);

    const fillLight = new DirectionalLight(0x65d6ff, 0.55);
    fillLight.position.set(-2.4, 2.2, 1.9);
    this.scene.add(fillLight);

    const grid = new GridHelper(4.2, 28, 0x4f74ce, 0x2a386d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.0005;
    this.scene.add(grid);

    const floor = new Mesh(
      new PlaneGeometry(6, 6),
      new MeshStandardMaterial({
        color: 0x1a2454,
        metalness: 0.08,
        roughness: 0.88,
        transparent: true,
        opacity: 0.92,
      }),
    );
    floor.position.z = -0.001;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.resizeObserver = new ResizeObserver(() => {
      const width = this.container.clientWidth;
      const height = this.container.clientHeight;
      if (width === 0 || height === 0) {
        return;
      }
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height, false);
    });
    this.resizeObserver.observe(container);

    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });

    this.ready = this.robot.loadMeshes();
  }

  setJoints(joints: JointVector): void {
    this.robot.setJoints(joints);
  }

  setTargetPose(pose: EulerPose): void {
    applyPose(this.targetFrame, pose);
  }

  setWaypoints(waypoints: Waypoint[]): void {
    this.waypointGroup.clear();
    waypoints.forEach((waypoint, index) => {
      this.waypointGroup.add(makeWaypointFrame(index, waypoint));
    });
  }

  setTrail(points: Vec3[]): void {
    if (this.trailLine) {
      this.scene.remove(this.trailLine);
      this.trailLine.geometry.dispose();
      (this.trailLine.material as LineBasicMaterial).dispose();
      this.trailLine = null;
    }

    if (points.length < 2) {
      return;
    }

    const pathPoints =
      points.length > 2
        ? new CatmullRomCurve3(
            points.map((point) => new Vector3(point[0], point[1], point[2])),
            false,
            "catmullrom",
            0.1,
          ).getPoints(Math.max(32, points.length * 8))
        : points.map((point) => new Vector3(point[0], point[1], point[2]));

    const geometry = new BufferGeometry().setFromPoints(pathPoints);
    const material = new LineBasicMaterial({ color: 0xffc05e, linewidth: 2 });
    this.trailLine = new Line(geometry, material);
    this.scene.add(this.trailLine);
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.renderer.setAnimationLoop(null);
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
