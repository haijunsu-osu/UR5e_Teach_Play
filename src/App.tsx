import {
  useEffect,
  useRef,
  useState,
  type CSSProperties,
  type KeyboardEvent as ReactKeyboardEvent,
  type PointerEvent as ReactPointerEvent,
} from "react";
import ThreeViewport from "./components/ThreeViewport";
import {
  DEFAULT_JOINTS,
  JOINT_DISPLAY_OFFSETS,
  JOINT_LABELS,
  JOINT_LIMITS,
} from "./data/ur5e";
import { forwardKinematics, inverseKinematics } from "./kinematics/ur5eKinematics";
import {
  clamp,
  degToRad,
  formatRadians,
  radToDeg,
  rowMajorToPose,
} from "./lib/math";
import { buildMoveJTrajectory } from "./motion/movej";
import { buildMoveLTrajectory } from "./motion/movel";
import type {
  EulerPose,
  IKSolution,
  JointIndex,
  JointVector,
  MotionTrajectory,
  TrajectorySample,
  Waypoint,
} from "./types";

const INITIAL_TARGET_POSE = rowMajorToPose(forwardKinematics(DEFAULT_JOINTS));
const POSITION_FIELDS = [
  { label: "X", axis: 0 as const, suffix: "m" },
  { label: "Y", axis: 1 as const, suffix: "m" },
  { label: "Z", axis: 2 as const, suffix: "m" },
] as const;
const ORIENTATION_FIELDS = [
  { label: "Roll", axis: 0 as const, suffix: "deg" },
  { label: "Pitch", axis: 1 as const, suffix: "deg" },
  { label: "Yaw", axis: 2 as const, suffix: "deg" },
] as const;
const DEFAULT_TEACH_PANEL_WIDTH = 250;
const DEFAULT_ROBOT_PANEL_WIDTH = 430;
const MIN_TEACH_PANEL_WIDTH = 210;
const MIN_ROBOT_PANEL_WIDTH = 340;
const MIN_VIEWER_CANVAS_WIDTH = 360;
const MIN_VIEWER_SHELL_WIDTH = 560;
const SPLITTER_SIZE = 12;

type DividerTarget = "teach" | "robot";

interface ResizeSession {
  target: DividerTarget;
  startX: number;
  startWidth: number;
  min: number;
  max: number;
}

function cloneJoints(joints: JointVector): JointVector {
  return [...joints] as JointVector;
}

function poseFromJoints(joints: JointVector): EulerPose {
  return rowMajorToPose(forwardKinematics(joints));
}

function toDisplayJointValue(joint: JointIndex, internalValue: number): number {
  return internalValue - JOINT_DISPLAY_OFFSETS[joint];
}

function toInternalJointValue(joint: JointIndex, displayValue: number): number {
  return displayValue + JOINT_DISPLAY_OFFSETS[joint];
}

function formatDisplayJointValue(joint: JointIndex, internalValue: number): string {
  return formatRadians(toDisplayJointValue(joint, internalValue));
}

function updateJointValue(
  joints: JointVector,
  index: JointIndex,
  nextValue: number,
): JointVector {
  const next = cloneJoints(joints);
  const [lower, upper] = JOINT_LIMITS[index]!;
  next[index] = clamp(nextValue, lower, upper);
  return next;
}

function sampleAtTime(samples: TrajectorySample[], elapsed: number): TrajectorySample {
  for (const sample of samples) {
    if (sample.time >= elapsed) {
      return sample;
    }
  }
  const last = samples.at(-1);
  if (!last) {
    throw new Error("Cannot sample an empty trajectory.");
  }
  return last;
}

export default function App() {
  const [currentJoints, setCurrentJoints] = useState<JointVector>(DEFAULT_JOINTS);
  const [targetPose, setTargetPose] = useState<EulerPose>(INITIAL_TARGET_POSE);
  const [selectedSolutionId, setSelectedSolutionId] = useState<string | null>(null);
  const [isIkDropdownOpen, setIsIkDropdownOpen] = useState(false);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [speedScale, setSpeedScale] = useState(0.5);
  const [activeTrajectory, setActiveTrajectory] = useState<MotionTrajectory | null>(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [elapsedTime, setElapsedTime] = useState(0);
  const [motionError, setMotionError] = useState<string | null>(null);
  const [teachPanelWidth, setTeachPanelWidth] = useState(DEFAULT_TEACH_PANEL_WIDTH);
  const [robotPanelWidth, setRobotPanelWidth] = useState(DEFAULT_ROBOT_PANEL_WIDTH);
  const [activeDivider, setActiveDivider] = useState<DividerTarget | null>(null);
  const frameRef = useRef<number | null>(null);
  const studioLayoutRef = useRef<HTMLElement | null>(null);
  const viewerBodyRef = useRef<HTMLDivElement | null>(null);
  const resizeSessionRef = useRef<ResizeSession | null>(null);

  const currentPose = poseFromJoints(currentJoints);
  const solutions = inverseKinematics(targetPose, currentJoints);
  const selectedSolution =
    solutions.find((solution) => solution.id === selectedSolutionId) ?? solutions[0] ?? null;
  const selectedSolutionIndex = selectedSolution
    ? solutions.findIndex((solution) => solution.id === selectedSolution.id)
    : -1;
  const analyticalBranchLabel = `${solutions.length} analytical branch${
    solutions.length === 1 ? "" : "es"
  }`;
  const trail = activeTrajectory
    ? activeTrajectory.samples.map((sample) => sample.tcpPose.position)
    : [currentPose.position, ...waypoints.map((waypoint) => waypoint.pose.position)];

  useEffect(() => {
    if (!solutions.length) {
      setSelectedSolutionId(null);
      return;
    }
    if (!selectedSolutionId || !solutions.some((solution) => solution.id === selectedSolutionId)) {
      const firstSolution = solutions[0];
      if (firstSolution) {
        setSelectedSolutionId(firstSolution.id);
      }
    }
  }, [selectedSolutionId, solutions]);

  useEffect(() => {
    setTargetPose(poseFromJoints(currentJoints));
  }, [currentJoints]);

  useEffect(() => {
    const syncPanelWidths = () => {
      const viewerWidth = viewerBodyRef.current?.clientWidth ?? 0;
      const teachMax = Math.max(
        MIN_TEACH_PANEL_WIDTH,
        viewerWidth - MIN_VIEWER_CANVAS_WIDTH - SPLITTER_SIZE,
      );
      setTeachPanelWidth((previous) =>
        clamp(previous, MIN_TEACH_PANEL_WIDTH, teachMax),
      );

      const studioWidth = studioLayoutRef.current?.clientWidth ?? 0;
      const robotMax = Math.max(
        MIN_ROBOT_PANEL_WIDTH,
        studioWidth - MIN_VIEWER_SHELL_WIDTH - SPLITTER_SIZE,
      );
      setRobotPanelWidth((previous) =>
        clamp(previous, MIN_ROBOT_PANEL_WIDTH, robotMax),
      );
    };

    syncPanelWidths();
    window.addEventListener("resize", syncPanelWidths);
    return () => {
      window.removeEventListener("resize", syncPanelWidths);
    };
  }, []);

  useEffect(() => {
    const handlePointerMove = (event: PointerEvent) => {
      const session = resizeSessionRef.current;
      if (!session) {
        return;
      }

      const delta = event.clientX - session.startX;
      const nextWidth =
        session.target === "teach"
          ? session.startWidth + delta
          : session.startWidth - delta;
      const clampedWidth = clamp(nextWidth, session.min, session.max);

      if (session.target === "teach") {
        setTeachPanelWidth(clampedWidth);
        return;
      }
      setRobotPanelWidth(clampedWidth);
    };

    const endResize = () => {
      resizeSessionRef.current = null;
      setActiveDivider(null);
      document.body.classList.remove("is-resizing");
    };

    window.addEventListener("pointermove", handlePointerMove);
    window.addEventListener("pointerup", endResize);
    window.addEventListener("pointercancel", endResize);

    return () => {
      window.removeEventListener("pointermove", handlePointerMove);
      window.removeEventListener("pointerup", endResize);
      window.removeEventListener("pointercancel", endResize);
      document.body.classList.remove("is-resizing");
    };
  }, []);

  useEffect(() => {
    if (!isPlaying || !activeTrajectory) {
      return undefined;
    }

    let startTime = 0;
    const samples = activeTrajectory.samples;
    const duration = activeTrajectory.duration;

    const tick = (timestamp: number) => {
      if (startTime === 0) {
        startTime = timestamp;
      }
      const nextElapsed = Math.min((timestamp - startTime) / 1000, duration);
      const sample = sampleAtTime(samples, nextElapsed);
      setElapsedTime(nextElapsed);
      setCurrentJoints(sample.joints);

      if (nextElapsed >= duration) {
        setIsPlaying(false);
        return;
      }
      frameRef.current = requestAnimationFrame(tick);
    };

    frameRef.current = requestAnimationFrame(tick);
    return () => {
      if (frameRef.current !== null) {
        cancelAnimationFrame(frameRef.current);
      }
    };
  }, [activeTrajectory, isPlaying]);

  const stopPlayback = () => {
    setIsPlaying(false);
    if (frameRef.current !== null) {
      cancelAnimationFrame(frameRef.current);
    }
  };

  const scrubTrajectory = (nextElapsed: number) => {
    if (!activeTrajectory?.samples.length) {
      return;
    }

    stopPlayback();
    const boundedElapsed = clamp(nextElapsed, 0, activeTrajectory.duration);
    const sample = sampleAtTime(activeTrajectory.samples, boundedElapsed);
    setElapsedTime(boundedElapsed);
    setCurrentJoints(sample.joints);
  };

  const getTeachPanelBounds = (): readonly [number, number] => {
    const viewerWidth = viewerBodyRef.current?.clientWidth ?? 0;
    return [
      MIN_TEACH_PANEL_WIDTH,
      Math.max(MIN_TEACH_PANEL_WIDTH, viewerWidth - MIN_VIEWER_CANVAS_WIDTH - SPLITTER_SIZE),
    ] as const;
  };

  const getRobotPanelBounds = (): readonly [number, number] => {
    const studioWidth = studioLayoutRef.current?.clientWidth ?? 0;
    return [
      MIN_ROBOT_PANEL_WIDTH,
      Math.max(MIN_ROBOT_PANEL_WIDTH, studioWidth - MIN_VIEWER_SHELL_WIDTH - SPLITTER_SIZE),
    ] as const;
  };

  const startResize =
    (target: DividerTarget) => (event: ReactPointerEvent<HTMLDivElement>) => {
      const [min, max] =
        target === "teach" ? getTeachPanelBounds() : getRobotPanelBounds();
      resizeSessionRef.current = {
        target,
        startX: event.clientX,
        startWidth: target === "teach" ? teachPanelWidth : robotPanelWidth,
        min,
        max,
      };
      setActiveDivider(target);
      document.body.classList.add("is-resizing");
    };

  const nudgeDivider =
    (target: DividerTarget) => (event: ReactKeyboardEvent<HTMLDivElement>) => {
      if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") {
        return;
      }

      event.preventDefault();
      const delta = event.key === "ArrowRight" ? 24 : -24;
      if (target === "teach") {
        const [min, max] = getTeachPanelBounds();
        setTeachPanelWidth((previous) => clamp(previous + delta, min, max));
        return;
      }

      const [min, max] = getRobotPanelBounds();
      setRobotPanelWidth((previous) => clamp(previous - delta, min, max));
    };

  const playTrajectory = (trajectory: MotionTrajectory) => {
    stopPlayback();

    if (!trajectory.samples.length) {
      setActiveTrajectory(null);
      setElapsedTime(0);
      setMotionError(trajectory.error ?? "The motion planner returned no trajectory.");
      return;
    }

    setMotionError(null);
    setActiveTrajectory(trajectory);
    setElapsedTime(0);
    setIsPlaying(true);
  };

  const clearMotionPlan = () => {
    stopPlayback();
    setWaypoints([]);
    setActiveTrajectory(null);
    setElapsedTime(0);
    setMotionError(null);
  };

  const setTargetField = (axis: 0 | 1 | 2, value: number) => {
    stopPlayback();
    setMotionError(null);
    setTargetPose((previous) => ({
      ...previous,
      position: previous.position.map((entry, index) =>
        index === axis ? value : entry,
      ) as EulerPose["position"],
    }));
  };

  const setTargetRotation = (axis: 0 | 1 | 2, degrees: number) => {
    stopPlayback();
    setMotionError(null);
    setTargetPose((previous) => ({
      ...previous,
      rpy: previous.rpy.map((entry, index) =>
        index === axis ? degToRad(degrees) : entry,
      ) as EulerPose["rpy"],
    }));
  };

  const addWaypoint = (name: string, joints: JointVector, source: Waypoint["source"]) => {
    setMotionError(null);
    setWaypoints((previous) => [
      ...previous,
      {
        id: crypto.randomUUID(),
        name: `${name} ${previous.length + 1}`,
        joints: cloneJoints(joints),
        pose: poseFromJoints(joints),
        source,
      },
    ]);
  };

  const startMoveJ = () => {
    const points = [currentJoints, ...waypoints.map((waypoint) => waypoint.joints)];
    const trajectory = buildMoveJTrajectory(points, speedScale);
    playTrajectory(trajectory);
  };

  const startMoveL = () => {
    const trajectory = buildMoveLTrajectory(
      currentJoints,
      waypoints.map((waypoint) => waypoint.pose),
      speedScale,
    );
    playTrajectory(trajectory);
  };

  const loadWaypoint = (waypoint: Waypoint) => {
    stopPlayback();
    setMotionError(null);
    setCurrentJoints(cloneJoints(waypoint.joints));
  };

  const visualizeSolution = (solutionId: string) => {
    const solution = solutions.find((entry) => entry.id === solutionId);
    if (!solution) {
      return;
    }

    stopPlayback();
    setMotionError(null);
    setSelectedSolutionId(solution.id);
    setCurrentJoints(cloneJoints(solution.joints));
    setIsIkDropdownOpen(false);
  };

  const studioLayoutStyle = {
    "--robot-panel-width": `${robotPanelWidth}px`,
  } as CSSProperties;
  const viewerBodyStyle = {
    "--teach-panel-width": `${teachPanelWidth}px`,
  } as CSSProperties;

  return (
    <div className="workcell-shell">
      <main
        ref={studioLayoutRef}
        className="studio-layout"
        style={studioLayoutStyle}
      >
        <section className="viewer-shell">
          <div className="viewer-panel">
            <div className="viewer-header viewer-header-panel">
              <div className="viewer-title-block">
                <p className="panel-kicker">Robot Cell</p>
                <h1>UR5e Simulation Workcell</h1>
                <span className="viewer-meta-line">
                  Perspective view | Base frame | Tool flange / tool0
                </span>
              </div>
              <div className="viewer-tags">
                <span className="viewer-tag">Official UR meshes</span>
                <span className="viewer-tag">Analytical IK x8</span>
                <span className="viewer-tag">MoveJ + MoveL</span>
              </div>
            </div>
            <div
              ref={viewerBodyRef}
              className="viewer-body"
              style={viewerBodyStyle}
            >
              <aside className="viewer-side-panel">
                <div className="dock-heading">
                  <h3>Teach &amp; Motion</h3>
                  <button
                    className="ghost-button"
                    type="button"
                    onClick={clearMotionPlan}
                  >
                    Clear
                  </button>
                </div>

                <div className="button-row tight viewer-side-actions">
                  <button
                    className="accent-button"
                    type="button"
                    disabled={!selectedSolution}
                    onClick={() => {
                      if (!selectedSolution) {
                        return;
                      }
                      addWaypoint("IK", selectedSolution.joints, "ik");
                    }}
                  >
                    Add selected branch
                  </button>
                  <button
                    className="ghost-button"
                    type="button"
                    onClick={() => addWaypoint("TCP", currentJoints, "current")}
                  >
                    Capture TCP
                  </button>
                </div>

                <label className="inline-slider">
                  <span>Speed scale</span>
                  <input
                    type="range"
                    min="0.1"
                    max="1.0"
                    step="0.05"
                    value={speedScale}
                    onChange={(event) => setSpeedScale(Number(event.target.value))}
                  />
                  <strong>{speedScale.toFixed(2)}x</strong>
                </label>

                <div className="button-row tight">
                  <button
                    className="accent-button"
                    type="button"
                    onClick={startMoveJ}
                    disabled={waypoints.length === 0}
                  >
                    Play MoveJ
                  </button>
                  <button
                    className="accent-button"
                    type="button"
                    onClick={startMoveL}
                    disabled={waypoints.length === 0}
                  >
                    Play MoveL
                  </button>
                  <button className="ghost-button" type="button" onClick={stopPlayback}>
                    Stop
                  </button>
                </div>

                <label className="timeline-slider">
                  <span>
                    Trajectory {activeTrajectory ? `(${activeTrajectory.motionType})` : ""}
                  </span>
                  <div className="timeline-slider-row">
                    <input
                      type="range"
                      min="0"
                      max={activeTrajectory?.duration ?? 1}
                      step="0.01"
                      value={activeTrajectory ? elapsedTime : 0}
                      disabled={!activeTrajectory || Boolean(motionError)}
                      onChange={(event) => scrubTrajectory(Number(event.target.value))}
                    />
                    <strong>
                      {activeTrajectory
                        ? `${elapsedTime.toFixed(2)} / ${activeTrajectory.duration.toFixed(2)} s`
                        : "No plan"}
                    </strong>
                  </div>
                </label>

                {waypoints.length ? (
                  <div className="waypoint-table">
                    {waypoints.map((waypoint) => (
                      <article key={waypoint.id} className="waypoint-card">
                        <div className="waypoint-copy">
                          <strong>{waypoint.name}</strong>
                          <p>
                            {waypoint.source === "ik" ? "IK branch" : "Current TCP"} |{" "}
                            {waypoint.pose.position
                              .map((value) => `${(value * 1000).toFixed(0)} mm`)
                              .join(" / ")}
                          </p>
                        </div>
                        <div className="button-row compact">
                          <button
                            className="ghost-button"
                            type="button"
                            onClick={() => loadWaypoint(waypoint)}
                          >
                            Load
                          </button>
                          <button
                            className="ghost-button"
                            type="button"
                            onClick={() =>
                              setWaypoints((previous) =>
                                previous.filter((entry) => entry.id !== waypoint.id),
                              )
                            }
                          >
                            Remove
                          </button>
                        </div>
                      </article>
                    ))}
                  </div>
                ) : (
                  <p className="empty-state">
                    Capture at least one waypoint to generate MoveJ or MoveL trajectories.
                  </p>
                )}
              </aside>

              <div
                className={`panel-splitter ${activeDivider === "teach" ? "is-active" : ""}`}
                role="separator"
                tabIndex={0}
                aria-label="Resize teach panel"
                aria-orientation="vertical"
                aria-valuenow={Math.round(teachPanelWidth)}
                onPointerDown={startResize("teach")}
                onKeyDown={nudgeDivider("teach")}
              />

              <div className="viewer-canvas-area">
                <ThreeViewport
                  joints={currentJoints}
                  targetPose={targetPose}
                  waypoints={waypoints}
                  trail={trail}
                />
              </div>
            </div>

            <div className="status-bar">
              <span>
                TCP:{" "}
                {currentPose.position
                  .map((value) => `${(value * 1000).toFixed(1)} mm`)
                  .join(" / ")}
              </span>
              <span>Selected IK: {selectedSolution ? selectedSolution.label : "none"}</span>
              <span>
                Motion:{" "}
                {motionError
                  ? motionError
                  : activeTrajectory
                    ? `${activeTrajectory.motionType} ${elapsedTime.toFixed(2)} / ${activeTrajectory.duration.toFixed(2)} s`
                  : "idle"}
              </span>
            </div>
          </div>
        </section>

        <div
          className={`panel-splitter panel-splitter-main ${
            activeDivider === "robot" ? "is-active" : ""
          }`}
          role="separator"
          tabIndex={0}
          aria-label="Resize robot controls panel"
          aria-orientation="vertical"
          aria-valuenow={Math.round(robotPanelWidth)}
          onPointerDown={startResize("robot")}
          onKeyDown={nudgeDivider("robot")}
        />

        <aside className="robot-panel">
          <div className="robot-panel-header">
            <div>
              <p className="panel-kicker">UR5e Panel</p>
              <h2>Robot Controls</h2>
            </div>
            <button
              className="ghost-button"
              type="button"
              onClick={() => {
                stopPlayback();
                setMotionError(null);
                setTargetPose(currentPose);
              }}
            >
              Sync from TCP
            </button>
          </div>

          <section className="dock-section">
            <div className="dock-heading">
              <h3>Cartesian Jog</h3>
              <span>Tool frame with respect to robot base</span>
            </div>
            <div className="pose-grid">
              {POSITION_FIELDS.map(({ label, axis, suffix }) => (
                <label key={label} className={`pose-cell pose-cell-${label.toLowerCase()}`}>
                  <span>{label}</span>
                  <input
                    type="number"
                    step="0.001"
                    value={targetPose.position[axis].toFixed(3)}
                    onChange={(event) => setTargetField(axis, Number(event.target.value))}
                  />
                  <small>{suffix}</small>
                </label>
              ))}
              {ORIENTATION_FIELDS.map(({ label, axis, suffix }) => (
                <label key={label} className={`pose-cell pose-cell-${label.toLowerCase()}`}>
                  <span>{label}</span>
                  <input
                    type="number"
                    step="0.1"
                    value={radToDeg(targetPose.rpy[axis]).toFixed(1)}
                    onChange={(event) => setTargetRotation(axis, Number(event.target.value))}
                  />
                  <small>{suffix}</small>
                </label>
              ))}
            </div>
          </section>

          <section className="dock-section">
            <div className="dock-heading">
              <h3>Joint Axis Jog</h3>
              <button
                className="ghost-button"
                type="button"
                onClick={() => {
                  stopPlayback();
                  setMotionError(null);
                  setCurrentJoints(DEFAULT_JOINTS);
                }}
              >
                Home
              </button>
            </div>
            <div className="joint-jog-list">
              {JOINT_LABELS.map((label, index) => {
                const jointIndex = index as JointIndex;
                const [lower, upper] = JOINT_LIMITS[jointIndex]!;
                const currentValue = currentJoints[jointIndex];
                const displayValue = toDisplayJointValue(jointIndex, currentValue);
                return (
                  <div key={label} className="joint-jog-row">
                    <strong className="joint-jog-label">
                      {index + 1}: {label}
                    </strong>
                    <input
                      type="range"
                      min={radToDeg(toDisplayJointValue(jointIndex, lower))}
                      max={radToDeg(toDisplayJointValue(jointIndex, upper))}
                      step={0.1}
                      value={radToDeg(displayValue)}
                      onChange={(event) => {
                        stopPlayback();
                        setMotionError(null);
                        setCurrentJoints((previous) =>
                          updateJointValue(
                            previous,
                            jointIndex,
                            toInternalJointValue(
                              jointIndex,
                              degToRad(Number(event.target.value)),
                            ),
                          ),
                        );
                      }}
                    />
                    <input
                      className="joint-number"
                      type="number"
                      step="0.1"
                      value={radToDeg(displayValue).toFixed(1)}
                      onChange={(event) => {
                        stopPlayback();
                        setMotionError(null);
                        setCurrentJoints((previous) =>
                          updateJointValue(
                            previous,
                            jointIndex,
                            toInternalJointValue(
                              jointIndex,
                              degToRad(Number(event.target.value)),
                            ),
                          ),
                        );
                      }}
                    />
                  </div>
                );
              })}
            </div>
          </section>

          <section className="dock-section">
            <div className="dock-heading">
              <h3>IK Configurations</h3>
              <span>{analyticalBranchLabel}</span>
            </div>
            {solutions.length ? (
              <>
                <button
                  className={`ik-dropdown-trigger ${isIkDropdownOpen ? "is-open" : ""}`}
                  type="button"
                  onClick={() => setIsIkDropdownOpen((previous) => !previous)}
                >
                  <span className="ik-dropdown-copy">
                    <strong>{selectedSolution?.label ?? "Select IK branch"}</strong>
                    <span>
                      {selectedSolutionIndex >= 0
                        ? `Branch ${selectedSolutionIndex + 1} of ${solutions.length}`
                        : "Closed-form branch list"}
                    </span>
                  </span>
                  <span className="ik-dropdown-caret">
                    {isIkDropdownOpen ? "Hide list" : "Show list"}
                  </span>
                </button>

                {isIkDropdownOpen ? (
                  <div className="ik-dropdown-list" role="listbox" aria-label="IK branches">
                    {solutions.map((solution, index) => (
                      <button
                        key={solution.id}
                        className={`ik-option ${
                          selectedSolution?.id === solution.id ? "is-active" : ""
                        }`}
                        type="button"
                        onClick={() => visualizeSolution(solution.id)}
                      >
                        <span className="ik-option-line">
                          <strong>{`#${index + 1}`}</strong>
                          <code>
                            {solution.joints
                              .map((value, jointIndex) =>
                                formatDisplayJointValue(jointIndex as JointIndex, value),
                              )
                              .join("  ")}
                          </code>
                        </span>
                      </button>
                    ))}
                  </div>
                ) : null}
              </>
            ) : (
              <p className="empty-state">
                No closed-form solution for the current target pose within the configured
                joint limits.
              </p>
            )}
          </section>
        </aside>
      </main>
    </div>
  );
}
