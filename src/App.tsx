import { useEffect, useRef, useState } from "react";
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
import { buildMoveJTrajectory, type MoveJTrajectory } from "./motion/movej";
import type {
  EulerPose,
  IKSolution,
  JointIndex,
  JointVector,
  MoveJSample,
  Waypoint,
} from "./types";

const INITIAL_TARGET_POSE = rowMajorToPose(forwardKinematics(DEFAULT_JOINTS));
const MENU_ITEMS = ["File", "Edit", "Program", "View", "Tools", "Utilities", "Connect", "Help"];
const TOOLBAR_ACTIONS = [
  "Open",
  "Save",
  "Undo",
  "Frame",
  "Target",
  "Run",
  "Pause",
  "Python",
  "Docs",
] as const;
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

function formatBranchSignature(solution: IKSolution): string {
  const shoulder = solution.branch.shoulder > 0 ? "S+" : "S-";
  const wrist = solution.branch.wrist > 0 ? "W+" : "W-";
  const elbow = solution.branch.elbow > 0 ? "E+" : "E-";
  return `${shoulder} ${wrist} ${elbow}`;
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

function sampleAtTime(samples: MoveJSample[], elapsed: number): MoveJSample {
  for (const sample of samples) {
    if (sample.time >= elapsed) {
      return sample;
    }
  }
  const last = samples.at(-1);
  if (!last) {
    throw new Error("Cannot sample an empty MoveJ trajectory.");
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
  const [activeTrajectory, setActiveTrajectory] = useState<MoveJTrajectory | null>(
    null,
  );
  const [isPlaying, setIsPlaying] = useState(false);
  const [elapsedTime, setElapsedTime] = useState(0);
  const frameRef = useRef<number | null>(null);

  const currentPose = poseFromJoints(currentJoints);
  const solutions = inverseKinematics(targetPose, currentJoints);
  const selectedSolution =
    solutions.find((solution) => solution.id === selectedSolutionId) ?? solutions[0] ?? null;
  const selectedSolutionIndex = selectedSolution
    ? solutions.findIndex((solution) => solution.id === selectedSolution.id)
    : -1;
  const ikBranchCountLabel = `${solutions.length} IK branch${solutions.length === 1 ? "" : "es"}`;
  const analyticalBranchLabel = `${solutions.length} analytical branch${
    solutions.length === 1 ? "" : "es"
  }`;
  const waypointCountLabel = `${waypoints.length} waypoint${waypoints.length === 1 ? "" : "s"}`;
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

  const setTargetField = (axis: 0 | 1 | 2, value: number) => {
    stopPlayback();
    setTargetPose((previous) => ({
      ...previous,
      position: previous.position.map((entry, index) =>
        index === axis ? value : entry,
      ) as EulerPose["position"],
    }));
  };

  const setTargetRotation = (axis: 0 | 1 | 2, degrees: number) => {
    stopPlayback();
    setTargetPose((previous) => ({
      ...previous,
      rpy: previous.rpy.map((entry, index) =>
        index === axis ? degToRad(degrees) : entry,
      ) as EulerPose["rpy"],
    }));
  };

  const addWaypoint = (name: string, joints: JointVector, source: Waypoint["source"]) => {
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
    if (!trajectory.samples.length) {
      return;
    }
    setActiveTrajectory(trajectory);
    setElapsedTime(0);
    setIsPlaying(true);
  };

  const loadWaypoint = (waypoint: Waypoint) => {
    stopPlayback();
    setCurrentJoints(cloneJoints(waypoint.joints));
  };

  const visualizeSolution = (solutionId: string) => {
    const solution = solutions.find((entry) => entry.id === solutionId);
    if (!solution) {
      return;
    }

    stopPlayback();
    setSelectedSolutionId(solution.id);
    setCurrentJoints(cloneJoints(solution.joints));
    setIsIkDropdownOpen(false);
  };

  return (
    <div className="workcell-shell">
      <header className="app-frame">
        <div className="menu-strip">
          {MENU_ITEMS.map((item) => (
            <button key={item} className="menu-item" type="button">
              {item}
            </button>
          ))}
        </div>
        <div className="toolbar-strip">
          {TOOLBAR_ACTIONS.map((action) => (
            <button key={action} className="tool-button" type="button">
              <span className="tool-icon" aria-hidden="true" />
              <span>{action}</span>
            </button>
          ))}
        </div>
      </header>

      <main className="studio-layout">
        <aside className="station-panel">
          <div className="panel-caption">
            <span className="panel-kicker">Station Tree</span>
            <strong>New Station (1)</strong>
          </div>
          <div className="station-tree">
            <button className="tree-item is-root" type="button">
              <span className="tree-chevron">v</span>
              <span>UR5e Base</span>
            </button>
            <button className="tree-item is-active" type="button">
              <span className="tree-indent" />
              <span>UR5e</span>
            </button>
          </div>
          <div className="station-summary">
            <div className="summary-chip">Official UR visuals</div>
            <div className="summary-chip">{ikBranchCountLabel}</div>
            <div className="summary-chip">{waypointCountLabel}</div>
          </div>
        </aside>

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
                <span className="viewer-tag">MoveJ preview</span>
              </div>
            </div>
            <ThreeViewport
              joints={currentJoints}
              targetPose={targetPose}
              waypoints={waypoints}
              trail={trail}
            />
            <div className="status-bar">
              <span>
                TCP:{" "}
                {currentPose.position
                  .map((value) => `${(value * 1000).toFixed(1)} mm`)
                  .join(" / ")}
              </span>
              <span>Selected IK: {selectedSolution ? selectedSolution.label : "none"}</span>
              <span>
                Playback:{" "}
                {activeTrajectory
                  ? `${elapsedTime.toFixed(2)} / ${activeTrajectory.duration.toFixed(2)} s`
                  : "idle"}
              </span>
            </div>
          </div>
        </section>

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
                    <div className="joint-jog-meta">
                      <strong>
                        {index + 1}: {label}
                      </strong>
                      <span>{formatDisplayJointValue(jointIndex, currentValue)}</span>
                    </div>
                    <div className="joint-jog-controls">
                      <input
                        type="range"
                        min={radToDeg(toDisplayJointValue(jointIndex, lower))}
                        max={radToDeg(toDisplayJointValue(jointIndex, upper))}
                        step={0.1}
                        value={radToDeg(displayValue)}
                        onChange={(event) => {
                          stopPlayback();
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
                        <span className="ik-option-head">
                          <strong>{solution.label}</strong>
                          <span>{`#${index + 1}`}</span>
                        </span>
                        <span className="ik-option-meta">
                          {formatBranchSignature(solution)} |{" "}
                          {solution.positionErrorMm.toFixed(3)} mm /{" "}
                          {solution.orientationErrorDeg.toFixed(3)} deg
                        </span>
                        <code>
                          {solution.joints
                            .map((value, jointIndex) =>
                              formatDisplayJointValue(jointIndex as JointIndex, value),
                            )
                            .join("  ")}
                        </code>
                      </button>
                    ))}
                  </div>
                ) : null}

                {selectedSolution ? (
                  <>
                    <div className="ik-summary-grid">
                      <div className="ik-summary-card">
                        <span>Configuration</span>
                        <strong>{formatBranchSignature(selectedSolution)}</strong>
                      </div>
                      <div className="ik-summary-card">
                        <span>Residual</span>
                        <strong>
                          {selectedSolution.positionErrorMm.toFixed(3)} mm /{" "}
                          {selectedSolution.orientationErrorDeg.toFixed(3)} deg
                        </strong>
                      </div>
                    </div>
                    <div className="ik-selected-joints">
                      <span>Selected joint values</span>
                      <code>
                        {selectedSolution.joints
                          .map((value, index) =>
                            formatDisplayJointValue(index as JointIndex, value),
                          )
                          .join(" · ")}
                      </code>
                    </div>
                  </>
                ) : null}
              </>
            ) : (
              <p className="empty-state">
                No closed-form solution for the current target pose within the configured
                joint limits.
              </p>
            )}
            <div className="button-row tight">
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
            </div>
          </section>

          <section className="dock-section">
            <div className="dock-heading">
              <h3>MoveJ</h3>
              <button
                className="ghost-button"
                type="button"
                onClick={() => setWaypoints([])}
              >
                Clear
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
                className="ghost-button"
                type="button"
                onClick={() => addWaypoint("TCP", currentJoints, "current")}
              >
                Capture TCP
              </button>
              <button
                className="accent-button"
                type="button"
                onClick={startMoveJ}
                disabled={waypoints.length === 0}
              >
                Play MoveJ
              </button>
              <button className="ghost-button" type="button" onClick={stopPlayback}>
                Stop
              </button>
            </div>

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
                Capture at least one waypoint to generate a joint-space MoveJ trajectory.
              </p>
            )}
          </section>
        </aside>
      </main>
    </div>
  );
}
