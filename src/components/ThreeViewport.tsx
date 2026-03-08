import { useEffect, useRef, useState } from "react";
import { Ur5eScene } from "../scene/Ur5eScene";
import type { EulerPose, JointVector, Vec3, Waypoint } from "../types";

interface ThreeViewportProps {
  joints: JointVector;
  targetPose: EulerPose;
  waypoints: Waypoint[];
  trail: Vec3[];
}

export default function ThreeViewport({
  joints,
  targetPose,
  waypoints,
  trail,
}: ThreeViewportProps) {
  const hostRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<Ur5eScene | null>(null);
  const [sceneReady, setSceneReady] = useState(false);
  const [sceneError, setSceneError] = useState<string | null>(null);

  useEffect(() => {
    if (!hostRef.current) {
      return undefined;
    }

    const scene = new Ur5eScene(hostRef.current);
    sceneRef.current = scene;
    scene
      .ready.then(() => setSceneReady(true))
      .catch((error: unknown) => {
        console.error(error);
        setSceneError("Failed to load the UR5e mesh assets.");
      });

    return () => {
      scene.dispose();
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    sceneRef.current?.setJoints(joints);
  }, [joints]);

  useEffect(() => {
    sceneRef.current?.setTargetPose(targetPose);
  }, [targetPose]);

  useEffect(() => {
    sceneRef.current?.setWaypoints(waypoints);
  }, [waypoints]);

  useEffect(() => {
    sceneRef.current?.setTrail(trail);
  }, [trail]);

  return (
    <div className="viewport">
      <div className="viewport-canvas" ref={hostRef} />
      {!sceneReady && !sceneError ? (
        <div className="viewport-status">Loading UR5e visual meshes…</div>
      ) : null}
      {sceneError ? <div className="viewport-status viewport-error">{sceneError}</div> : null}
    </div>
  );
}
