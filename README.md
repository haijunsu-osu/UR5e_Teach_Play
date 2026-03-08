# UR5e Visualizer

Browser-based UR5e workspace built with React, TypeScript, Vite, and Three.js.

## Included

- Official public UR5e visual meshes from the Universal Robots ROS 2 description package.
- Interactive joint sliders for all 6 UR5e joints.
- Closed-form UR analytical inverse kinematics with up to 8 branches.
- IK branch selection with direct visualization on the robot model.
- Waypoint placement from the current TCP or the selected IK branch.
- MoveJ joint-space interpolation and playback.

## Sources

- Universal Robots ROS 2 description package for the public UR5e mesh assets and kinematic metadata:
  - `vendor/Universal_Robots_ROS2_Description_full`
- ROS-Industrial `ur_kinematics` reference for the analytical UR inverse-kinematics equations:
  - `vendor/universal_robot_ref`

## Official CAD note

Universal Robots publishes official UR5e STEP and JT CAD files, but those downloads are account-gated on the UR site and are not directly renderable in a browser. This app is set up so converted per-link browser meshes can be dropped into:

- `public/assets/ur5e/visual`

If you later export the official STEP/JT assets into per-link `glTF`, `DAE`, or similar web-friendly meshes with the same link naming, the scene loader can be swapped over with minimal code change.

## Local development

```bash
npm install
npm run dev
```

Production build:

```bash
npm run build
```

## Current scope

- Implemented now: joint-space waypoint execution (`MoveJ`)
- Not implemented yet: Cartesian interpolation (`MoveL` / Cartesian path planning), dynamics limits beyond a simple speed scaling factor, collision checking
