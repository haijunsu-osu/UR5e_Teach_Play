# UR5e Teach & Play

Browser-based UR5e simulation workspace built with React, TypeScript, Vite, and Three.js.

## Launch App

Public app URL:

[https://haijunsu-osu.github.io/UR5e_Teach_Play/](https://haijunsu-osu.github.io/UR5e_Teach_Play/)

If the page is not live yet, wait for the latest GitHub Actions workflow to finish deploying GitHub Pages.

## What The App Does

- Visualizes the UR5e robot in a browser with official public Universal Robots mesh assets.
- Lets you jog all 6 joints with GUI sliders.
- Computes analytical inverse kinematics branches for a target pose.
- Lets you select an IK branch to visualize on the robot.
- Captures waypoints from the current TCP or the selected IK branch.
- Plays a `MoveJ` joint-space interpolation through the stored waypoints.

## How To Use

### 1. Inspect the robot

- The center viewport shows the UR5e model, TCP frame, waypoint frames, and MoveJ trail.
- Drag with the mouse to orbit the camera.
- Use the mouse wheel to zoom.

### 2. Jog the robot

- In `Joint Axis Jog`, move any joint slider or type a value directly.
- The robot updates immediately.
- The TCP pose updates automatically in the `Cartesian Jog` panel.

### 3. Solve IK

- Set a target pose in `Cartesian Jog`.
- Open `IK Configurations`.
- Select any available analytical branch from the dropdown list.
- Click `Add selected branch` if you want to store that branch as a waypoint.

Note:
- A UR5e can have up to 8 analytical branches, but singular poses can collapse some branches into fewer distinct solutions.

### 4. Build a waypoint sequence

- Click `Capture TCP` to save the robot's current joint state as a waypoint.
- Or add the currently selected IK branch as a waypoint.
- Use `Load` to jump back to a saved waypoint.
- Use `Remove` to delete one waypoint.

### 5. Play MoveJ

- Set `Speed scale`.
- Click `Play MoveJ`.
- The app interpolates in joint space through the saved waypoint list.

## UI Layout

- Left panel: station tree and project summary
- Center panel: 3D workcell viewport
- Right panel: Cartesian jog, joint jog, IK branch selector, and MoveJ controls

## Local Development

```bash
npm install
npm run dev
```

Open the local dev server URL printed by Vite, usually:

[http://localhost:5173](http://localhost:5173)

## Production Build

```bash
npm run build
```

## GitHub Pages Deployment

This repository includes a GitHub Actions workflow that builds the app and deploys `dist/` to GitHub Pages whenever `main` is updated.

Files involved:

- `.github/workflows/deploy-pages.yml`
- `vite.config.ts`

## Mesh Source

The browser model currently uses the official public UR5e visual meshes from the Universal Robots ROS 2 description package.

## Official CAD Note

Universal Robots also publishes official UR5e STEP and JT CAD files, but those downloads are account-gated and are not directly renderable in a browser. This app is set up so converted per-link browser meshes can be swapped in under:

- `public/assets/ur5e/visual`

## Technical References

- Universal Robots ROS 2 description package for public UR5e visual assets and kinematic metadata
- ROS-Industrial `ur_kinematics` reference for the analytical UR inverse-kinematics equations

## Current Scope

- Implemented now: UR5e visualization, joint jogging, analytical IK branch selection, waypoint capture, `MoveJ` playback
- Not implemented yet: Cartesian interpolation (`MoveL`), collision checking, time-optimal motion planning, robot controller connectivity
