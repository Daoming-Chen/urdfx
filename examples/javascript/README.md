# JavaScript/WebAssembly Examples for urdfx

## Overview

JavaScript examples using the WebAssembly bindings for urdfx.

## Planned Examples

1. **forward_kinematics.js** - Basic forward kinematics in Node.js
2. **inverse_kinematics.js** - IK solving in the browser
3. **robot_visualization.html** - Interactive 3D robot visualization
4. **performance_benchmark.js** - Performance testing

## Setup

```bash
npm install urdfx-wasm
```

## Basic Usage (Future)

```javascript
import urdfx from 'urdfx-wasm';

// Initialize module
const robot = await urdfx.parseURDF('robot.urdf');

// Compute forward kinematics
const jointAngles = [0, 1.57, -1.57, 0, 0, 0];
const pose = robot.forwardKinematics(jointAngles);

console.log('End effector position:', pose.position);
console.log('End effector orientation:', pose.orientation);
```

## Live Demo

See `apps/visualization/` for a full-featured React application using urdfx-wasm.
