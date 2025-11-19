# JavaScript/WebAssembly Examples for urdfx

## Overview

JavaScript examples using the WebAssembly bindings for urdfx.

## Examples

1. **forward_kinematics.js** - Basic forward kinematics in Node.js
2. **visualization.html** - Interactive 3D robot visualization with Three.js
3. **inverse_kinematics.js** - IK solving in the browser (Planned)
4. **performance_benchmark.js** - Performance testing (Planned)

## Setup

1. Build the WASM module:
   ```bash
   ./scripts/build-wasm.ps1
   ```

2. Install dependencies (if any):
   ```bash
   cd examples/javascript
   npm install
   ```

## Running Examples

### Forward Kinematics

This example loads the UR5e robot model and computes forward kinematics for a given joint configuration.

```bash
cd examples/javascript
npm run fk
# OR
node forward_kinematics.js
```

### Visualization Demo

This demo visualizes the robot using Three.js and allows control via sliders.
Since it loads local files (WASM, URDF), you must run it using a local web server.

Using Python:
```bash
# Run from the repository root
python -m http.server
# Open http://localhost:8000/examples/javascript/visualization.html
```

Using Node.js (serve):
```bash
npx serve .
# Open the provided URL and navigate to examples/javascript/visualization.html
```

## Basic Usage

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
