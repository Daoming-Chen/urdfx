# Capability: Visualization App

## ADDED Requirements

### Requirement: System SHALL provide Three.js-based robot visualization
The system SHALL provide a web application for visualizing robot kinematics using Three.js.

#### Scenario: Load and display UR5e robot
**Given** the visualization app is running  
**When** the user loads the UR5e URDF  
**Then** the robot is rendered in 3D with correct geometry  
**And** mesh files are loaded and displayed  
**And** the robot is centered in the viewport

#### Scenario: Render robot with textures
**Given** a URDF with mesh files (OBJ format)  
**When** the robot is loaded  
**Then** meshes are displayed with correct colors/materials  
**And** lighting is applied for 3D depth perception  
**And** the frame rate is > 30 FPS

### Requirement: System SHALL provide interactive joint control via sliders
The system SHALL provide UI controls for manipulating joint angles.

#### Scenario: Adjust joint angles with sliders
**Given** a loaded robot with 6 joints  
**When** the user drags a slider for joint 2  
**Then** the robot updates in real-time (< 50ms latency)  
**And** the joint moves smoothly  
**And** joint limits are respected in slider range

#### Scenario: Display current joint angles
**Given** the user adjusts multiple joints  
**When** viewing the UI  
**Then** current joint angle values are displayed in degrees  
**And** values update live as sliders move

### Requirement: System SHALL visualize forward kinematics visualization
The system SHALL visualize FK computation results in real-time.

#### Scenario: Show end-effector pose
**Given** the robot visualization with active FK  
**When** joint angles change  
**Then** the end-effector pose (position + orientation) is displayed  
**And** a coordinate frame gizmo is shown at the end-effector  
**And** position is shown in meters, orientation in Euler angles

### Requirement: System SHALL support inverse kinematics mode with target manipulation
The system SHALL allow dragging the end-effector to solve IK.

#### Scenario: Drag end-effector to target
**Given** the app is in IK mode  
**When** the user drags the end-effector gizmo to a new position  
**Then** IK is solved using the WASM urdfx module  
**And** the robot joints update to reach the target  
**And** the solution updates in real-time (< 100ms)

#### Scenario: Visual feedback for unreachable targets
**Given** the user drags the end-effector to an unreachable pose  
**When** IK fails to converge  
**Then** the end-effector gizmo turns red  
**And** the robot moves to the closest reachable pose  
**And** an error message is displayed

### Requirement: System SHALL display kinematic information
The system SHALL show kinematic data during manipulation.

#### Scenario: Show Jacobian condition number
**Given** FK computation is active  
**When** the robot is in a configuration  
**Then** the Jacobian condition number is computed and displayed  
**And** the display changes color near singularities (red for singular)

#### Scenario: Show manipulability ellipsoid
**Given** an advanced visualization mode  
**When** enabled  
**Then** an ellipsoid representing manipulability is drawn at end-effector  
**And** the ellipsoid shape reflects directional manipulability

### Requirement: System SHALL support URDF file upload
The system SHALL allow users to upload custom URDF files.

#### Scenario: Upload URDF from local file
**Given** the visualization app  
**When** the user clicks "Upload URDF" and selects a file  
**Then** the file is read into the browser  
**And** the robot is parsed and displayed  
**And** mesh paths are resolved relative to URDF location

#### Scenario: Load URDF from URL
**Given** the user provides a URL to a URDF file  
**When** the URL is submitted  
**Then** the file is fetched via AJAX  
**And** the robot is loaded and displayed

### Requirement: System SHALL provide camera controls for 3D navigation
The system SHALL provide intuitive 3D camera controls.

#### Scenario: Orbit camera around robot
**Given** the 3D viewport  
**When** the user drags with left mouse button  
**Then** the camera orbits around the robot center  
**And** the robot remains in view

#### Scenario: Zoom camera
**Given** the 3D viewport  
**When** the user scrolls the mouse wheel  
**Then** the camera zooms in/out smoothly  
**And** zoom limits prevent clipping through objects

#### Scenario: Pan camera
**Given** the 3D viewport  
**When** the user drags with right mouse button (or Shift+left)  
**Then** the camera pans in the viewport plane  
**And** the robot moves accordingly

### Requirement: System SHALL provide responsive UI layout
The system SHALL work on different screen sizes.

#### Scenario: Desktop layout
**Given** a desktop browser with 1920×1080 resolution  
**When** the app loads  
**Then** the 3D viewport occupies the main area  
**And** controls are in a sidebar  
**And** all UI elements are accessible

#### Scenario: Tablet layout
**Given** a tablet with 1024×768 resolution  
**When** the app loads  
**Then** the UI adapts to smaller screen  
**And** controls may be in a collapsible panel  
**And** the viewport remains functional

### Requirement: System SHALL optimize performance optimization for real-time interaction
The system SHALL maintain smooth frame rates during interaction.

#### Scenario: 60 FPS during joint manipulation
**Given** the user rapidly adjusts joint sliders  
**When** monitored with browser performance tools  
**Then** the frame rate stays above 60 FPS  
**And** FK/IK computations don't block rendering

#### Scenario: Efficient WASM module usage
**Given** FK/IK computations via WASM  
**When** called repeatedly (e.g., 60 times per second)  
**Then** no memory leaks occur  
**And** computation overhead is < 5ms per call

### Requirement: System SHALL use modern frontend framework
The system SHALL use React for UI component management.

#### Scenario: Modular React components
**Given** the visualization app source code  
**When** inspected  
**Then** the UI is composed of reusable React components  
**And** state management is handled via hooks  
**And** the code follows React best practices

### Requirement: System SHALL build with Vite
The system SHALL use Vite for fast development and production builds.

#### Scenario: Development server with HMR
**Given** the visualization app source  
**When** the user runs `npm run dev`  
**Then** Vite starts a development server  
**And** hot module replacement (HMR) updates the page on code changes  
**And** changes are reflected in < 1 second

#### Scenario: Production build
**Given** the visualization app source  
**When** the user runs `npm run build`  
**Then** Vite creates an optimized production bundle  
**And** assets are minified and hashed  
**And** the bundle size is reasonable (< 5MB)

### Requirement: System SHALL provide example usage documentation
The system SHALL include documentation for using the visualization app.

#### Scenario: Access user guide
**Given** the running visualization app  
**When** the user clicks "Help" or "?"  
**Then** a modal displays usage instructions  
**And** keyboard shortcuts are listed  
**And** links to documentation are provided
