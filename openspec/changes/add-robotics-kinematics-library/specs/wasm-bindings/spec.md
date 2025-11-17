# Capability: WebAssembly Bindings

## ADDED Requirements

### Requirement: System SHALL compile library to WebAssembly using Emscripten
The system SHALL compile the C++ library to WebAssembly for browser usage.

#### Scenario: Build WASM module with Emscripten
**Given** the urdfx C++ library  
**When** the user runs the WASM build script  
**Then** a urdfx.js and urdfx.wasm file are generated  
**And** the combined size is < 2MB (uncompressed)  
**And** the module works in modern browsers

#### Scenario: Load WASM module in browser
**Given** the built urdfx WASM module  
**When** a web page loads the module with `import urdfx from './urdfx.js'`  
**Then** the module loads successfully  
**And** all exported functions are accessible  
**And** no errors appear in browser console

### Requirement: System SHALL expose essential C++ API via Embind
The system SHALL expose core kinematics functions to JavaScript using Embind.

#### Scenario: Create Robot from URDF string in JS
**Given** URDF content as a JavaScript string  
**When** the user calls `const robot = urdfx.Robot.fromURDFString(urdfContent)`  
**Then** a Robot instance is created in JavaScript  
**And** the robot properties are accessible

#### Scenario: Compute forward kinematics in JS
**Given** a Robot and ForwardKinematics object in JavaScript  
**And** joint angles as a JavaScript array [0, -1.57, 0, 0, 0, 0]  
**When** the user calls `const pose = fk.compute(jointAngles)`  
**Then** the system returns a 4×4 transformation matrix as nested arrays  
**And** the computation completes in < 5ms

### Requirement: System SHALL provide TypeScript type definitions
The system SHALL include TypeScript declaration files (.d.ts) for type safety.

#### Scenario: TypeScript type checking
**Given** TypeScript code using urdfx WASM module  
**When** the user compiles with tsc  
**Then** all urdfx types are recognized  
**And** type errors are caught at compile time  
**And** IDE provides accurate autocomplete

### Requirement: System SHALL optimize WASM binary size
The system SHALL minimize WebAssembly binary size through optimization.

#### Scenario: WASM binary size under limit
**Given** the compiled urdfx.wasm file  
**When** measured after compression (gzip)  
**Then** the size is < 500KB  
**And** the module loads in < 1 second on a typical connection

#### Scenario: Tree-shake unused code
**Given** WASM build configuration  
**When** only FK is used (not IK)  
**Then** the IK solver code is eliminated from the binary  
**And** the binary size is smaller than the full build

### Requirement: System SHALL enable SIMD for performance
The system SHALL use WebAssembly SIMD instructions for Eigen operations.

#### Scenario: SIMD-optimized matrix operations
**Given** a WASM build with SIMD enabled  
**When** FK is computed with matrix multiplications  
**Then** SIMD instructions are used (verify with WASM inspector)  
**And** performance is 2-3× faster than scalar version

### Requirement: System SHALL handle memory management safely
The system SHALL properly manage memory allocated in WASM heap.

#### Scenario: No memory leaks in repeated FK calls
**Given** a loaded WASM module  
**When** FK is computed 10,000 times in a loop  
**Then** WASM heap usage remains constant  
**And** no memory leaks are detected

#### Scenario: Explicit resource cleanup
**Given** a Robot object created in JavaScript  
**When** the user calls `robot.delete()`  
**Then** the C++ object is destroyed  
**And** memory is freed in the WASM heap

### Requirement: System SHALL support file system access via Emscripten FS
The system SHALL support loading URDF files from virtual filesystem.

#### Scenario: Load URDF from virtual FS
**Given** a URDF file preloaded into Emscripten's virtual filesystem  
**When** the user calls `Robot.fromURDF("/models/ur5e.urdf")`  
**Then** the file is read from virtual FS  
**And** the robot is parsed correctly

#### Scenario: Fetch URDF from URL
**Given** a URDF file at a remote URL  
**When** the user fetches the content and creates robot from string  
**Then** the WASM module parses it successfully  
**And** the robot model is usable

### Requirement: System SHALL provide JavaScript-friendly API
The system SHALL adapt C++ API to JavaScript idioms.

#### Scenario: Return plain JavaScript objects
**Given** a FK computation result  
**When** returned to JavaScript  
**Then** the pose is a plain JS object `{position: [x,y,z], rotation: [qw,qx,qy,qz]}`  
**And** no C++ proxy objects are returned (for serialization)

#### Scenario: Accept JavaScript arrays for vectors
**Given** a function expecting joint angles  
**When** the user passes a JavaScript array [0, 1, 2, 3, 4, 5]  
**Then** the function accepts it without conversion  
**And** the array is automatically converted to std::vector

### Requirement: System SHALL support promise-based async operations
The system SHALL support asynchronous operations for expensive computations.

#### Scenario: Async IK solving
**Given** an IK problem that may take 100ms  
**When** the user calls `await ikSolver.solveAsync(target)`  
**Then** the computation runs without blocking the main thread  
**And** the result is returned via Promise

### Requirement: System SHALL be compatible with modern JavaScript bundlers
The system SHALL work with Webpack, Vite, and other bundlers.

#### Scenario: Import in Vite application
**Given** a Vite React project  
**When** the user imports urdfx with `import urdfx from 'urdfx'`  
**Then** Vite bundles the WASM module correctly  
**And** the WASM file is copied to output directory  
**And** the module loads at runtime

### Requirement: System SHALL provide usage examples and documentation
The system SHALL include JavaScript/TypeScript examples.

#### Scenario: Access example code
**Given** the urdfx repository  
**When** the user browses the examples/ directory  
**Then** JavaScript examples are available for FK, IK, and Jacobian  
**And** each example includes clear comments  
**And** examples can run in browser or Node.js
