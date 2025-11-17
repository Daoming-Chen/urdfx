# Capability: URDF Parsing

## ADDED Requirements

### Requirement: System SHALL parse URDF from file
The system SHALL parse URDF XML files into an in-memory robot model structure.

#### Scenario: Load UR5e robot from URDF file
**Given** a valid URDF file at path "ur5_urdf/ur5e.urdf"  
**When** the user calls `Robot::fromURDF("ur5_urdf/ur5e.urdf")`  
**Then** the system returns a Robot object containing all links and joints  
**And** the robot has 6 revolute joints  
**And** each joint has valid axis, limits, and origin data

#### Scenario: Handle invalid URDF file
**Given** a malformed URDF file with missing required tags  
**When** the user attempts to parse the file  
**Then** the system throws a `URDFParseException` with descriptive error message  
**And** the error indicates which tag is missing and at which line number

### Requirement: System SHALL extract robot structure
The system SHALL extract the complete robot structure including links, joints, and their hierarchical relationships.

#### Scenario: Build kinematic tree from URDF
**Given** a parsed Robot object from UR5e URDF  
**When** the user queries the kinematic structure  
**Then** the system provides the complete tree from "world" link to "tool0" link  
**And** each joint connects exactly one parent link to one child link  
**And** the tree has no cycles

#### Scenario: Access joint properties
**Given** a parsed Robot object  
**When** the user queries joint "shoulder_pan_joint"  
**Then** the system returns joint type as "revolute"  
**And** returns joint axis as [0, 0, 1]  
**And** returns joint limits (e.g., [-2π, 2π])  
**And** returns joint origin transformation

### Requirement: System SHALL parse visual geometry
The system SHALL parse visual geometry elements including mesh references.

#### Scenario: Load mesh file paths
**Given** a URDF with visual meshes  
**When** the system parses the file  
**Then** each visual element contains the mesh file path  
**And** the mesh origin transformation is stored  
**And** relative paths are resolved relative to URDF directory

### Requirement: System SHALL parse joint limits
The system SHALL extract and validate joint limit specifications.

#### Scenario: Extract revolute joint limits
**Given** a revolute joint with limits [-π, π]  
**When** the system parses the joint  
**Then** the joint lower limit is stored as -π  
**And** the joint upper limit is stored as π  
**And** velocity and effort limits are also extracted if present

#### Scenario: Handle unlimited joints
**Given** a continuous joint without limits  
**When** the system parses the joint  
**Then** the joint is marked as unlimited  
**And** the joint type is stored as "continuous"

### Requirement: System SHALL support URDF from string
The system SHALL support parsing URDF content from in-memory strings.

#### Scenario: Parse URDF from string buffer
**Given** a string containing valid URDF XML content  
**When** the user calls `Robot::fromURDFString(urdf_content)`  
**Then** the system returns a valid Robot object  
**And** the parsing behavior is identical to file-based parsing

### Requirement: System SHALL validate robot model
The system SHALL validate the parsed robot model for common errors.

#### Scenario: Detect disconnected links
**Given** a URDF with links not connected to the root  
**When** the system validates the robot model  
**Then** a warning is logged using spdlog about disconnected links  
**And** only the connected subtree is used for kinematics

#### Scenario: Detect joint limit violations
**Given** a joint with lower limit > upper limit  
**When** the system parses the URDF  
**Then** an exception is thrown with a descriptive error  
**And** the error message identifies the problematic joint
