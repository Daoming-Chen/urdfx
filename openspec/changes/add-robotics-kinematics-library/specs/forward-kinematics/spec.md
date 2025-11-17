# Capability: Forward Kinematics

## ADDED Requirements

### Requirement: System SHALL compute forward kinematics for kinematic chain
The system SHALL compute the end-effector pose given joint angles using Eigen transformations.

#### Scenario: Compute FK for UR5e at zero configuration
**Given** a UR5e robot model  
**And** joint angles q = [0, 0, 0, 0, 0, 0]  
**When** the user calls `forwardKinematics.compute(q)`  
**Then** the system returns a 4x4 transformation matrix  
**And** the position matches the expected end-effector location  
**And** the orientation is the identity rotation

#### Scenario: Compute FK for arbitrary configuration
**Given** a UR5e robot model  
**And** joint angles q = [0, -π/2, 0, -π/2, 0, 0]  
**When** the user computes forward kinematics  
**Then** the returned pose is within 1mm of the analytical solution  
**And** the orientation quaternion is normalized

### Requirement: System SHALL support multiple end-effectors
The system SHALL compute FK for any link in the kinematic chain.

#### Scenario: Compute FK to intermediate link
**Given** a UR5e robot model  
**When** the user requests FK to link "forearm_link" instead of tool  
**Then** the system returns the transformation from base to forearm_link  
**And** the result excludes downstream joints

### Requirement: System SHALL provide efficient FK computation
The system SHALL compute FK efficiently by caching static transformations.

#### Scenario: Multiple FK calls with same robot
**Given** a ForwardKinematics object initialized with a robot  
**When** the user calls compute() 1000 times with different joint angles  
**Then** the average computation time is < 1ms per call  
**And** no dynamic memory allocation occurs during compute()

### Requirement: System SHALL return pose in multiple formats
The system SHALL provide FK results in various transformation representations.

#### Scenario: Get FK as 4x4 matrix
**Given** a computed FK result  
**When** the user calls `result.asMatrix()`  
**Then** the system returns an Eigen::Matrix4d homogeneous transformation

#### Scenario: Get FK as position and quaternion
**Given** a computed FK result  
**When** the user calls `result.asPositionQuaternion()`  
**Then** the system returns a std::pair<Eigen::Vector3d, Eigen::Quaterniond>  
**And** the quaternion is normalized

#### Scenario: Get FK as position and Euler angles
**Given** a computed FK result  
**When** the user calls `result.asPositionEuler()`  
**Then** the system returns position and roll-pitch-yaw Euler angles  
**And** the angles are in radians

### Requirement: System SHALL validate joint angle bounds
The system SHALL optionally validate joint angles against limits during FK computation.

#### Scenario: FK with joints within limits
**Given** a robot with joint limits  
**And** joint angles within the limits  
**When** FK is computed with bounds checking enabled  
**Then** the computation succeeds normally

#### Scenario: FK with joints exceeding limits
**Given** a robot with joint limits [-π, π]  
**And** joint angles including a value of 2π  
**When** FK is computed with strict bounds checking  
**Then** the system throws a `JointLimitException`  
**And** the exception identifies which joint violated limits

#### Scenario: FK without bounds checking
**Given** joint angles exceeding limits  
**When** FK is computed with bounds checking disabled  
**Then** the computation proceeds and returns a result  
**And** no exception is thrown

### Requirement: System SHALL provide thread-safe FK computation
The system SHALL allow concurrent FK computations on the same robot model.

#### Scenario: Parallel FK calls from multiple threads
**Given** a single ForwardKinematics object  
**When** 10 threads call compute() simultaneously with different joint angles  
**Then** all calls complete successfully  
**And** each result corresponds to its input joint angles  
**And** no race conditions occur
