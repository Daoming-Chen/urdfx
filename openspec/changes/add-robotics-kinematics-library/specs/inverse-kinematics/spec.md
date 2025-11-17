# Capability: Inverse Kinematics

## ADDED Requirements

### Requirement: System SHALL solve IK using SQP algorithm with DaQP
The system SHALL solve inverse kinematics using Sequential Quadratic Programming with joint limit constraints.

#### Scenario: Solve IK for reachable pose
**Given** a UR5e robot model  
**And** a target pose within the robot's workspace  
**When** the user calls `ikSolver.solve(targetPose, initialGuess)`  
**Then** the system returns a solution q within 10 iterations  
**And** FK(q) is within 1mm position error and 0.01 rad orientation error of target  
**And** all joint angles respect joint limits

#### Scenario: Solve IK with warm start
**Given** a sequence of target poses along a trajectory  
**When** the user solves IK for each pose using the previous solution as initial guess  
**Then** each IK solve converges faster than cold start  
**And** the trajectory is smooth (no large joint jumps)

### Requirement: System SHALL respect joint limits as constraints
The system SHALL enforce joint limits as hard constraints during IK solving.

#### Scenario: IK solution respects joint limits
**Given** a UR5e robot with joint limits [-π, π]  
**And** a target pose that would require joint angles exceeding limits without constraints  
**When** IK is solved with joint limit enforcement  
**Then** the solution has all joint angles within [-π, π]  
**And** the solution is the best reachable pose within limits

#### Scenario: Unreachable pose due to joint limits
**Given** a target pose only reachable with joints outside limits  
**When** IK is solved  
**Then** the solver returns failure status after max iterations  
**And** the best effort solution is returned  
**And** the error residual is reported

### Requirement: System SHALL handle multiple solution candidates
The system SHALL support providing different initial guesses to find alternative IK solutions.

#### Scenario: Find elbow-up vs elbow-down solutions
**Given** a UR5e robot and a target pose with multiple IK solutions  
**When** the user solves IK with different initial guesses (elbow-up and elbow-down)  
**Then** the solver converges to different valid solutions  
**And** both solutions satisfy FK(q) ≈ target

### Requirement: System SHALL configure solver parameters
The system SHALL allow configuration of solver parameters for different use cases.

#### Scenario: Set convergence tolerance
**Given** an IKSolver instance  
**When** the user sets position tolerance to 0.001m and orientation tolerance to 0.001 rad  
**Then** the solver only returns success if errors are below these thresholds

#### Scenario: Set maximum iterations
**Given** an IKSolver instance  
**When** the user sets max iterations to 50  
**Then** the solver terminates after 50 iterations if not converged  
**And** returns the best solution found

#### Scenario: Configure step size limits
**Given** an IKSolver instance  
**When** the user sets max joint step to 0.1 rad  
**Then** each SQP iteration limits Δq to 0.1 rad per joint  
**And** this prevents large jumps for numerical stability

### Requirement: System SHALL report solver diagnostics
The system SHALL provide detailed diagnostics about the solving process.

#### Scenario: Query solver status after solving
**Given** a completed IK solve  
**When** the user queries the solver status  
**Then** the status includes convergence flag, iterations taken, final error, and constraint violations  
**And** the status indicates if max iterations was reached

#### Scenario: Access convergence history
**Given** an IK solve with history logging enabled  
**When** the user queries the convergence history  
**Then** the system returns error norms and joint angles for each iteration  
**And** this data can be used for debugging or visualization  
**And** iteration details are logged using spdlog at DEBUG level

### Requirement: System SHALL handle unreachable poses gracefully
The system SHALL handle targets outside the workspace without crashing.

#### Scenario: Target beyond reach
**Given** a target pose 10 meters away from robot base  
**When** IK is solved  
**Then** the solver returns failure status  
**And** the best effort solution points toward the target  
**And** no exceptions are thrown

### Requirement: System SHALL support position-only and orientation-only IK
The system SHALL allow solving IK for position or orientation independently.

#### Scenario: Solve IK for position only
**Given** a target position without orientation constraint  
**When** the user calls `ikSolver.solvePosition(targetPosition)`  
**Then** the solver finds q such that FK(q).position ≈ targetPosition  
**And** the orientation is free (not constrained)

#### Scenario: Solve IK for orientation only
**Given** a target orientation without position constraint  
**When** the user calls `ikSolver.solveOrientation(targetOrientation)`  
**Then** the solver finds q such that FK(q).orientation ≈ targetOrientation  
**And** the position is free (not constrained)

### Requirement: System SHALL optimize for smoothness in trajectory IK
The system SHALL minimize joint velocity when solving sequential IK problems.

#### Scenario: Trajectory IK with smoothness penalty
**Given** a sequence of target poses  
**When** IK is solved with a smoothness weight parameter  
**Then** the joint trajectory minimizes ||q_i - q_{i-1}||  
**And** the trajectory has no unnecessary oscillations

### Requirement: System SHALL provide thread-safe IK solving
The system SHALL support concurrent IK solving from multiple threads.

#### Scenario: Parallel IK solving
**Given** multiple IKSolver instances (one per thread)  
**When** 10 threads solve IK simultaneously for different targets  
**Then** all solves complete successfully  
**And** results are independent and correct  
**And** no data races occur
