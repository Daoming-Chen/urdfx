import numpy as np
from typing import Optional, List, Tuple
import urdfx

class FKOracle:
    """
    Wrapper around urdfx Forward Kinematics for benchmarking.
    """
    def __init__(self, robot: urdfx.Robot, end_link: Optional[str] = None):
        self.robot = robot
        self.end_link = end_link or self._find_leaf_link(robot)
        
        if not self.end_link:
            raise ValueError("Could not determine end link automatically.")
            
        self.fk = urdfx.ForwardKinematics(robot, self.end_link)
        self.dof = robot.dof

    def _find_leaf_link(self, robot: urdfx.Robot) -> str:
        """Find the first leaf link (no child joints)."""
        # Simple traversal or check all links
        # Assuming single chain for now or taking the "last" one
        links = robot.get_links()
        for link in links:
            name = link.get_name()
            children = robot.get_child_joints(name)
            if not children:
                return name
        # If no leaf found (loop?), return last link name
        return links[-1].get_name()

    def compute_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute FK and return (position, rotation_matrix).
        """
        tf = self.fk.compute(q)
        return tf.translation(), tf.rotation()

    def compute_transform(self, q: np.ndarray) -> urdfx.Transform:
        return self.fk.compute(q)

class JointSampler:
    """
    Samples joint configurations within limits.
    """
    def __init__(self, robot: urdfx.Robot, rng: np.random.RandomState = None):
        self.robot = robot
        self.dof = robot.dof
        self.limits = robot.get_joint_limits() # Expected shape (dof, 2) or specialized object
        # urdfx Robot.get_joint_limits return numpy array? Check pyi.
        # pyi says: def get_joint_limits(self) -> np.ndarray: ...
        # Assuming it returns (dof, 2) array of [lower, upper]
        
        self.rng = rng or np.random.RandomState()

    def sample(self, n_samples: int = 1) -> np.ndarray:
        """Sample N random configurations uniformly within limits."""
        lower = self.limits[:, 0]
        upper = self.limits[:, 1]
        
        # Handle unbounded or very large limits if necessary
        # Spec says "generate reasonable limits".
        
        return self.rng.uniform(lower, upper, size=(n_samples, self.dof))

    def sample_gaussian(self, n_samples: int = 1, sigma: float = 0.5) -> np.ndarray:
        """
        Sample N random configurations using Gaussian distribution centered at the middle of the range.
        """
        lower = self.limits[:, 0]
        upper = self.limits[:, 1]
        center = (lower + upper) / 2.0
        range_width = upper - lower
        
        # Scale sigma by range width
        std_dev = range_width * sigma
        
        samples = self.rng.normal(center, std_dev, size=(n_samples, self.dof))
        
        # Clip to limits
        return np.clip(samples, lower, upper)

