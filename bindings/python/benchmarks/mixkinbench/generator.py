import random
import numpy as np
from typing import List, Tuple, Optional, Dict
import xml.etree.ElementTree as ET
from xml.dom import minidom

class MixedChainGenerator:
    """
    Generates synthetic serial kinematic chains with mixed joint types (revolute/prismatic).
    Outputs URDF format compatible with urdfx.
    """

    def __init__(
        self,
        dof: int,
        prismatic_prob: float = 0.2,
        seed: Optional[int] = None,
        link_length_range: Tuple[float, float] = (0.1, 0.5),
    ):
        """
        Initialize the generator.

        Args:
            dof: Number of degrees of freedom (joints).
            prismatic_prob: Probability of a joint being prismatic (0.0 to 1.0).
            seed: Random seed for reproducibility.
            link_length_range: Min and max length of links in meters.
        """
        self.dof = dof
        self.prismatic_prob = prismatic_prob
        self.rng = random.Random(seed)
        self.link_length_range = link_length_range
        
        # Generate robot immediately to ensure consistent state
        self._generate_robot()

    def _generate_robot(self):
        """Internal method to generate robot parameters."""
        self.joint_types = []
        self.joint_axes = []
        self.joint_limits = []
        self.link_lengths = []
        self.link_rpy = []  # Roll-Pitch-Yaw for link visuals/collisions

        for i in range(self.dof):
            # Determine joint type
            is_prismatic = self.rng.random() < self.prismatic_prob
            joint_type = "prismatic" if is_prismatic else "revolute"
            self.joint_types.append(joint_type)

            # Determine axis
            # Simple randomization: choose one of principal axes or random vector
            # Spec suggests: alternating X/Y/Z to ensure spatial complexity
            axis_choice = self.rng.choice([(1, 0, 0), (0, 1, 0), (0, 0, 1)])
            self.joint_axes.append(axis_choice)

            # Determine limits
            if is_prismatic:
                # Spec: -0.2 to 0.5 meters
                lower = -0.2
                upper = 0.5
            else:
                # Spec: -pi to pi radians
                lower = -np.pi
                upper = np.pi
            self.joint_limits.append((lower, upper))

            # Determine link length (distance to next joint)
            length = self.rng.uniform(*self.link_length_range)
            self.link_lengths.append(length)
            
            # Random orientation for visual variety
            self.link_rpy.append((0, 0, 0))

    def to_urdf_string(self) -> str:
        """
        Generate URDF XML string for the robot.
        """
        robot_name = f"mixed_chain_{self.dof}dof"
        robot = ET.Element("robot", name=robot_name)

        # Create base link
        base_link_name = "base_link"
        base_link = ET.SubElement(robot, "link", name=base_link_name)
        self._add_visual_collision(base_link, length=0.1, radius=0.1, color="0.8 0.8 0.8 1")

        parent_link = base_link_name

        for i in range(self.dof):
            child_link_name = f"link_{i+1}"
            joint_name = f"joint_{i+1}"
            
            # Create Link
            link = ET.SubElement(robot, "link", name=child_link_name)
            # Visuals - simple cylinder along Z or X
            length = self.link_lengths[i]
            self._add_visual_collision(link, length=length, radius=0.05, color=self._random_color())

            # Create Joint
            joint = ET.SubElement(robot, "joint", name=joint_name, type=self.joint_types[i])
            ET.SubElement(joint, "parent", link=parent_link)
            ET.SubElement(joint, "child", link=child_link_name)
            
            # Origin: We stack links along Z for simplicity in base pose, 
            # or we could rotate them. 
            # To make it a "chain", let's say the joint is at the end of the previous link.
            # For simplicity, let's assume standard DH-like structure or just translation along Z of previous link.
            # Here we place joint at (0, 0, prev_length) relative to parent
            prev_length = 0.0 if i == 0 else self.link_lengths[i-1]
            # Actually, for the first joint, it can be at 0,0,0 of base. 
            # But let's give base some height.
            xyz = f"0 0 {prev_length}" if i > 0 else "0 0 0.1" 
            ET.SubElement(joint, "origin", xyz=xyz, rpy="0 0 0")
            
            # Axis
            axis = self.joint_axes[i]
            ET.SubElement(joint, "axis", xyz=f"{axis[0]} {axis[1]} {axis[2]}")
            
            # Limits
            lower, upper = self.joint_limits[i]
            ET.SubElement(joint, "limit", lower=str(lower), upper=str(upper), effort="100.0", velocity="1.0")

            parent_link = child_link_name

        # Pretty print
        xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
        return xmlstr

    def save_urdf(self, filepath: str):
        """Save URDF to a file."""
        with open(filepath, "w") as f:
            f.write(self.to_urdf_string())

    def _add_visual_collision(self, link_elem, length, radius, color):
        """Helper to add visual and collision elements to a link."""
        # Visual
        visual = ET.SubElement(link_elem, "visual")
        # Shift cylinder center so it starts at origin and extends along Z
        ET.SubElement(visual, "origin", xyz=f"0 0 {length/2}", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "cylinder", length=str(length), radius=str(radius))
        material = ET.SubElement(visual, "material", name=f"mat_{id(link_elem)}")
        ET.SubElement(material, "color", rgba=color)

        # Collision (same as visual)
        collision = ET.SubElement(link_elem, "collision")
        ET.SubElement(collision, "origin", xyz=f"0 0 {length/2}", rpy="0 0 0")
        geometry_col = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry_col, "cylinder", length=str(length), radius=str(radius))

    def _random_color(self):
        r = self.rng.random()
        g = self.rng.random()
        b = self.rng.random()
        return f"{r:.2f} {g:.2f} {b:.2f} 1.0"

    def get_statistics(self) -> Dict:
        """Return statistics about the generated robot."""
        return {
            "total_dof": self.dof,
            "num_revolute": self.joint_types.count("revolute"),
            "num_prismatic": self.joint_types.count("prismatic"),
            "total_chain_length": sum(self.link_lengths)
        }

