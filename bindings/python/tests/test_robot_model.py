import pytest
import urdfx
import numpy as np

def test_robot_creation():
    robot = urdfx.Robot("test_robot")
    assert robot.get_name() == "test_robot"
    assert len(robot.get_links()) == 0
    assert len(robot.get_joints()) == 0

def test_urdf_parsing(ur5_robot):
    assert ur5_robot.get_name() == "ur5"
    assert len(ur5_robot.get_links()) > 0
    assert len(ur5_robot.get_joints()) > 0
    
    # Check for specific links
    link_names = [l.get_name() for l in ur5_robot.get_links()]
    assert "base_link" in link_names
    assert "tool0" in link_names
    
    # Check DOF
    assert ur5_robot.dof == 6
    
    # Check limits
    limits = ur5_robot.get_joint_limits()
    assert limits.shape == (6, 2)

def test_urdf_parsing_string():
    urdf_str = """
    <robot name="test_robot">
        <link name="base_link"/>
        <link name="child_link"/>
        <joint name="joint1" type="fixed">
            <parent link="base_link"/>
            <child link="child_link"/>
        </joint>
    </robot>
    """
    robot = urdfx.Robot.from_urdf_string(urdf_str)
    assert robot.get_name() == "test_robot"
    assert len(robot.get_links()) == 2
    assert len(robot.get_joints()) == 1

