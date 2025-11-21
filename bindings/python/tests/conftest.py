import pytest
import os
import urdfx

@pytest.fixture
def test_data_dir():
    # Assuming we run tests from bindings/python or root
    # Try to find examples/models/ur5/ur5.urdf
    possible_paths = [
        "../../../examples/models/ur5",  # relative to bindings/python/tests
        "examples/models/ur5",           # relative to root
    ]
    
    for path in possible_paths:
        if os.path.exists(os.path.join(path, "ur5.urdf")):
            return os.path.abspath(path)
            
    # If not found, rely on environment variable or fail
    if "URDFX_MODELS_DIR" in os.environ:
        return os.environ["URDFX_MODELS_DIR"]
        
    pytest.skip("UR5 model not found. Please set URDFX_MODELS_DIR env var.")

@pytest.fixture
def ur5_urdf_path(test_data_dir):
    return os.path.join(test_data_dir, "ur5.urdf")

@pytest.fixture
def ur5_robot(ur5_urdf_path):
    return urdfx.Robot.from_urdf_file(ur5_urdf_path)

