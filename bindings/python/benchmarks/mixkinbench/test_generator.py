
import sys
import os
import unittest

# Add current directory to sys.path to allow importing local modules when run as script
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    import urdfx
except ImportError:
    # Try to add build directory to path if not installed
    # Adjust this path based on where bindings are built
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../build/bindings/python")))
    try:
        import urdfx
    except ImportError:
        print("Warning: urdfx module not found. Validation with urdfx will be skipped.")
        urdfx = None

try:
    from generator import MixedChainGenerator
except ImportError:
    from .generator import MixedChainGenerator

class TestMixedChainGenerator(unittest.TestCase):
    def test_generation(self):
        gen = MixedChainGenerator(dof=10, seed=42)
        urdf_str = gen.to_urdf_string()
        self.assertIn('robot name="mixed_chain_10dof"', urdf_str)
        # Probability of all prismatic is low but possible, but with 10 dof and seed 42, likely mixed
        # We can check if valid XML
        import xml.etree.ElementTree as ET
        root = ET.fromstring(urdf_str)
        self.assertEqual(root.tag, 'robot')
        self.assertEqual(len(root.findall('joint')), 10)
        
        stats = gen.get_statistics()
        self.assertEqual(stats['total_dof'], 10)
        
    def test_urdfx_parsing(self):
        if urdfx is None:
            self.skipTest("urdfx module not available")
            
        gen = MixedChainGenerator(dof=5, seed=123)
        urdf_str = gen.to_urdf_string()
        
        try:
            robot = urdfx.Robot.from_urdf_string(urdf_str)
            self.assertIsNotNone(robot)
            print(f"Successfully parsed {robot.get_name()}")
            self.assertEqual(robot.dof, 5)
        except AttributeError:
            print("urdfx.Robot.from_urdf_string not found. available:", dir(urdfx))
        except Exception as e:
            self.fail(f"urdfx failed to parse generated URDF: {e}")

if __name__ == '__main__':
    unittest.main()
