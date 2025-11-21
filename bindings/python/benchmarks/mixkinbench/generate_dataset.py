import os
import argparse
import sys

# Add current directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
    
# Add bindings build path
build_path = os.path.abspath(os.path.join(current_dir, "../../../../../build/bindings/python"))
if build_path not in sys.path:
    sys.path.append(build_path)

try:
    import urdfx
except ImportError:
    print(f"Error: urdfx module not found in {build_path}. Please build Python bindings first.")
    sys.exit(1)

from generator import MixedChainGenerator
from oracle import FKOracle, JointSampler
from dataset import BenchmarkDataset

def generate_tier_b(output_dir: str):
    """Generate Tier B synthetic robot datasets."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Configs: (DOF, sample_count), here DOF from 8 to 20, all with 1000 samples
    configs = [(dof, 1000) for dof in range(8, 21)]
    
    for dof, samples in configs:
        print(f"Generating {dof}-DOF dataset with {samples} samples...")
        
        # 1. Generate Robot
        # Use fixed seed for reproducibility of the robot structure
        gen = MixedChainGenerator(dof=dof, prismatic_prob=0.3, seed=42+dof)
        urdf_str = gen.to_urdf_string()
        
        # Save URDF
        urdf_filename = f"mixed_{dof}dof.urdf"
        urdf_path = os.path.join(output_dir, urdf_filename)
        gen.save_urdf(urdf_path)
        print(f"  Saved robot to {urdf_path}")
        
        # 2. Load Robot into urdfx
        robot = urdfx.Robot.from_urdf_string(urdf_str)
        
        # 3. Setup Oracle and Sampler
        oracle = FKOracle(robot)
        sampler = JointSampler(robot)
        
        # 4. Generate Dataset with variations
        dataset_name = f"tier_b_{dof}dof"
        dataset = BenchmarkDataset(dataset_name)
        dataset.generate_variations(oracle, sampler, count=samples)
        
        # 5. Save Dataset
        dataset_path = os.path.join(output_dir, f"{dataset_name}.npz")
        dataset.save_npz(dataset_path)
        bin_path = os.path.join(output_dir, f"{dataset_name}.bin")
        dataset.save_binary(bin_path)
        print(f"  Saved dataset to {dataset_path} and {bin_path} ({len(dataset.cases)} cases)")

def main():
    parser = argparse.ArgumentParser(description="Generate MixKinBench datasets")
    parser.add_argument("--output", "-o", default="benchmarks/datasets", help="Output directory")
    args = parser.parse_args()
    
    generate_tier_b(args.output)

if __name__ == "__main__":
    main()

