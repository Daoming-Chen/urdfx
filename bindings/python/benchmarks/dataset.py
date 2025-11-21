import numpy as np
import json
import struct
from typing import List, Dict, Optional, Union
import os
from dataclasses import dataclass

@dataclass
class BenchmarkCase:
    target_pos: List[float]
    target_rot: List[List[float]] # 3x3 matrix
    initial_guess: List[float]
    ground_truth_q: List[float]
    robot_name: str
    dof: int
    metadata: Dict

class BenchmarkDataset:
    """
    Manages generation, storage, and loading of IK benchmark datasets.
    """
    def __init__(self, name: str):
        self.name = name
        self.cases: List[BenchmarkCase] = []

    def add_case(self, case: BenchmarkCase):
        self.cases.append(case)

    def save_npz(self, filepath: str):
        """Save dataset to compressed NPZ format."""
        # Convert to numpy arrays for efficient storage
        target_pos = np.array([c.target_pos for c in self.cases])
        target_rot = np.array([c.target_rot for c in self.cases])
        initial_guess = np.array([c.initial_guess for c in self.cases])
        ground_truth_q = np.array([c.ground_truth_q for c in self.cases])
        
        # Metadata as JSON string
        meta = {
            "name": self.name,
            "robot_name": self.cases[0].robot_name if self.cases else "",
            "dof": self.cases[0].dof if self.cases else 0,
            "count": len(self.cases),
            "case_metadata": [c.metadata for c in self.cases]
        }
        
        np.savez_compressed(
            filepath,
            target_pos=target_pos,
            target_rot=target_rot,
            initial_guess=initial_guess,
            ground_truth_q=ground_truth_q,
            metadata=json.dumps(meta)
        )

    def save_binary(self, filepath: str):
        """
        Save dataset to a simple custom binary format for C++ consumption.
        Format:
        - Magic "MKBN" (4 bytes)
        - Version (int32)
        - NumCases (int32)
        - DOF (int32)
        - Data block:
          For each case:
            - TargetPos (3 doubles)
            - TargetRot (9 doubles, row-major)
            - InitialGuess (DOF doubles)
            - GroundTruth (DOF doubles)
        """
        if not self.cases:
            return

        dof = self.cases[0].dof
        num_cases = len(self.cases)
        
        with open(filepath, 'wb') as f:
            f.write(b'MKBN') # Magic
            f.write(struct.pack('<i', 1)) # Version 1
            f.write(struct.pack('<i', num_cases))
            f.write(struct.pack('<i', dof))
            
            for case in self.cases:
                # Target Pos (3)
                f.write(struct.pack('<3d', *case.target_pos))
                # Target Rot (9) - Flatten 3x3
                rot_flat = [val for row in case.target_rot for val in row]
                f.write(struct.pack('<9d', *rot_flat))
                # Initial Guess (DOF)
                f.write(struct.pack(f'<{dof}d', *case.initial_guess))
                # Ground Truth (DOF)
                f.write(struct.pack(f'<{dof}d', *case.ground_truth_q))

    @staticmethod
    def load_npz(filepath: str) -> 'BenchmarkDataset':
        data = np.load(filepath)
        metadata = json.loads(str(data['metadata']))
        
        dataset = BenchmarkDataset(metadata['name'])
        
        target_pos = data['target_pos']
        target_rot = data['target_rot']
        initial_guess = data['initial_guess']
        ground_truth_q = data['ground_truth_q']
        case_metas = metadata.get("case_metadata", [{} for _ in range(len(target_pos))])
        
        robot_name = metadata.get("robot_name", "")
        dof = metadata.get("dof", 0)
        
        for i in range(len(target_pos)):
            dataset.add_case(BenchmarkCase(
                target_pos=target_pos[i].tolist(),
                target_rot=target_rot[i].tolist(),
                initial_guess=initial_guess[i].tolist(),
                ground_truth_q=ground_truth_q[i].tolist(),
                robot_name=robot_name,
                dof=dof,
                metadata=case_metas[i]
            ))
            
        return dataset

    def generate(self, 
                 oracle: 'FKOracle', 
                 sampler: 'JointSampler', 
                 count: int, 
                 strategy: str = "reachable"):
        """
        Generate dataset using the provided oracle and sampler.
        """
        q_samples = sampler.sample(count)
        
        for i in range(count):
            q_gt = q_samples[i]
            pos, rot = oracle.compute_pose(q_gt)
            
            # Cold start (zeros)
            q_init = np.zeros_like(q_gt)
            
            self.add_case(BenchmarkCase(
                target_pos=pos.tolist(),
                target_rot=rot.tolist(),
                initial_guess=q_init.tolist(),
                ground_truth_q=q_gt.tolist(),
                robot_name=oracle.robot.get_name(),
                dof=oracle.dof,
                metadata={"type": strategy, "index": i}
            ))

    def generate_variations(self, 
                          oracle: 'FKOracle', 
                          sampler: 'JointSampler', 
                          count: int):
        """
        Generate dataset with variations (cold/warm starts).
        """
        q_samples = sampler.sample(count)
        rng = np.random.RandomState(42)
        
        for i in range(count):
            q_gt = q_samples[i]
            pos, rot = oracle.compute_pose(q_gt)
            
            # 1. Cold Start (Zeros)
            self.add_case(BenchmarkCase(
                target_pos=pos.tolist(),
                target_rot=rot.tolist(),
                initial_guess=np.zeros_like(q_gt).tolist(),
                ground_truth_q=q_gt.tolist(),
                robot_name=oracle.robot.get_name(),
                dof=oracle.dof,
                metadata={"type": "cold_start_zero", "index": i}
            ))
            
            # 2. Cold Start (Random)
            q_random = sampler.sample(1)[0]
            self.add_case(BenchmarkCase(
                target_pos=pos.tolist(),
                target_rot=rot.tolist(),
                initial_guess=q_random.tolist(),
                ground_truth_q=q_gt.tolist(),
                robot_name=oracle.robot.get_name(),
                dof=oracle.dof,
                metadata={"type": "cold_start_random", "index": i}
            ))
            
            # 3. Warm Start (Noisy GT)
            noise = rng.normal(0, 0.1, size=q_gt.shape)
            q_warm = q_gt + noise
            self.add_case(BenchmarkCase(
                target_pos=pos.tolist(),
                target_rot=rot.tolist(),
                initial_guess=q_warm.tolist(),
                ground_truth_q=q_gt.tolist(),
                robot_name=oracle.robot.get_name(),
                dof=oracle.dof,
                metadata={"type": "warm_start", "index": i}
            ))

    def generate_unreachable(self, 
                           oracle: 'FKOracle', 
                           sampler: 'JointSampler', 
                           count: int,
                           scale_factor: float = 1.5):
        """
        Generate dataset with unreachable targets by scaling reachable positions.
        """
        q_samples = sampler.sample(count)
        
        for i in range(count):
            q_gt = q_samples[i]
            pos, rot = oracle.compute_pose(q_gt)
            
            # Make unreachable by scaling the position vector from origin
            # This assumes the base is at (0,0,0). If not, we should subtract base pos first.
            # For benchmarks, base is usually at identity.
            
            pos_vec = np.array(pos)
            dist = np.linalg.norm(pos_vec)
            
            if dist < 1e-6:
                # If at origin, move to a random direction with large distance
                # Estimate max reach? Just use a large number like 10.0 meters
                pos_vec = np.array([10.0, 0.0, 0.0])
            else:
                pos_vec = pos_vec * scale_factor
                
            # Initial guess: random or zero
            q_init = np.zeros_like(q_gt)
            
            self.add_case(BenchmarkCase(
                target_pos=pos_vec.tolist(),
                target_rot=rot.tolist(),
                initial_guess=q_init.tolist(),
                ground_truth_q=q_gt.tolist(), # Note: this is NOT the solution for the target, just the seed
                robot_name=oracle.robot.get_name(),
                dof=oracle.dof,
                metadata={"type": "unreachable", "scale": scale_factor, "index": i}
            ))
