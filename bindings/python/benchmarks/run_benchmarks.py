import subprocess
import sys
import os

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    benchmarks = [
        "bench_forward_kinematics.py",
        "bench_jacobian.py",
        "bench_inverse_kinematics.py"
    ]
    
    print("Running all benchmarks...")
    print("-" * 50)
    
    for bench in benchmarks:
        print(f"Benchmark: {bench}")
        bench_path = os.path.join(script_dir, bench)
        try:
            subprocess.run([sys.executable, bench_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running {bench}: {e}")
        print("-" * 50)

if __name__ == "__main__":
    main()

