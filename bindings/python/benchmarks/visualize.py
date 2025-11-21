import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def load_benchmark_results(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data.get('benchmarks', [])

def extract_metrics(benchmarks):
    """Extract metrics by benchmark type and DOF."""
    results = {
        'BM_MixedChainIK': {},
        'BM_MixedChainIK_ColdStart': {},
        'BM_MixedChainIK_WarmStart': {}
    }
    
    for bm in benchmarks:
        name = bm['name']
        parts = name.split('/')
        
        if len(parts) < 3:
            continue
        
        bench_type_prefix = parts[0]  # BM_MixedChainIK, BM_MixedChainIK_ColdStart, etc.
        dof = int(parts[2])
        
        # Map the benchmark type to our result keys
        if bench_type_prefix not in results:
            continue
        
        # For each DOF, aggregate or average across different types
        if dof not in results[bench_type_prefix]:
            results[bench_type_prefix][dof] = {
                'time_ms': bm.get('real_time', 0),
                'success_rate': bm.get('success_rate', 0),
                'avg_iter': bm.get('avg_iter', 0),
                'avg_pos_err_mm': bm.get('avg_pos_err_mm', 0),
                'avg_rot_err_deg': bm.get('avg_rot_err_deg', 0),
                'avg_rev_err_deg': bm.get('avg_rev_err_deg', 0),
                'avg_pris_err_mm': bm.get('avg_pris_err_mm', 0),
                'count': 1
            }
        else:
            # Average with existing data
            existing = results[bench_type_prefix][dof]
            count = existing['count']
            existing['time_ms'] = (existing['time_ms'] * count + bm.get('real_time', 0)) / (count + 1)
            existing['success_rate'] = (existing['success_rate'] * count + bm.get('success_rate', 0)) / (count + 1)
            existing['avg_iter'] = (existing['avg_iter'] * count + bm.get('avg_iter', 0)) / (count + 1)
            existing['avg_pos_err_mm'] = (existing['avg_pos_err_mm'] * count + bm.get('avg_pos_err_mm', 0)) / (count + 1)
            existing['avg_rot_err_deg'] = (existing['avg_rot_err_deg'] * count + bm.get('avg_rot_err_deg', 0)) / (count + 1)
            existing['avg_rev_err_deg'] = (existing['avg_rev_err_deg'] * count + bm.get('avg_rev_err_deg', 0)) / (count + 1)
            existing['avg_pris_err_mm'] = (existing['avg_pris_err_mm'] * count + bm.get('avg_pris_err_mm', 0)) / (count + 1)
            existing['count'] = count + 1
    
    # Remove the count field from final results
    for bench_type in results:
        for dof in results[bench_type]:
            if 'count' in results[bench_type][dof]:
                del results[bench_type][dof]['count']
    
    return results

def plot_results(results, output_dir):
    """Generate comprehensive visualization plots."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Mixed-Chain IK Benchmark Results (8-20 DOF)', fontsize=16, fontweight='bold')
    
    colors = {
        'BM_MixedChainIK': '#1f77b4',
        'BM_MixedChainIK_ColdStart': '#ff7f0e',
        'BM_MixedChainIK_WarmStart': '#2ca02c'
    }
    
    labels = {
        'BM_MixedChainIK': 'Mixed (Cold+Warm)',
        'BM_MixedChainIK_ColdStart': 'Cold Start',
        'BM_MixedChainIK_WarmStart': 'Warm Start'
    }
    
    # Plot 1: Execution Time vs DOF
    ax = axes[0, 0]
    for bench_type, data in results.items():
        if not data:
            continue
        dofs = sorted(data.keys())
        times = [data[d]['time_ms'] for d in dofs]
        ax.plot(dofs, times, 'o-', label=labels[bench_type], color=colors[bench_type], linewidth=2, markersize=6)
    
    ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
    ax.set_ylabel('Time (ms)', fontsize=12, fontweight='bold')
    ax.set_title('Execution Time vs DOF', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xticks(range(8, 21))
    
    # Plot 2: Success Rate vs DOF
    ax = axes[0, 1]
    for bench_type, data in results.items():
        if not data:
            continue
        dofs = sorted(data.keys())
        success = [data[d]['success_rate'] for d in dofs]
        ax.plot(dofs, success, 'o-', label=labels[bench_type], color=colors[bench_type], linewidth=2, markersize=6)
    
    ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
    ax.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
    ax.set_title('Success Rate vs DOF', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 105])
    ax.set_xticks(range(8, 21))
    
    # Plot 3: Average Iterations vs DOF
    ax = axes[0, 2]
    for bench_type, data in results.items():
        if not data:
            continue
        dofs = sorted(data.keys())
        iters = [data[d]['avg_iter'] for d in dofs]
        ax.plot(dofs, iters, 'o-', label=labels[bench_type], color=colors[bench_type], linewidth=2, markersize=6)
    
    ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
    ax.set_ylabel('Average Iterations', fontsize=12, fontweight='bold')
    ax.set_title('Convergence Iterations vs DOF', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xticks(range(8, 21))
    
    # Plot 4: Position Error vs DOF
    ax = axes[1, 0]
    for bench_type, data in results.items():
        if not data:
            continue
        dofs = sorted(data.keys())
        errors = [data[d]['avg_pos_err_mm'] for d in dofs if data[d]['avg_pos_err_mm'] > 0]
        valid_dofs = [d for d in dofs if data[d]['avg_pos_err_mm'] > 0]
        if valid_dofs:
            ax.plot(valid_dofs, errors, 'o-', label=labels[bench_type], color=colors[bench_type], linewidth=2, markersize=6)
    
    ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
    ax.set_ylabel('Position Error (mm)', fontsize=12, fontweight='bold')
    ax.set_title('Position Error vs DOF', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xticks(range(8, 21))
    
    # Plot 5: Rotation Error vs DOF
    ax = axes[1, 1]
    for bench_type, data in results.items():
        if not data:
            continue
        dofs = sorted(data.keys())
        errors = [data[d]['avg_rot_err_deg'] for d in dofs if data[d]['avg_rot_err_deg'] > 0]
        valid_dofs = [d for d in dofs if data[d]['avg_rot_err_deg'] > 0]
        if valid_dofs:
            ax.plot(valid_dofs, errors, 'o-', label=labels[bench_type], color=colors[bench_type], linewidth=2, markersize=6)
    
    ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
    ax.set_ylabel('Rotation Error (deg)', fontsize=12, fontweight='bold')
    ax.set_title('Rotation Error vs DOF', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xticks(range(8, 21))
    
    # Plot 6: Joint Type Sensitivity (Revolute vs Prismatic)
    ax = axes[1, 2]
    bench_type = 'BM_MixedChainIK'
    if bench_type in results and results[bench_type]:
        data = results[bench_type]
        dofs = sorted(data.keys())
        rev_errors = [data[d]['avg_rev_err_deg'] for d in dofs if data[d]['avg_rev_err_deg'] > 0]
        pris_errors = [data[d]['avg_pris_err_mm'] for d in dofs if data[d]['avg_pris_err_mm'] > 0]
        valid_dofs_rev = [d for d in dofs if data[d]['avg_rev_err_deg'] > 0]
        valid_dofs_pris = [d for d in dofs if data[d]['avg_pris_err_mm'] > 0]
        
        ax2 = ax.twinx()
        if valid_dofs_rev:
            line1 = ax.plot(valid_dofs_rev, rev_errors, 'o-', label='Revolute Error', color='#d62728', linewidth=2, markersize=6)
        if valid_dofs_pris:
            line2 = ax2.plot(valid_dofs_pris, pris_errors, 's-', label='Prismatic Error', color='#9467bd', linewidth=2, markersize=6)
        
        ax.set_xlabel('Degrees of Freedom', fontsize=12, fontweight='bold')
        ax.set_ylabel('Revolute Error (deg)', fontsize=12, fontweight='bold', color='#d62728')
        ax2.set_ylabel('Prismatic Error (mm)', fontsize=12, fontweight='bold', color='#9467bd')
        ax.set_title('Joint Type Sensitivity', fontsize=13, fontweight='bold')
        ax.tick_params(axis='y', labelcolor='#d62728')
        ax2.tick_params(axis='y', labelcolor='#9467bd')
        
        # Combine legends
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
        
        ax.grid(True, alpha=0.3)
        ax.set_xticks(range(8, 21))
    
    plt.tight_layout()
    
    # Save figure
    output_path = output_dir / 'mixkinbench_visualization.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    
    # Also save as PDF
    output_path_pdf = output_dir / 'mixkinbench_visualization.pdf'
    plt.savefig(output_path_pdf, bbox_inches='tight')
    print(f"PDF saved to: {output_path_pdf}")
    
    plt.show()

def generate_summary_table(results, output_dir):
    """Generate a summary table in markdown format."""
    output_dir = Path(output_dir)
    output_path = output_dir / 'benchmark_summary.md'
    
    with open(output_path, 'w') as f:
        f.write("# Mixed-Chain IK Benchmark Summary (8-20 DOF)\n\n")
        
        for bench_type, data in results.items():
            if not data:
                continue
                
            f.write(f"## {bench_type}\n\n")
            f.write("| DOF | Time (ms) | Success Rate (%) | Avg Iterations | Pos Error (mm) | Rot Error (deg) |\n")
            f.write("|-----|-----------|------------------|----------------|----------------|------------------|\n")
            
            for dof in sorted(data.keys()):
                metrics = data[dof]
                f.write(f"| {dof} | {metrics['time_ms']:.3f} | {metrics['success_rate']:.1f} | "
                       f"{metrics['avg_iter']:.1f} | {metrics['avg_pos_err_mm']:.4f} | "
                       f"{metrics['avg_rot_err_deg']:.2f} |\n")
            
            f.write("\n")
    
    print(f"Summary table saved to: {output_path}")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python visualize.py <benchmark_results.json> [output_dir]")
        sys.exit(1)
    
    json_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "benchmarks/results"
    
    benchmarks = load_benchmark_results(json_path)
    results = extract_metrics(benchmarks)
    
    plot_results(results, output_dir)
    generate_summary_table(results, output_dir)
