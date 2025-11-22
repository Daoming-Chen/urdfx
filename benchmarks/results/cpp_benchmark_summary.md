# C++ Benchmark Summary

**System Information:**
- Host: Asus4090
- CPUs: 24
- CPU MHz: 3187
- Build Type: release

## IK Solver Benchmarks (UR5e Robot)

| Scenario | Time (µs) | Iterations | Success Rate | Pos Error (mm) | Rot Error (deg) |
|----------|-----------|------------|--------------|----------------|------------------|
| ColdStart | 73.57 | 9.5 | 100% | 0.0227 | 0.0021 |
| WarmStart | 36.30 | 5.0 | 100% | 0.1139 | 0.0017 |
| Trajectory | 59.15 | 2.4 | 100% | 0.0860 | 0.0051 |

## Jacobian Computation Benchmarks

| Benchmark | Time (µs) | Throughput (calls/sec) |
|-----------|-----------|------------------------|
| BM_Jacobian | 0.2370 | 4219454 |

## Key Findings

### IK Solver Performance
- **Warm start** provides ~2x speedup over cold start
- **Trajectory optimization** mode reduces iterations significantly (2.4 vs 5-9 iterations)
- All scenarios achieve 100% success rate with sub-millimeter accuracy

### Jacobian Computation
- Extremely fast: < 0.25 µs per computation
- Can handle >4 million Jacobian computations per second
- Suitable for real-time control applications

