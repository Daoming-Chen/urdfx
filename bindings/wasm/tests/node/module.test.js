const fs = require('node:fs');
const path = require('node:path');

const modulePath = process.env.URDFX_WASM_BUNDLE;
const wasmPath = process.env.URDFX_WASM_BINARY;
const hasArtifacts = Boolean(modulePath && wasmPath);

const SIMPLE_URDF = `<?xml version="1.0"?>
<robot name="two_link">
  <link name="base" />
  <link name="link_1" />
  <joint name="joint_1" type="revolute">
    <parent link="base" />
    <child link="link_1" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10" />
  </joint>
</robot>`;

const describeIf = hasArtifacts ? describe : describe.skip;

function vectorToArray(vec) {
  if (!vec) {
    return [];
  }
  if (Array.isArray(vec)) {
    return vec;
  }
  if (typeof vec.length === 'number') {
    return Array.from(vec);
  }
  if (typeof vec.size === 'function' && typeof vec.get === 'function') {
    const length = vec.size();
    return Array.from({ length }, (_, index) => vec.get(index));
  }
  return Array.from(vec);
}

describeIf('URDFX WASM module (Node)', () => {
  let module;

  beforeAll(async () => {
    const wasmStat = await fs.promises.stat(wasmPath);
    if (!wasmStat.isFile()) {
      throw new Error('URDFX_WASM_BINARY must point to the generated urdfx.wasm file.');
    }

    const resolvedModulePath = path.resolve(modulePath);
    const createModule = require(resolvedModulePath);
    module = await createModule({
      locateFile(file) {
        if (file.endsWith('.wasm')) {
          return path.resolve(wasmPath);
        }
        return path.resolve(path.dirname(resolvedModulePath), file);
      },
    });
  }, 60000);

  afterAll(() => {
    module = undefined;
  });

  test('creates robot from URDF string', () => {
    const robot = module.Robot.fromURDFString(SIMPLE_URDF);
    expect(robot.getName()).toBe('two_link');
    expect(vectorToArray(robot.getJointNames())).toEqual(['joint_1']);
    expect(robot.getDOF()).toBe(1);
    robot.dispose();
  });

  test('computes forward kinematics at zero angles', () => {
    const robot = module.Robot.fromURDFString(SIMPLE_URDF);
    const fk = new module.ForwardKinematics(robot, 'link_1', 'base');
    const pose = fk.compute([0.0]);
    const position = vectorToArray(pose.position);
    expect(position.length).toBe(3);
    expect(position[0]).toBeCloseTo(0.0, 6);
    expect(position[1]).toBeCloseTo(0.0, 6);
    expect(position[2]).toBeCloseTo(0.1, 6);
    fk.dispose();
    robot.dispose();
  });

  test('computes Jacobian and solves IK', () => {
    const robot = module.Robot.fromURDFString(SIMPLE_URDF);
    const jac = new module.JacobianCalculator(robot, 'link_1', 'base');
    const matrix = jac.compute([0.0], module.JacobianType.Analytic);
    expect(matrix.rows).toBe(6);
    expect(matrix.cols).toBe(1);
    expect(vectorToArray(matrix.data).every((value) => Number.isFinite(value))).toBe(true);

    const solver = new module.SQPIKSolver(robot, 'link_1', 'base');
    const result = solver.solve(
      {
        position: [0.0, 0.0, 0.1],
        quaternion: [1.0, 0.0, 0.0, 0.0],
      },
      [0.0],
    );

    expect(result.converged).toBe(true);
    expect(result.iterations).toBeGreaterThanOrEqual(0);
    const solution = vectorToArray(result.solution);
    expect(solution.length).toBe(1);
    expect(solution[0]).toBeCloseTo(0.0, 4);

    solver.dispose();
    jac.dispose();
    robot.dispose();
  });
});

describeIf('URDFX WASM artifacts availability', () => {
  test('requires URDFX_WASM_BUNDLE and URDFX_WASM_BINARY environment variables', () => {
    expect(hasArtifacts).toBe(true);
  });
});
