const fs = require('node:fs');
const path = require('node:path');

const modulePath = process.env.URDFX_WASM_BUNDLE;
const wasmPath = process.env.URDFX_WASM_BINARY;
const hasArtifacts = Boolean(modulePath && wasmPath);

const TEST_URDF = `<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual name="base_visual">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision name="base_collision">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.5" length="1"/>
      </geometry>
    </collision>
  </link>
  <link name="child_link">
     <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    <dynamics damping="0.1" friction="0.1"/>
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

describeIf('URDFX WASM Full Bindings (Node)', () => {
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

  test('Robot traversal', () => {
    const robot = module.Robot.fromURDFString(TEST_URDF);
    expect(robot.getName()).toBe('test_robot');
    expect(robot.getRootLink()).toBe('base_link');

    const links = vectorToArray(robot.getLinks());
    expect(links.length).toBe(2);
    expect(links.map(l => l.getName()).sort()).toEqual(['base_link', 'child_link']);

    const joints = vectorToArray(robot.getJoints());
    expect(joints.length).toBe(1);
    expect(joints[0].getName()).toBe('joint1');

    const baseLink = robot.getLink('base_link');
    expect(baseLink).toBeDefined();
    expect(baseLink.getName()).toBe('base_link');

    const joint1 = robot.getJoint('joint1');
    expect(joint1).toBeDefined();
    expect(joint1.getName()).toBe('joint1');

    robot.dispose();
  });

  test('Link properties', () => {
    const robot = module.Robot.fromURDFString(TEST_URDF);
    const baseLink = robot.getLink('base_link');

    // Inertial
    const inertial = baseLink.getInertial();
    expect(inertial).toBeDefined();
    expect(inertial.mass).toBe(1.0);
    const inertia = inertial.getInertia();
    expect(inertia.rows).toBe(3);
    expect(inertia.cols).toBe(3);
    // Identity matrix check
    const inertiaData = vectorToArray(inertia.data);
    expect(inertiaData[0]).toBe(1);
    expect(inertiaData[4]).toBe(1);
    expect(inertiaData[8]).toBe(1);

    // Visuals
    const visuals = vectorToArray(baseLink.getVisuals());
    expect(visuals.length).toBe(1);
    const visual = visuals[0];
    expect(visual.name).toBe('base_visual');
    expect(visual.geometry.type).toBe(module.GeometryType.Box);
    expect(vectorToArray(visual.geometry.box_size)).toEqual([1, 1, 1]);
    expect(vectorToArray(visual.getColor())).toEqual([0, 0, 1, 1]);
    expect(visual.getMaterialName()).toBe('blue');

    // Collisions
    const collisions = vectorToArray(baseLink.getCollisions());
    expect(collisions.length).toBe(1);
    const collision = collisions[0];
    expect(collision.name).toBe('base_collision');
    expect(collision.geometry.type).toBe(module.GeometryType.Cylinder);
    expect(collision.geometry.cylinder_radius).toBe(0.5);
    expect(collision.geometry.cylinder_length).toBe(1.0);

    robot.dispose();
  });

  test('Joint properties', () => {
    const robot = module.Robot.fromURDFString(TEST_URDF);
    const joint = robot.getJoint('joint1');

    expect(joint.getType()).toBe(module.JointType.Revolute);
    expect(joint.getParentLink()).toBe('base_link');
    expect(joint.getChildLink()).toBe('child_link');
    
    const axis = vectorToArray(joint.getAxis());
    expect(axis).toEqual([0, 0, 1]);

    const limits = joint.getLimits();
    expect(limits).toBeDefined();
    expect(limits.lower).toBe(-1.57);
    expect(limits.upper).toBe(1.57);
    expect(limits.effort).toBe(10);
    expect(limits.velocity).toBe(1);

    const dynamics = joint.getDynamics();
    expect(dynamics).toBeDefined();
    expect(dynamics.damping).toBe(0.1);
    expect(dynamics.friction).toBe(0.1);

    const origin = joint.getOrigin();
    const translation = vectorToArray(origin.translation());
    expect(translation).toEqual([0, 0, 1]);

    robot.dispose();
  });

  test('Transform class', () => {
    const t = new module.Transform();
    const translation = vectorToArray(t.translation());
    expect(translation).toEqual([0, 0, 0]);

    const t2 = module.Transform.fromPositionQuaternion([1, 2, 3], [1, 0, 0, 0]);
    const t2Trans = vectorToArray(t2.translation());
    expect(t2Trans).toEqual([1, 2, 3]);
    
    const pose = t2.asPose();
    expect(vectorToArray(pose.position)).toEqual([1, 2, 3]);
    expect(vectorToArray(pose.quaternion)).toEqual([1, 0, 0, 0]);

    const matrix = t2.asMatrix();
    expect(matrix.rows).toBe(4);
    expect(matrix.cols).toBe(4);
  });
});
