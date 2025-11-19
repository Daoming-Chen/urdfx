const http = require('node:http');
const path = require('node:path');
const fs = require('node:fs');

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

describeIf('URDFX WASM module (Browser)', () => {
  let puppeteer;
  let browser;
  let server;
  let serverUrl;

  const resolvedModulePath = modulePath ? path.resolve(modulePath) : '';
  const resolvedWasmPath = wasmPath ? path.resolve(wasmPath) : '';

  beforeAll(async () => {
    puppeteer = require('puppeteer');

    server = http.createServer((req, res) => {
      const { url } = req;
      if (url === '/urdfx.js') {
        res.writeHead(200, {
          'Content-Type': 'application/javascript',
          'Cache-Control': 'no-store',
        });
        fs.createReadStream(resolvedModulePath).pipe(res);
        return;
      }

      if (url === '/urdfx.wasm') {
        res.writeHead(200, {
          'Content-Type': 'application/wasm',
          'Cache-Control': 'no-store',
        });
        fs.createReadStream(resolvedWasmPath).pipe(res);
        return;
      }

      if (url === '/test.html' || url === '/') {
        res.writeHead(200, {
          'Content-Type': 'text/html; charset=utf-8',
          'Cache-Control': 'no-store',
        });
        res.end(`<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<title>urdfx wasm browser test</title>
<script src="/urdfx.js"></script>
</head>
<body>
<script>
const SAMPLE_URDF = ${JSON.stringify(SIMPLE_URDF)};
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
    const data = [];
    for (let i = 0; i < length; i += 1) {
      data.push(vec.get(i));
    }
    return data;
  }
  return Array.from(vec);
}
window.runUrdfxSmokeTest = async function runUrdfxSmokeTest() {
  const module = await createUrdfxModule({
    locateFile(file) {
      if (file.endsWith('.wasm')) {
        return '/urdfx.wasm';
      }
      return file;
    },
  });
  const robot = module.Robot.fromURDFString(SAMPLE_URDF);
  const dof = robot.getDOF();
  const name = robot.getName();
  const fk = new module.ForwardKinematics(robot, 'link_1', 'base');
  const pose = fk.compute([0.0]);
  const posePosition = vectorToArray(pose.position);

  const fkStart = performance.now();
  for (let i = 0; i < 64; i += 1) {
    fk.compute([0.01 * i]);
  }
  const fkDuration = performance.now() - fkStart;

  const solver = new module.SQPIKSolver(robot, 'link_1', 'base');
  const ikStart = performance.now();
  const ikResult = solver.solve(
    { position: [0.0, 0.0, 0.1], quaternion: [1.0, 0.0, 0.0, 0.0] },
    [0.0],
  );
  const ikDuration = performance.now() - ikStart;
  const solution = vectorToArray(ikResult.solution);

  solver.dispose();
  fk.dispose();
  robot.dispose();

  return {
    robotName: name,
    dof,
    posePosition,
    fkDuration,
    ikDuration,
    ikConverged: Boolean(ikResult.converged),
    ikSolution: solution,
  };
};
</script>
</body>
</html>`);
        return;
      }

      res.writeHead(404, { 'Content-Type': 'text/plain' });
      res.end('Not found');
    });

    await new Promise((resolve) => {
      server.listen(0, '127.0.0.1', () => {
        const address = server.address();
        serverUrl = `http://127.0.0.1:${address.port}`;
        resolve();
      });
    });

    browser = await puppeteer.launch({
      headless: 'new',
      args: ['--no-sandbox', '--disable-setuid-sandbox'],
    });
  }, 60000);

  afterAll(async () => {
    if (browser) {
      await browser.close();
    }
    if (server) {
      await new Promise((resolve) => server.close(resolve));
    }
  });

  test('loads module and runs kinematics in browser', async () => {
    const page = await browser.newPage();
    await page.goto(`${serverUrl}/test.html`, { waitUntil: 'networkidle0' });
    const result = await page.evaluate(() => window.runUrdfxSmokeTest());
    expect(result.robotName).toBe('two_link');
    expect(result.dof).toBe(1);
    expect(Array.isArray(result.posePosition)).toBe(true);
    expect(result.posePosition[2]).toBeCloseTo(0.1, 6);
    expect(result.ikConverged).toBe(true);
    expect(Array.isArray(result.ikSolution)).toBe(true);
    expect(result.ikSolution.length).toBe(1);
    expect(result.ikSolution[0]).toBeCloseTo(0.0, 4);
    expect(result.fkDuration).toBeLessThan(25);
    expect(result.ikDuration).toBeLessThan(25);
    await page.close();
  });
});

describeIf('URDFX WASM artifacts availability (browser)', () => {
  test('requires URDFX_WASM_BUNDLE and URDFX_WASM_BINARY environment variables', () => {
    expect(hasArtifacts).toBe(true);
  });
});
