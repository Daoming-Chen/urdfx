const fs = require('fs');
const path = require('path');

// Path to the built WASM module
// Try to find it in the build directory
const BUILD_WASM_PATH = path.join(__dirname, '../../build-wasm/wasm/urdfx.js');
const LOCAL_WASM_PATH = path.join(__dirname, './urdfx.js');

let createUrdfxModule;
let wasmPath;

if (fs.existsSync(BUILD_WASM_PATH)) {
    console.log(`Using WASM module from build directory: ${BUILD_WASM_PATH}`);
    createUrdfxModule = require(BUILD_WASM_PATH);
    wasmPath = path.dirname(BUILD_WASM_PATH);
} else if (fs.existsSync(LOCAL_WASM_PATH)) {
    console.log(`Using local WASM module: ${LOCAL_WASM_PATH}`);
    createUrdfxModule = require(LOCAL_WASM_PATH);
    wasmPath = __dirname;
} else {
    console.error('Could not find urdfx.js. Please build the WASM module first.');
    console.error('Expected at:', BUILD_WASM_PATH);
    // For development, we might want to continue if the user plans to put it there
    // but for running it will fail.
    process.exit(1);
}

async function main() {
    try {
        // Initialize the module
        // We need to help it find the .wasm file if it's not in the current working directory
        const urdfx = await createUrdfxModule({
            locateFile: (path, prefix) => {
                if (path.endsWith('.wasm')) {
                    return require('path').join(wasmPath, path);
                }
                return prefix + path;
            }
        });
        console.log('urdfx module loaded successfully');

        // Load URDF
        const urdfPath = path.join(__dirname, '../../examples/models/ur5/ur5e.urdf');
        if (!fs.existsSync(urdfPath)) {
            console.error(`URDF file not found at ${urdfPath}`);
            return;
        }
        const urdfContent = fs.readFileSync(urdfPath, 'utf8');

        // Create Robot from URDF string
        const robot = urdfx.Robot.fromURDFString(urdfContent);
        console.log(`Robot loaded: ${robot.getName()}`);
        console.log(`DOF: ${robot.getDOF()}`);

        // Print joint names
        const jointNames = robot.getJointNames();
        console.log('Joints:');
        // Handle EmbindVector or Array
        const numJoints = jointNames.size ? jointNames.size() : jointNames.length;
        for (let i = 0; i < numJoints; i++) {
            const name = jointNames.get ? jointNames.get(i) : jointNames[i];
            console.log(`  - ${name}`);
        }

        // Get all links
        const links = robot.getLinks();
        const numLinks = links.size();
        const baseLinkName = robot.getRootLink();

        // Define joint angles (e.g., all zeros)
        const dof = robot.getDOF();
        const jointAngles = new Float64Array(dof).fill(0);

        // Set some non-zero angles for demonstration
        if (dof >= 6) {
            jointAngles[1] = -1.57; // Shoulder lift
            jointAngles[2] = 1.57;  // Elbow
        }

        console.log('Joint Angles:', jointAngles);

        // 获取所有驱动关节对应的 child links
        // 这些是真正在运动链中会移动的 links
        const movableLinks = [];
        for (let i = 0; i < numJoints; i++) {
            const jointName = jointNames.get ? jointNames.get(i) : jointNames[i];
            const joint = robot.getJoint(jointName);
            const childLinkName = joint.getChildLink();
            if (!movableLinks.includes(childLinkName)) {
                movableLinks.push(childLinkName);
            }
        }

        // 遍历可移动的 links，计算并输出其位姿
        console.log('\nLink Poses (movable links only):');
        for (const linkName of movableLinks) {
            try {
                // 创建 FK solver: base -> 当前 link
                const fk = new urdfx.ForwardKinematics(robot, linkName, baseLinkName);
                const chainDOF = fk.getNumJoints();
                // 只传入这条运动链需要的关节角度
                const chainJointAngles = jointAngles.slice(0, chainDOF);
                const pose = fk.compute(chainJointAngles);
                console.log(`- ${linkName} (chain DOF: ${chainDOF})`);
                console.log(`    Position:   [${pose.position.map(n => n.toFixed(4)).join(', ')}]`);
                console.log(`    Quaternion: [${pose.quaternion.map(n => n.toFixed(4)).join(', ')}]`);
                fk.dispose();
            } catch (err) {
                // 正确显示错误信息
                const errorMessage = err.message || err.toString() || String(err);
                console.log(`- ${linkName} (error: ${errorMessage})`);
            }
        }

        // Clean up
        robot.dispose();

    } catch (error) {
        console.error('Error running example:', error);
    }
}

main();
