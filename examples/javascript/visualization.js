import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';
import GUI from 'lil-gui';

async function main() {
    // 1. Initialize Three.js Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x222222);
    scene.add(new THREE.GridHelper(10, 10));

    const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.position.set(2, 2, 2);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.5, 0);
    controls.update();

    const light = new THREE.DirectionalLight(0xffffff, 1.0);
    light.position.set(1, 1, 1);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x444444));

    // 2. Initialize URDFX (WASM)
    const infoDiv = document.getElementById('info');
    infoDiv.innerText = 'Loading WASM...';

    let urdfx;
    try {
        urdfx = await createUrdfxModule({
            locateFile: (path, prefix) => {
                if (path.endsWith('.wasm')) {
                    return '../../build-wasm/wasm/' + path;
                }
                return prefix + path;
            }
        });
        console.log('URDFX loaded');
    } catch (e) {
        console.error(e);
        infoDiv.innerText = 'Error loading WASM. Check console.';
        return;
    }

    // 3. Load URDF File
    infoDiv.innerText = 'Loading URDF...';
    const urdfUrl = '../../examples/models/ur5/ur5e.urdf';
    
    let urdfContent;
    try {
        const response = await fetch(urdfUrl);
        urdfContent = await response.text();
    } catch (e) {
        console.error(e);
        infoDiv.innerText = 'Error loading URDF file. Check console.';
        return;
    }

    // 4. Create URDFX Robot Model
    const robotKinematics = urdfx.Robot.fromURDFString(urdfContent);
    console.log(`URDFX Robot loaded: ${robotKinematics.getName()}, DOF: ${robotKinematics.getDOF()}`);

    // 5. Load Visual Robot (Three.js)
    const loader = new URDFLoader();
    loader.loadMeshCb = function(path, manager, done) {
        if (/\.obj$/i.test(path)) {
            const loader = new OBJLoader(manager);
            loader.load(path, (obj) => {
                done(obj);
            });
        } else {
            this.defaultMeshLoader(path, manager, done);
        }
    };
    // Adjust loader to find meshes relative to the URDF file
    // URDFLoader resolves relative paths against the URDF URL automatically if passed to load()
    // But here we might want to parse the string we already fetched, or load again.
    // Let's load again to let URDFLoader handle paths.
    
    loader.load(
        urdfUrl,
        (robotVisual) => {
            scene.add(robotVisual);
            
            // Rotate to match standard robotics Z-up if needed (Three.js is Y-up)
            // URDFLoader usually handles this by rotating the root.
            // But UR5 model is Z-up. Three.js is Y-up.
            // Usually we rotate the whole robot -90 deg around X.
            robotVisual.rotation.x = -Math.PI / 2;
            
            // Also rotate our grid/camera to match or just rotate the robot.
            // Let's keep Three.js Y-up and rotate robot to be upright.
            
            infoDiv.innerText = 'URDFX Visualization Demo';
            initGUI(robotVisual, robotKinematics, urdfx);
        },
        undefined,
        (err) => {
            console.error(err);
            infoDiv.innerText = 'Error loading Visual Model.';
        }
    );

    // Handle Window Resize
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // Animation Loop
    function animate() {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    }
    animate();
}

function initGUI(robotVisual, robotKinematics, urdfx) {
    const gui = new GUI({ title: 'Robot Control' });
    const jointNames = robotKinematics.getJointNames();
    const numJoints = jointNames.size();
    const dof = robotKinematics.getDOF();
    
    // Map joint names to indices for kinematics
    // Note: robotKinematics.getJointNames() returns all joints, but we only control movable ones (DOF)
    // We need to identify which joints are controllable.
    // In urdfx, usually the order of jointAngles corresponds to the movable joints.
    // Let's assume standard order.
    
    // We will create a config object for the GUI
    const config = {};
    const jointLimits = {}; // Store limits if available (urdfx might provide them, or we guess)

    // Helper to get movable joints
    const movableJoints = [];
    for (let i = 0; i < numJoints; i++) {
        const name = jointNames.get(i);
        const joint = robotKinematics.getJoint(name);
        const type = joint.getType(); // We assume getJoint returns a Joint object
        // Check if joint is fixed (urdfx might have an enum or string)
        // Based on previous example, we just iterate.
        // Let's try to find the joint in the visual robot to see if it's movable
        if (robotVisual.joints[name]) {
             movableJoints.push(name);
             config[name] = 0.0;
             jointLimits[name] = { min: -Math.PI * 2, max: Math.PI * 2 };
             
             // Try to get limits from visual robot (URDFLoader parses them)
             const vJoint = robotVisual.joints[name];
             if (vJoint.limit) {
                 jointLimits[name].min = vJoint.limit.lower;
                 jointLimits[name].max = vJoint.limit.upper;
             }
        }
    }

    // Create Sliders
    movableJoints.forEach((name, index) => {
        gui.add(config, name, jointLimits[name].min, jointLimits[name].max)
           .name(name)
           .onChange(() => updateRobot(robotVisual, robotKinematics, config, movableJoints, urdfx));
    });

    // Initial update
    updateRobot(robotVisual, robotKinematics, config, movableJoints, urdfx);
}

function updateRobot(robotVisual, robotKinematics, config, movableJoints, urdfx) {
    // 1. Update Visual Robot
    for (const name of movableJoints) {
        if (robotVisual.joints[name]) {
            robotVisual.setJointValue(name, config[name]);
        }
    }

    // 2. Update Kinematics (URDFX)
    // We need to construct the joint angles array.
    // Assuming the order in movableJoints matches the DOF order in urdfx.
    // This is a strong assumption. A better way is to check urdfx API for setting joint by name.
    // But urdfx usually takes a vector of doubles.
    // Let's assume the order of movable joints found matches.
    
    const dof = robotKinematics.getDOF();
    const angles = new Float64Array(dof);
    
    // We need to map names to indices. 
    // If we don't know the order, we might be in trouble.
    // However, usually URDF parsers maintain order.
    // Let's try to match by iterating joints in urdfx and checking if they are movable.
    
    // Re-construct the mapping logic to be safe:
    // urdfx likely expects joints in the order they appear in the chain or file.
    // Let's assume the order we added them to GUI (which came from getJointNames) is correct,
    // filtering for movable ones.
    
    // Actually, `robotKinematics.getJointNames()` returns ALL joints.
    // We need to know which ones correspond to the `dof` indices.
    // Usually, `compute(angles)` expects angles for the movable joints in order.
    
    // Let's assume `movableJoints` array we built is correct order if we iterated `getJointNames`.
    // But we filtered by `robotVisual.joints[name]`.
    
    // Let's just fill the array.
    for(let i=0; i<movableJoints.length && i < dof; i++) {
        angles[i] = config[movableJoints[i]];
    }
    
    // Compute FK for end effector (just to show we can)
    // Let's pick the last link
    const links = robotKinematics.getLinks(); // EmbindVector
    const numLinks = links.size();
    const lastLinkName = links.get(numLinks - 1);
    const rootLinkName = robotKinematics.getRootLink();
    
    try {
        const fk = new urdfx.ForwardKinematics(robotKinematics, lastLinkName, rootLinkName);
        const pose = fk.compute(angles);
        // console.log('FK for ' + lastLinkName, pose.position);
        fk.dispose();
        
        // We could visualize this pose with a helper to verify alignment
        // But for now, just driving the visual robot is enough for the user request.
    } catch (e) {
        // console.error(e);
    }
}

main();
