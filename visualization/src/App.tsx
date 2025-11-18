import { useState, useEffect, useCallback, useRef } from 'react';
import { RobotRenderer } from './components/RobotRenderer';
import { JointControlPanel } from './components/JointControlPanel';
import { ControlModeSelector } from './components/ControlModeSelector';
import { PoseDisplay } from './components/PoseDisplay';
import { IKControlPanel } from './components/IKControlPanel';
import { loadUrdfxModule } from './utils/wasmLoader';
import { URDFParser } from './utils/urdfParser';
import { embindVectorToArray } from './utils/embind';
import type { AppState, ControlMode, JointInfo, ParsedURDF } from './types';
import type { ForwardKinematics, SQPIKSolver, UrdfxModule } from '../public/urdfx';
import './App.css';

function App() {
  const [state, setState] = useState<AppState>({
    mode: 'FK',
    urdfContent: null,
    parsedURDF: null,
    robotState: {
      robot: null,
      jointAngles: [],
      jointInfo: [],
      endEffectorPose: null,
    },
    isLoading: true,
    error: null,
  });

  const [ikState, setIKState] = useState<{
    targetPose: { position: [number, number, number]; quaternion: [number, number, number, number] } | null;
    solutionFound?: boolean;
  }>({
    targetPose: null,
    solutionFound: undefined,
  });

  const moduleRef = useRef<UrdfxModule | null>(null);
  const fkSolverRef = useRef<ForwardKinematics | null>(null);
  const ikSolverRef = useRef<SQPIKSolver | null>(null);
  const urdfParserRef = useRef<URDFParser>(new URDFParser());

  // Load WASM module on mount
  useEffect(() => {
    const loadModule = async () => {
      try {
        const module = await loadUrdfxModule();
        moduleRef.current = module;
        console.log('WASM module loaded successfully');
        
        // Load default URDF (UR5e)
        await loadDefaultURDF();
      } catch (error) {
        console.error('Failed to load WASM module:', error);
        setState(prev => ({
          ...prev,
          error: `Failed to load WASM module: ${error}`,
          isLoading: false,
        }));
      }
    };

    loadModule();

    return () => {
      // Cleanup
      if (fkSolverRef.current) fkSolverRef.current.dispose();
      if (ikSolverRef.current) ikSolverRef.current.dispose();
      if (state.robotState.robot) state.robotState.robot.dispose();
    };
  }, []);

  const loadDefaultURDF = async () => {
    try {
      const response = await fetch('/ur5e.urdf');
      if (!response.ok) {
        throw new Error('Failed to fetch UR5e URDF');
      }
      const urdfContent = await response.text();
      await loadURDF(urdfContent);
    } catch (error) {
      console.error('Failed to load default URDF:', error);
      setState(prev => ({
        ...prev,
        error: `Failed to load default URDF: ${error}`,
        isLoading: false,
      }));
    }
  };

  const loadURDF = async (urdfContent: string) => {
    if (!moduleRef.current) {
      throw new Error('WASM module not loaded');
    }

    console.log('[App] Starting to load URDF, content length:', urdfContent.length);
    setState(prev => ({ ...prev, isLoading: true, error: null }));

    try {
      // Parse URDF with our parser for visualization
      console.log('[App] Parsing URDF for visualization...');
      const parsedURDF = urdfParserRef.current.parse(urdfContent);
      console.log('[App] URDF parsed, links:', parsedURDF.links.size, 'joints:', parsedURDF.joints.size);

      // Create robot instance in WASM
      console.log('[App] Creating robot instance in WASM...');
      const robot = moduleRef.current.Robot.fromURDFString(urdfContent);
      const jointNames = embindVectorToArray(robot.getJointNames());
      const lowerLimits = embindVectorToArray(robot.getLowerLimits());
      const upperLimits = embindVectorToArray(robot.getUpperLimits());

      // Initialize joint info
      const jointInfo: JointInfo[] = jointNames.map((name, index) => ({
        name,
        index,
        lowerLimit: lowerLimits[index],
        upperLimit: upperLimits[index],
        value: 0, // Start at zero position
      }));

      const initialAngles = new Array(jointNames.length).fill(0);

      // Create FK solver
      const endLink = findEndEffectorLink(parsedURDF);
      const fkSolver = new moduleRef.current.ForwardKinematics(robot, endLink, '');
      fkSolverRef.current = fkSolver;

      // Create IK solver
      const ikSolver = new moduleRef.current.SQPIKSolver(robot, endLink, '');
      ikSolverRef.current = ikSolver;

      // Compute initial pose
      const initialPose = fkSolver.compute(initialAngles);

      setState({
        mode: 'FK',
        urdfContent,
        parsedURDF,
        robotState: {
          robot,
          jointAngles: initialAngles,
          jointInfo,
          endEffectorPose: initialPose,
        },
        isLoading: false,
        error: null,
      });
    } catch (error) {
      console.error('Failed to load URDF:', error);
      setState(prev => ({
        ...prev,
        error: `Failed to load URDF: ${error}`,
        isLoading: false,
      }));
    }
  };

  const findEndEffectorLink = (urdf: ParsedURDF): string => {
    // Find the leaf link (link that is not a parent of any joint)
    const parentLinks = new Set<string>();
    urdf.joints.forEach(joint => parentLinks.add(joint.parent));
    
    for (const linkName of urdf.links.keys()) {
      if (!parentLinks.has(linkName) && linkName !== urdf.rootLink) {
        return linkName;
      }
    }
    
    // Fallback to last link
    return Array.from(urdf.links.keys()).pop() || urdf.rootLink;
  };

  const handleJointChange = useCallback((index: number, value: number) => {
    if (!fkSolverRef.current || state.mode !== 'FK') return;

    const newAngles = [...state.robotState.jointAngles];
    newAngles[index] = value;

    const newJointInfo = [...state.robotState.jointInfo];
    newJointInfo[index] = { ...newJointInfo[index], value };

    try {
      const newPose = fkSolverRef.current.compute(newAngles);

      setState(prev => ({
        ...prev,
        robotState: {
          ...prev.robotState,
          jointAngles: newAngles,
          jointInfo: newJointInfo,
          endEffectorPose: newPose,
        },
      }));
    } catch (error) {
      console.error('FK computation failed:', error);
    }
  }, [state.mode, state.robotState.jointAngles, state.robotState.jointInfo]);

  const handleModeChange = useCallback((mode: ControlMode) => {
    setState(prev => ({ ...prev, mode }));
    
    // Initialize IK target when switching to IK mode
    if (mode === 'IK' && state.robotState.endEffectorPose) {
      setIKState({
        targetPose: {
          position: [...state.robotState.endEffectorPose.position],
          quaternion: [...state.robotState.endEffectorPose.quaternion],
        },
        solutionFound: undefined,
      });
    }
  }, [state.robotState.endEffectorPose]);

  const handleFileUpload = useCallback(async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    try {
      const content = await file.text();
      await loadURDF(content);
    } catch (error) {
      console.error('Failed to load uploaded URDF:', error);
      setState(prev => ({
        ...prev,
        error: `Failed to load uploaded URDF: ${error}`,
      }));
    }
  }, []);

  const handleIKTargetChange = useCallback((pose: { position: [number, number, number]; quaternion: [number, number, number, number] }) => {
    setIKState(prev => ({
      ...prev,
      targetPose: pose,
      solutionFound: undefined,
    }));
  }, []);

  const handleIKTargetPositionChange = useCallback((position: [number, number, number]) => {
    setIKState(prev => ({
      ...prev,
      targetPose: prev.targetPose ? { ...prev.targetPose, position } : null,
      solutionFound: undefined,
    }));
  }, []);

  const handleIKSolve = useCallback(() => {
    if (!ikSolverRef.current || !ikState.targetPose || !fkSolverRef.current) return;

    try {
      const { position, quaternion } = ikState.targetPose;
      
      // Use current joint angles as initial guess
      const initialGuess = state.robotState.jointAngles;
      
      // Solve IK
      const result = ikSolverRef.current.solve(
        position,
        quaternion,
        initialGuess,
        100, // max iterations
        1e-6 // tolerance
      );

      const solution = embindVectorToArray(result);
      
      // Check if solution is valid (within joint limits)
      const isValid = solution.every((angle, i) => {
        const joint = state.robotState.jointInfo[i];
        return angle >= joint.lowerLimit && angle <= joint.upperLimit;
      });

      if (isValid && solution.length === state.robotState.jointAngles.length) {
        // Update joint angles and compute FK to verify
        const newJointInfo = state.robotState.jointInfo.map((joint, i) => ({
          ...joint,
          value: solution[i],
        }));

        const verifyPose = fkSolverRef.current.compute(solution);

        setState(prev => ({
          ...prev,
          robotState: {
            ...prev.robotState,
            jointAngles: solution,
            jointInfo: newJointInfo,
            endEffectorPose: verifyPose,
          },
        }));

        setIKState(prev => ({
          ...prev,
          solutionFound: true,
        }));
      } else {
        setIKState(prev => ({
          ...prev,
          solutionFound: false,
        }));
      }
    } catch (error) {
      console.error('IK solve failed:', error);
      setIKState(prev => ({
        ...prev,
        solutionFound: false,
      }));
    }
  }, [ikState.targetPose, state.robotState.jointAngles, state.robotState.jointInfo]);

  if (state.error) {
    return (
      <div className="app error-state">
        <div className="error-message">
          <h2>Error</h2>
          <p>{state.error}</p>
          <button onClick={() => window.location.reload()}>Reload Page</button>
        </div>
      </div>
    );
  }

  if (state.isLoading) {
    return (
      <div className="app loading-state">
        <div className="loading-message">
          <div className="spinner"></div>
          <p>Loading urdfx...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="app">
      <header className="app-header">
        <h1>urdfx Visualization</h1>
        <div className="file-upload">
          <label htmlFor="urdf-upload" className="upload-button">
            Upload URDF
          </label>
          <input
            id="urdf-upload"
            type="file"
            accept=".urdf"
            onChange={handleFileUpload}
            style={{ display: 'none' }}
          />
        </div>
      </header>

      <div className="app-content">
        <aside className="sidebar">
          <ControlModeSelector
            mode={state.mode}
            onModeChange={handleModeChange}
            disabled={!state.parsedURDF}
          />
          
          <PoseDisplay
            pose={state.robotState.endEffectorPose}
          />
          
          {state.mode === 'FK' ? (
            <JointControlPanel
              joints={state.robotState.jointInfo}
              onJointChange={handleJointChange}
              disabled={!state.parsedURDF}
            />
          ) : (
            ikState.targetPose && (
              <IKControlPanel
                targetPose={ikState.targetPose}
                onTargetChange={handleIKTargetChange}
                onSolve={handleIKSolve}
                disabled={!state.parsedURDF}
                solutionFound={ikState.solutionFound}
              />
            )
          )}
        </aside>

        <main className="viewer">
          <RobotRenderer
            urdf={state.parsedURDF}
            jointAngles={state.robotState.jointAngles}
            jointNames={state.robotState.jointInfo.map(j => j.name)}
            endEffectorPose={state.robotState.endEffectorPose}
            showEndEffector={true}
            ikMode={state.mode === 'IK'}
            ikTarget={ikState.targetPose}
            onIKTargetChange={handleIKTargetPositionChange}
          />
        </main>
      </div>
    </div>
  );
}

export default App;
