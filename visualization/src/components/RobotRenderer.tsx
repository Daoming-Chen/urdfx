import { useEffect, useRef, useState } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import * as THREE from 'three';
import type { ParsedURDF } from '../types';
import { URDFSceneBuilder } from '../utils/sceneBuilder';

interface RobotSceneProps {
  urdf: ParsedURDF;
  jointAngles: number[];
  jointNames: string[];
}

function RobotModel({ urdf, jointAngles, jointNames }: RobotSceneProps) {
  const [robotGroup, setRobotGroup] = useState<THREE.Group | null>(null);
  const sceneBuilderRef = useRef<URDFSceneBuilder>(new URDFSceneBuilder());
  const { scene } = useThree();

  useEffect(() => {
    const loadRobot = async () => {
      console.log('[RobotRenderer] Starting to load robot, urdf:', urdf);
      try {
        const group = await sceneBuilderRef.current.buildScene(urdf);
        console.log('[RobotRenderer] Robot scene built successfully, group children:', group.children.length);
        
        // Compute bounding box
        const bbox = new THREE.Box3().setFromObject(group);
        console.log('[RobotRenderer] Robot bounding box:', {
          min: { x: bbox.min.x.toFixed(3), y: bbox.min.y.toFixed(3), z: bbox.min.z.toFixed(3) },
          max: { x: bbox.max.x.toFixed(3), y: bbox.max.y.toFixed(3), z: bbox.max.z.toFixed(3) },
          size: {
            x: (bbox.max.x - bbox.min.x).toFixed(3),
            y: (bbox.max.y - bbox.min.y).toFixed(3),
            z: (bbox.max.z - bbox.min.z).toFixed(3),
          }
        });
        
        setRobotGroup(group);
        scene.add(group);
        console.log('[RobotRenderer] Robot group added to scene');
      } catch (error) {
        console.error('[RobotRenderer] Failed to build robot scene:', error);
      }
    };

    loadRobot();

    return () => {
      if (robotGroup) {
        console.log('[RobotRenderer] Removing robot group from scene');
        scene.remove(robotGroup);
      }
    };
  }, [urdf, scene]);

  useEffect(() => {
    if (robotGroup && jointNames.length > 0) {
      const angleMap = new Map<string, number>();
      jointNames.forEach((name, index) => {
        angleMap.set(name, jointAngles[index] || 0);
      });
      sceneBuilderRef.current.updateJointAngles(robotGroup, angleMap);
    }
  }, [robotGroup, jointAngles, jointNames]);

  return null;
}

interface EndEffectorFrameProps {
  position: [number, number, number];
  quaternion: [number, number, number, number];
}

function EndEffectorFrame({ position, quaternion }: EndEffectorFrameProps) {
  const meshRef = useRef<THREE.Group>(null);

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.position.set(position[0], position[1], position[2]);
      meshRef.current.quaternion.set(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    }
  });

  return (
    <group ref={meshRef}>
      <primitive object={new THREE.AxesHelper(0.2)} />
      <mesh>
        <sphereGeometry args={[0.02, 16, 16]} />
        <meshBasicMaterial color="red" />
      </mesh>
    </group>
  );
}

interface DraggableTargetProps {
  position: [number, number, number];
  quaternion: [number, number, number, number];
  onPositionChange: (position: [number, number, number]) => void;
  enabled: boolean;
}

function DraggableTarget({ position, quaternion, onPositionChange, enabled }: DraggableTargetProps) {
  const meshRef = useRef<THREE.Group>(null);
  const [isDragging, setIsDragging] = useState(false);
  const { camera, gl } = useThree();
  const planeRef = useRef(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0));
  const intersectionPoint = useRef(new THREE.Vector3());

  useFrame(() => {
    if (meshRef.current && !isDragging) {
      meshRef.current.position.set(position[0], position[1], position[2]);
      meshRef.current.quaternion.set(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    }
  });

  const handlePointerDown = (e: any) => {
    if (!enabled) return;
    e.stopPropagation();
    setIsDragging(true);
    (e.target as any).setPointerCapture(e.pointerId);
  };

  const handlePointerMove = (e: any) => {
    if (!isDragging || !enabled) return;
    e.stopPropagation();

    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2(
      (e.pointer.x / gl.domElement.clientWidth) * 2 - 1,
      -(e.pointer.y / gl.domElement.clientHeight) * 2 + 1
    );
    
    raycaster.setFromCamera(mouse, camera);
    
    // Update plane to face camera
    const cameraDirection = new THREE.Vector3();
    camera.getWorldDirection(cameraDirection);
    planeRef.current.setFromNormalAndCoplanarPoint(
      cameraDirection.negate(),
      new THREE.Vector3(position[0], position[1], position[2])
    );

    if (raycaster.ray.intersectPlane(planeRef.current, intersectionPoint.current)) {
      const newPosition: [number, number, number] = [
        intersectionPoint.current.x,
        intersectionPoint.current.y,
        intersectionPoint.current.z
      ];
      onPositionChange(newPosition);
      
      if (meshRef.current) {
        meshRef.current.position.set(newPosition[0], newPosition[1], newPosition[2]);
      }
    }
  };

  const handlePointerUp = (e: any) => {
    if (!enabled) return;
    e.stopPropagation();
    setIsDragging(false);
    (e.target as any).releasePointerCapture(e.pointerId);
  };

  return (
    <group ref={meshRef}>
      <primitive object={new THREE.AxesHelper(0.2)} />
      <mesh
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerMissed={() => setIsDragging(false)}
      >
        <sphereGeometry args={[0.03, 32, 32]} />
        <meshStandardMaterial 
          color={enabled ? (isDragging ? "#ff4444" : "#ff0000") : "#666666"}
          emissive={enabled ? (isDragging ? "#ff4444" : "#880000") : "#000000"}
          emissiveIntensity={isDragging ? 0.5 : 0.2}
        />
      </mesh>
      {enabled && (
        <mesh>
          <sphereGeometry args={[0.04, 32, 32]} />
          <meshBasicMaterial color="#ff0000" wireframe opacity={0.3} transparent />
        </mesh>
      )}
    </group>
  );
}

interface RobotRendererProps {
  urdf: ParsedURDF | null;
  jointAngles: number[];
  jointNames: string[];
  endEffectorPose?: { position: [number, number, number]; quaternion: [number, number, number, number] } | null;
  showEndEffector?: boolean;
  ikMode?: boolean;
  ikTarget?: { position: [number, number, number]; quaternion: [number, number, number, number] } | null;
  onIKTargetChange?: (position: [number, number, number]) => void;
}

export function RobotRenderer({
  urdf,
  jointAngles,
  jointNames,
  endEffectorPose,
  showEndEffector = true,
  ikMode = false,
  ikTarget,
  onIKTargetChange,
}: RobotRendererProps) {
  return (
    <div style={{ width: '100%', height: '100%' }}>
      <Canvas
        camera={{ position: [2, 2, 2], fov: 50 }}
        gl={{ antialias: true }}
        shadows
      >
        {/* Lighting */}
        <ambientLight intensity={0.5} />
        <directionalLight
          position={[5, 5, 5]}
          intensity={0.8}
          castShadow
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
        />
        <pointLight position={[-5, 5, -5]} intensity={0.3} />

        {/* Grid and axes */}
        <Grid args={[10, 10]} cellColor="#6f6f6f" sectionColor="#9d4b4b" />
        <primitive object={new THREE.AxesHelper(1)} />

        {/* Robot model */}
        {urdf && (
          <RobotModel
            urdf={urdf}
            jointAngles={jointAngles}
            jointNames={jointNames}
          />
        )}

        {/* End effector frame (FK mode or reference in IK mode) */}
        {showEndEffector && endEffectorPose && !ikMode && (
          <EndEffectorFrame
            position={endEffectorPose.position}
            quaternion={endEffectorPose.quaternion}
          />
        )}

        {/* IK Target (draggable in IK mode) */}
        {ikMode && ikTarget && onIKTargetChange && (
          <DraggableTarget
            position={ikTarget.position}
            quaternion={ikTarget.quaternion}
            onPositionChange={onIKTargetChange}
            enabled={true}
          />
        )}

        {/* Camera controls */}
        <OrbitControls
          enableDamping
          dampingFactor={0.05}
          minDistance={0.5}
          maxDistance={10}
        />
      </Canvas>
    </div>
  );
}
