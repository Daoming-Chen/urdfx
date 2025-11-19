import { useEffect, useState, useRef, useMemo, forwardRef } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three-stdlib';
import { useControls } from 'leva';
import { TransformControls } from '@react-three/drei';
import * as THREE from 'three';

interface RobotRendererProps {
  urdfx: any;
  urdfUrl?: string;
  urdfContent?: string | null;
}

export function RobotRenderer({ urdfx, urdfUrl, urdfContent }: RobotRendererProps) {
  const [robot, setRobot] = useState<any>(null);
  const [fk, setFk] = useState<any>(null);
  const [ikSolver, setIkSolver] = useState<any>(null);
  const [jacobianCalc, setJacobianCalc] = useState<any>(null);
  const [jointNames, setJointNames] = useState<string[]>([]);
  const [links, setLinks] = useState<any[]>([]);
  const linkRefs = useRef<{[name: string]: THREE.Group}>({});
  const targetRef = useRef<THREE.Object3D>(new THREE.Object3D());

  useEffect(() => {
    const load = async () => {
        let content = urdfContent;
        if (!content && urdfUrl) {
            const res = await fetch(urdfUrl);
            content = await res.text();
        }
        
        if (content) {
            try {
                if (robot) robot.dispose();
                if (fk) fk.dispose();
                if (ikSolver) ikSolver.dispose();

                const r = urdfx.Robot.fromURDFString(content);
                setRobot(r);
                
                const jNamesVector = r.getJointNames();
                const jNames: string[] = [];
                for (let i = 0; i < jNamesVector.size(); i++) {
                    jNames.push(jNamesVector.get(i));
                }
                setJointNames(jNames);

                const linksVector = r.getLinks();
                const l: any[] = [];
                for(let i=0; i<linksVector.size(); i++) {
                    l.push(linksVector.get(i));
                }
                setLinks(l);
                
                const lastLinkName = l[l.length - 1].getName();
                const f = new urdfx.ForwardKinematics(r, lastLinkName);
                setFk(f);
                
                const s = new urdfx.SQPIKSolver(r, lastLinkName);
                setIkSolver(s);
                
                const j = new urdfx.JacobianCalculator(r, lastLinkName);
                setJacobianCalc(j);

            } catch (e) {
                console.error("Failed to parse URDF", e);
            }
        }
    };
    load();
    return () => {
        // Cleanup handled by next effect or unmount
    };
  }, [urdfx, urdfUrl, urdfContent]);

  // Controls
  const controlsConfig = useMemo(() => {
    if (!robot) return {};
    return jointNames.reduce((acc, name) => {
      const joint = robot.getJoint(name);
      const limits = joint?.getLimits();
      const min = limits ? limits.lower : -Math.PI * 2;
      const max = limits ? limits.upper : Math.PI * 2;
      
      acc[name] = { value: 0, min, max, step: 0.01 };
      return acc;
    }, {} as any);
  }, [robot, jointNames]);

  const [jointValues, setJointValues] = useControls('Joints', () => controlsConfig, [controlsConfig]);
  const { ikEnabled } = useControls('IK', { ikEnabled: false });
  const [, setInfo] = useControls('Info', () => ({
      Manipulability: { value: 0, editable: false }
  }));

  // IK Handler
  const onIKDrag = () => {
      if (!ikEnabled || !ikSolver || !targetRef.current) return;
      
      const target = targetRef.current;
      const pos = target.position;
      const quat = target.quaternion;
      
      const t = urdfx.Transform.fromPositionQuaternion(
          [pos.x, pos.y, pos.z], 
          [quat.w, quat.x, quat.y, quat.z]
      );
      
      const currentAngles = jointNames.map(name => (jointValues as any)[name]);
      
      const result = ikSolver.solve(t, currentAngles);
      
      if (result.converged || result.solution) {
          const sol = result.solution;
          const newValues: any = {};
          for(let i=0; i<jointNames.length; i++) {
              const val = (sol as any).get ? (sol as any).get(i) : (sol as any)[i];
              newValues[jointNames[i]] = val;
          }
          setJointValues(newValues);
      }
      
      t.delete();
  };

  // Initialize IK target when enabling IK
  useEffect(() => {
      if (ikEnabled && fk && links.length) {
          const lastLinkName = links[links.length - 1].getName();
          const angles = jointNames.map(name => (jointValues as any)[name]);
          const pose = fk.computeToLink(angles, lastLinkName);
          
          if (targetRef.current) {
              targetRef.current.position.set(pose.position[0], pose.position[1], pose.position[2]);
              targetRef.current.quaternion.set(pose.quaternion[1], pose.quaternion[2], pose.quaternion[3], pose.quaternion[0]);
          }
      }
  }, [ikEnabled, fk, links, jointNames, jointValues]);

  // Update robot pose
  useFrame(() => {
    if (!robot || !fk || !links.length) return;
    
    const angles = jointNames.map(name => (jointValues as any)[name]);
    
    links.forEach(link => {
        const linkName = link.getName();
        const group = linkRefs.current[linkName];
        if (group) {
            const pose = fk.computeToLink(angles, linkName);
            group.position.set(pose.position[0], pose.position[1], pose.position[2]);
            group.quaternion.set(pose.quaternion[1], pose.quaternion[2], pose.quaternion[3], pose.quaternion[0]);
        }
    });
    
    if (jacobianCalc) {
        const m = jacobianCalc.getManipulability(angles);
        setInfo({ Manipulability: m });
    }
  });

  if (!robot) return null;

  return (
    <group>
      {links.map(link => (
        <LinkVisuals 
            key={link.getName()} 
            link={link} 
            urdfx={urdfx}
            ref={(el: THREE.Group) => { if (el) linkRefs.current[link.getName()] = el; }} 
        />
      ))}
      
      {ikEnabled && (
          <TransformControls 
            object={targetRef.current} 
            mode="translate"
            onChange={onIKDrag}
          />
      )}
      {ikEnabled && <primitive object={targetRef.current} visible={false} />}
    </group>
  )
}

const LinkVisuals = forwardRef(({ link, urdfx }: {link: any, urdfx: any}, ref: any) => {
  const visualsVector = link.getVisuals();
  const visuals: any[] = [];
  for(let i=0; i<visualsVector.size(); i++) {
    visuals.push(visualsVector.get(i));
  }

  return (
    <group ref={ref}>
      {visuals.map((visual: any, idx: number) => (
        <VisualRenderer key={idx} visual={visual} urdfx={urdfx} />
      ))}
    </group>
  );
});

function VisualRenderer({ visual, urdfx }: { visual: any, urdfx: any }) {
    const geometry = visual.geometry;
    const type = geometry.type; // 0: Box, 1: Cylinder, 2: Sphere, 3: Mesh
    
    // Transform of the visual relative to the link
    const origin = visual.origin;
    const pos = origin.translation();
    const quat = origin.asPose().quaternion; // [w, x, y, z]

    const groupRef = useRef<THREE.Group>(null);

    useEffect(() => {
        if (groupRef.current) {
            groupRef.current.position.set(pos[0], pos[1], pos[2]);
            groupRef.current.quaternion.set(quat[1], quat[2], quat[3], quat[0]);
        }
    }, [pos, quat]);

    if (type === 3 || type === urdfx.GeometryType.Mesh) { // Mesh
        const filename = geometry.mesh_filename;
        let path = filename;
        if (path.startsWith('package://')) {
             // Simple heuristic: strip package:// and maybe the package name if needed.
             // For now, just strip package://
             path = path.replace('package://', '');
        }
        if (!path.startsWith('/')) path = '/' + path;
        
        const obj = useLoader(OBJLoader, path) as THREE.Group;
        const scale = geometry.mesh_scale;
        
        return (
            <group ref={groupRef}>
                <primitive object={obj.clone()} scale={[scale[0], scale[1], scale[2]]} />
            </group>
        );
    } else if (type === 0 || type === urdfx.GeometryType.Box) { // Box
        const size = geometry.box_size;
        return (
            <group ref={groupRef}>
                <mesh>
                    <boxGeometry args={[size[0], size[1], size[2]]} />
                    <meshStandardMaterial color="orange" />
                </mesh>
            </group>
        );
    } else if (type === 1 || type === urdfx.GeometryType.Cylinder) { // Cylinder
        const r = geometry.cylinder_radius;
        const l = geometry.cylinder_length;
        return (
            <group ref={groupRef}>
                <mesh rotation={[Math.PI/2, 0, 0]}> 
                    <cylinderGeometry args={[r, r, l, 32]} />
                    <meshStandardMaterial color="orange" />
                </mesh>
            </group>
        );
    } else if (type === 2 || type === urdfx.GeometryType.Sphere) { // Sphere
        const r = geometry.sphere_radius;
        return (
            <group ref={groupRef}>
                <mesh>
                    <sphereGeometry args={[r, 32, 32]} />
                    <meshStandardMaterial color="orange" />
                </mesh>
            </group>
        );
    }

    return null;
}
