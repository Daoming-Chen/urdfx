import type * as THREE from 'three';
import type { Robot, Pose } from '../../public/urdfx';

export interface JointInfo {
  name: string;
  index: number;
  lowerLimit: number;
  upperLimit: number;
  value: number;
}

export interface RobotState {
  robot: Robot | null;
  jointAngles: number[];
  jointInfo: JointInfo[];
  endEffectorPose: Pose | null;
}

export interface URDFLink {
  name: string;
  visual: URDFVisual[];
  collision: URDFCollision[];
}

export interface URDFVisual {
  geometry: URDFGeometry;
  origin: THREE.Matrix4;
  material?: URDFMaterial;
}

export interface URDFCollision {
  geometry: URDFGeometry;
  origin: THREE.Matrix4;
}

export interface URDFGeometry {
  type: 'box' | 'cylinder' | 'sphere' | 'mesh';
  size?: THREE.Vector3; // For box
  radius?: number; // For sphere and cylinder
  length?: number; // For cylinder
  filename?: string; // For mesh
}

export interface URDFMaterial {
  color?: THREE.Color;
  texture?: string;
}

export interface URDFJoint {
  name: string;
  type: 'revolute' | 'continuous' | 'prismatic' | 'fixed' | 'floating' | 'planar';
  parent: string;
  child: string;
  origin: THREE.Matrix4;
  axis: THREE.Vector3;
  lowerLimit?: number;
  upperLimit?: number;
  velocity?: number;
  effort?: number;
}

export interface ParsedURDF {
  robotName: string;
  links: Map<string, URDFLink>;
  joints: Map<string, URDFJoint>;
  rootLink: string;
}

export type ControlMode = 'FK' | 'IK';

export interface AppState {
  mode: ControlMode;
  urdfContent: string | null;
  parsedURDF: ParsedURDF | null;
  robotState: RobotState;
  isLoading: boolean;
  error: string | null;
}
