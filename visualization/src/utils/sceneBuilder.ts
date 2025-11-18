import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import type { ParsedURDF, URDFGeometry, URDFMaterial } from '../types';

/**
 * Loads and builds Three.js scene from parsed URDF
 */
export class URDFSceneBuilder {
  private objLoader: OBJLoader;
  private meshCache: Map<string, THREE.Object3D>;

  constructor() {
    this.objLoader = new OBJLoader();
    this.meshCache = new Map();
  }

  /**
   * Build Three.js scene graph from parsed URDF
   */
  async buildScene(urdf: ParsedURDF): Promise<THREE.Group> {
    console.log('[SceneBuilder] Starting to build scene for robot:', urdf.robotName);
    console.log('[SceneBuilder] Total links to process:', urdf.links.size);
    
    const robotGroup = new THREE.Group();
    robotGroup.name = urdf.robotName;

    // Create link objects
    const linkObjects = new Map<string, THREE.Group>();
    
    for (const [linkName, link] of urdf.links) {
      console.log(`[SceneBuilder] Processing link: ${linkName}, visual elements: ${link.visual.length}`);
      
      const linkGroup = new THREE.Group();
      linkGroup.name = linkName;
      linkGroup.userData.linkName = linkName;

      // Add visual geometries
      for (const visual of link.visual) {
        try {
          console.log(`[SceneBuilder] Creating mesh for link ${linkName}, geometry type: ${visual.geometry.type}`);
          const mesh = await this.createMeshFromGeometry(visual.geometry, visual.material);
          
          // Log the origin transformation
          const position = new THREE.Vector3();
          const quaternion = new THREE.Quaternion();
          const scale = new THREE.Vector3();
          visual.origin.decompose(position, quaternion, scale);
          console.log(`[SceneBuilder] Origin transform - pos: (${position.x.toFixed(3)}, ${position.y.toFixed(3)}, ${position.z.toFixed(3)})`);
          
          mesh.applyMatrix4(visual.origin);
          linkGroup.add(mesh);
          console.log(`[SceneBuilder] Successfully added mesh to link ${linkName}, mesh children count:`, mesh.children.length);
        } catch (error) {
          console.error(`[SceneBuilder] Failed to create visual for link ${linkName}:`, error);
        }
      }

      linkObjects.set(linkName, linkGroup);
    }

    // Build kinematic tree
    const addedLinks = new Set<string>();
    
    const addLinkWithParents = (linkName: string) => {
      if (addedLinks.has(linkName)) return;
      
      // Find joint where this link is a child
      let parentJoint = null;
      for (const [, joint] of urdf.joints) {
        if (joint.child === linkName) {
          parentJoint = joint;
          break;
        }
      }

      if (parentJoint) {
        // Add parent first
        addLinkWithParents(parentJoint.parent);
        
        // Get parent and child link objects
        const parentLinkObj = linkObjects.get(parentJoint.parent);
        const childLinkObj = linkObjects.get(linkName);
        
        if (parentLinkObj && childLinkObj) {
          // Create joint transform
          const jointGroup = new THREE.Group();
          jointGroup.name = `joint_${parentJoint.name}`;
          jointGroup.userData.jointName = parentJoint.name;
          jointGroup.userData.jointType = parentJoint.type;
          jointGroup.userData.axis = parentJoint.axis;
          jointGroup.applyMatrix4(parentJoint.origin);
          
          // Add child link to joint
          jointGroup.add(childLinkObj);
          
          // Add joint to parent link
          parentLinkObj.add(jointGroup);
        }
      } else {
        // This is root link
        const linkObj = linkObjects.get(linkName);
        if (linkObj) {
          robotGroup.add(linkObj);
        }
      }
      
      addedLinks.add(linkName);
    };

    // Start from root link
    addLinkWithParents(urdf.rootLink);

    console.log('[SceneBuilder] Scene building complete. Robot group children:', robotGroup.children.length);
    console.log('[SceneBuilder] Robot group structure:');
    robotGroup.traverse((obj) => {
      if (obj instanceof THREE.Mesh) {
        console.log('  - Mesh:', obj.name, 'vertices:', obj.geometry.attributes.position?.count || 0);
      } else if (obj instanceof THREE.Group) {
        console.log('  - Group:', obj.name, 'children:', obj.children.length);
      }
    });

    return robotGroup;
  }

  /**
   * Update robot joint angles
   */
  updateJointAngles(robotGroup: THREE.Group, jointAngles: Map<string, number>) {
    console.log('[SceneBuilder] Updating joint angles, joints to update:', jointAngles.size);
    robotGroup.traverse((obj) => {
      if (obj.userData.jointName) {
        const angle = jointAngles.get(obj.userData.jointName);
        if (angle !== undefined && obj.userData.axis) {
          const axis = obj.userData.axis as THREE.Vector3;
          const jointType = obj.userData.jointType as string;
          
          console.log(`[SceneBuilder] Updating joint ${obj.userData.jointName}, type: ${jointType}, angle: ${angle}`);
          
          if (jointType === 'revolute' || jointType === 'continuous') {
            // Use quaternion to set rotation around arbitrary axis
            const quaternion = new THREE.Quaternion();
            quaternion.setFromAxisAngle(axis.clone().normalize(), angle);
            obj.quaternion.copy(quaternion);
            console.log(`[SceneBuilder] Set rotation for ${obj.userData.jointName}, axis:`, axis, 'angle:', angle);
          } else if (jointType === 'prismatic') {
            obj.position.copy(axis.clone().multiplyScalar(angle));
            console.log(`[SceneBuilder] Set position for ${obj.userData.jointName}, pos:`, obj.position);
          }
        }
      }
    });
  }

  private async createMeshFromGeometry(
    geometry: URDFGeometry,
    material?: URDFMaterial
  ): Promise<THREE.Object3D> {
    let threeGeometry: THREE.BufferGeometry;

    switch (geometry.type) {
      case 'box':
        threeGeometry = new THREE.BoxGeometry(
          geometry.size?.x || 1,
          geometry.size?.y || 1,
          geometry.size?.z || 1
        );
        break;

      case 'cylinder':
        threeGeometry = new THREE.CylinderGeometry(
          geometry.radius || 1,
          geometry.radius || 1,
          geometry.length || 1,
          32
        );
        break;

      case 'sphere':
        threeGeometry = new THREE.SphereGeometry(geometry.radius || 1, 32, 32);
        break;

      case 'mesh':
        if (!geometry.filename) {
          throw new Error('Mesh geometry missing filename');
        }
        return this.loadMesh(geometry.filename);

      default:
        threeGeometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
    }

    const threeMaterial = this.createMaterial(material);
    const mesh = new THREE.Mesh(threeGeometry, threeMaterial);
    
    return mesh;
  }

  private createMaterial(urdfMaterial?: URDFMaterial): THREE.Material {
    const color = urdfMaterial?.color || new THREE.Color(0.8, 0.8, 0.8);
    
    return new THREE.MeshPhongMaterial({
      color,
      shininess: 30,
      flatShading: false,
    });
  }

  private async loadMesh(filename: string): Promise<THREE.Object3D> {
    console.log('[SceneBuilder] loadMesh called with filename:', filename);
    
    // Check cache
    if (this.meshCache.has(filename)) {
      console.log('[SceneBuilder] Using cached mesh for:', filename);
      return this.meshCache.get(filename)!.clone();
    }

    // Convert package:// URLs to relative paths
    let meshPath = filename;
    if (meshPath.startsWith('package://')) {
      meshPath = meshPath.replace('package://', '/meshes/');
      console.log('[SceneBuilder] Converted package:// URL to:', meshPath);
    } else if (!meshPath.startsWith('/') && !meshPath.startsWith('http://') && !meshPath.startsWith('https://')) {
      // If it's a relative path, make it absolute from the public directory
      meshPath = '/' + meshPath;
      console.log('[SceneBuilder] Converted relative path to:', meshPath);
    }

    console.log('[SceneBuilder] Loading mesh from:', meshPath);

    try {
      const obj = await this.objLoader.loadAsync(meshPath);
      console.log('[SceneBuilder] Successfully loaded mesh:', meshPath);
      
      // Count meshes and vertices
      let meshCount = 0;
      let totalVertices = 0;
      obj.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          meshCount++;
          const vertexCount = child.geometry.attributes.position?.count || 0;
          totalVertices += vertexCount;
          child.material = new THREE.MeshPhongMaterial({
            color: 0x888888,
            shininess: 30,
          });
        }
      });
      console.log(`[SceneBuilder] Mesh stats - meshes: ${meshCount}, total vertices: ${totalVertices}`);

      this.meshCache.set(filename, obj);
      return obj.clone();
    } catch (error) {
      console.error(`[SceneBuilder] Failed to load mesh ${meshPath}:`, error);
      console.error('[SceneBuilder] Using fallback red box geometry');
      // Fallback to box
      const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
      const material = new THREE.MeshPhongMaterial({ color: 0xff0000 });
      return new THREE.Mesh(geometry, material);
    }
  }
}
