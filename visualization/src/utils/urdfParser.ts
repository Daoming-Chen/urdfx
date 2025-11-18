import * as THREE from 'three';
import type { ParsedURDF, URDFLink, URDFJoint, URDFGeometry, URDFMaterial } from '../types';

/**
 * Parses a URDF XML string into a structured format
 */
export class URDFParser {
  private parser: DOMParser;

  constructor() {
    this.parser = new DOMParser();
  }

  /**
   * Parse URDF string into structured data
   */
  parse(urdfContent: string): ParsedURDF {
    const xmlDoc = this.parser.parseFromString(urdfContent, 'text/xml');
    
    const parserError = xmlDoc.querySelector('parsererror');
    if (parserError) {
      throw new Error(`URDF XML parsing error: ${parserError.textContent}`);
    }

    const robotElement = xmlDoc.querySelector('robot');
    if (!robotElement) {
      throw new Error('No <robot> element found in URDF');
    }

    const robotName = robotElement.getAttribute('name') || 'unnamed_robot';
    
    const links = this.parseLinks(robotElement);
    const joints = this.parseJoints(robotElement);
    
    // Find root link (link that is not a child of any joint)
    const childLinks = new Set<string>();
    joints.forEach(joint => childLinks.add(joint.child));
    
    let rootLink = '';
    for (const linkName of links.keys()) {
      if (!childLinks.has(linkName)) {
        rootLink = linkName;
        break;
      }
    }

    if (!rootLink && links.size > 0) {
      rootLink = links.keys().next().value || '';
    }

    return {
      robotName,
      links,
      joints,
      rootLink,
    };
  }

  private parseLinks(robotElement: Element): Map<string, URDFLink> {
    const links = new Map<string, URDFLink>();
    const linkElements = robotElement.querySelectorAll('link');

    linkElements.forEach(linkEl => {
      const name = linkEl.getAttribute('name');
      if (!name) return;

      const visual = this.parseVisualElements(linkEl);
      const collision = this.parseCollisionElements(linkEl);

      links.set(name, { name, visual, collision });
    });

    return links;
  }

  private parseVisualElements(linkElement: Element) {
    const visuals: Array<{ geometry: URDFGeometry; origin: THREE.Matrix4; material?: URDFMaterial }> = [];
    const visualElements = linkElement.querySelectorAll('visual');
    const linkName = linkElement.getAttribute('name');

    console.log(`[URDFParser] Parsing ${visualElements.length} visual elements for link: ${linkName}`);

    visualElements.forEach(visualEl => {
      const geometry = this.parseGeometry(visualEl);
      const origin = this.parseOrigin(visualEl);
      const material = this.parseMaterial(visualEl);

      if (geometry) {
        visuals.push({ geometry, origin, material });
      }
    });

    return visuals;
  }

  private parseCollisionElements(linkElement: Element) {
    const collisions: Array<{ geometry: URDFGeometry; origin: THREE.Matrix4 }> = [];
    const collisionElements = linkElement.querySelectorAll('collision');

    collisionElements.forEach(collisionEl => {
      const geometry = this.parseGeometry(collisionEl);
      const origin = this.parseOrigin(collisionEl);

      if (geometry) {
        collisions.push({ geometry, origin });
      }
    });

    return collisions;
  }

  private parseGeometry(element: Element): URDFGeometry | null {
    const geometryEl = element.querySelector('geometry');
    if (!geometryEl) return null;

    // Check for box
    const boxEl = geometryEl.querySelector('box');
    if (boxEl) {
      const size = this.parseVector3(boxEl.getAttribute('size') || '1 1 1');
      console.log('[URDFParser] Found box geometry:', size);
      return { type: 'box', size };
    }

    // Check for cylinder
    const cylinderEl = geometryEl.querySelector('cylinder');
    if (cylinderEl) {
      const radius = parseFloat(cylinderEl.getAttribute('radius') || '1');
      const length = parseFloat(cylinderEl.getAttribute('length') || '1');
      console.log('[URDFParser] Found cylinder geometry:', { radius, length });
      return { type: 'cylinder', radius, length };
    }

    // Check for sphere
    const sphereEl = geometryEl.querySelector('sphere');
    if (sphereEl) {
      const radius = parseFloat(sphereEl.getAttribute('radius') || '1');
      console.log('[URDFParser] Found sphere geometry:', { radius });
      return { type: 'sphere', radius };
    }

    // Check for mesh
    const meshEl = geometryEl.querySelector('mesh');
    if (meshEl) {
      const filename = meshEl.getAttribute('filename') || '';
      console.log('[URDFParser] Found mesh geometry:', filename);
      return { type: 'mesh', filename };
    }

    return null;
  }

  private parseMaterial(element: Element): URDFMaterial | undefined {
    const materialEl = element.querySelector('material');
    if (!materialEl) return undefined;

    const colorEl = materialEl.querySelector('color');
    if (colorEl) {
      const rgba = this.parseVector4(colorEl.getAttribute('rgba') || '0.8 0.8 0.8 1');
      return {
        color: new THREE.Color(rgba.x, rgba.y, rgba.z),
      };
    }

    return undefined;
  }

  private parseOrigin(element: Element): THREE.Matrix4 {
    const originEl = element.querySelector('origin');
    if (!originEl) return new THREE.Matrix4();

    const xyz = this.parseVector3(originEl.getAttribute('xyz') || '0 0 0');
    const rpy = this.parseVector3(originEl.getAttribute('rpy') || '0 0 0');

    const matrix = new THREE.Matrix4();
    matrix.makeRotationFromEuler(new THREE.Euler(rpy.x, rpy.y, rpy.z, 'XYZ'));
    matrix.setPosition(xyz);

    return matrix;
  }

  private parseJoints(robotElement: Element): Map<string, URDFJoint> {
    const joints = new Map<string, URDFJoint>();
    const jointElements = robotElement.querySelectorAll('joint');

    jointElements.forEach(jointEl => {
      const name = jointEl.getAttribute('name');
      const type = jointEl.getAttribute('type') as URDFJoint['type'];
      if (!name || !type) return;

      const parentEl = jointEl.querySelector('parent');
      const childEl = jointEl.querySelector('child');
      if (!parentEl || !childEl) return;

      const parent = parentEl.getAttribute('link') || '';
      const child = childEl.getAttribute('link') || '';

      const origin = this.parseOrigin(jointEl);
      const axis = this.parseAxis(jointEl);

      const limitEl = jointEl.querySelector('limit');
      let lowerLimit: number | undefined;
      let upperLimit: number | undefined;
      let velocity: number | undefined;
      let effort: number | undefined;

      if (limitEl) {
        const lower = limitEl.getAttribute('lower');
        const upper = limitEl.getAttribute('upper');
        if (lower) lowerLimit = parseFloat(lower);
        if (upper) upperLimit = parseFloat(upper);
        
        const vel = limitEl.getAttribute('velocity');
        const eff = limitEl.getAttribute('effort');
        if (vel) velocity = parseFloat(vel);
        if (eff) effort = parseFloat(eff);
      }

      joints.set(name, {
        name,
        type,
        parent,
        child,
        origin,
        axis,
        lowerLimit,
        upperLimit,
        velocity,
        effort,
      });
    });

    return joints;
  }

  private parseAxis(element: Element): THREE.Vector3 {
    const axisEl = element.querySelector('axis');
    if (!axisEl) return new THREE.Vector3(1, 0, 0);
    return this.parseVector3(axisEl.getAttribute('xyz') || '1 0 0');
  }

  private parseVector3(str: string): THREE.Vector3 {
    const parts = str.trim().split(/\s+/).map(parseFloat);
    return new THREE.Vector3(parts[0] || 0, parts[1] || 0, parts[2] || 0);
  }

  private parseVector4(str: string): THREE.Vector4 {
    const parts = str.trim().split(/\s+/).map(parseFloat);
    return new THREE.Vector4(parts[0] || 0, parts[1] || 0, parts[2] || 0, parts[3] || 1);
  }
}
