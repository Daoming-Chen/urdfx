/*
 * Type definitions for the urdfx WebAssembly module.
 *
 * The module is generated with Emscripten using MODULARIZE=1 and EXPORT_NAME=createUrdfxModule.
 * Use the default export factory to instantiate the module:
 *
 * import createUrdfxModule from './urdfx.js';
 * const module = await createUrdfxModule();
 */

export type NumericArray = ArrayLike<number>;

export interface EmbindVector<T> {
    size(): number;
    get(index: number): T;
    set(index: number, value: T): void;
    push_back(value: T): void;
    delete?(): void;
}

export interface Pose {
    position: [number, number, number];
    quaternion: [number, number, number, number]; // w, x, y, z
}

export interface Matrix {
    data: Float64Array | number[] | EmbindVector<number>;
    rows: number;
    cols: number;
}

export interface SolverConfig {
    max_iterations: number;
    tolerance: number;
    regularization: number;
    max_step_size: number;
    max_line_search_steps: number;
    line_search_shrink: number;
    line_search_min_alpha: number;
    line_search_improvement: number;
    position_weight: number;
    orientation_weight: number;
    position_anchor_weight: number;
    orientation_anchor_weight: number;
    joint_limit_margin: number;
    unbounded_joint_limit: number;
    enable_warm_start: boolean;
}

export interface IKResult {
    converged: boolean;
    iterations: number;
    final_error_norm: number;
    final_step_norm: number;
    qp_status: number;
    message: string;
    solution: Float64Array | number[] | EmbindVector<number>;
    error_history: Float64Array | number[] | EmbindVector<number>;
}

export const enum JacobianType {
    Analytic = 0,
    Geometric = 1,
}

export const enum GeometryType {
    Box = 0,
    Cylinder = 1,
    Sphere = 2,
    Mesh = 3,
}

export const enum JointType {
    Fixed = 0,
    Revolute = 1,
    Continuous = 2,
    Prismatic = 3,
    Floating = 4,
    Planar = 5,
}

export class Transform {
    constructor();
    static fromPositionQuaternion(position: [number, number, number], quaternion: [number, number, number, number]): Transform;
    asMatrix(): Matrix;
    asPose(): Pose;
    translation(): [number, number, number];
}

export interface JointLimits {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
}

export interface JointDynamics {
    damping: number;
    friction: number;
}

export interface Geometry {
    type: GeometryType;
    box_size: [number, number, number];
    cylinder_radius: number;
    cylinder_length: number;
    sphere_radius: number;
    mesh_filename: string;
    mesh_scale: [number, number, number];
}

export class Visual {
    get name(): string;
    set name(value: string);
    get origin(): Transform;
    set origin(value: Transform);
    get geometry(): Geometry;
    set geometry(value: Geometry);
    getColor(): [number, number, number, number] | null;
    getMaterialName(): string | null;
}

export class Collision {
    get name(): string;
    set name(value: string);
    get origin(): Transform;
    set origin(value: Transform);
    get geometry(): Geometry;
    set geometry(value: Geometry);
}

export class Inertial {
    get origin(): Transform;
    set origin(value: Transform);
    get mass(): number;
    set mass(value: number);
    getInertia(): Matrix;
}

export class Link {
    getName(): string;
    getInertial(): Inertial | null;
    getVisuals(): EmbindVector<Visual>;
    getCollisions(): EmbindVector<Collision>;
}

export class Joint {
    getName(): string;
    getType(): JointType;
    getParentLink(): string;
    getChildLink(): string;
    getOrigin(): Transform;
    getAxis(): [number, number, number];
    getLimits(): JointLimits | null;
    getDynamics(): JointDynamics | null;
}

export class Robot {
    static fromURDFString(urdf: string, baseDir?: string): Robot;

    getName(): string;
    getJointNames(): string[] | EmbindVector<string>;
    getDOF(): number;
    getLowerLimits(): Float64Array | number[] | EmbindVector<number>;
    getUpperLimits(): Float64Array | number[] | EmbindVector<number>;
    getRootLink(): string;
    getLinks(): EmbindVector<Link>;
    getJoints(): EmbindVector<Joint>;
    getLink(name: string): Link | null;
    getJoint(name: string): Joint | null;
    dispose(): void;
    isDisposed(): boolean;
}

export class ForwardKinematics {
    constructor(robot: Robot, endLink: string, baseLink?: string);

    compute(jointAngles: NumericArray | EmbindVector<number>, checkBounds?: boolean): Pose;
    computeToLink(jointAngles: NumericArray | EmbindVector<number>, targetLink: string, checkBounds?: boolean): Pose;
    computeMatrix(jointAngles: NumericArray, checkBounds?: boolean): Matrix;
    getNumJoints(): number;
    dispose(): void;
}

export class JacobianCalculator {
    constructor(robot: Robot, endLink: string, baseLink?: string);

    compute(jointAngles: NumericArray | EmbindVector<number>, type?: JacobianType, targetLink?: string): Matrix;
    isSingular(jointAngles: NumericArray | EmbindVector<number>, threshold?: number, type?: JacobianType, targetLink?: string): boolean;
    getManipulability(jointAngles: NumericArray | EmbindVector<number>, type?: JacobianType, targetLink?: string): number;
    getConditionNumber(jointAngles: NumericArray | EmbindVector<number>, type?: JacobianType, targetLink?: string): number;
    dispose(): void;
}

export class SQPIKSolver {
    constructor(robot: Robot, endLink: string, baseLink?: string);

    setConfig(config: SolverConfig): void;
    getConfig(): SolverConfig;
    setPositionOnly(enable: boolean): void;
    setOrientationOnly(enable: boolean): void;
    setWarmStart(jointAngles: NumericArray | EmbindVector<number>): void;
    solve(targetPose: Pose, initialGuess: NumericArray | EmbindVector<number>): IKResult;
    dispose(): void;
}

export interface UrdfxModule {
    Robot: typeof Robot;
    ForwardKinematics: typeof ForwardKinematics;
    JacobianCalculator: typeof JacobianCalculator;
    SQPIKSolver: typeof SQPIKSolver;
    Transform: typeof Transform;
    Link: typeof Link;
    Joint: typeof Joint;
    Visual: typeof Visual;
    Collision: typeof Collision;
    Inertial: typeof Inertial;
    JacobianType: {
        Analytic: JacobianType;
        Geometric: JacobianType;
    };
    GeometryType: {
        Box: GeometryType;
        Cylinder: GeometryType;
        Sphere: GeometryType;
        Mesh: GeometryType;
    };
    JointType: {
        Fixed: JointType;
        Revolute: JointType;
        Continuous: JointType;
        Prismatic: JointType;
        Floating: JointType;
        Planar: JointType;
    };
    HEAPF64: Float64Array;
    _malloc(size: number): number;
    _free(ptr: number): void;
}

export interface ModuleFactoryOptions {
    locateFile?(path: string, prefix?: string): string;
    print?(text: string): void;
    printErr?(text: string): void;
}

export type UrdfxModuleFactory = (options?: ModuleFactoryOptions) => Promise<UrdfxModule>;

declare const createUrdfxModule: UrdfxModuleFactory;

export default createUrdfxModule;
