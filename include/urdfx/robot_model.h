#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <unordered_map>

namespace urdfx {

/**
 * @brief Wrapper around Eigen::Isometry3d for representing 3D transformations
 */
class Transform {
public:
    Transform() : transform_(Eigen::Isometry3d::Identity()) {}
    explicit Transform(const Eigen::Isometry3d& t) : transform_(t) {}
    
    /**
     * @brief Create transform from position and quaternion
     * @param position XYZ position vector
     * @param quaternion Rotation quaternion (w, x, y, z)
     */
    static Transform fromPositionQuaternion(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& quaternion);
    
    /**
     * @brief Create transform from position and RPY (roll-pitch-yaw) angles
     * @param position XYZ position vector
     * @param rpy Roll-pitch-yaw angles in radians
     */
    static Transform fromPositionRPY(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& rpy);
    
    /**
     * @brief Get the underlying Eigen transform
     */
    const Eigen::Isometry3d& getTransform() const { return transform_; }
    Eigen::Isometry3d& getTransform() { return transform_; }
    
    /**
     * @brief Get 4x4 transformation matrix
     */
    Eigen::Matrix4d asMatrix() const;
    
    /**
     * @brief Get position and orientation as quaternion
     */
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> asPositionQuaternion() const;
    
    /**
     * @brief Get position and orientation as RPY angles
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> asPositionRPY() const;
    
    /**
     * @brief Get translation vector
     */
    Eigen::Vector3d translation() const { return transform_.translation(); }
    
    /**
     * @brief Get rotation matrix
     */
    Eigen::Matrix3d rotation() const { return transform_.rotation(); }
    
    /**
     * @brief Compose two transforms (T1 * T2)
     */
    Transform operator*(const Transform& other) const;
    
    /**
     * @brief Get inverse transform
     */
    Transform inverse() const;

private:
    Eigen::Isometry3d transform_;
};

/**
 * @brief Inertial properties of a link
 */
struct Inertial {
    Transform origin;  // Origin of inertial frame relative to link frame
    double mass = 0.0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // 3x3 inertia matrix
};

/**
 * @brief Geometry types for visual and collision elements
 */
enum class GeometryType {
    Box,
    Cylinder,
    Sphere,
    Mesh
};

/**
 * @brief Geometric shape definition
 */
struct Geometry {
    GeometryType type;
    
    // Box dimensions (x, y, z)
    Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
    
    // Cylinder properties
    double cylinder_radius = 0.0;
    double cylinder_length = 0.0;
    
    // Sphere properties
    double sphere_radius = 0.0;
    
    // Mesh properties
    std::string mesh_filename;
    Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
};

/**
 * @brief Visual element of a link
 */
struct Visual {
    std::string name;
    Transform origin;
    Geometry geometry;
    
    // Optional material properties
    std::optional<Eigen::Vector4d> color;  // RGBA
    std::optional<std::string> material_name;
};

/**
 * @brief Collision element of a link
 */
struct Collision {
    std::string name;
    Transform origin;
    Geometry geometry;
};

/**
 * @brief Robot link with physical and geometric properties
 */
class Link {
public:
    explicit Link(const std::string& name) : name_(name) {}
    
    const std::string& getName() const { return name_; }
    
    void setInertial(const Inertial& inertial) { inertial_ = inertial; }
    const std::optional<Inertial>& getInertial() const { return inertial_; }
    
    void addVisual(const Visual& visual) { visuals_.push_back(visual); }
    const std::vector<Visual>& getVisuals() const { return visuals_; }
    
    void addCollision(const Collision& collision) { collisions_.push_back(collision); }
    const std::vector<Collision>& getCollisions() const { return collisions_; }

private:
    std::string name_;
    std::optional<Inertial> inertial_;
    std::vector<Visual> visuals_;
    std::vector<Collision> collisions_;
};

/**
 * @brief Joint types in URDF
 */
enum class JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar
};

/**
 * @brief Joint limits
 */
struct JointLimits {
    double lower = 0.0;
    double upper = 0.0;
    double effort = 0.0;  // Maximum effort
    double velocity = 0.0;  // Maximum velocity
};

/**
 * @brief Joint dynamics properties
 */
struct JointDynamics {
    double damping = 0.0;
    double friction = 0.0;
};

/**
 * @brief Robot joint connecting two links
 */
class Joint {
public:
    Joint(const std::string& name, JointType type) 
        : name_(name), type_(type) {}
    
    const std::string& getName() const { return name_; }
    JointType getType() const { return type_; }
    
    void setParentLink(const std::string& parent) { parent_link_ = parent; }
    const std::string& getParentLink() const { return parent_link_; }
    
    void setChildLink(const std::string& child) { child_link_ = child; }
    const std::string& getChildLink() const { return child_link_; }
    
    void setOrigin(const Transform& origin) { origin_ = origin; }
    const Transform& getOrigin() const { return origin_; }
    
    void setAxis(const Eigen::Vector3d& axis) { axis_ = axis; }
    const Eigen::Vector3d& getAxis() const { return axis_; }
    
    void setLimits(const JointLimits& limits) { limits_ = limits; }
    const std::optional<JointLimits>& getLimits() const { return limits_; }
    
    void setDynamics(const JointDynamics& dynamics) { dynamics_ = dynamics; }
    const std::optional<JointDynamics>& getDynamics() const { return dynamics_; }
    
    /**
     * @brief Check if joint is actuated (not fixed)
     */
    bool isActuated() const { 
        return type_ != JointType::Fixed; 
    }
    
    /**
     * @brief Get transformation for given joint value
     * @param value Joint value (angle for revolute, distance for prismatic)
     */
    Transform getTransform(double value) const;

private:
    std::string name_;
    JointType type_;
    std::string parent_link_;
    std::string child_link_;
    Transform origin_;  // Fixed transform from parent to child
    Eigen::Vector3d axis_ = Eigen::Vector3d::UnitZ();  // Joint axis
    std::optional<JointLimits> limits_;
    std::optional<JointDynamics> dynamics_;
};

/**
 * @brief Complete robot model with links and joints
 */
class Robot {
public:
    explicit Robot(const std::string& name) : name_(name) {}
    
    const std::string& getName() const { return name_; }
    
    void addLink(std::shared_ptr<Link> link);
    std::shared_ptr<Link> getLink(const std::string& name) const;
    const std::vector<std::shared_ptr<Link>>& getLinks() const { return links_; }
    
    void addJoint(std::shared_ptr<Joint> joint);
    std::shared_ptr<Joint> getJoint(const std::string& name) const;
    const std::vector<std::shared_ptr<Joint>>& getJoints() const { return joints_; }
    
    void setRootLink(const std::string& root) { root_link_ = root; }
    const std::string& getRootLink() const { return root_link_; }
    
    /**
     * @brief Get all actuated joints in order
     */
    std::vector<std::shared_ptr<Joint>> getActuatedJoints() const;
    
    /**
     * @brief Get child joints of a link
     */
    std::vector<std::shared_ptr<Joint>> getChildJoints(const std::string& link_name) const;
    
    /**
     * @brief Get parent joint of a link (if any)
     */
    std::shared_ptr<Joint> getParentJoint(const std::string& link_name) const;
    
    /**
     * @brief Validate robot structure
     * @return true if valid, false otherwise
     */
    bool validate() const;

private:
    std::string name_;
    std::string root_link_;
    std::vector<std::shared_ptr<Link>> links_;
    std::vector<std::shared_ptr<Joint>> joints_;
    
    // Internal lookup maps (built lazily)
    mutable std::unordered_map<std::string, std::shared_ptr<Link>> link_map_;
    mutable std::unordered_map<std::string, std::shared_ptr<Joint>> joint_map_;
    mutable bool maps_built_ = false;
    
    void buildMaps() const;
};

} // namespace urdfx
