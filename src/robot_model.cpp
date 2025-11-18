#include "urdfx/robot_model.h"
#include "urdfx/logging.h"
#include <cmath>
#include <unordered_set>

namespace urdfx {

// ============================================================================
// Transform Implementation
// ============================================================================

Transform Transform::fromPositionQuaternion(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& quaternion) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translate(position);
    transform.rotate(quaternion.normalized());
    return Transform(transform);
}

Transform Transform::fromPositionRPY(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& rpy) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translate(position);
    
    // Apply rotations in order: roll (X), pitch (Y), yaw (Z)
    Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    transform.rotate(q);
    
    return Transform(transform);
}

Eigen::Matrix4d Transform::asMatrix() const {
    return transform_.matrix();
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> Transform::asPositionQuaternion() const {
    Eigen::Vector3d position = transform_.translation();
    Eigen::Quaterniond quaternion(transform_.rotation());
    quaternion.normalize();
    return {position, quaternion};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Transform::asPositionRPY() const {
    Eigen::Vector3d position = transform_.translation();
    Eigen::Matrix3d rotation = transform_.rotation();
    
    // Extract RPY angles from rotation matrix
    // Using ZYX Euler angle convention
    Eigen::Vector3d rpy;
    rpy(0) = std::atan2(rotation(2, 1), rotation(2, 2));  // roll
    rpy(1) = std::atan2(-rotation(2, 0), 
                        std::sqrt(rotation(2, 1) * rotation(2, 1) + 
                                 rotation(2, 2) * rotation(2, 2)));  // pitch
    rpy(2) = std::atan2(rotation(1, 0), rotation(0, 0));  // yaw
    
    return {position, rpy};
}

Transform Transform::operator*(const Transform& other) const {
    return Transform(transform_ * other.transform_);
}

Transform Transform::inverse() const {
    return Transform(transform_.inverse());
}

// ============================================================================
// Joint Implementation
// ============================================================================

Transform Joint::getTransform(double value) const {
    Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
    
    switch (type_) {
        case JointType::Fixed:
            // No additional transformation for fixed joints
            break;
            
        case JointType::Revolute:
        case JointType::Continuous:
            // Rotation around joint axis
            joint_transform.rotate(Eigen::AngleAxisd(value, axis_));
            break;
            
        case JointType::Prismatic:
            // Translation along joint axis
            joint_transform.translate(value * axis_);
            break;
            
        case JointType::Floating:
        case JointType::Planar:
            URDFX_WARN("Joint type {} not fully supported yet", 
                       static_cast<int>(type_));
            break;
    }
    
    // Compose with the fixed origin transform
    return origin_ * Transform(joint_transform);
}

// ============================================================================
// Robot Implementation
// ============================================================================

void Robot::addLink(std::shared_ptr<Link> link) {
    links_.push_back(link);
    maps_built_ = false;
}

std::shared_ptr<Link> Robot::getLink(const std::string& name) const {
    if (!maps_built_) {
        buildMaps();
    }
    
    auto it = link_map_.find(name);
    if (it != link_map_.end()) {
        return it->second;
    }
    return nullptr;
}

void Robot::addJoint(std::shared_ptr<Joint> joint) {
    joints_.push_back(joint);
    maps_built_ = false;
}

std::shared_ptr<Joint> Robot::getJoint(const std::string& name) const {
    if (!maps_built_) {
        buildMaps();
    }
    
    auto it = joint_map_.find(name);
    if (it != joint_map_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<std::shared_ptr<Joint>> Robot::getActuatedJoints() const {
    std::vector<std::shared_ptr<Joint>> actuated;
    for (const auto& joint : joints_) {
        if (joint->isActuated()) {
            actuated.push_back(joint);
        }
    }
    return actuated;
}

std::vector<std::shared_ptr<Joint>> Robot::getChildJoints(const std::string& link_name) const {
    std::vector<std::shared_ptr<Joint>> children;
    for (const auto& joint : joints_) {
        if (joint->getParentLink() == link_name) {
            children.push_back(joint);
        }
    }
    return children;
}

std::shared_ptr<Joint> Robot::getParentJoint(const std::string& link_name) const {
    for (const auto& joint : joints_) {
        if (joint->getChildLink() == link_name) {
            return joint;
        }
    }
    return nullptr;
}

bool Robot::validate() const {
    if (links_.empty()) {
        URDFX_ERROR("Robot has no links");
        return false;
    }
    
    if (root_link_.empty()) {
        URDFX_ERROR("Robot has no root link specified");
        return false;
    }
    
    // Check root link exists
    if (!getLink(root_link_)) {
        URDFX_ERROR("Root link '{}' not found in robot", root_link_);
        return false;
    }
    
    // Check all joint parent/child links exist
    for (const auto& joint : joints_) {
        if (!getLink(joint->getParentLink())) {
            URDFX_ERROR("Joint '{}' references non-existent parent link '{}'",
                       joint->getName(), joint->getParentLink());
            return false;
        }
        if (!getLink(joint->getChildLink())) {
            URDFX_ERROR("Joint '{}' references non-existent child link '{}'",
                       joint->getName(), joint->getChildLink());
            return false;
        }
    }
    
    // Check for cycles in kinematic tree
    std::unordered_set<std::string> visited;
    std::unordered_set<std::string> rec_stack;
    
    std::function<bool(const std::string&)> hasCycle = [&](const std::string& link_name) -> bool {
        visited.insert(link_name);
        rec_stack.insert(link_name);
        
        for (const auto& child_joint : getChildJoints(link_name)) {
            const std::string& child_link = child_joint->getChildLink();
            
            if (rec_stack.find(child_link) != rec_stack.end()) {
                URDFX_ERROR("Cycle detected in kinematic tree at link '{}'", child_link);
                return true;
            }
            
            if (visited.find(child_link) == visited.end()) {
                if (hasCycle(child_link)) {
                    return true;
                }
            }
        }
        
        rec_stack.erase(link_name);
        return false;
    };
    
    if (hasCycle(root_link_)) {
        return false;
    }
    
    // Check for disconnected links (warning only)
    for (const auto& link : links_) {
        if (link->getName() != root_link_ && !getParentJoint(link->getName())) {
            URDFX_WARN("Link '{}' is disconnected from kinematic tree", link->getName());
        }
    }
    
    URDFX_INFO("Robot '{}' validation passed", name_);
    return true;
}

void Robot::buildMaps() const {
    link_map_.clear();
    joint_map_.clear();
    
    for (const auto& link : links_) {
        link_map_[link->getName()] = link;
    }
    
    for (const auto& joint : joints_) {
        joint_map_[joint->getName()] = joint;
    }
    
    maps_built_ = true;
}

} // namespace urdfx
