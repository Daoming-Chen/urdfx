#include "urdfx/kinematics.h"
#include "urdfx/logging.h"
#include <stdexcept>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <limits>
#include <vector>
#include <cmath>
#include <utility>
#include <mutex>
#include <iostream>

namespace urdfx {

// ============================================================================
// KinematicChain Implementation
// ============================================================================

KinematicChain::KinematicChain(
    std::shared_ptr<const Robot> robot,
    const std::string& end_link,
    const std::string& base_link)
    : robot_(robot)
    , base_link_(base_link.empty() ? robot->getRootLink() : base_link)
    , end_link_(end_link)
{
    if (!robot_) {
        throw std::invalid_argument("Robot pointer cannot be null");
    }
    
    if (!robot_->getLink(end_link_)) {
        throw std::invalid_argument("End link '" + end_link_ + "' not found in robot");
    }
    
    if (!robot_->getLink(base_link_)) {
        throw std::invalid_argument("Base link '" + base_link_ + "' not found in robot");
    }
    
    URDFX_LOG_INFO("Building kinematic chain from '{}' to '{}'", base_link_, end_link_);
    buildChain();
    URDFX_LOG_INFO("Kinematic chain built with {} actuated joints", joints_.size());
}

void KinematicChain::buildChain() {
    // Find path from end_link to base_link by traversing parent joints
    std::vector<std::shared_ptr<Joint>> path_joints;
    std::vector<std::string> path_links;
    
    std::string current_link = end_link_;
    path_links.push_back(current_link);
    
    // Traverse from end to base
    while (current_link != base_link_) {
        auto parent_joint = robot_->getParentJoint(current_link);
        if (!parent_joint) {
            throw std::runtime_error(
                "No path found from '" + end_link_ + "' to '" + base_link_ + 
                "': link '" + current_link + "' has no parent joint");
        }
        
        path_joints.push_back(parent_joint);
        current_link = parent_joint->getParentLink();
        path_links.push_back(current_link);
    }
    
    // Reverse to get base-to-end order
    std::reverse(path_joints.begin(), path_joints.end());
    std::reverse(path_links.begin(), path_links.end());
    
    // Filter to only actuated joints and build static transforms
    link_names_ = path_links;
    
    for (auto& joint : path_joints) {
        if (joint->isActuated()) {
            joints_.push_back(joint);
        }
        // Store the joint's origin transform (static part)
        static_transforms_.push_back(joint->getOrigin());
    }
    
    URDFX_LOG_DEBUG("Chain has {} total joints, {} actuated", 
                    path_joints.size(), joints_.size());
}

std::vector<std::shared_ptr<Joint>> KinematicChain::findPath(
    const std::string& from_link,
    const std::string& to_link)
{
    // This is a helper method for potential future use
    // Current implementation uses direct parent traversal in buildChain
    std::vector<std::shared_ptr<Joint>> path;
    
    std::string current = from_link;
    while (current != to_link) {
        auto parent_joint = robot_->getParentJoint(current);
        if (!parent_joint) {
            return {};  // No path found
        }
        path.push_back(parent_joint);
        current = parent_joint->getParentLink();
    }
    
    return path;
}

// ============================================================================
// ForwardKinematics Implementation
// ============================================================================

ForwardKinematics::ForwardKinematics(
    std::shared_ptr<const Robot> robot,
    const std::string& end_link,
    const std::string& base_link)
    : robot_(robot)
    , chain_(robot, end_link, base_link)
{
    URDFX_LOG_INFO("ForwardKinematics initialized for chain with {} DOF",
                   chain_.getNumJoints());
}

Transform ForwardKinematics::compute(
    const Eigen::VectorXd& joint_angles,
    bool check_bounds) const
{
    if (static_cast<size_t>(joint_angles.size()) != chain_.getNumJoints()) {
        throw std::invalid_argument(
            "Joint angles size (" + std::to_string(joint_angles.size()) + 
            ") does not match number of joints (" + 
            std::to_string(chain_.getNumJoints()) + ")");
    }
    
    if (check_bounds) {
        checkJointLimits(joint_angles);
    }
    
    // Start with identity transform
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    
    const auto& joints = chain_.getJoints();
    
    // Accumulate transformations through the chain
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        double q = joint_angles[i];
        
        // Apply the joint's full transformation:
        // 1. Static origin transform (from joint definition)
        // 2. Joint-dependent transform (rotation or translation)
        
        // Get the static origin transform
        T = T * joint->getOrigin().getTransform();
        
        // Apply joint-dependent transform based on type
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        switch (joint->getType()) {
            case JointType::Revolute:
            case JointType::Continuous: {
                // Rotation around joint axis
                Eigen::AngleAxisd rotation(q, joint->getAxis());
                joint_transform.linear() = rotation.toRotationMatrix();
                break;
            }
            case JointType::Prismatic: {
                // Translation along joint axis
                joint_transform.translation() = q * joint->getAxis();
                break;
            }
            case JointType::Fixed:
                // No additional transform (but this shouldn't happen as fixed joints are filtered)
                break;
            case JointType::Floating:
            case JointType::Planar:
                URDFX_LOG_WARN("Joint type not fully supported: {}", 
                               static_cast<int>(joint->getType()));
                break;
        }
        
        T = T * joint_transform;
    }
    
    return Transform(T);
}

Transform ForwardKinematics::computeToLink(
    const Eigen::VectorXd& joint_angles,
    const std::string& target_link,
    bool check_bounds) const
{
    if (static_cast<size_t>(joint_angles.size()) != chain_.getNumJoints()) {
        throw std::invalid_argument(
            "Joint angles size does not match number of joints");
    }
    
    if (check_bounds) {
        checkJointLimits(joint_angles);
    }
    
    // Find target link in chain
    const auto& link_names = chain_.getLinkNames();
    auto it = std::find(link_names.begin(), link_names.end(), target_link);
    
    if (it == link_names.end()) {
        throw std::invalid_argument(
            "Target link '" + target_link + "' not found in kinematic chain");
    }
    
    size_t target_idx = std::distance(link_names.begin(), it);
    
    // Compute FK up to this link
    // We need to figure out how many joints to include
    const auto& joints = chain_.getJoints();
    
    // Count actuated joints up to target link
    size_t num_joints_to_include = 0;
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        // Find the child link of this joint
        const std::string& child = joint->getChildLink();
        
        // Find position of child in link_names
        auto child_it = std::find(link_names.begin(), link_names.end(), child);
        if (child_it != link_names.end()) {
            size_t child_idx = std::distance(link_names.begin(), child_it);
            if (child_idx <= target_idx) {
                num_joints_to_include = i + 1;
            }
        }
    }
    
    return computeToJointIndex(joint_angles, num_joints_to_include - 1);
}

Transform ForwardKinematics::computeToJointIndex(
    const Eigen::VectorXd& joint_angles,
    size_t end_joint_idx) const
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    
    const auto& joints = chain_.getJoints();
    
    // Accumulate transformations up to end_joint_idx
    for (size_t i = 0; i <= end_joint_idx && i < joints.size(); ++i) {
        const auto& joint = joints[i];
        double q = joint_angles[i];
        
        // Apply static origin transform
        T = T * joint->getOrigin().getTransform();
        
        // Apply joint-dependent transform
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        switch (joint->getType()) {
            case JointType::Revolute:
            case JointType::Continuous: {
                // Rotation around joint axis
                Eigen::AngleAxisd rotation(q, joint->getAxis());
                joint_transform.linear() = rotation.toRotationMatrix();
                break;
            }
            case JointType::Prismatic: {
                // Translation along joint axis
                joint_transform.translation() = q * joint->getAxis();
                break;
            }
            default:
                break;
        }
        
        T = T * joint_transform;
    }
    
    return Transform(T);
}

void ForwardKinematics::checkJointLimits(const Eigen::VectorXd& joint_angles) const {
    const auto& joints = chain_.getJoints();
    
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        const auto& limits = joint->getLimits();
        
        if (limits && joint->getType() == JointType::Revolute) {
            double q = joint_angles[i];
            if (q < limits->lower || q > limits->upper) {
                throw std::runtime_error(
                    "Joint '" + joint->getName() + "' value " + 
                    std::to_string(q) + " is out of bounds [" + 
                    std::to_string(limits->lower) + ", " + 
                    std::to_string(limits->upper) + "]");
            }
        }
    }
}

namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Matrix<double, 6, 6> adjointMatrix(const Transform& T) {
    Eigen::Matrix<double, 6, 6> adj = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix3d R = T.rotation();
    const Eigen::Vector3d p = T.translation();
    adj.block<3, 3>(0, 0) = R;
    adj.block<3, 3>(3, 3) = R;
    adj.block<3, 3>(3, 0) = skew(p) * R;
    return adj;
}

Eigen::Matrix<double, 6, 6> adjointInverseMatrix(const Transform& T) {
    return adjointMatrix(T.inverse());
}

} // namespace

// ============================================================================
// JacobianCalculator Implementation
// ============================================================================

JacobianCalculator::JacobianCalculator(
    std::shared_ptr<const Robot> robot,
    const std::string& end_link,
    const std::string& base_link)
    : robot_(std::move(robot))
    , base_link_(base_link.empty() && robot_ ? robot_->getRootLink() : base_link)
    , default_end_link_(end_link)
{
    if (!robot_) {
        throw std::invalid_argument("Robot pointer cannot be null for JacobianCalculator");
    }

    if (default_end_link_.empty()) {
        default_end_link_ = robot_->getRootLink();
    }

    if (!robot_->getLink(default_end_link_)) {
        throw std::invalid_argument("End link not found in robot model: " + default_end_link_);
    }

    if (!robot_->getLink(base_link_)) {
        throw std::invalid_argument("Base link not found in robot model: " + base_link_);
    }
}

std::string JacobianCalculator::resolveLink(const std::string& target_link) const {
    return target_link.empty() ? default_end_link_ : target_link;
}

const KinematicChain& JacobianCalculator::ensureChain(const std::string& link) const {
    auto it = chain_cache_.find(link);
    if (it == chain_cache_.end()) {
        auto chain = std::make_unique<KinematicChain>(robot_, link, base_link_);
        it = chain_cache_.emplace(link, std::move(chain)).first;
    }
    return *it->second;
}

JacobianCalculator::JointFrameCache& JacobianCalculator::ensureCache(const std::string& link, size_t dof) const {
    auto it = frame_cache_.find(link);
    if (it == frame_cache_.end()) {
        auto [new_it, _] = frame_cache_.emplace(link, JointFrameCache());
        it = new_it;
    }
    it->second.resize(dof);
    return it->second;
}

Eigen::MatrixXd JacobianCalculator::compute(
    const Eigen::VectorXd& joint_angles,
    JacobianType type,
    const std::string& target_link) const
{
    const std::string link = resolveLink(target_link);
    const auto& chain = ensureChain(link);
    const size_t dof = chain.getNumJoints();

    if (static_cast<size_t>(joint_angles.size()) < dof) {
        throw std::invalid_argument("Joint vector size is smaller than chain DOF for Jacobian computation");
    }

    // Get thread-local cache for this link
    auto& cache = ensureCache(link, dof);
    
    Eigen::MatrixXd J(6, dof);
    computeAnalyticalJacobian(joint_angles, chain, cache, J);

    if (type == JacobianType::Analytic) {
        return J;
    } else {
        // Ensure FK exists for conversion
        auto fk_it = fk_cache_.find(link);
        if (fk_it == fk_cache_.end()) {
            auto fk = std::make_unique<ForwardKinematics>(robot_, link, base_link_);
            fk_it = fk_cache_.emplace(link, std::move(fk)).first;
        }
        Transform pose = fk_it->second->compute(joint_angles.head(dof));
        
        return convertJacobian(J, pose, JacobianType::Analytic, JacobianType::Geometric);
    }
}

Eigen::MatrixXd JacobianCalculator::computeJacobianDerivative(
    const Eigen::VectorXd& joint_angles,
    const Eigen::VectorXd& joint_velocities,
    JacobianType type,
    const std::string& target_link) const
{
    (void)joint_angles;
    (void)joint_velocities;
    (void)type;
    (void)target_link;
    throw std::runtime_error("computeJacobianDerivative not implemented for Analytical Jacobian");
}

void JacobianCalculator::computeAnalyticalJacobian(
    const Eigen::VectorXd& joint_angles,
    const KinematicChain& chain,
    JointFrameCache& cache,
    Eigen::MatrixXd& J_out) const
{
    const auto& joints = chain.getJoints();
    size_t dof = joints.size();

    // Initialize world transform accumulator
    Eigen::Isometry3d T_world = Eigen::Isometry3d::Identity();

    // Forward pass: Compute all joint frames in world coordinates
    for (size_t i = 0; i < dof; ++i) {
        const auto& joint = joints[i];
        double q = joint_angles[i];

        // 1. Apply static origin transform
        T_world = T_world * joint->getOrigin().getTransform();

        // 2. Cache joint axis and position in world frame BEFORE applying joint rotation/translation
        Eigen::Vector3d axis_local = joint->getAxis();
        cache.z_world[i] = T_world.linear() * axis_local;
        cache.p_world[i] = T_world.translation();

        // 3. Apply joint-dependent transform
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        switch (joint->getType()) {
            case JointType::Revolute:
            case JointType::Continuous: {
                // Optimization for common Z-axis case (0, 0, 1)
                if (axis_local.x() == 0 && axis_local.y() == 0 && axis_local.z() == 1) {
                     double c = std::cos(q);
                     double s = std::sin(q);
                     joint_transform.linear() << c, -s, 0,
                                                 s,  c, 0,
                                                 0,  0, 1;
                } else {
                    Eigen::AngleAxisd rotation(q, axis_local);
                    joint_transform.linear() = rotation.toRotationMatrix();
                }
                break;
            }
            case JointType::Prismatic: {
                joint_transform.translation() = q * axis_local;
                break;
            }
            default:
                break;
        }
        
        T_world = T_world * joint_transform;
    }

    // Store end-effector position
    cache.p_ee = T_world.translation();

    // Backward pass/Fill: Compute Jacobian columns
    for (size_t i = 0; i < dof; ++i) {
        const auto& joint = joints[i];
        const Eigen::Vector3d& z_i = cache.z_world[i];
        const Eigen::Vector3d& p_i = cache.p_world[i];

        if (joint->getType() == JointType::Prismatic) {
            // Prismatic: Linear velocity is z_i, Angular velocity is 0
            J_out.col(i) << z_i, Eigen::Vector3d::Zero();
        } else {
            // Revolute: Linear velocity is z_i x (p_ee - p_i), Angular velocity is z_i
            Eigen::Vector3d p_diff = cache.p_ee - p_i;
            J_out.col(i) << z_i.cross(p_diff), z_i;
        }
    }
}

bool JacobianCalculator::isSingular(
    const Eigen::VectorXd& joint_angles,
    double threshold,
    JacobianType type,
    const std::string& target_link) const
{
    Eigen::MatrixXd J = compute(joint_angles, type, target_link);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double min_sv = svd.singularValues().minCoeff();
    return min_sv < threshold;
}

double JacobianCalculator::getManipulability(
    const Eigen::VectorXd& joint_angles,
    JacobianType type,
    const std::string& target_link) const
{
    Eigen::MatrixXd J = compute(joint_angles, type, target_link);
    Eigen::MatrixXd gram;
    if (J.rows() <= J.cols()) {
        gram = J * J.transpose();
    } else {
        gram = J.transpose() * J;
    }
    double det = gram.determinant();
    if (det < 0.0) {
        det = 0.0;
    }
    return std::sqrt(det);
}

double JacobianCalculator::getConditionNumber(
    const Eigen::VectorXd& joint_angles,
    JacobianType type,
    const std::string& target_link) const
{
    Eigen::MatrixXd J = compute(joint_angles, type, target_link);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& singular = svd.singularValues();
    double min_sv = singular.minCoeff();
    double max_sv = singular.maxCoeff();
    if (min_sv <= std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::infinity();
    }
    return max_sv / min_sv;
}

Eigen::MatrixXd JacobianCalculator::convertJacobian(
    const Eigen::MatrixXd& jacobian,
    const Transform& pose,
    JacobianType from,
    JacobianType to)
{
    if (from == to) {
        return jacobian;
    }

    if (from == JacobianType::Analytic && to == JacobianType::Geometric) {
        return adjointInverseMatrix(pose) * jacobian;
    }

    if (from == JacobianType::Geometric && to == JacobianType::Analytic) {
        return adjointMatrix(pose) * jacobian;
    }

    throw std::invalid_argument("Unsupported Jacobian conversion requested");
}

} // namespace urdfx
