#include "urdfx/kinematics.h"
#include "urdfx/logging.h"
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4244)  // Conversion warnings
#pragma warning(disable: 4267)  // size_t conversion warnings
#endif
#include <cppad/cppad.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
#include <stdexcept>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <limits>
#include <vector>
#include <cmath>
#include <utility>
#include <mutex>

namespace urdfx {

void JacobianCalculator::TapeData::TapeDeleter::operator()(void* ptr) const {
    delete static_cast<CppAD::ADFun<double, double>*>(ptr);
}

CppAD::ADFun<double, double>* JacobianCalculator::getTapePointer(const TapeData& data) {
    return static_cast<CppAD::ADFun<double, double>*>(data.tape.get());
}

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
                Eigen::AngleAxisd rotation(q, joint->getAxis());
                joint_transform.linear() = rotation.toRotationMatrix();
                break;
            }
            case JointType::Prismatic: {
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

constexpr size_t kPoseVectorSize = 12;
std::mutex g_cppad_mutex;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& m) {
    return Eigen::Vector3d(m(2, 1), m(0, 2), m(1, 0));
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

Eigen::Matrix<double, 6, 6> adMatrix(const Eigen::Matrix<double, 6, 1>& twist) {
    Eigen::Matrix<double, 6, 6> ad = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Vector3d v = twist.head<3>();
    const Eigen::Vector3d w = twist.tail<3>();
    ad.block<3, 3>(0, 0) = skew(w);
    ad.block<3, 3>(3, 0) = skew(v);
    ad.block<3, 3>(3, 3) = skew(w);
    return ad;
}

} // namespace

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

    {
        std::lock_guard<std::mutex> lock(g_cppad_mutex);
        ensureTape(default_end_link_);
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

const ForwardKinematics& JacobianCalculator::ensureForwardKinematics(const std::string& link) const {
    auto it = fk_cache_.find(link);
    if (it == fk_cache_.end()) {
        auto fk = std::make_unique<ForwardKinematics>(robot_, link, base_link_);
        it = fk_cache_.emplace(link, std::move(fk)).first;
    }
    return *it->second;
}

JacobianCalculator::TapeData& JacobianCalculator::ensureTape(const std::string& link) const {
    auto it = tape_cache_.find(link);
    if (it != tape_cache_.end()) {
        return *it->second;
    }

    const auto& chain = ensureChain(link);
    const size_t dof = chain.getNumJoints();
    if (dof == 0) {
        throw std::runtime_error("JacobianCalculator requires at least one actuated joint");
    }

    std::vector<CppAD::AD<double>> ad_q(dof);
    for (size_t i = 0; i < dof; ++i) {
        ad_q[i] = 0.0;
    }
    CppAD::Independent(ad_q);

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1> joint_vec(dof);
    for (size_t i = 0; i < dof; ++i) {
        joint_vec[i] = ad_q[i];
    }

    auto T = ad_fk_.compute(chain, joint_vec);

    std::vector<CppAD::AD<double>> outputs(kPoseVectorSize);
    for (int i = 0; i < 3; ++i) {
        outputs[i] = T.translation()(i);
    }

    size_t cursor = 3;
    const auto& rot = T.rotation();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            outputs[cursor++] = rot(r, c);
        }
    }

    auto tape = std::make_unique<CppAD::ADFun<double, double>>(ad_q, outputs);
    tape->optimize();

    auto data = std::make_unique<TapeData>();
    data->tape.reset(static_cast<void*>(tape.release()));
    data->dof = dof;

    auto [new_it, inserted] = tape_cache_.emplace(link, std::move(data));
    (void)inserted;
    return *new_it->second;
}

Eigen::MatrixXd JacobianCalculator::compute(
    const Eigen::VectorXd& joint_angles,
    JacobianType type,
    const std::string& target_link) const
{
    std::lock_guard<std::mutex> lock(g_cppad_mutex);
    const std::string link = resolveLink(target_link);
    auto& tape = ensureTape(link);

    if (static_cast<size_t>(joint_angles.size()) < tape.dof) {
        throw std::invalid_argument("Joint vector size is smaller than chain DOF for Jacobian computation");
    }

    Eigen::VectorXd joint_subset = joint_angles.head(tape.dof);

    std::vector<double> x(tape.dof);
    for (size_t i = 0; i < tape.dof; ++i) {
        x[i] = joint_subset[i];
    }

    std::vector<double> raw = getTapePointer(tape)->Jacobian(x);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> raw_map(
        raw.data(), kPoseVectorSize, tape.dof);
    Eigen::MatrixXd raw_matrix = raw_map;

    const auto& fk = ensureForwardKinematics(link);
    Transform pose = fk.compute(joint_subset);
    Eigen::MatrixXd spatial = buildSpatialJacobian(raw_matrix, pose);
    return convertIfNeeded(spatial, pose, type);
}

Eigen::MatrixXd JacobianCalculator::computeJacobianDerivative(
    const Eigen::VectorXd& joint_angles,
    const Eigen::VectorXd& joint_velocities,
    JacobianType type,
    const std::string& target_link) const
{
    std::lock_guard<std::mutex> lock(g_cppad_mutex);
    const std::string link = resolveLink(target_link);
    auto& tape = ensureTape(link);

    if (static_cast<size_t>(joint_angles.size()) < tape.dof ||
        static_cast<size_t>(joint_velocities.size()) < tape.dof) {
        throw std::invalid_argument("Joint vectors must be at least as large as chain DOF for Jacobian derivative");
    }

    Eigen::VectorXd joint_subset = joint_angles.head(tape.dof);
    Eigen::VectorXd velocity_subset = joint_velocities.head(tape.dof);

    std::vector<double> x(tape.dof);
    for (size_t i = 0; i < tape.dof; ++i) {
        x[i] = joint_subset[i];
    }

    std::vector<double> raw = getTapePointer(tape)->Jacobian(x);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> raw_map(
        raw.data(), kPoseVectorSize, tape.dof);
    Eigen::MatrixXd raw_matrix = raw_map;

    const auto& fk = ensureForwardKinematics(link);
    Transform pose = fk.compute(joint_subset);
    Eigen::MatrixXd spatial = buildSpatialJacobian(raw_matrix, pose);

    Eigen::MatrixXd raw_dot(kPoseVectorSize, tape.dof);
    std::vector<double> w(kPoseVectorSize, 0.0);
    for (size_t output_idx = 0; output_idx < kPoseVectorSize; ++output_idx) {
        w[output_idx] = 1.0;
        std::vector<double> hess = getTapePointer(tape)->Hessian(x, w);
        w[output_idx] = 0.0;

        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> H(
            hess.data(), tape.dof, tape.dof);
        Eigen::VectorXd grad_dot = H * velocity_subset;
        raw_dot.row(output_idx) = grad_dot.transpose();
    }

    Eigen::MatrixXd spatial_dot = buildSpatialJacobianDerivative(raw_dot, pose);
    return convertDerivativeIfNeeded(spatial, spatial_dot, pose, velocity_subset, type);
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

Eigen::MatrixXd JacobianCalculator::buildSpatialJacobian(
    const Eigen::MatrixXd& raw_pose_jac,
    const Transform& pose) const
{
    Eigen::MatrixXd J(6, raw_pose_jac.cols());
    J.topRows(3) = raw_pose_jac.topRows(3);

    const Eigen::Matrix3d R = pose.rotation();
    for (int col = 0; col < J.cols(); ++col) {
        Eigen::Matrix3d dR = Eigen::Matrix3d::Zero();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                dR(r, c) = raw_pose_jac(3 + r * 3 + c, col);
            }
        }
        Eigen::Matrix3d omega_skew = dR * R.transpose();
        J.block<3, 1>(3, col) = vee(omega_skew);
    }

    return J;
}

Eigen::MatrixXd JacobianCalculator::buildSpatialJacobianDerivative(
    const Eigen::MatrixXd& raw_pose_jac_dot,
    const Transform& pose) const
{
    return buildSpatialJacobian(raw_pose_jac_dot, pose);
}

Eigen::MatrixXd JacobianCalculator::convertIfNeeded(
    const Eigen::MatrixXd& spatial_jac,
    const Transform& pose,
    JacobianType type) const
{
    if (type == JacobianType::Analytic) {
        return spatial_jac;
    }
    return convertJacobian(spatial_jac, pose, JacobianType::Analytic, JacobianType::Geometric);
}

Eigen::MatrixXd JacobianCalculator::convertDerivativeIfNeeded(
    const Eigen::MatrixXd& spatial_jac,
    const Eigen::MatrixXd& spatial_jac_dot,
    const Transform& pose,
    const Eigen::VectorXd& joint_velocities,
    JacobianType type) const
{
    if (type == JacobianType::Analytic) {
        return spatial_jac_dot;
    }

    Eigen::MatrixXd J_body = convertJacobian(spatial_jac, pose, JacobianType::Analytic, JacobianType::Geometric);
    Eigen::MatrixXd J_body_dot = convertJacobian(spatial_jac_dot, pose, JacobianType::Analytic, JacobianType::Geometric);

    Eigen::Matrix<double, 6, 1> twist_body = J_body * joint_velocities;
    J_body_dot -= adMatrix(twist_body) * J_body;
    return J_body_dot;
}

} // namespace urdfx
