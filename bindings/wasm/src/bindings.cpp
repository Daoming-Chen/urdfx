#include <emscripten/bind.h>

#include <emscripten/val.h>

#include "urdfx/inverse_kinematics.h"
#include "urdfx/kinematics.h"
#include "urdfx/robot_model.h"
#include "urdfx/urdf_parser.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace emscripten;

namespace {

bool isNullish(const val& value) {
    return value.isNull() || value.isUndefined();
}

bool isLengthBasedArrayLike(const val& value) {
    return !value["length"].isUndefined();
}

std::vector<double> toVectorDouble(const val& value, std::size_t expected_size, const char* context) {
    if (isNullish(value)) {
        throw std::invalid_argument(std::string("Expected ") + context + " to be array-like.");
    }

    std::vector<double> result;

    if (isLengthBasedArrayLike(value)) {
        const auto length = value["length"].as<std::size_t>();
        result.reserve(length);
        for (std::size_t i = 0; i < length; ++i) {
            result.push_back(value[i].as<double>());
        }
    } else if (!value["size"].isUndefined() && !value["get"].isUndefined()) {
        const auto length = value.call<std::size_t>("size");
        result.reserve(length);
        for (std::size_t i = 0; i < length; ++i) {
            result.push_back(value.call<double>("get", i));
        }
    } else {
        throw std::invalid_argument(std::string("Expected ") + context + " to be array-like.");
    }

    if (expected_size != 0 && result.size() != expected_size) {
        throw std::invalid_argument(std::string(context) + " must contain exactly " + std::to_string(expected_size) + " elements.");
    }

    return result;
}

bool toBoolOrDefault(const val& maybe_value, bool default_value) {
    return isNullish(maybe_value) ? default_value : maybe_value.as<bool>();
}

double toDoubleOrDefault(const val& maybe_value, double default_value) {
    return isNullish(maybe_value) ? default_value : maybe_value.as<double>();
}

std::string toStringOrDefault(const val& maybe_value, const std::string& default_value) {
    return isNullish(maybe_value) ? default_value : maybe_value.as<std::string>();
}

template <typename TEnum>
TEnum toEnumOrDefault(const val& maybe_value, TEnum default_value) {
    return isNullish(maybe_value) ? default_value : maybe_value.as<TEnum>();
}

constexpr std::size_t kMaxPosePositionSize = 3;
constexpr std::size_t kMaxPoseQuaternionSize = 4;

struct PoseData {
    std::array<double, kMaxPosePositionSize> position{};
    std::array<double, kMaxPoseQuaternionSize> quaternion{}; // w, x, y, z
};

struct MatrixData {
    std::vector<double> data;
    int rows = 0;
    int cols = 0;
};

struct SolverConfigData {
    std::size_t max_iterations = 64;
    double tolerance = 1e-4;
    double regularization = 1e-5;
    double max_step_size = 0.5;
    std::size_t max_line_search_steps = 6;
    double line_search_shrink = 0.5;
    double line_search_min_alpha = 0.05;
    double line_search_improvement = 1e-6;
    double position_weight = 1.0;
    double orientation_weight = 1.0;
    double position_anchor_weight = 1e-2;
    double orientation_anchor_weight = 1e-6;
    double joint_limit_margin = 1e-4;
    double unbounded_joint_limit = 50.0;
    bool enable_warm_start = true;
};

struct IKResultData {
    bool converged = false;
    std::size_t iterations = 0;
    double final_error_norm = 0.0;
    double final_step_norm = 0.0;
    int qp_status = 0;
    std::string message;
    std::vector<double> solution;
    std::vector<double> error_history;
};

class RobotHandle {
public:
    static std::shared_ptr<RobotHandle> fromURDFString(const std::string& urdf_string, const std::string& base_dir = "") {
        urdfx::URDFParser parser;
        if (!base_dir.empty()) {
            parser.setBaseDirectory(base_dir);
        }
        auto robot = parser.parseString(urdf_string);
        if (!robot) {
            throw std::runtime_error("Failed to parse URDF string.");
        }
        return std::shared_ptr<RobotHandle>(new RobotHandle(std::move(robot)));
    }

    explicit RobotHandle(std::shared_ptr<urdfx::Robot> robot)
        : robot_(std::move(robot)) {}

    void ensureAlive() const {
        if (!robot_) {
            throw std::runtime_error("RobotHandle has been disposed.");
        }
    }

    const std::string& getName() const {
        ensureAlive();
        return robot_->getName();
    }

    std::vector<std::string> getJointNames() const {
        ensureAlive();
        std::vector<std::string> names;
        auto joints = robot_->getActuatedJoints();
        names.reserve(joints.size());
        for (const auto& joint : joints) {
            names.push_back(joint->getName());
        }
        return names;
    }

    std::size_t getDOF() const {
        ensureAlive();
        return robot_->getActuatedJoints().size();
    }

    std::vector<double> getLowerLimits() const {
        ensureAlive();
        std::vector<double> limits;
        auto joints = robot_->getActuatedJoints();
        limits.reserve(joints.size());
        for (const auto& joint : joints) {
            const auto& joint_limits = joint->getLimits();
            if (joint_limits.has_value()) {
                limits.push_back(joint_limits->lower);
            } else {
                limits.push_back(-std::numeric_limits<double>::infinity());
            }
        }
        return limits;
    }

    std::vector<double> getUpperLimits() const {
        ensureAlive();
        std::vector<double> limits;
        auto joints = robot_->getActuatedJoints();
        limits.reserve(joints.size());
        for (const auto& joint : joints) {
            const auto& joint_limits = joint->getLimits();
            if (joint_limits.has_value()) {
                limits.push_back(joint_limits->upper);
            } else {
                limits.push_back(std::numeric_limits<double>::infinity());
            }
        }
        return limits;
    }

    void dispose() {
        robot_.reset();
    }

    bool isDisposed() const {
        return robot_ == nullptr;
    }

    std::shared_ptr<const urdfx::Robot> getRobot() const {
        ensureAlive();
        return robot_;
    }

private:
    std::shared_ptr<urdfx::Robot> robot_;
};

class ForwardKinematicsHandle {
public:
    ForwardKinematicsHandle(std::shared_ptr<RobotHandle> robot, const std::string& end_link, const std::string& base_link = "")
        : robot_(std::move(robot))
        , fk_(std::make_unique<urdfx::ForwardKinematics>(robot_->getRobot(), end_link, base_link)) {}

    PoseData compute(const std::vector<double>& joint_angles, bool check_bounds = false) const {
        ensureAlive();
        auto q = toEigenVector(joint_angles, fk_->getNumJoints());
        auto transform = fk_->compute(q, check_bounds);
        return toPoseData(transform);
    }

    PoseData computeToLink(const std::vector<double>& joint_angles, const std::string& target_link, bool check_bounds = false) const {
        ensureAlive();
        auto q = toEigenVector(joint_angles, fk_->getNumJoints());
        auto transform = fk_->computeToLink(q, target_link, check_bounds);
        return toPoseData(transform);
    }

    MatrixData computeMatrix(const std::vector<double>& joint_angles, bool check_bounds = false) const {
        ensureAlive();
        auto q = toEigenVector(joint_angles, fk_->getNumJoints());
        auto transform = fk_->compute(q, check_bounds);
        return toMatrixData(transform.asMatrix());
    }

    std::size_t getNumJoints() const {
        ensureAlive();
        return fk_->getNumJoints();
    }

    void dispose() {
        fk_.reset();
        robot_.reset();
    }

private:
    void ensureAlive() const {
        if (!fk_) {
            throw std::runtime_error("ForwardKinematicsHandle has been disposed.");
        }
        if (!robot_ || robot_->isDisposed()) {
            throw std::runtime_error("Associated RobotHandle has been disposed.");
        }
    }

    static Eigen::VectorXd toEigenVector(const std::vector<double>& values, std::size_t expected_size) {
        if (values.size() != expected_size) {
            throw std::invalid_argument("Joint angle vector size does not match chain degrees of freedom.");
        }
        Eigen::VectorXd vec(static_cast<Eigen::Index>(values.size()));
        for (Eigen::Index i = 0; i < vec.size(); ++i) {
            vec[i] = values[static_cast<std::size_t>(i)];
        }
        return vec;
    }

    static PoseData toPoseData(const urdfx::Transform& transform) {
        PoseData data;
        const auto [position, quaternion] = transform.asPositionQuaternion();
        data.position = {position.x(), position.y(), position.z()};
        data.quaternion = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
        return data;
    }

    static MatrixData toMatrixData(const Eigen::Matrix4d& matrix) {
        MatrixData data;
        data.rows = static_cast<int>(matrix.rows());
        data.cols = static_cast<int>(matrix.cols());
        data.data.resize(static_cast<std::size_t>(matrix.size()));
        std::size_t index = 0;
        for (int r = 0; r < matrix.rows(); ++r) {
            for (int c = 0; c < matrix.cols(); ++c) {
                data.data[index++] = matrix(r, c);
            }
        }
        return data;
    }

    std::shared_ptr<RobotHandle> robot_;
    std::unique_ptr<urdfx::ForwardKinematics> fk_;
};

class JacobianCalculatorHandle {
public:
    JacobianCalculatorHandle(std::shared_ptr<RobotHandle> robot, const std::string& end_link, const std::string& base_link = "")
        : robot_(std::move(robot))
        , calculator_(std::make_unique<urdfx::JacobianCalculator>(robot_->getRobot(), end_link, base_link)) {}

    MatrixData compute(const std::vector<double>& joint_angles, urdfx::JacobianType type = urdfx::JacobianType::Analytic, const std::string& target_link = "") const {
        ensureAlive();
        auto q = toEigenVector(joint_angles);
        auto matrix = calculator_->compute(q, type, target_link);
        return toMatrixData(matrix);
    }

    bool isSingular(const std::vector<double>& joint_angles, double threshold = 1e-6, urdfx::JacobianType type = urdfx::JacobianType::Analytic, const std::string& target_link = "") const {
        ensureAlive();
        auto q = toEigenVector(joint_angles);
        return calculator_->isSingular(q, threshold, type, target_link);
    }

    double getManipulability(const std::vector<double>& joint_angles, urdfx::JacobianType type = urdfx::JacobianType::Analytic, const std::string& target_link = "") const {
        ensureAlive();
        auto q = toEigenVector(joint_angles);
        return calculator_->getManipulability(q, type, target_link);
    }

    double getConditionNumber(const std::vector<double>& joint_angles, urdfx::JacobianType type = urdfx::JacobianType::Analytic, const std::string& target_link = "") const {
        ensureAlive();
        auto q = toEigenVector(joint_angles);
        return calculator_->getConditionNumber(q, type, target_link);
    }

    void dispose() {
        calculator_.reset();
        robot_.reset();
    }

private:
    void ensureAlive() const {
        if (!calculator_) {
            throw std::runtime_error("JacobianCalculatorHandle has been disposed.");
        }
        if (!robot_ || robot_->isDisposed()) {
            throw std::runtime_error("Associated RobotHandle has been disposed.");
        }
    }

    Eigen::VectorXd toEigenVector(const std::vector<double>& values) const {
        const std::size_t dof = robot_->getDOF();
        if (values.size() != dof) {
            throw std::invalid_argument("Joint state vector size does not match robot degrees of freedom.");
        }
        Eigen::VectorXd vec(static_cast<Eigen::Index>(values.size()));
        for (Eigen::Index i = 0; i < vec.size(); ++i) {
            vec[i] = values[static_cast<std::size_t>(i)];
        }
        return vec;
    }

    static MatrixData toMatrixData(const Eigen::MatrixXd& matrix) {
        MatrixData data;
        data.rows = static_cast<int>(matrix.rows());
        data.cols = static_cast<int>(matrix.cols());
        data.data.resize(static_cast<std::size_t>(matrix.size()));
        std::size_t index = 0;
        for (int r = 0; r < matrix.rows(); ++r) {
            for (int c = 0; c < matrix.cols(); ++c) {
                data.data[index++] = matrix(r, c);
            }
        }
        return data;
    }

    std::shared_ptr<RobotHandle> robot_;
    std::unique_ptr<urdfx::JacobianCalculator> calculator_;
};

class SQPIKSolverHandle {
public:
    SQPIKSolverHandle(std::shared_ptr<RobotHandle> robot, const std::string& end_link, const std::string& base_link = "")
        : robot_(std::move(robot))
        , solver_(std::make_unique<urdfx::SQPIKSolver>(robot_->getRobot(), end_link, base_link)) {}

    void setConfig(const SolverConfigData& config) {
        ensureAlive();
        solver_->setSolverConfig(toSolverConfig(config));
    }

    SolverConfigData getConfig() const {
        ensureAlive();
        return fromSolverConfig(solver_->getSolverConfig());
    }

    void setPositionOnly(bool enable) {
        ensureAlive();
        solver_->setPositionOnly(enable);
    }

    void setOrientationOnly(bool enable) {
        ensureAlive();
        solver_->setOrientationOnly(enable);
    }

    void setWarmStart(const std::vector<double>& guess) {
        ensureAlive();
        auto eigen_guess = toEigenVector(guess);
        solver_->setWarmStart(eigen_guess);
    }

    IKResultData solve(const PoseData& target_pose, const std::vector<double>& initial_guess) {
        ensureAlive();
        const auto q0 = toEigenVector(initial_guess);
        Eigen::VectorXd solution = q0;
        const urdfx::Transform target = toTransform(target_pose);
        auto status = solver_->solve(target, q0, solution);

        IKResultData result;
        result.converged = status.converged;
        result.iterations = status.iterations;
        result.final_error_norm = status.final_error_norm;
        result.final_step_norm = status.final_step_norm;
        result.qp_status = status.qp_status;
        result.message = status.message;
        result.solution.assign(solution.data(), solution.data() + solution.size());
        result.error_history = status.error_history;
        return result;
    }

    void dispose() {
        solver_.reset();
        robot_.reset();
    }

private:
    void ensureAlive() const {
        if (!solver_) {
            throw std::runtime_error("SQPIKSolverHandle has been disposed.");
        }
        if (!robot_ || robot_->isDisposed()) {
            throw std::runtime_error("Associated RobotHandle has been disposed.");
        }
    }

    Eigen::VectorXd toEigenVector(const std::vector<double>& values) const {
        const std::size_t dof = robot_->getDOF();
        if (values.size() != dof) {
            throw std::invalid_argument("Joint vector size does not match robot degrees of freedom.");
        }
        Eigen::VectorXd vec(static_cast<Eigen::Index>(values.size()));
        for (Eigen::Index i = 0; i < vec.size(); ++i) {
            vec[i] = values[static_cast<std::size_t>(i)];
        }
        return vec;
    }

    static urdfx::Transform toTransform(const PoseData& pose) {
        const Eigen::Vector3d position(pose.position[0], pose.position[1], pose.position[2]);
        const Eigen::Quaterniond quaternion(pose.quaternion[0], pose.quaternion[1], pose.quaternion[2], pose.quaternion[3]);
        return urdfx::Transform::fromPositionQuaternion(position, quaternion);
    }

    static SolverConfigData fromSolverConfig(const urdfx::SolverConfig& config) {
        SolverConfigData data;
        data.max_iterations = config.max_iterations;
        data.tolerance = config.tolerance;
        data.regularization = config.regularization;
        data.max_step_size = config.max_step_size;
        data.max_line_search_steps = config.max_line_search_steps;
        data.line_search_shrink = config.line_search_shrink;
        data.line_search_min_alpha = config.line_search_min_alpha;
        data.line_search_improvement = config.line_search_improvement;
        data.position_weight = config.position_weight;
        data.orientation_weight = config.orientation_weight;
        data.position_anchor_weight = config.position_anchor_weight;
        data.orientation_anchor_weight = config.orientation_anchor_weight;
        data.joint_limit_margin = config.joint_limit_margin;
        data.unbounded_joint_limit = config.unbounded_joint_limit;
        data.enable_warm_start = config.enable_warm_start;
        return data;
    }

    static urdfx::SolverConfig toSolverConfig(const SolverConfigData& data) {
        urdfx::SolverConfig config;
        config.max_iterations = data.max_iterations;
        config.tolerance = data.tolerance;
        config.regularization = data.regularization;
        config.max_step_size = data.max_step_size;
        config.max_line_search_steps = data.max_line_search_steps;
        config.line_search_shrink = data.line_search_shrink;
        config.line_search_min_alpha = data.line_search_min_alpha;
        config.line_search_improvement = data.line_search_improvement;
        config.position_weight = data.position_weight;
        config.orientation_weight = data.orientation_weight;
        config.position_anchor_weight = data.position_anchor_weight;
        config.orientation_anchor_weight = data.orientation_anchor_weight;
        config.joint_limit_margin = data.joint_limit_margin;
        config.unbounded_joint_limit = data.unbounded_joint_limit;
        config.enable_warm_start = data.enable_warm_start;
        return config;
    }

    std::shared_ptr<RobotHandle> robot_;
    std::unique_ptr<urdfx::SQPIKSolver> solver_;
};

} // namespace

val getBoxSize(const urdfx::Geometry& g) {
    return val(std::array<double, 3>{g.box_size.x(), g.box_size.y(), g.box_size.z()});
}
void setBoxSize(urdfx::Geometry& g, val v) {
    auto arr = toVectorDouble(v, 3, "box_size");
    g.box_size = Eigen::Vector3d(arr[0], arr[1], arr[2]);
}
val getMeshScale(const urdfx::Geometry& g) {
    return val(std::array<double, 3>{g.mesh_scale.x(), g.mesh_scale.y(), g.mesh_scale.z()});
}
void setMeshScale(urdfx::Geometry& g, val v) {
    auto arr = toVectorDouble(v, 3, "mesh_scale");
    g.mesh_scale = Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

EMSCRIPTEN_BINDINGS(urdfx_wasm_bindings) {
    value_array<std::array<double, kMaxPosePositionSize>>("Vec3")
        .element(emscripten::index<0>())
        .element(emscripten::index<1>())
        .element(emscripten::index<2>());

    value_array<std::array<double, kMaxPoseQuaternionSize>>("Quat")
        .element(emscripten::index<0>())
        .element(emscripten::index<1>())
        .element(emscripten::index<2>())
        .element(emscripten::index<3>());

    value_object<PoseData>("Pose")
        .field("position", &PoseData::position)
        .field("quaternion", &PoseData::quaternion);

    value_object<MatrixData>("Matrix")
        .field("data", &MatrixData::data)
        .field("rows", &MatrixData::rows)
        .field("cols", &MatrixData::cols);

    value_object<SolverConfigData>("SolverConfig")
        .field("max_iterations", &SolverConfigData::max_iterations)
        .field("tolerance", &SolverConfigData::tolerance)
        .field("regularization", &SolverConfigData::regularization)
        .field("max_step_size", &SolverConfigData::max_step_size)
        .field("max_line_search_steps", &SolverConfigData::max_line_search_steps)
        .field("line_search_shrink", &SolverConfigData::line_search_shrink)
        .field("line_search_min_alpha", &SolverConfigData::line_search_min_alpha)
        .field("line_search_improvement", &SolverConfigData::line_search_improvement)
        .field("position_weight", &SolverConfigData::position_weight)
        .field("orientation_weight", &SolverConfigData::orientation_weight)
        .field("position_anchor_weight", &SolverConfigData::position_anchor_weight)
        .field("orientation_anchor_weight", &SolverConfigData::orientation_anchor_weight)
        .field("joint_limit_margin", &SolverConfigData::joint_limit_margin)
        .field("unbounded_joint_limit", &SolverConfigData::unbounded_joint_limit)
        .field("enable_warm_start", &SolverConfigData::enable_warm_start);

    value_object<IKResultData>("IKResult")
        .field("converged", &IKResultData::converged)
        .field("iterations", &IKResultData::iterations)
        .field("final_error_norm", &IKResultData::final_error_norm)
        .field("final_step_norm", &IKResultData::final_step_norm)
        .field("qp_status", &IKResultData::qp_status)
        .field("message", &IKResultData::message)
        .field("solution", &IKResultData::solution)
        .field("error_history", &IKResultData::error_history);

    value_object<urdfx::JointLimits>("JointLimits")
        .field("lower", &urdfx::JointLimits::lower)
        .field("upper", &urdfx::JointLimits::upper)
        .field("effort", &urdfx::JointLimits::effort)
        .field("velocity", &urdfx::JointLimits::velocity);

    value_object<urdfx::JointDynamics>("JointDynamics")
        .field("damping", &urdfx::JointDynamics::damping)
        .field("friction", &urdfx::JointDynamics::friction);

    value_object<urdfx::Geometry>("Geometry")
        .field("type", &urdfx::Geometry::type)
        .field("box_size", 
            std::function<val(const urdfx::Geometry&)>(getBoxSize), 
            std::function<void(urdfx::Geometry&, val)>(setBoxSize))
        .field("cylinder_radius", &urdfx::Geometry::cylinder_radius)
        .field("cylinder_length", &urdfx::Geometry::cylinder_length)
        .field("sphere_radius", &urdfx::Geometry::sphere_radius)
        .field("mesh_filename", &urdfx::Geometry::mesh_filename)
        .field("mesh_scale", 
            std::function<val(const urdfx::Geometry&)>(getMeshScale), 
            std::function<void(urdfx::Geometry&, val)>(setMeshScale));

    enum_<urdfx::JacobianType>("JacobianType")
        .value("Analytic", urdfx::JacobianType::Analytic)
        .value("Geometric", urdfx::JacobianType::Geometric);

    enum_<urdfx::GeometryType>("GeometryType")
        .value("Box", urdfx::GeometryType::Box)
        .value("Cylinder", urdfx::GeometryType::Cylinder)
        .value("Sphere", urdfx::GeometryType::Sphere)
        .value("Mesh", urdfx::GeometryType::Mesh);

    enum_<urdfx::JointType>("JointType")
        .value("Fixed", urdfx::JointType::Fixed)
        .value("Revolute", urdfx::JointType::Revolute)
        .value("Continuous", urdfx::JointType::Continuous)
        .value("Prismatic", urdfx::JointType::Prismatic)
        .value("Floating", urdfx::JointType::Floating)
        .value("Planar", urdfx::JointType::Planar);

    class_<urdfx::Transform>("Transform")
        .constructor<>()
        .class_function("fromPositionQuaternion", emscripten::optional_override([](const std::array<double, 3>& pos, const std::array<double, 4>& quat) {
            return urdfx::Transform::fromPositionQuaternion(
                Eigen::Vector3d(pos[0], pos[1], pos[2]),
                Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]));
        }))
        .function("asMatrix", emscripten::optional_override([](const urdfx::Transform& self) {
            MatrixData md;
            md.rows = 4;
            md.cols = 4;
            Eigen::Matrix4d m = self.asMatrix();
            md.data.assign(m.data(), m.data() + 16);
            return md;
        }))
        .function("asPose", emscripten::optional_override([](const urdfx::Transform& self) {
            auto [pos, quat] = self.asPositionQuaternion();
            PoseData pd;
            pd.position = {pos.x(), pos.y(), pos.z()};
            pd.quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};
            return pd;
        }))
        .function("translation", emscripten::optional_override([](const urdfx::Transform& self) {
            Eigen::Vector3d t = self.translation();
            return std::array<double, 3>{t.x(), t.y(), t.z()};
        }));

    class_<urdfx::Visual>("Visual")
        .property("name", &urdfx::Visual::name)
        .property("origin", &urdfx::Visual::origin)
        .property("geometry", &urdfx::Visual::geometry)
        .function("getColor", emscripten::optional_override([](const urdfx::Visual& v) { 
            if (v.color) return val(std::array<double, 4>{v.color->x(), v.color->y(), v.color->z(), v.color->w()});
            return val::null();
        }))
        .function("getMaterialName", emscripten::optional_override([](const urdfx::Visual& v) {
            if (v.material_name) return val(*v.material_name);
            return val::null();
        }));

    class_<urdfx::Collision>("Collision")
        .property("name", &urdfx::Collision::name)
        .property("origin", &urdfx::Collision::origin)
        .property("geometry", &urdfx::Collision::geometry);

    class_<urdfx::Inertial>("Inertial")
        .property("origin", &urdfx::Inertial::origin)
        .property("mass", &urdfx::Inertial::mass)
        .function("getInertia", emscripten::optional_override([](const urdfx::Inertial& i) {
            MatrixData md; md.rows = 3; md.cols = 3;
            md.data.assign(i.inertia.data(), i.inertia.data() + 9);
            return md;
        }));

    class_<urdfx::Link>("Link")
        .smart_ptr<std::shared_ptr<urdfx::Link>>("Link")
        .function("getName", &urdfx::Link::getName)
        .function("getInertial", emscripten::optional_override([](const urdfx::Link& self) {
            if (self.getInertial()) return val(*self.getInertial());
            return val::null();
        }))
        .function("getVisuals", &urdfx::Link::getVisuals)
        .function("getCollisions", &urdfx::Link::getCollisions);

    class_<urdfx::Joint>("Joint")
        .smart_ptr<std::shared_ptr<urdfx::Joint>>("Joint")
        .function("getName", &urdfx::Joint::getName)
        .function("getType", &urdfx::Joint::getType)
        .function("getParentLink", &urdfx::Joint::getParentLink)
        .function("getChildLink", &urdfx::Joint::getChildLink)
        .function("getOrigin", &urdfx::Joint::getOrigin)
        .function("getAxis", emscripten::optional_override([](const urdfx::Joint& self) {
            Eigen::Vector3d a = self.getAxis();
            return std::array<double, 3>{a.x(), a.y(), a.z()};
        }))
        .function("getLimits", emscripten::optional_override([](const urdfx::Joint& self) {
            if (self.getLimits()) return val(*self.getLimits());
            return val::null();
        }))
        .function("getDynamics", emscripten::optional_override([](const urdfx::Joint& self) {
            if (self.getDynamics()) return val(*self.getDynamics());
            return val::null();
        }));

    class_<RobotHandle>("Robot")
        .smart_ptr<std::shared_ptr<RobotHandle>>("Robot")
        .class_function("fromURDFString", emscripten::optional_override([](const std::string& urdf, emscripten::val maybe_base_dir) {
            if (maybe_base_dir.isNull() || maybe_base_dir.isUndefined()) {
                return RobotHandle::fromURDFString(urdf);
            }
            return RobotHandle::fromURDFString(urdf, maybe_base_dir.as<std::string>());
        }))
        .function("getName", &RobotHandle::getName)
        .function("getJointNames", &RobotHandle::getJointNames)
        .function("getDOF", &RobotHandle::getDOF)
        .function("getLowerLimits", &RobotHandle::getLowerLimits)
        .function("getUpperLimits", &RobotHandle::getUpperLimits)
        .function("getRootLink", emscripten::optional_override([](const RobotHandle& self) {
            self.ensureAlive();
            return self.getRobot()->getRootLink();
        }))
        .function("getLinks", emscripten::optional_override([](const RobotHandle& self) {
            self.ensureAlive();
            return self.getRobot()->getLinks();
        }))
        .function("getJoints", emscripten::optional_override([](const RobotHandle& self) {
            self.ensureAlive();
            return self.getRobot()->getJoints();
        }))
        .function("getLink", emscripten::optional_override([](const RobotHandle& self, const std::string& name) {
            self.ensureAlive();
            return self.getRobot()->getLink(name);
        }))
        .function("getJoint", emscripten::optional_override([](const RobotHandle& self, const std::string& name) {
            self.ensureAlive();
            return self.getRobot()->getJoint(name);
        }))
        .function("dispose", &RobotHandle::dispose)
        .function("isDisposed", &RobotHandle::isDisposed);

    class_<ForwardKinematicsHandle>("ForwardKinematics")
        .smart_ptr<std::shared_ptr<ForwardKinematicsHandle>>("ForwardKinematics")
        .constructor<std::shared_ptr<RobotHandle>, std::string, std::string>()
        .function("compute", emscripten::optional_override([](ForwardKinematicsHandle& self, val joint_angles, val maybe_check_bounds) {
            const auto angles = toVectorDouble(joint_angles, self.getNumJoints(), "jointAngles");
            const bool check_bounds = toBoolOrDefault(maybe_check_bounds, false);
            return self.compute(angles, check_bounds);
        }))
        .function("computeToLink", emscripten::optional_override([](ForwardKinematicsHandle& self, val joint_angles, const std::string& target_link, val maybe_check_bounds) {
            const auto angles = toVectorDouble(joint_angles, self.getNumJoints(), "jointAngles");
            const bool check_bounds = toBoolOrDefault(maybe_check_bounds, false);
            return self.computeToLink(angles, target_link, check_bounds);
        }))
        .function("computeMatrix", emscripten::optional_override([](ForwardKinematicsHandle& self, val joint_angles, val maybe_check_bounds) {
            const auto angles = toVectorDouble(joint_angles, self.getNumJoints(), "jointAngles");
            const bool check_bounds = toBoolOrDefault(maybe_check_bounds, false);
            return self.computeMatrix(angles, check_bounds);
        }))
        .function("getNumJoints", &ForwardKinematicsHandle::getNumJoints)
        .function("dispose", &ForwardKinematicsHandle::dispose);

    class_<JacobianCalculatorHandle>("JacobianCalculator")
        .smart_ptr<std::shared_ptr<JacobianCalculatorHandle>>("JacobianCalculator")
        .constructor<std::shared_ptr<RobotHandle>, std::string, std::string>()
        .function("compute", emscripten::optional_override([](JacobianCalculatorHandle& self, val joint_angles, val maybe_type, val maybe_target_link) {
            const auto angles = toVectorDouble(joint_angles, 0, "jointAngles");
            const auto type = toEnumOrDefault<urdfx::JacobianType>(maybe_type, urdfx::JacobianType::Analytic);
            const auto target_link = toStringOrDefault(maybe_target_link, std::string());
            return self.compute(angles, type, target_link);
        }))
        .function("isSingular", emscripten::optional_override([](JacobianCalculatorHandle& self, val joint_angles, val maybe_threshold, val maybe_type, val maybe_target_link) {
            const auto angles = toVectorDouble(joint_angles, 0, "jointAngles");
            const double threshold = toDoubleOrDefault(maybe_threshold, 1e-6);
            const auto type = toEnumOrDefault<urdfx::JacobianType>(maybe_type, urdfx::JacobianType::Analytic);
            const auto target_link = toStringOrDefault(maybe_target_link, std::string());
            return self.isSingular(angles, threshold, type, target_link);
        }))
        .function("getManipulability", emscripten::optional_override([](JacobianCalculatorHandle& self, val joint_angles, val maybe_type, val maybe_target_link) {
            const auto angles = toVectorDouble(joint_angles, 0, "jointAngles");
            const auto type = toEnumOrDefault<urdfx::JacobianType>(maybe_type, urdfx::JacobianType::Analytic);
            const auto target_link = toStringOrDefault(maybe_target_link, std::string());
            return self.getManipulability(angles, type, target_link);
        }))
        .function("getConditionNumber", emscripten::optional_override([](JacobianCalculatorHandle& self, val joint_angles, val maybe_type, val maybe_target_link) {
            const auto angles = toVectorDouble(joint_angles, 0, "jointAngles");
            const auto type = toEnumOrDefault<urdfx::JacobianType>(maybe_type, urdfx::JacobianType::Analytic);
            const auto target_link = toStringOrDefault(maybe_target_link, std::string());
            return self.getConditionNumber(angles, type, target_link);
        }))
        .function("dispose", &JacobianCalculatorHandle::dispose);

    class_<SQPIKSolverHandle>("SQPIKSolver")
        .smart_ptr<std::shared_ptr<SQPIKSolverHandle>>("SQPIKSolver")
        .constructor<std::shared_ptr<RobotHandle>, std::string, std::string>()
        .function("setConfig", &SQPIKSolverHandle::setConfig)
        .function("getConfig", &SQPIKSolverHandle::getConfig)
        .function("setPositionOnly", &SQPIKSolverHandle::setPositionOnly)
        .function("setOrientationOnly", &SQPIKSolverHandle::setOrientationOnly)
        .function("setWarmStart", emscripten::optional_override([](SQPIKSolverHandle& self, val initial_guess) {
            const auto guess = toVectorDouble(initial_guess, 0, "initialGuess");
            self.setWarmStart(guess);
        }))
        .function("solve", emscripten::optional_override([](SQPIKSolverHandle& self, const PoseData& target_pose, val initial_guess) {
            const auto guess = toVectorDouble(initial_guess, 0, "initialGuess");
            return self.solve(target_pose, guess);
        }))
        .function("dispose", &SQPIKSolverHandle::dispose);

    register_vector<double>("VectorDouble");
    register_vector<std::string>("VectorString");
    register_vector<urdfx::Visual>("VectorVisual");
    register_vector<urdfx::Collision>("VectorCollision");
    register_vector<std::shared_ptr<urdfx::Link>>("VectorLink");
    register_vector<std::shared_ptr<urdfx::Joint>>("VectorJoint");
}
