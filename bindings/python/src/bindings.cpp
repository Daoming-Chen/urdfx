#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/unique_ptr.h>

#include "urdfx/robot_model.h"
#include "urdfx/kinematics.h"
#include "urdfx/urdf_parser.h"
#include "urdfx/inverse_kinematics.h"

namespace nb = nanobind;
using namespace urdfx;

struct IKResult {
    SolverStatus status;
    Eigen::VectorXd solution;
};

#ifndef URDFX_VERSION
#define URDFX_VERSION "0.0.0-dev"
#endif

NB_MODULE(_urdfx, m) {
    m.doc() = "Python bindings for urdfx robotics library";
    m.attr("__version__") = URDFX_VERSION;

    // 2.4 Enums
    nb::enum_<JointType>(m, "JointType")
        .value("Fixed", JointType::Fixed)
        .value("Revolute", JointType::Revolute)
        .value("Continuous", JointType::Continuous)
        .value("Prismatic", JointType::Prismatic)
        .value("Floating", JointType::Floating)
        .value("Planar", JointType::Planar)
        .export_values();

    nb::enum_<GeometryType>(m, "GeometryType")
        .value("Box", GeometryType::Box)
        .value("Cylinder", GeometryType::Cylinder)
        .value("Sphere", GeometryType::Sphere)
        .value("Mesh", GeometryType::Mesh)
        .export_values();

    nb::enum_<JacobianType>(m, "JacobianType")
        .value("Analytic", JacobianType::Analytic)
        .value("Geometric", JacobianType::Geometric)
        .export_values();

    // 2.3 Transform
    nb::class_<Transform>(m, "Transform")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&>())
        .def_static("from_position_quaternion", &Transform::fromPositionQuaternion,
            nb::arg("position"), nb::arg("quaternion"),
            "Create transform from position and quaternion")
        .def_static("from_position_rpy", &Transform::fromPositionRPY,
            nb::arg("position"), nb::arg("rpy"),
            "Create transform from position and RPY angles")
        .def("inverse", &Transform::inverse, "Get inverse transform")
        .def("translation", &Transform::translation, "Get translation vector")
        .def("rotation", &Transform::rotation, "Get rotation matrix")
        .def("as_matrix", &Transform::asMatrix, "Get 4x4 transformation matrix")
        .def("as_position_quaternion", [](const Transform& t) {
            auto [pos, quat] = t.asPositionQuaternion();
            Eigen::Vector4d quat_xyzw;
            quat_xyzw << quat.x(), quat.y(), quat.z(), quat.w();
            return std::make_pair(pos, quat_xyzw);
        }, "Get position and orientation as quaternion (x,y,z,w)")
        .def("as_position_rpy", &Transform::asPositionRPY,
             "Get position and orientation as RPY angles")
        .def("__mul__", [](const Transform& a, const Transform& b) { return a * b; })
        .def("__repr__", [](const Transform& t) {
            auto [pos, rpy] = t.asPositionRPY();
            return "Transform(pos=[" + std::to_string(pos.x()) + ", " + 
                   std::to_string(pos.y()) + ", " + std::to_string(pos.z()) + 
                   "], rpy=[" + std::to_string(rpy.x()) + ", " + 
                   std::to_string(rpy.y()) + ", " + std::to_string(rpy.z()) + "])";
        });

    // 2.5 Data Structures
    nb::class_<JointLimits>(m, "JointLimits")
        .def(nb::init<>())
        .def_rw("lower", &JointLimits::lower)
        .def_rw("upper", &JointLimits::upper)
        .def_rw("effort", &JointLimits::effort)
        .def_rw("velocity", &JointLimits::velocity);

    nb::class_<JointDynamics>(m, "JointDynamics")
        .def(nb::init<>())
        .def_rw("damping", &JointDynamics::damping)
        .def_rw("friction", &JointDynamics::friction);

    nb::class_<Geometry>(m, "Geometry")
        .def(nb::init<>())
        .def_rw("type", &Geometry::type)
        .def_rw("box_size", &Geometry::box_size)
        .def_rw("cylinder_radius", &Geometry::cylinder_radius)
        .def_rw("cylinder_length", &Geometry::cylinder_length)
        .def_rw("sphere_radius", &Geometry::sphere_radius)
        .def_rw("mesh_filename", &Geometry::mesh_filename)
        .def_rw("mesh_scale", &Geometry::mesh_scale);

    nb::class_<Visual>(m, "Visual")
        .def(nb::init<>())
        .def_rw("name", &Visual::name)
        .def_rw("origin", &Visual::origin)
        .def_rw("geometry", &Visual::geometry)
        .def_rw("color", &Visual::color)
        .def_rw("material_name", &Visual::material_name);

    nb::class_<Collision>(m, "Collision")
        .def(nb::init<>())
        .def_rw("name", &Collision::name)
        .def_rw("origin", &Collision::origin)
        .def_rw("geometry", &Collision::geometry);

    nb::class_<Inertial>(m, "Inertial")
        .def(nb::init<>())
        .def_rw("origin", &Inertial::origin)
        .def_rw("mass", &Inertial::mass)
        .def_rw("inertia", &Inertial::inertia);

    // 3.1 Link
    nb::class_<Link>(m, "Link")
        .def(nb::init<const std::string&>())
        .def("get_name", &Link::getName)
        .def("get_inertial", &Link::getInertial)
        .def("set_inertial", &Link::setInertial)
        .def("get_visuals", &Link::getVisuals)
        .def("add_visual", &Link::addVisual)
        .def("get_collisions", &Link::getCollisions)
        .def("add_collision", &Link::addCollision);

    // 3.2 Joint
    nb::class_<Joint>(m, "Joint")
        .def(nb::init<const std::string&, JointType>())
        .def("get_name", &Joint::getName)
        .def("get_type", &Joint::getType)
        .def("get_parent_link", &Joint::getParentLink)
        .def("set_parent_link", &Joint::setParentLink)
        .def("get_child_link", &Joint::getChildLink)
        .def("set_child_link", &Joint::setChildLink)
        .def("get_origin", &Joint::getOrigin)
        .def("set_origin", &Joint::setOrigin)
        .def("get_axis", &Joint::getAxis)
        .def("set_axis", &Joint::setAxis)
        .def("get_limits", &Joint::getLimits)
        .def("set_limits", &Joint::setLimits)
        .def("get_dynamics", &Joint::getDynamics)
        .def("set_dynamics", &Joint::setDynamics)
        .def("is_actuated", &Joint::isActuated)
        .def("get_transform", &Joint::getTransform, "Get transformation for given joint value");

    // 3.3 Robot
    nb::class_<Robot>(m, "Robot")
        .def(nb::init<const std::string&>())
        .def("get_name", &Robot::getName)
        .def("get_root_link", &Robot::getRootLink)
        .def("set_root_link", &Robot::setRootLink)
        .def("add_link", &Robot::addLink)
        .def("get_link", &Robot::getLink)
        .def("get_links", &Robot::getLinks)
        .def("add_joint", &Robot::addJoint)
        .def("get_joint", &Robot::getJoint)
        .def("get_joints", &Robot::getJoints)
        .def("get_actuated_joints", &Robot::getActuatedJoints)
        .def("get_child_joints", &Robot::getChildJoints)
        .def("get_parent_joint", &Robot::getParentJoint)
        .def("validate", &Robot::validate)
        .def_prop_ro("dof", [](const Robot& r) { return r.getActuatedJoints().size(); })
        .def("get_joint_names", [](const Robot& r) {
            std::vector<std::string> names;
            for (const auto& j : r.getActuatedJoints()) {
                names.push_back(j->getName());
            }
            return names;
        })
        .def("get_joint_limits", [](const Robot& r) {
            auto joints = r.getActuatedJoints();
            Eigen::MatrixXd limits(joints.size(), 2);
            for (size_t i = 0; i < joints.size(); ++i) {
                if (auto l = joints[i]->getLimits()) {
                    limits(i, 0) = l->lower;
                    limits(i, 1) = l->upper;
                } else {
                    limits(i, 0) = -std::numeric_limits<double>::infinity();
                    limits(i, 1) = std::numeric_limits<double>::infinity();
                }
            }
            return limits;
        })
        .def_static("from_urdf_file", [](const std::string& path) {
             URDFParser parser;
             return parser.parseFile(path);
        }, "Parse robot from URDF file")
        .def_static("from_urdf_string", [](const std::string& xml) {
             URDFParser parser;
             return parser.parseString(xml);
        }, "Parse robot from URDF string");

    // 3.4 URDFParser
    nb::class_<URDFParser>(m, "URDFParser")
        .def(nb::init<>())
        .def("parse_file", &URDFParser::parseFile)
        .def("parse_string", &URDFParser::parseString)
        .def_prop_rw("base_directory", &URDFParser::getBaseDirectory, &URDFParser::setBaseDirectory);

    // 4. Kinematics
    nb::class_<KinematicChain>(m, "KinematicChain")
        .def(nb::init<std::shared_ptr<const Robot>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("get_num_joints", &KinematicChain::getNumJoints)
        .def("get_joints", &KinematicChain::getJoints)
        .def("get_link_names", &KinematicChain::getLinkNames)
        .def("get_end_link", &KinematicChain::getEndLink)
        .def("get_base_link", &KinematicChain::getBaseLink)
        .def("get_static_transforms", &KinematicChain::getStaticTransforms);

    nb::class_<ForwardKinematics>(m, "ForwardKinematics")
        .def(nb::init<std::shared_ptr<const Robot>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("compute", &ForwardKinematics::compute,
             nb::arg("joint_angles"), nb::arg("check_bounds") = false)
        .def("compute_to_link", &ForwardKinematics::computeToLink,
             nb::arg("joint_angles"), nb::arg("target_link"), nb::arg("check_bounds") = false)
        .def("get_chain", &ForwardKinematics::getChain)
        .def("get_num_joints", &ForwardKinematics::getNumJoints)
        .def_prop_ro("num_joints", &ForwardKinematics::getNumJoints);

    nb::class_<JacobianCalculator>(m, "JacobianCalculator")
        .def(nb::init<std::shared_ptr<const Robot>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("compute", &JacobianCalculator::compute,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("compute_jacobian_derivative", &JacobianCalculator::computeJacobianDerivative,
             nb::arg("joint_angles"), nb::arg("joint_velocities"),
             nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("is_singular", &JacobianCalculator::isSingular,
             nb::arg("joint_angles"), nb::arg("threshold") = 1e-6,
             nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("get_manipulability", &JacobianCalculator::getManipulability,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("get_condition_number", &JacobianCalculator::getConditionNumber,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def_static("convert_jacobian", &JacobianCalculator::convertJacobian,
             nb::arg("jacobian"), nb::arg("pose"), nb::arg("from_type"), nb::arg("to_type"));

    // 5. Inverse Kinematics
    nb::class_<SolverConfig>(m, "SolverConfig")
        .def(nb::init<>())
        .def_rw("max_iterations", &SolverConfig::max_iterations)
        .def_rw("tolerance", &SolverConfig::tolerance)
        .def_rw("regularization", &SolverConfig::regularization)
        .def_rw("max_step_size", &SolverConfig::max_step_size)
        .def_rw("max_line_search_steps", &SolverConfig::max_line_search_steps)
        .def_rw("line_search_shrink", &SolverConfig::line_search_shrink)
        .def_rw("line_search_min_alpha", &SolverConfig::line_search_min_alpha)
        .def_rw("line_search_improvement", &SolverConfig::line_search_improvement)
        .def_rw("position_weight", &SolverConfig::position_weight)
        .def_rw("orientation_weight", &SolverConfig::orientation_weight)
        .def_rw("position_anchor_weight", &SolverConfig::position_anchor_weight)
        .def_rw("orientation_anchor_weight", &SolverConfig::orientation_anchor_weight)
        .def_rw("joint_limit_margin", &SolverConfig::joint_limit_margin)
        .def_rw("unbounded_joint_limit", &SolverConfig::unbounded_joint_limit)
        .def_rw("enable_warm_start", &SolverConfig::enable_warm_start);

    nb::class_<SolverStatus>(m, "SolverStatus")
        .def(nb::init<>())
        .def_rw("converged", &SolverStatus::converged)
        .def_rw("iterations", &SolverStatus::iterations)
        .def_rw("final_error_norm", &SolverStatus::final_error_norm)
        .def_rw("final_step_norm", &SolverStatus::final_step_norm)
        .def_rw("qp_status", &SolverStatus::qp_status)
        .def_rw("message", &SolverStatus::message)
        .def_rw("error_history", &SolverStatus::error_history)
        .def("__repr__", [](const SolverStatus& s) {
             return "SolverStatus(converged=" + std::string(s.converged ? "True" : "False") + 
                    ", iterations=" + std::to_string(s.iterations) + 
                    ", error=" + std::to_string(s.final_error_norm) + ")";
        });

    nb::class_<IKResult>(m, "IKResult")
        .def_rw("status", &IKResult::status)
        .def_rw("solution", &IKResult::solution)
        .def("__repr__", [](const IKResult& r) {
            return "IKResult(status=" + std::string(r.status.converged ? "Converged" : "Failed") + ")";
        });

    nb::class_<IKSolver>(m, "IKSolver")
        .def("set_solver_config", &IKSolver::setSolverConfig)
        .def("get_solver_config", &IKSolver::getSolverConfig)
        .def("set_position_only", &IKSolver::setPositionOnly)
        .def("set_orientation_only", &IKSolver::setOrientationOnly)
        .def("set_warm_start", &IKSolver::setWarmStart)
        .def_prop_rw("tolerance", 
            [](const IKSolver& s) { return s.getSolverConfig().tolerance; },
            [](IKSolver& s, double t) { 
                auto c = s.getSolverConfig(); 
                c.tolerance = t; 
                s.setSolverConfig(c); 
            })
        .def_prop_rw("max_iterations", 
            [](const IKSolver& s) { return s.getSolverConfig().max_iterations; },
            [](IKSolver& s, size_t i) { 
                auto c = s.getSolverConfig(); 
                c.max_iterations = i; 
                s.setSolverConfig(c); 
            });

    nb::class_<SQPIKSolver, IKSolver>(m, "SQPIKSolver")
        .def(nb::init<std::shared_ptr<const Robot>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("solve", [](SQPIKSolver& self, const Transform& target_pose, const Eigen::VectorXd& initial_guess) {
             Eigen::VectorXd solution;
             SolverStatus status = self.solve(target_pose, initial_guess, solution);
             return IKResult{status, solution};
        }, nb::arg("target_pose"), nb::arg("initial_guess"));
}
