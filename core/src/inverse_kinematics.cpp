#include "urdfx/inverse_kinematics.h"
#include "api.h"
#include "constants.h"
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace urdfx {

class DaQPSolver {
public:
    explicit DaQPSolver(size_t dof)
        : dof_(static_cast<int>(dof))
    {
        result_.x = nullptr;
        result_.lam = nullptr;
    }

    bool solve(
        const Eigen::MatrixXd& H,
        const Eigen::VectorXd& g,
        const Eigen::VectorXd& lower,
        const Eigen::VectorXd& upper,
        Eigen::VectorXd& delta,
        int& exitflag)
    {
        if (dof_ == 0) {
            exitflag = EXIT_INFEASIBLE;
            return false;
        }

        if (lower.size() != dof_ || upper.size() != dof_) {
            throw std::invalid_argument("DaQPSolver bounds size mismatch");
        }

        if (delta.size() != dof_) {
            delta.resize(dof_);
        }

        lower_.assign(lower.data(), lower.data() + dof_);
        upper_.assign(upper.data(), upper.data() + dof_);
        x_.assign(dof_, 0.0);
        lam_.assign(dof_, 0.0);

        DAQPProblem problem{};
        problem.n = dof_;
        problem.m = dof_;
        problem.ms = dof_;
        problem.H = const_cast<double*>(H.data());
        problem.f = const_cast<double*>(g.data());
        problem.A = nullptr;
        problem.bupper = upper_.data();
        problem.blower = lower_.data();
        problem.sense = nullptr;
        problem.break_points = nullptr;
        problem.nh = 0;

        result_.x = x_.data();
        result_.lam = lam_.data();
        result_.fval = 0.0;
        result_.soft_slack = 0.0;
        result_.exitflag = EXIT_ITERLIMIT;
        result_.iter = 0;
        result_.nodes = 0;
        result_.solve_time = 0.0;
        result_.setup_time = 0.0;

        daqp_quadprog(&result_, &problem, nullptr);
        exitflag = result_.exitflag;

        if (exitflag >= EXIT_OPTIMAL) {
            Eigen::Map<const Eigen::VectorXd> mapped(result_.x, dof_);
            delta = mapped;
            return true;
        }

        return false;
    }

private:
    int dof_;
    DAQPResult result_{};
    std::vector<double> lower_;
    std::vector<double> upper_;
    std::vector<double> x_;
    std::vector<double> lam_;
};

namespace {

Eigen::Vector3d orientationError(
    const Eigen::Matrix3d& current,
    const Eigen::Matrix3d& target)
{
    Eigen::Matrix3d R_err = current.transpose() * target;
    Eigen::AngleAxisd axis_angle(R_err);
    double angle = axis_angle.angle();
    if (std::abs(angle) < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d axis = axis_angle.axis();
    return current * (axis * angle);
}

} // namespace

IKSolver::IKSolver(
    std::shared_ptr<const Robot> robot,
    std::string end_link,
    std::string base_link)
    : robot_(std::move(robot))
    , end_link_(std::move(end_link))
    , base_link_(std::move(base_link))
{
    if (!robot_) {
        throw std::invalid_argument("IKSolver requires valid robot pointer");
    }

    if (base_link_.empty()) {
        base_link_ = robot_->getRootLink();
    }
    if (end_link_.empty()) {
        end_link_ = robot_->getRootLink();
    }

    if (!robot_->getLink(base_link_)) {
        throw std::invalid_argument("Base link not present in robot model");
    }
    if (!robot_->getLink(end_link_)) {
        throw std::invalid_argument("End link not present in robot model");
    }
}

void IKSolver::setSolverConfig(const SolverConfig& config) {
    config_ = config;
}

void IKSolver::setPositionOnly(bool enable) {
    if (enable && orientation_only_) {
        throw std::invalid_argument("Cannot enable position-only and orientation-only simultaneously");
    }
    position_only_ = enable;
}

void IKSolver::setOrientationOnly(bool enable) {
    if (enable && position_only_) {
        throw std::invalid_argument("Cannot enable position-only and orientation-only simultaneously");
    }
    orientation_only_ = enable;
}

void IKSolver::setWarmStart(const Eigen::VectorXd& guess) {
    warm_start_ = guess;
}

SQPIKSolver::SQPIKSolver(
    std::shared_ptr<const Robot> robot,
    const std::string& end_link,
    const std::string& base_link)
    : IKSolver(std::move(robot), end_link, base_link)
    , fk_(robot_, end_link_, base_link_)
    , jacobian_(robot_, end_link_, base_link_)
    , qp_solver_(std::make_unique<DaQPSolver>(fk_.getNumJoints()))
{
}

SQPIKSolver::~SQPIKSolver() = default;

Eigen::VectorXd SQPIKSolver::buildTaskError(
    const Transform& current_pose,
    const Transform& target_pose) const
{
    Eigen::VectorXd error(6);
    error.head<3>() = target_pose.translation() - current_pose.translation();
    error.tail<3>() = orientationError(current_pose.rotation(), target_pose.rotation());
    return error;
}

Eigen::VectorXd SQPIKSolver::weightError(const Eigen::VectorXd& full_error) const {
    Eigen::VectorXd weighted = full_error;
    const double pos_weight = orientation_only_ ? config_.position_anchor_weight : config_.position_weight;
    const double ori_weight = position_only_ ? config_.orientation_anchor_weight : config_.orientation_weight;
    weighted.head<3>() *= pos_weight;
    weighted.tail<3>() *= ori_weight;
    return weighted;
}

Eigen::MatrixXd SQPIKSolver::weightJacobian(const Eigen::MatrixXd& full_jacobian) const {
    Eigen::MatrixXd weighted = full_jacobian;
    const double pos_weight = orientation_only_ ? config_.position_anchor_weight : config_.position_weight;
    const double ori_weight = position_only_ ? config_.orientation_anchor_weight : config_.orientation_weight;
    weighted.topRows(3) *= pos_weight;
    weighted.bottomRows(3) *= ori_weight;
    return weighted;
}

void SQPIKSolver::computeJointBounds(
    Eigen::VectorXd& lower,
    Eigen::VectorXd& upper) const
{
    const auto& joints = fk_.getChain().getJoints();
    const size_t dof = joints.size();
    lower.resize(static_cast<Eigen::Index>(dof));
    upper.resize(static_cast<Eigen::Index>(dof));
    for (size_t i = 0; i < dof; ++i) {
        const auto& joint = joints[i];
        const auto& limits = joint->getLimits();
        if (limits) {
            lower[static_cast<Eigen::Index>(i)] = limits->lower - config_.joint_limit_margin;
            upper[static_cast<Eigen::Index>(i)] = limits->upper + config_.joint_limit_margin;
        } else {
            lower[static_cast<Eigen::Index>(i)] = -config_.unbounded_joint_limit;
            upper[static_cast<Eigen::Index>(i)] = config_.unbounded_joint_limit;
        }
    }
}

void SQPIKSolver::clampToJointLimits(Eigen::VectorXd& joints) const {
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
    computeJointBounds(lower, upper);
    joints = joints.cwiseMax(lower).cwiseMin(upper);
}

SolverStatus SQPIKSolver::solve(
    const Transform& target_pose,
    const Eigen::VectorXd& initial_guess,
    Eigen::VectorXd& solution)
{
    SolverStatus status;
    const size_t dof = fk_.getNumJoints();
    if (dof == 0) {
        status.message = "No actuated joints in chain";
        return status;
    }

    Eigen::VectorXd current;
    if (initial_guess.size() == static_cast<Eigen::Index>(dof)) {
        current = initial_guess;
    } else if (initial_guess.size() == 0 && warm_start_ && warm_start_->size() == static_cast<Eigen::Index>(dof)) {
        current = *warm_start_;
    } else if (initial_guess.size() == 0) {
        current = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof));
    } else {
        throw std::invalid_argument("Initial guess dimension mismatch");
    }

    clampToJointLimits(current);

    Eigen::VectorXd lower_bounds;
    Eigen::VectorXd upper_bounds;
    computeJointBounds(lower_bounds, upper_bounds);

    Eigen::VectorXd weighted_error(6);
    Eigen::MatrixXd weighted_jac(6, static_cast<Eigen::Index>(dof));
    Eigen::VectorXd delta(static_cast<Eigen::Index>(dof));

    status.error_history.reserve(config_.max_iterations);

    for (size_t iter = 0; iter < config_.max_iterations; ++iter) {
        Transform current_pose = fk_.compute(current);
        weighted_error = weightError(buildTaskError(current_pose, target_pose));
        double error_norm = weighted_error.norm();
        status.iterations = iter + 1;
        status.error_history.push_back(error_norm);

        if (error_norm < config_.tolerance) {
            status.converged = true;
            status.final_error_norm = error_norm;
            break;
        }

        Eigen::MatrixXd full_jacobian = jacobian_.compute(current, JacobianType::Analytic);
        weighted_jac = weightJacobian(full_jacobian);

        Eigen::MatrixXd H = weighted_jac.transpose() * weighted_jac;
        H.diagonal().array() += config_.regularization;
        Eigen::VectorXd g = -weighted_jac.transpose() * weighted_error;

        Eigen::VectorXd lower_delta = lower_bounds - current;
        Eigen::VectorXd upper_delta = upper_bounds - current;

        bool qp_ok = qp_solver_ && qp_solver_->solve(H, g, lower_delta, upper_delta, delta, status.qp_status);
        if (!qp_ok) {
            status.qp_status = EXIT_NONCONVEX;
            delta = H.ldlt().solve(-g);
            status.message = "Fallback damping step";
        }

        double step_norm = delta.norm();
        status.final_step_norm = step_norm;
        if (step_norm > config_.max_step_size && step_norm > 1e-12) {
            delta *= config_.max_step_size / step_norm;
        }

        double alpha = 1.0;
        bool accepted = false;
        double current_norm = error_norm;
        for (size_t ls = 0; ls < config_.max_line_search_steps; ++ls) {
            if (alpha < config_.line_search_min_alpha) {
                break;
            }
            Eigen::VectorXd candidate = current + alpha * delta;
            candidate = candidate.cwiseMax(lower_bounds).cwiseMin(upper_bounds);
            Transform pose_candidate = fk_.compute(candidate);
            Eigen::VectorXd candidate_error = weightError(buildTaskError(pose_candidate, target_pose));
            double candidate_norm = candidate_error.norm();
            if (candidate_norm + config_.line_search_improvement < current_norm) {
                current = candidate;
                error_norm = candidate_norm;
                accepted = true;
                break;
            }
            alpha *= config_.line_search_shrink;
        }

        if (!accepted) {
            current += delta;
            current = current.cwiseMax(lower_bounds).cwiseMin(upper_bounds);
        }

        status.final_error_norm = error_norm;
    }

    solution = current;
    if (config_.enable_warm_start) {
        warm_start_ = current;
    }

    if (!status.converged) {
        Transform final_pose = fk_.compute(current);
        status.final_error_norm = weightError(buildTaskError(final_pose, target_pose)).norm();
        status.message = "Maximum iterations reached";
    } else {
        status.message = "IK converged";
    }

    return status;
}

} // namespace urdfx
