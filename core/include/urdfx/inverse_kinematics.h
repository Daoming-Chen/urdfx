#pragma once

#include "urdfx/export.h"
#include "urdfx/kinematics.h"
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace urdfx {

class DaQPSolver;

struct SolverConfig {
    size_t max_iterations = 64;
    double tolerance = 1e-4;
    double regularization = 1e-5;
    double max_step_size = 0.5;
    size_t max_line_search_steps = 6;
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

struct SolverStatus {
    bool converged = false;
    size_t iterations = 0;
    double final_error_norm = 0.0;
    double final_step_norm = 0.0;
    int qp_status = 0;
    std::string message;
    std::vector<double> error_history;
};

class URDFX_API IKSolver {
public:
    IKSolver(
        std::shared_ptr<const Robot> robot,
        std::string end_link,
        std::string base_link = "");
    virtual ~IKSolver() = default;

    virtual SolverStatus solve(
        const Transform& target_pose,
        const Eigen::VectorXd& initial_guess,
        Eigen::VectorXd& solution) = 0;

    void setSolverConfig(const SolverConfig& config);
    const SolverConfig& getSolverConfig() const { return config_; }

    std::shared_ptr<const Robot> getRobot() const { return robot_; }
    const std::string& getEndLink() const { return end_link_; }
    const std::string& getBaseLink() const { return base_link_; }

    void setPositionOnly(bool enable);
    void setOrientationOnly(bool enable);

    void setWarmStart(const Eigen::VectorXd& guess);
    std::optional<Eigen::VectorXd> getWarmStart() const { return warm_start_; }

protected:
    std::shared_ptr<const Robot> robot_;
    std::string end_link_;
    std::string base_link_;
    SolverConfig config_;
    bool position_only_ = false;
    bool orientation_only_ = false;
    std::optional<Eigen::VectorXd> warm_start_;
};

class URDFX_API SQPIKSolver final : public IKSolver {
public:
    SQPIKSolver(
        std::shared_ptr<const Robot> robot,
        const std::string& end_link,
        const std::string& base_link = "");

    ~SQPIKSolver() override;

    SolverStatus solve(
        const Transform& target_pose,
        const Eigen::VectorXd& initial_guess,
        Eigen::VectorXd& solution) override;

private:
    ForwardKinematics fk_;
    JacobianCalculator jacobian_;
    std::unique_ptr<class DaQPSolver> qp_solver_;

    Eigen::VectorXd buildTaskError(
        const Transform& current_pose,
        const Transform& target_pose) const;

    Eigen::VectorXd weightError(const Eigen::VectorXd& full_error) const;
    Eigen::MatrixXd weightJacobian(const Eigen::MatrixXd& full_jacobian) const;

    void computeJointBounds(
        Eigen::VectorXd& lower,
        Eigen::VectorXd& upper) const;

    void clampToJointLimits(Eigen::VectorXd& joints) const;
};

} // namespace urdfx
