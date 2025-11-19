#include <gtest/gtest.h>
#include "urdfx/inverse_kinematics.h"
#include "urdfx/kinematics.h"
#include "urdfx/urdf_parser.h"
#include "urdfx/logging.h"
#include <Eigen/Geometry>

// Define M_PI for MSVC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <filesystem>
#include <future>
#include <random>

using namespace urdfx;

namespace {

std::string resolveEndLink(const std::shared_ptr<Robot>& robot) {
    std::vector<std::string> candidates = {"tool0", "ee_link", "wrist_3_link"};
    for (const auto& name : candidates) {
        if (robot->getLink(name)) {
            return name;
        }
    }
    // Fallback to first non-root link
    for (const auto& link : robot->getLinks()) {
        if (link->getName() != robot->getRootLink()) {
            return link->getName();
        }
    }
    return robot->getRootLink();
}

Transform makeTransform(const Eigen::Vector3d& position, const Eigen::Quaterniond& quaternion) {
    Transform result = Transform::fromPositionQuaternion(position, quaternion.normalized());
    return result;
}

double orientationDistance(const Transform& a, const Transform& b) {
    Eigen::Matrix3d delta = a.rotation().transpose() * b.rotation();
    Eigen::AngleAxisd aa(delta);
    return std::abs(aa.angle());
}

} // namespace

class InverseKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        setLogLevel(spdlog::level::err);
        auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
        urdf_path_ = root / "examples" / "models" / "ur5" / "ur5e.urdf";
        if (!std::filesystem::exists(urdf_path_)) {
            GTEST_SKIP() << "UR5e URDF not available";
        }

        URDFParser parser;
        robot_ = parser.parseFile(urdf_path_.string());
        ASSERT_NE(robot_, nullptr);

        end_link_ = resolveEndLink(robot_);
        base_link_ = robot_->getRootLink();

        fk_ = std::make_unique<ForwardKinematics>(robot_, end_link_, base_link_);
        dof_ = fk_->getNumJoints();
        ASSERT_GT(dof_, 0U);

        lower_limits_.resize(static_cast<Eigen::Index>(dof_));
        upper_limits_.resize(static_cast<Eigen::Index>(dof_));
        const auto& joints = fk_->getChain().getJoints();
        for (size_t i = 0; i < dof_; ++i) {
            if (auto limits = joints[i]->getLimits()) {
                lower_limits_[static_cast<Eigen::Index>(i)] = limits->lower;
                upper_limits_[static_cast<Eigen::Index>(i)] = limits->upper;
            } else {
                lower_limits_[static_cast<Eigen::Index>(i)] = -M_PI;
                upper_limits_[static_cast<Eigen::Index>(i)] = M_PI;
            }
        }
    }

    Eigen::VectorXd clampToLimits(const Eigen::VectorXd& q) const {
        Eigen::VectorXd clamped = q;
        return clamped.cwiseMax(lower_limits_).cwiseMin(upper_limits_);
    }

    Eigen::VectorXd randomConfiguration(double scale = 0.8) {
        Eigen::VectorXd config(static_cast<Eigen::Index>(dof_));
        std::uniform_real_distribution<double> dist(-scale, scale);
        for (size_t i = 0; i < dof_; ++i) {
            double mid = 0.5 * (upper_limits_[static_cast<Eigen::Index>(i)] + lower_limits_[static_cast<Eigen::Index>(i)]);
            double span = 0.5 * (upper_limits_[static_cast<Eigen::Index>(i)] - lower_limits_[static_cast<Eigen::Index>(i)]);
            config[static_cast<Eigen::Index>(i)] = mid + dist(rng_) * span;
        }
        return clampToLimits(config);
    }

    Transform targetFromConfiguration(const Eigen::VectorXd& q) const {
        return fk_->compute(q);
    }

    Eigen::VectorXd zeroConfiguration() const {
        return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));
    }

    std::shared_ptr<Robot> robot_;
    std::unique_ptr<ForwardKinematics> fk_;
    std::string base_link_;
    std::string end_link_;
    size_t dof_ = 0;
    Eigen::VectorXd lower_limits_;
    Eigen::VectorXd upper_limits_;
    std::filesystem::path urdf_path_;
    std::mt19937 rng_{1337u};
};

TEST_F(InverseKinematicsTest, SolveReachablePoseMatchesForwardKinematics) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    Eigen::VectorXd reference(static_cast<Eigen::Index>(dof_));
    reference << 0.1, -1.1, 1.2, -1.4, 0.8, 0.2;
    reference = clampToLimits(reference);
    Transform target = targetFromConfiguration(reference);

    Eigen::VectorXd result;
    auto status = solver.solve(target, zeroConfiguration(), result);
    EXPECT_TRUE(status.converged);
    EXPECT_LT(status.final_error_norm, 1e-3);

    Transform actual = fk_->compute(result);
    EXPECT_LT((actual.translation() - target.translation()).norm(), 1e-4);
    EXPECT_LT(orientationDistance(actual, target), 1e-3);
}

TEST_F(InverseKinematicsTest, WarmStartReducesIterations) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    Eigen::VectorXd reference(static_cast<Eigen::Index>(dof_));
    reference << -0.3, -1.2, 1.5, -1.3, 0.9, -0.2;
    reference = clampToLimits(reference);
    Transform target = targetFromConfiguration(reference);

    Eigen::VectorXd cold;
    auto cold_status = solver.solve(target, zeroConfiguration(), cold);
    EXPECT_TRUE(cold_status.converged);

    Eigen::VectorXd warm;
    auto warm_status = solver.solve(target, Eigen::VectorXd(), warm);
    EXPECT_TRUE(warm_status.converged);
    EXPECT_LE(warm_status.iterations, cold_status.iterations);
    EXPECT_LT((fk_->compute(warm).translation() - target.translation()).norm(), 1e-4);
}

TEST_F(InverseKinematicsTest, RespectsJointLimits) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    Eigen::VectorXd near_bounds = zeroConfiguration();
    near_bounds[0] = upper_limits_[0] - 5e-3;
    if (dof_ > 1) {
        near_bounds[1] = lower_limits_[1] + 0.4;
    }
    if (dof_ > 2) {
        near_bounds[2] = 0.8;
    }
    near_bounds = clampToLimits(near_bounds);
    Transform target = targetFromConfiguration(near_bounds);

    Eigen::VectorXd result;
    auto status = solver.solve(target, zeroConfiguration(), result);
    EXPECT_TRUE(status.converged);

    for (size_t i = 0; i < dof_; ++i) {
        EXPECT_LE(result[static_cast<Eigen::Index>(i)], upper_limits_[static_cast<Eigen::Index>(i)] + 1e-3);
        EXPECT_GE(result[static_cast<Eigen::Index>(i)], lower_limits_[static_cast<Eigen::Index>(i)] - 1e-3);
    }
}

TEST_F(InverseKinematicsTest, PositionOnlyIgnoresOrientationDifferences) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    solver.setPositionOnly(true);

    Eigen::VectorXd reference = clampToLimits(randomConfiguration());
    Transform baseline = targetFromConfiguration(reference);
    Eigen::Isometry3d rotated = baseline.getTransform();
    rotated.linear() = Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitX()).toRotationMatrix() * rotated.linear();
    Transform target(rotated);

    Eigen::VectorXd result;
    auto status = solver.solve(target, zeroConfiguration(), result);
    EXPECT_TRUE(status.converged);

    Transform actual = fk_->compute(result);
    EXPECT_LT((actual.translation() - target.translation()).norm(), 5e-4);
    EXPECT_GT(orientationDistance(actual, target), 1e-2);
}

TEST_F(InverseKinematicsTest, OrientationOnlyAdjustsOrientationWithMinimalTranslation) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    solver.setOrientationOnly(true);

    Eigen::VectorXd reference = clampToLimits(randomConfiguration());
    Transform baseline = targetFromConfiguration(reference);
    Eigen::Isometry3d rotated = baseline.getTransform();
    constexpr double kRotation = M_PI / 12.0;
    rotated.linear() = rotated.linear() * Eigen::AngleAxisd(kRotation, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Transform target(rotated);

    Eigen::VectorXd result;
    auto status = solver.solve(target, reference, result);
    EXPECT_TRUE(status.converged);

    Transform actual = fk_->compute(result);
    EXPECT_LT(orientationDistance(actual, target), 5e-3);
    EXPECT_LT((actual.translation() - baseline.translation()).norm(), 2e-2);
}

TEST_F(InverseKinematicsTest, MultipleInitialGuessesFindDifferentSolutions) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    Eigen::VectorXd goal(static_cast<Eigen::Index>(dof_));
    goal << -0.5, -0.8, 1.2, -1.1, 0.7, 0.3;
    goal = clampToLimits(goal);
    Transform target = targetFromConfiguration(goal);

    Eigen::VectorXd alt = goal;
    alt[2] *= -1.0; // flip elbow to encourage alternate branch
    alt = clampToLimits(alt);

    Eigen::VectorXd first;
    auto s1 = solver.solve(target, goal, first);
    EXPECT_TRUE(s1.converged);

    Eigen::VectorXd second;
    auto s2 = solver.solve(target, alt, second);
    EXPECT_TRUE(s2.converged);

    EXPECT_LT((fk_->compute(first).translation() - target.translation()).norm(), 1e-4);
    EXPECT_LT((fk_->compute(second).translation() - target.translation()).norm(), 1e-4);

    // Different configurations should emerge
    EXPECT_GT((first - second).norm(), 0.3);
}

TEST_F(InverseKinematicsTest, ReportsFailureForUnreachablePose) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    Eigen::Vector3d far_position(5.0, 5.0, 5.0);
    Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
    Transform target = makeTransform(far_position, identity);

    Eigen::VectorXd result;
    auto status = solver.solve(target, zeroConfiguration(), result);
    EXPECT_FALSE(status.converged);
    EXPECT_GT(status.final_error_norm, 1e-2);
}

TEST_F(InverseKinematicsTest, ConvergesForRandomTargets) {
    SQPIKSolver solver(robot_, end_link_, base_link_);
    for (int i = 0; i < 3; ++i) {
        Eigen::VectorXd reference = randomConfiguration();
        Transform target = targetFromConfiguration(reference);
        Eigen::VectorXd result;
        auto status = solver.solve(target, zeroConfiguration(), result);
        EXPECT_TRUE(status.converged);
        EXPECT_LT(status.iterations, solver.getSolverConfig().max_iterations);
        EXPECT_LT(status.final_error_norm, 1e-3);
    }
}

TEST_F(InverseKinematicsTest, ThreadSafeAcrossSolvers) {
    auto task = [&](Eigen::VectorXd seed) {
        SQPIKSolver solver(robot_, end_link_, base_link_);
        Transform target = targetFromConfiguration(seed);
        Eigen::VectorXd result;
        auto status = solver.solve(target, zeroConfiguration(), result);
        return status.converged;
    };

    Eigen::VectorXd seed1 = randomConfiguration();
    Eigen::VectorXd seed2 = randomConfiguration();

    auto fut1 = std::async(std::launch::async, task, seed1);
    auto fut2 = std::async(std::launch::async, task, seed2);

    EXPECT_TRUE(fut1.get());
    EXPECT_TRUE(fut2.get());
}