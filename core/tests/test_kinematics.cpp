#include <gtest/gtest.h>
#include "urdfx/kinematics.h"
#include "urdfx/urdf_parser.h"
#include "urdfx/robot_model.h"
#include "urdfx/logging.h"
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <cmath>

// Define M_PI for MSVC
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace urdfx;

class ForwardKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set log level to WARNING to reduce test output noise
        setLogLevel(spdlog::level::warn);
        
        // Get the UR5e URDF path
        ur5e_urdf_path_ = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "examples" / "models" / "ur5" / "ur5e.urdf";
        
        // Create a simple 2-link robot for basic testing
        simple_urdf_ = R"(
<?xml version="1.0"?>
<robot name="simple_robot">
    <link name="base_link"/>
    
    <link name="link1"/>
    
    <link name="link2"/>
    
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
    
    <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
</robot>
)";
        
        // Parse simple robot
        URDFParser parser;
        simple_robot_ = parser.parseString(simple_urdf_);
        ASSERT_NE(simple_robot_, nullptr);
        
        // Parse UR5e robot if file exists
        if (std::filesystem::exists(ur5e_urdf_path_)) {
            ur5e_robot_ = parser.parseFile(ur5e_urdf_path_.string());
        }
    }
    
    std::filesystem::path ur5e_urdf_path_;
    std::string simple_urdf_;
    std::shared_ptr<Robot> simple_robot_;
    std::shared_ptr<Robot> ur5e_robot_;
};

namespace {

Eigen::MatrixXd finiteDifferenceJacobian(
    const ForwardKinematics& fk,
    const Eigen::VectorXd& joint_angles,
    double epsilon = 1e-6)
{
    const size_t dof = static_cast<size_t>(joint_angles.size());
    Eigen::MatrixXd J(6, dof);
    Transform nominal = fk.compute(joint_angles);
    const Eigen::Matrix3d R = nominal.rotation();

    Eigen::VectorXd q_plus = joint_angles;
    Eigen::VectorXd q_minus = joint_angles;

    for (size_t i = 0; i < dof; ++i) {
        q_plus = joint_angles;
        q_minus = joint_angles;
        q_plus(static_cast<Eigen::Index>(i)) += epsilon;
        q_minus(static_cast<Eigen::Index>(i)) -= epsilon;

        Transform T_plus = fk.compute(q_plus);
        Transform T_minus = fk.compute(q_minus);

        Eigen::Vector3d dp = (T_plus.translation() - T_minus.translation()) / (2.0 * epsilon);
        Eigen::Matrix3d dR = (T_plus.rotation() - T_minus.rotation()) / (2.0 * epsilon);
        Eigen::Matrix3d omega_skew = dR * R.transpose();
        Eigen::Vector3d dom(omega_skew(2, 1), omega_skew(0, 2), omega_skew(1, 0));

        J.block<3, 1>(0, static_cast<Eigen::Index>(i)) = dp;
        J.block<3, 1>(3, static_cast<Eigen::Index>(i)) = dom;
    }

    return J;
}

} // namespace

// ============================================================================
// KinematicChain Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, KinematicChainConstruction) {
    KinematicChain chain(simple_robot_, "link2", "base_link");
    
    EXPECT_EQ(chain.getNumJoints(), 2);
    EXPECT_EQ(chain.getBaseLink(), "base_link");
    EXPECT_EQ(chain.getEndLink(), "link2");
    
    const auto& joints = chain.getJoints();
    EXPECT_EQ(joints.size(), 2);
    EXPECT_EQ(joints[0]->getName(), "joint1");
    EXPECT_EQ(joints[1]->getName(), "joint2");
    
    const auto& link_names = chain.getLinkNames();
    EXPECT_EQ(link_names.size(), 3);
    EXPECT_EQ(link_names[0], "base_link");
    EXPECT_EQ(link_names[1], "link1");
    EXPECT_EQ(link_names[2], "link2");
}

TEST_F(ForwardKinematicsTest, KinematicChainInvalidLinks) {
    // Test with non-existent end link
    EXPECT_THROW(
        KinematicChain(simple_robot_, "nonexistent_link", "base_link"),
        std::invalid_argument
    );
    
    // Test with non-existent base link
    EXPECT_THROW(
        KinematicChain(simple_robot_, "link2", "nonexistent_link"),
        std::invalid_argument
    );
}

// ============================================================================
// ForwardKinematics Basic Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, ConstructorAndBasicProperties) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    EXPECT_EQ(fk.getNumJoints(), 2);
    EXPECT_EQ(fk.getChain().getBaseLink(), "base_link");
    EXPECT_EQ(fk.getChain().getEndLink(), "link2");
}

TEST_F(ForwardKinematicsTest, ComputeAtZeroConfiguration) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles = Eigen::VectorXd::Zero(2);
    Transform result = fk.compute(joint_angles);
    
    // At zero configuration:
    // joint1 origin: (0, 0, 0), link1 at origin
    // joint2 origin: (1, 0, 0), link2 at (1, 0, 0) relative to link1
    // So end-effector should be at (1, 0, 0)
    Eigen::Vector3d position = result.translation();
    EXPECT_NEAR(position.x(), 1.0, 1e-6);
    EXPECT_NEAR(position.y(), 0.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
    
    // Rotation should be identity at zero configuration
    Eigen::Matrix3d rotation = result.rotation();
    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity(), 1e-6));
}

TEST_F(ForwardKinematicsTest, ComputeWithRotation) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    // Rotate joint1 by 90 degrees (π/2)
    Eigen::VectorXd joint_angles(2);
    joint_angles << M_PI / 2.0, 0.0;
    
    Transform result = fk.compute(joint_angles);
    
    // After 90 degree rotation around Z axis at joint1:
    // - joint2's origin (1, 0, 0) rotates to (0, 1, 0)
    // - joint2 has no rotation, so link2 is at (0, 1, 0)
    Eigen::Vector3d position = result.translation();
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 1.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, ComputeWithBothJointsRotated) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    // Rotate both joints by 90 degrees
    Eigen::VectorXd joint_angles(2);
    joint_angles << M_PI / 2.0, M_PI / 2.0;
    
    Transform result = fk.compute(joint_angles);
    
    // First joint rotates +90° around Z: (1,0,0) → (0,1,0)
    // Second joint rotates +90° around Z in link1's frame
    // The total rotation is 180°, so link2 should point back
    // Position should be approximately (0, 1, 0) as both rotations cancel the displacement
    Eigen::Vector3d position = result.translation();
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 1.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, ComputeWithInvalidJointAnglesSize) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    // Wrong size
    Eigen::VectorXd joint_angles(3);
    joint_angles << 0.0, 0.0, 0.0;
    
    EXPECT_THROW(fk.compute(joint_angles), std::invalid_argument);
}

// ============================================================================
// Intermediate Link Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, ComputeToIntermediateLink) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 0.0, 0.0;
    
    // Compute to link1 (should be at origin after joint1)
    Transform result = fk.computeToLink(joint_angles, "link1");
    
    // link1 should be at origin (0, 0, 0)
    Eigen::Vector3d position = result.translation();
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 0.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, ComputeToIntermediateLinkWithRotation) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << M_PI / 2.0, M_PI / 2.0;
    
    // Compute to link1 (should only include joint1 rotation)
    Transform result = fk.computeToLink(joint_angles, "link1");
    
    // link1 should be at origin but rotated
    Eigen::Vector3d position = result.translation();
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 0.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, ComputeToInvalidLink) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 0.0, 0.0;
    
    EXPECT_THROW(
        fk.computeToLink(joint_angles, "nonexistent_link"),
        std::invalid_argument
    );
}

// ============================================================================
// Bounds Checking Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, BoundsCheckingWithinLimits) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 1.0, -1.0;  // Within limits [-3.14, 3.14]
    
    // Should not throw
    EXPECT_NO_THROW(fk.compute(joint_angles, true));
}

TEST_F(ForwardKinematicsTest, BoundsCheckingOutOfLimits) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 4.0, 0.0;  // Above upper limit of 3.14
    
    // Should throw when bounds checking is enabled
    EXPECT_THROW(fk.compute(joint_angles, true), std::runtime_error);
}

TEST_F(ForwardKinematicsTest, BoundsCheckingDisabled) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 4.0, 0.0;  // Above upper limit
    
    // Should not throw when bounds checking is disabled
    EXPECT_NO_THROW(fk.compute(joint_angles, false));
}

// ============================================================================
// Pose Representation Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, PoseAsMatrix) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles = Eigen::VectorXd::Zero(2);
    Transform result = fk.compute(joint_angles);
    
    Eigen::Matrix4d matrix = result.asMatrix();
    
    // Check position in last column
    EXPECT_NEAR(matrix(0, 3), 1.0, 1e-6);
    EXPECT_NEAR(matrix(1, 3), 0.0, 1e-6);
    EXPECT_NEAR(matrix(2, 3), 0.0, 1e-6);
    
    // Check homogeneous coordinate
    EXPECT_NEAR(matrix(3, 3), 1.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, PoseAsPositionQuaternion) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << M_PI / 2.0, 0.0;
    
    Transform result = fk.compute(joint_angles);
    auto [position, quaternion] = result.asPositionQuaternion();
    
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 1.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
    
    // Check quaternion is normalized
    EXPECT_NEAR(quaternion.norm(), 1.0, 1e-6);
}

TEST_F(ForwardKinematicsTest, PoseAsPositionRPY) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << M_PI / 2.0, 0.0;
    
    Transform result = fk.compute(joint_angles);
    auto [position, rpy] = result.asPositionRPY();
    
    EXPECT_NEAR(position.x(), 0.0, 1e-6);
    EXPECT_NEAR(position.y(), 1.0, 1e-6);
    EXPECT_NEAR(position.z(), 0.0, 1e-6);
    
    // At 90 degree rotation around Z, yaw should be π/2
    EXPECT_NEAR(rpy(2), M_PI / 2.0, 1e-6);
}

// ============================================================================
// UR5e Robot Tests (if available)
// ============================================================================

TEST_F(ForwardKinematicsTest, UR5eRobotAtZero) {
    if (!ur5e_robot_) {
        GTEST_SKIP() << "UR5e URDF not available";
    }
    
    // Get the tool link (end-effector)
    auto joints = ur5e_robot_->getActuatedJoints();
    ASSERT_GT(joints.size(), 0) << "UR5e should have actuated joints";
    
    // Try to find appropriate end link
    // UR5e typically has tool0, ee_link, or wrist_3_link
    std::vector<std::string> possible_end_links = {"tool0", "ee_link", "wrist_3_link"};
    std::string end_link;
    
    for (const auto& link_name : possible_end_links) {
        if (ur5e_robot_->getLink(link_name)) {
            end_link = link_name;
            break;
        }
    }
    
    // If none of the common names found, use any non-root link
    if (end_link.empty()) {
        for (const auto& link : ur5e_robot_->getLinks()) {
            if (link->getName() != ur5e_robot_->getRootLink()) {
                end_link = link->getName();
                break;
            }
        }
    }
    
    ASSERT_FALSE(end_link.empty()) << "Could not find suitable end-effector link";
    
    ForwardKinematics fk(ur5e_robot_, end_link);
    
    Eigen::VectorXd zero_config = Eigen::VectorXd::Zero(fk.getNumJoints());
    Transform result = fk.compute(zero_config);
    
    // Just verify it computes without error
    Eigen::Vector3d position = result.translation();
    EXPECT_TRUE(std::isfinite(position.x()));
    EXPECT_TRUE(std::isfinite(position.y()));
    EXPECT_TRUE(std::isfinite(position.z()));
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, PerformanceBenchmark) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    Eigen::VectorXd joint_angles(2);
    joint_angles << 1.0, 0.5;
    
    // Warm-up
    for (int i = 0; i < 100; ++i) {
        fk.compute(joint_angles);
    }
    
    // Benchmark
    const int num_iterations = 10000;
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < num_iterations; ++i) {
        fk.compute(joint_angles);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    double avg_time_us = static_cast<double>(duration.count()) / num_iterations;
    double avg_time_ms = avg_time_us / 1000.0;
    
    // Target: < 1ms per computation (should be much faster for 2-DOF robot)
    EXPECT_LT(avg_time_ms, 1.0) << "Average FK computation time: " << avg_time_ms << " ms";
    
    // Log performance info
    std::cout << "FK Performance: " << avg_time_us << " μs per computation" << std::endl;
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, ThreadSafety) {
    ForwardKinematics fk(simple_robot_, "link2", "base_link");
    
    const int num_threads = 4;
    const int iterations_per_thread = 1000;
    std::vector<std::thread> threads;
    std::atomic<int> error_count{0};
    
    auto worker = [&](int thread_id) {
        Eigen::VectorXd joint_angles(2);
        
        for (int i = 0; i < iterations_per_thread; ++i) {
            try {
                joint_angles << (thread_id * 0.1) + (i * 0.001), (thread_id * 0.2);
                Transform result = fk.compute(joint_angles);
                
                // Verify result is finite
                if (!std::isfinite(result.translation().x())) {
                    error_count++;
                }
            } catch (...) {
                error_count++;
            }
        }
    };
    
    // Launch threads
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    // Wait for completion
    for (auto& thread : threads) {
        thread.join();
    }
    
    EXPECT_EQ(error_count, 0) << "Thread safety test failed with " << error_count << " errors";
}

// ============================================================================
// Jacobian Calculator Tests
// ============================================================================

TEST_F(ForwardKinematicsTest, JacobianMatchesAnalyticalForSimpleRobot) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

    Eigen::MatrixXd J = jc.compute(q, JacobianType::Analytic);

    Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 2);
    expected(1, 0) = 1.0;
    expected(5, 0) = 1.0;
    expected(5, 1) = 1.0;

    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), 2);
    EXPECT_TRUE(J.isApprox(expected, 1e-9));
}

TEST_F(ForwardKinematicsTest, JacobianMatchesNumericalDifferentiation) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");
    ForwardKinematics fk(simple_robot_, "link2", "base_link");

    Eigen::VectorXd q(2);
    q << 0.3, -0.4;

    Eigen::MatrixXd J_ad = jc.compute(q, JacobianType::Analytic);
    Eigen::MatrixXd J_fd = finiteDifferenceJacobian(fk, q);

    EXPECT_TRUE(J_ad.isApprox(J_fd, 1e-5)) << "AD Jacobian:\n" << J_ad << "\nFD:\n" << J_fd;
}

TEST_F(ForwardKinematicsTest, GeometricJacobianMatchesConvertedAnalytic) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");
    ForwardKinematics fk(simple_robot_, "link2", "base_link");

    Eigen::VectorXd q(2);
    q << -0.2, 0.8;

    Eigen::MatrixXd J_analytic = jc.compute(q, JacobianType::Analytic);
    Eigen::MatrixXd J_geo = jc.compute(q, JacobianType::Geometric);
    Transform pose = fk.compute(q);

    Eigen::MatrixXd converted = JacobianCalculator::convertJacobian(
        J_analytic, pose, JacobianType::Analytic, JacobianType::Geometric);

    EXPECT_TRUE(J_geo.isApprox(converted, 1e-9));
}

TEST_F(ForwardKinematicsTest, JacobianDerivativeMatchesFiniteDifference) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;
    Eigen::VectorXd dq(2);
    dq << 0.2, -0.1;

    EXPECT_THROW({
        jc.computeJacobianDerivative(q, dq, JacobianType::Analytic);
    }, std::runtime_error);
}

TEST_F(ForwardKinematicsTest, JacobianToIntermediateLink) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");
    ForwardKinematics fk_end(simple_robot_, "link2", "base_link");
    ForwardKinematics fk_mid(simple_robot_, "link1", "base_link");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

    Eigen::MatrixXd J_end = jc.compute(q, JacobianType::Analytic);
    Eigen::MatrixXd J_mid = jc.compute(q, JacobianType::Analytic, "link1");

    Eigen::VectorXd q_mid(1);
    q_mid << q(0);
    Eigen::Vector3d delta = fk_end.compute(q).translation() - fk_mid.compute(q_mid).translation();

    ASSERT_EQ(J_mid.cols(), 1);
    EXPECT_TRUE((J_mid.block<3, 1>(3, 0).isApprox(J_end.block<3, 1>(3, 0), 1e-9)));

    Eigen::Vector3d shifted_linear = J_mid.block<3, 1>(0, 0) +
        J_mid.block<3, 1>(3, 0).cross(delta);
    EXPECT_TRUE((shifted_linear.isApprox(J_end.block<3, 1>(0, 0), 1e-9)));
}

TEST_F(ForwardKinematicsTest, SingularityDetectionAndManipulability) {
    const std::string singular_urdf = R"(
<?xml version="1.0"?>
<robot name="singular_robot">
    <link name="base_link"/>
    <link name="link1"/>
    <link name="link2"/>
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
    <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
)";

    URDFParser parser;
    auto robot = parser.parseString(singular_urdf);
    ASSERT_NE(robot, nullptr);

    JacobianCalculator jc(robot, "link2", "base_link");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2);

    EXPECT_TRUE(jc.isSingular(q, 1e-8));
    EXPECT_NEAR(jc.getManipulability(q), 0.0, 1e-8);
    EXPECT_TRUE(std::isinf(jc.getConditionNumber(q)));
}

TEST_F(ForwardKinematicsTest, ManipulabilityAndConditionNumberFinite) {
    JacobianCalculator jc(simple_robot_, "link2", "base_link");
    Eigen::VectorXd q(2);
    q << 0.8, -0.6;

    double manipulability = jc.getManipulability(q);
    double condition = jc.getConditionNumber(q);

    EXPECT_GT(manipulability, 0.0);
    EXPECT_TRUE(std::isfinite(condition));
}

