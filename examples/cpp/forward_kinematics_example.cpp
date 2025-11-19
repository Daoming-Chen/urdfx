#include "urdfx/urdf_parser.h"
#include "urdfx/kinematics.h"
#include "urdfx/logging.h"
#include <iostream>
#include <iomanip>

using namespace urdfx;

void printTransform(const Transform& transform) {
    auto [position, quaternion] = transform.asPositionQuaternion();
    
    std::cout << "Position: [" 
              << std::fixed << std::setprecision(4)
              << position.x() << ", "
              << position.y() << ", "
              << position.z() << "]" << std::endl;
    
    std::cout << "Quaternion (w,x,y,z): ["
              << quaternion.w() << ", "
              << quaternion.x() << ", "
              << quaternion.y() << ", "
              << quaternion.z() << "]" << std::endl;
    
    auto [pos_rpy, rpy] = transform.asPositionRPY();
    std::cout << "RPY: ["
              << rpy.x() << ", "
              << rpy.y() << ", "
              << rpy.z() << "] rad" << std::endl;
}

int main(int argc, char** argv) {
    // Set logging level
    setLogLevel(spdlog::level::info);
    
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <urdf_file>" << std::endl;
        return 1;
    }
    
    try {
        // Parse URDF file
        std::string urdf_path = argv[1];
        URDFX_LOG_INFO("Loading URDF file: {}", urdf_path);
        
        URDFParser parser;
        auto robot = parser.parseFile(urdf_path);
        
        URDFX_LOG_INFO("Robot '{}' loaded successfully", robot->getName());
        URDFX_LOG_INFO("Number of links: {}", robot->getLinks().size());
        URDFX_LOG_INFO("Number of joints: {}", robot->getJoints().size());
        
        // Get actuated joints
        auto actuated_joints = robot->getActuatedJoints();
        URDFX_LOG_INFO("Number of actuated joints: {}", actuated_joints.size());
        
        // Find end-effector link
        std::string end_link;
        std::vector<std::string> possible_end_links = {"tool0", "ee_link", "wrist_3_link"};
        
        for (const auto& link_name : possible_end_links) {
            if (robot->getLink(link_name)) {
                end_link = link_name;
                break;
            }
        }
        
        if (end_link.empty()) {
            // Use last link in chain
            for (const auto& link : robot->getLinks()) {
                if (link->getName() != robot->getRootLink() && 
                    robot->getChildJoints(link->getName()).empty()) {
                    end_link = link->getName();
                    break;
                }
            }
        }
        
        if (end_link.empty()) {
            URDFX_LOG_ERROR("Could not find end-effector link");
            return 1;
        }
        
        URDFX_LOG_INFO("Using end-effector link: {}", end_link);
        
        // Create forward kinematics solver
        ForwardKinematics fk(robot, end_link);
        URDFX_LOG_INFO("Forward kinematics solver created with {} DOF", fk.getNumJoints());
        
        // Test at zero configuration
        std::cout << "\n=== Zero Configuration ===" << std::endl;
        Eigen::VectorXd zero_config = Eigen::VectorXd::Zero(fk.getNumJoints());
        Transform zero_pose = fk.compute(zero_config);
        printTransform(zero_pose);
        
        // Test at random configuration
        std::cout << "\n=== Random Configuration ===" << std::endl;
        Eigen::VectorXd random_config(fk.getNumJoints());
        std::cout << "Joint angles: [";
        for (size_t i = 0; i < fk.getNumJoints(); ++i) {
            random_config[i] = (i + 1) * 0.1;
            std::cout << random_config[i];
            if (i < fk.getNumJoints() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        Transform random_pose = fk.compute(random_config);
        printTransform(random_pose);
        
        // Performance test
        std::cout << "\n=== Performance Test ===" << std::endl;
        const int num_iterations = 100000;
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < num_iterations; ++i) {
            fk.compute(random_config);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double avg_time_us = static_cast<double>(duration.count()) / num_iterations;
        
        std::cout << "Average FK computation time: " << avg_time_us << " μs" << std::endl;
        std::cout << "Throughput: " << (1000000.0 / avg_time_us) << " FK/second" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        URDFX_LOG_ERROR("Error: {}", e.what());
        return 1;
    }
}
