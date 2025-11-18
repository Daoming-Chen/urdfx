#include <gtest/gtest.h>
#include "urdfx/urdf_parser.h"
#include "urdfx/robot_model.h"
#include "urdfx/logging.h"
#include <fstream>
#include <filesystem>

using namespace urdfx;

class URDFParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set log level to WARNING to reduce test output noise
        setLogLevel(spdlog::level::warn);
        
        // Get the UR5e URDF path
        ur5e_urdf_path_ = std::filesystem::path(__FILE__).parent_path().parent_path() / "ur5_urdf" / "ur5e.urdf";
        
        // Create a simple test URDF
        simple_urdf_ = R"(
<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="1.0"/>
            <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
    
    <link name="link1">
        <inertial>
            <origin xyz="0 0 0.5" rpy="0 0 0"/>
            <mass value="2.0"/>
            <inertia ixx="0.1" iyy="0.1" izz="0.05" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0.5" rpy="0 0 0"/>
            <geometry>
                <cylinder radius="0.1" length="1.0"/>
            </geometry>
        </visual>
    </link>
    
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>
</robot>
)";
    }
    
    std::filesystem::path ur5e_urdf_path_;
    std::string simple_urdf_;
};

// Test parsing simple URDF from string
TEST_F(URDFParserTest, ParseSimpleURDFString) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(robot->getName(), "test_robot");
    
    // Check links
    EXPECT_EQ(robot->getLinks().size(), 2);
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    EXPECT_EQ(base_link->getName(), "base_link");
    
    auto link1 = robot->getLink("link1");
    ASSERT_NE(link1, nullptr);
    EXPECT_EQ(link1->getName(), "link1");
    
    // Check joints
    EXPECT_EQ(robot->getJoints().size(), 1);
    auto joint1 = robot->getJoint("joint1");
    ASSERT_NE(joint1, nullptr);
    EXPECT_EQ(joint1->getName(), "joint1");
    EXPECT_EQ(joint1->getType(), JointType::Revolute);
    EXPECT_EQ(joint1->getParentLink(), "base_link");
    EXPECT_EQ(joint1->getChildLink(), "link1");
    
    // Check root link
    EXPECT_EQ(robot->getRootLink(), "base_link");
    
    // Validate robot
    EXPECT_TRUE(robot->validate());
}

// Test parsing UR5e URDF from file
TEST_F(URDFParserTest, ParseUR5eURDFFile) {
    if (!std::filesystem::exists(ur5e_urdf_path_)) {
        GTEST_SKIP() << "UR5e URDF file not found: " << ur5e_urdf_path_;
    }
    
    URDFParser parser;
    auto robot = parser.parseFile(ur5e_urdf_path_.string());
    
    ASSERT_NE(robot, nullptr);
    EXPECT_EQ(robot->getName(), "converted_robot");
    
    // Check that we have links and joints
    EXPECT_GT(robot->getLinks().size(), 0);
    EXPECT_GT(robot->getJoints().size(), 0);
    
    // Check for expected links
    EXPECT_NE(robot->getLink("world"), nullptr);
    EXPECT_NE(robot->getLink("base"), nullptr);
    EXPECT_NE(robot->getLink("shoulder_link"), nullptr);
    
    // Validate robot
    EXPECT_TRUE(robot->validate());
    
    // Check actuated joints
    auto actuated = robot->getActuatedJoints();
    EXPECT_GT(actuated.size(), 0);
}

// Test link inertial properties
TEST_F(URDFParserTest, ParseInertialProperties) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    
    auto inertial = base_link->getInertial();
    ASSERT_TRUE(inertial.has_value());
    EXPECT_DOUBLE_EQ(inertial->mass, 1.0);
    EXPECT_DOUBLE_EQ(inertial->inertia(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(inertial->inertia(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(inertial->inertia(2, 2), 1.0);
}

// Test visual geometry parsing
TEST_F(URDFParserTest, ParseVisualGeometry) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    
    const auto& visuals = base_link->getVisuals();
    ASSERT_EQ(visuals.size(), 1);
    
    const auto& visual = visuals[0];
    EXPECT_EQ(visual.geometry.type, GeometryType::Box);
    EXPECT_DOUBLE_EQ(visual.geometry.box_size(0), 1.0);
    EXPECT_DOUBLE_EQ(visual.geometry.box_size(1), 1.0);
    EXPECT_DOUBLE_EQ(visual.geometry.box_size(2), 1.0);
    
    auto link1 = robot->getLink("link1");
    ASSERT_NE(link1, nullptr);
    
    const auto& visuals1 = link1->getVisuals();
    ASSERT_EQ(visuals1.size(), 1);
    
    const auto& visual1 = visuals1[0];
    EXPECT_EQ(visual1.geometry.type, GeometryType::Cylinder);
    EXPECT_DOUBLE_EQ(visual1.geometry.cylinder_radius, 0.1);
    EXPECT_DOUBLE_EQ(visual1.geometry.cylinder_length, 1.0);
}

// Test joint properties
TEST_F(URDFParserTest, ParseJointProperties) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    auto joint1 = robot->getJoint("joint1");
    ASSERT_NE(joint1, nullptr);
    
    EXPECT_TRUE(joint1->isActuated());
    
    auto limits = joint1->getLimits();
    ASSERT_TRUE(limits.has_value());
    EXPECT_DOUBLE_EQ(limits->lower, -3.14);
    EXPECT_DOUBLE_EQ(limits->upper, 3.14);
    EXPECT_DOUBLE_EQ(limits->effort, 100.0);
    EXPECT_DOUBLE_EQ(limits->velocity, 1.0);
    
    // Check joint axis
    Eigen::Vector3d axis = joint1->getAxis();
    EXPECT_DOUBLE_EQ(axis(0), 0.0);
    EXPECT_DOUBLE_EQ(axis(1), 0.0);
    EXPECT_DOUBLE_EQ(axis(2), 1.0);
}

// Test joint transform computation
TEST_F(URDFParserTest, JointTransform) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    auto joint1 = robot->getJoint("joint1");
    ASSERT_NE(joint1, nullptr);
    
    // Test at zero angle
    Transform t0 = joint1->getTransform(0.0);
    auto [pos0, quat0] = t0.asPositionQuaternion();
    EXPECT_NEAR(pos0(0), 0.0, 1e-6);
    EXPECT_NEAR(pos0(1), 0.0, 1e-6);
    EXPECT_NEAR(pos0(2), 0.0, 1e-6);
    
    // Test at 90 degrees (pi/2)
    Transform t90 = joint1->getTransform(M_PI / 2.0);
    auto [pos90, quat90] = t90.asPositionQuaternion();
    
    // Position should be unchanged (rotation about Z axis at origin)
    EXPECT_NEAR(pos90(0), 0.0, 1e-6);
    EXPECT_NEAR(pos90(1), 0.0, 1e-6);
    EXPECT_NEAR(pos90(2), 0.0, 1e-6);
}

// Test error handling - missing robot name
TEST_F(URDFParserTest, ErrorMissingRobotName) {
    std::string bad_urdf = R"(
<?xml version="1.0"?>
<robot>
    <link name="base_link"/>
</robot>
)";
    
    URDFParser parser;
    EXPECT_THROW({
        parser.parseString(bad_urdf);
    }, URDFParseException);
}

// Test error handling - missing link name
TEST_F(URDFParserTest, ErrorMissingLinkName) {
    std::string bad_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link/>
</robot>
)";
    
    URDFParser parser;
    EXPECT_THROW({
        parser.parseString(bad_urdf);
    }, URDFParseException);
}

// Test error handling - missing joint type
TEST_F(URDFParserTest, ErrorMissingJointType) {
    std::string bad_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base"/>
    <link name="link1"/>
    <joint name="joint1">
        <parent link="base"/>
        <child link="link1"/>
    </joint>
</robot>
)";
    
    URDFParser parser;
    EXPECT_THROW({
        parser.parseString(bad_urdf);
    }, URDFParseException);
}

// Test error handling - invalid XML
TEST_F(URDFParserTest, ErrorInvalidXML) {
    std::string bad_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base"
</robot>
)";
    
    URDFParser parser;
    EXPECT_THROW({
        parser.parseString(bad_urdf);
    }, URDFParseException);
}

// Test error handling - non-existent file
TEST_F(URDFParserTest, ErrorNonExistentFile) {
    URDFParser parser;
    EXPECT_THROW({
        parser.parseFile("/non/existent/path.urdf");
    }, URDFParseException);
}

// Test kinematic tree structure
TEST_F(URDFParserTest, KinematicTreeStructure) {
    URDFParser parser;
    auto robot = parser.parseString(simple_urdf_);
    
    // Test parent-child relationships
    auto child_joints = robot->getChildJoints("base_link");
    EXPECT_EQ(child_joints.size(), 1);
    EXPECT_EQ(child_joints[0]->getName(), "joint1");
    
    auto parent_joint = robot->getParentJoint("link1");
    ASSERT_NE(parent_joint, nullptr);
    EXPECT_EQ(parent_joint->getName(), "joint1");
    
    // Root link should have no parent
    auto root_parent = robot->getParentJoint("base_link");
    EXPECT_EQ(root_parent, nullptr);
}

// Test transform composition
TEST_F(URDFParserTest, TransformComposition) {
    Transform t1 = Transform::fromPositionRPY(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
    Transform t2 = Transform::fromPositionRPY(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0));
    
    Transform t3 = t1 * t2;
    auto [pos, quat] = t3.asPositionQuaternion();
    
    EXPECT_NEAR(pos(0), 1.0, 1e-6);
    EXPECT_NEAR(pos(1), 1.0, 1e-6);
    EXPECT_NEAR(pos(2), 0.0, 1e-6);
}

// Test transform inverse
TEST_F(URDFParserTest, TransformInverse) {
    Transform t = Transform::fromPositionRPY(
        Eigen::Vector3d(1, 2, 3),
        Eigen::Vector3d(0.1, 0.2, 0.3)
    );
    
    Transform t_inv = t.inverse();
    Transform identity = t * t_inv;
    
    auto [pos, quat] = identity.asPositionQuaternion();
    
    EXPECT_NEAR(pos(0), 0.0, 1e-6);
    EXPECT_NEAR(pos(1), 0.0, 1e-6);
    EXPECT_NEAR(pos(2), 0.0, 1e-6);
    EXPECT_NEAR(quat.w(), 1.0, 1e-6);
    EXPECT_NEAR(quat.x(), 0.0, 1e-6);
    EXPECT_NEAR(quat.y(), 0.0, 1e-6);
    EXPECT_NEAR(quat.z(), 0.0, 1e-6);
}

// Test mesh geometry parsing
TEST_F(URDFParserTest, ParseMeshGeometry) {
    std::string mesh_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base">
        <visual>
            <geometry>
                <mesh filename="meshes/base.obj" scale="1 2 3"/>
            </geometry>
        </visual>
    </link>
</robot>
)";
    
    URDFParser parser;
    parser.setBaseDirectory("/test/path");
    auto robot = parser.parseString(mesh_urdf);
    
    auto base_link = robot->getLink("base");
    ASSERT_NE(base_link, nullptr);
    
    const auto& visuals = base_link->getVisuals();
    ASSERT_EQ(visuals.size(), 1);
    
    const auto& geom = visuals[0].geometry;
    EXPECT_EQ(geom.type, GeometryType::Mesh);
    EXPECT_EQ(geom.mesh_filename, "/test/path/meshes/base.obj");
    EXPECT_DOUBLE_EQ(geom.mesh_scale(0), 1.0);
    EXPECT_DOUBLE_EQ(geom.mesh_scale(1), 2.0);
    EXPECT_DOUBLE_EQ(geom.mesh_scale(2), 3.0);
}

// Test fixed joint parsing
TEST_F(URDFParserTest, ParseFixedJoint) {
    std::string fixed_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base"/>
    <link name="fixed_link"/>
    <joint name="fixed_joint" type="fixed">
        <parent link="base"/>
        <child link="fixed_link"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
    </joint>
</robot>
)";
    
    URDFParser parser;
    auto robot = parser.parseString(fixed_urdf);
    
    auto joint = robot->getJoint("fixed_joint");
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->getType(), JointType::Fixed);
    EXPECT_FALSE(joint->isActuated());
    
    // Fixed joints should still compute transforms correctly
    Transform t = joint->getTransform(0.0);
    auto [pos, quat] = t.asPositionQuaternion();
    EXPECT_NEAR(pos(0), 1.0, 1e-6);
}

// Test continuous joint parsing
TEST_F(URDFParserTest, ParseContinuousJoint) {
    std::string continuous_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base"/>
    <link name="wheel"/>
    <joint name="wheel_joint" type="continuous">
        <parent link="base"/>
        <child link="wheel"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
</robot>
)";
    
    URDFParser parser;
    auto robot = parser.parseString(continuous_urdf);
    
    auto joint = robot->getJoint("wheel_joint");
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->getType(), JointType::Continuous);
    EXPECT_TRUE(joint->isActuated());
    
    // Continuous joints have no limits
    EXPECT_FALSE(joint->getLimits().has_value());
}

// Test prismatic joint parsing
TEST_F(URDFParserTest, ParsePrismaticJoint) {
    std::string prismatic_urdf = R"(
<?xml version="1.0"?>
<robot name="test">
    <link name="base"/>
    <link name="slider"/>
    <joint name="slide_joint" type="prismatic">
        <parent link="base"/>
        <child link="slider"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
    </joint>
</robot>
)";
    
    URDFParser parser;
    auto robot = parser.parseString(prismatic_urdf);
    
    auto joint = robot->getJoint("slide_joint");
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->getType(), JointType::Prismatic);
    EXPECT_TRUE(joint->isActuated());
    
    // Test prismatic transform (translation along axis)
    Transform t = joint->getTransform(0.5);
    auto [pos, quat] = t.asPositionQuaternion();
    EXPECT_NEAR(pos(0), 0.0, 1e-6);
    EXPECT_NEAR(pos(1), 0.0, 1e-6);
    EXPECT_NEAR(pos(2), 0.5, 1e-6);
}
