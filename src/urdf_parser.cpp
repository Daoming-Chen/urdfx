#include "urdfx/urdf_parser.h"
#include "urdfx/logging.h"
#include <pugixml.hpp>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>

namespace urdfx {

// Helper functions for parsing XML attributes
namespace {

std::string getAttribute(const pugi::xml_node& node, const char* name, const std::string& default_value = "") {
    pugi::xml_attribute attr = node.attribute(name);
    return attr ? attr.value() : default_value;
}

double getDoubleAttribute(const pugi::xml_node& node, const char* name, double default_value = 0.0) {
    pugi::xml_attribute attr = node.attribute(name);
    return attr ? attr.as_double(default_value) : default_value;
}

Eigen::Vector3d parseVector3(const std::string& str) {
    std::istringstream iss(str);
    Eigen::Vector3d vec;
    iss >> vec(0) >> vec(1) >> vec(2);
    return vec;
}

Eigen::Vector4d parseVector4(const std::string& str) {
    std::istringstream iss(str);
    Eigen::Vector4d vec;
    iss >> vec(0) >> vec(1) >> vec(2) >> vec(3);
    return vec;
}

Transform parseOrigin(const pugi::xml_node& origin_node) {
    if (!origin_node) {
        return Transform();  // Identity transform
    }
    
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    
    std::string xyz_str = getAttribute(origin_node, "xyz");
    if (!xyz_str.empty()) {
        xyz = parseVector3(xyz_str);
    }
    
    std::string rpy_str = getAttribute(origin_node, "rpy");
    if (!rpy_str.empty()) {
        rpy = parseVector3(rpy_str);
    }
    
    return Transform::fromPositionRPY(xyz, rpy);
}

Geometry parseGeometry(const pugi::xml_node& geometry_node, const std::string& base_dir) {
    Geometry geometry;
    
    if (!geometry_node) {
        throw URDFParseException("Geometry element is missing");
    }
    
    // Check for box
    pugi::xml_node box = geometry_node.child("box");
    if (box) {
        geometry.type = GeometryType::Box;
        std::string size_str = getAttribute(box, "size");
        if (size_str.empty()) {
            throw URDFParseException("Box geometry missing 'size' attribute");
        }
        geometry.box_size = parseVector3(size_str);
        return geometry;
    }
    
    // Check for cylinder
    pugi::xml_node cylinder = geometry_node.child("cylinder");
    if (cylinder) {
        geometry.type = GeometryType::Cylinder;
        geometry.cylinder_radius = getDoubleAttribute(cylinder, "radius");
        geometry.cylinder_length = getDoubleAttribute(cylinder, "length");
        return geometry;
    }
    
    // Check for sphere
    pugi::xml_node sphere = geometry_node.child("sphere");
    if (sphere) {
        geometry.type = GeometryType::Sphere;
        geometry.sphere_radius = getDoubleAttribute(sphere, "radius");
        return geometry;
    }
    
    // Check for mesh
    pugi::xml_node mesh = geometry_node.child("mesh");
    if (mesh) {
        geometry.type = GeometryType::Mesh;
        geometry.mesh_filename = getAttribute(mesh, "filename");
        
        if (geometry.mesh_filename.empty()) {
            throw URDFParseException("Mesh geometry missing 'filename' attribute");
        }
        
        // Resolve relative paths
        if (!base_dir.empty() && geometry.mesh_filename.find("://") == std::string::npos) {
            // Not a URI, treat as relative path
            std::filesystem::path mesh_path(geometry.mesh_filename);
            if (mesh_path.is_relative()) {
                geometry.mesh_filename = (std::filesystem::path(base_dir) / mesh_path).string();
            }
        }
        
        std::string scale_str = getAttribute(mesh, "scale");
        if (!scale_str.empty()) {
            geometry.mesh_scale = parseVector3(scale_str);
        }
        
        return geometry;
    }
    
    throw URDFParseException("Unknown geometry type");
}

Inertial parseInertial(const pugi::xml_node& inertial_node) {
    Inertial inertial;
    
    // Parse origin
    inertial.origin = parseOrigin(inertial_node.child("origin"));
    
    // Parse mass
    pugi::xml_node mass_node = inertial_node.child("mass");
    if (mass_node) {
        inertial.mass = getDoubleAttribute(mass_node, "value");
    }
    
    // Parse inertia matrix
    pugi::xml_node inertia_node = inertial_node.child("inertia");
    if (inertia_node) {
        double ixx = getDoubleAttribute(inertia_node, "ixx");
        double iyy = getDoubleAttribute(inertia_node, "iyy");
        double izz = getDoubleAttribute(inertia_node, "izz");
        double ixy = getDoubleAttribute(inertia_node, "ixy");
        double ixz = getDoubleAttribute(inertia_node, "ixz");
        double iyz = getDoubleAttribute(inertia_node, "iyz");
        
        inertial.inertia << ixx, ixy, ixz,
                           ixy, iyy, iyz,
                           ixz, iyz, izz;
    }
    
    return inertial;
}

Visual parseVisual(const pugi::xml_node& visual_node, const std::string& base_dir) {
    Visual visual;
    visual.name = getAttribute(visual_node, "name");
    
    // Parse origin
    visual.origin = parseOrigin(visual_node.child("origin"));
    
    // Parse geometry
    pugi::xml_node geometry_node = visual_node.child("geometry");
    visual.geometry = parseGeometry(geometry_node, base_dir);
    
    // Parse material (optional)
    pugi::xml_node material_node = visual_node.child("material");
    if (material_node) {
        visual.material_name = getAttribute(material_node, "name");
        
        pugi::xml_node color_node = material_node.child("color");
        if (color_node) {
            std::string rgba_str = getAttribute(color_node, "rgba");
            if (!rgba_str.empty()) {
                visual.color = parseVector4(rgba_str);
            }
        }
    }
    
    return visual;
}

Collision parseCollision(const pugi::xml_node& collision_node, const std::string& base_dir) {
    Collision collision;
    collision.name = getAttribute(collision_node, "name");
    
    // Parse origin
    collision.origin = parseOrigin(collision_node.child("origin"));
    
    // Parse geometry
    pugi::xml_node geometry_node = collision_node.child("geometry");
    collision.geometry = parseGeometry(geometry_node, base_dir);
    
    return collision;
}

JointType parseJointType(const std::string& type_str) {
    static const std::unordered_map<std::string, JointType> type_map = {
        {"fixed", JointType::Fixed},
        {"revolute", JointType::Revolute},
        {"continuous", JointType::Continuous},
        {"prismatic", JointType::Prismatic},
        {"floating", JointType::Floating},
        {"planar", JointType::Planar}
    };
    
    auto it = type_map.find(type_str);
    if (it != type_map.end()) {
        return it->second;
    }
    
    throw URDFParseException("Unknown joint type: " + type_str);
}

} // anonymous namespace

// ============================================================================
// URDFParser Implementation
// ============================================================================

class URDFParser::Impl {
public:
    std::shared_ptr<Robot> parse(const std::string& content, const std::string& source, const std::string& base_dir) {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_string(content.c_str());
        
        if (!result) {
            throw URDFParseException(
                std::string("XML parse error: ") + result.description(),
                result.offset
            );
        }
        
        pugi::xml_node robot_node = doc.child("robot");
        if (!robot_node) {
            throw URDFParseException("Missing <robot> root element");
        }
        
        std::string robot_name = getAttribute(robot_node, "name");
        if (robot_name.empty()) {
            throw URDFParseException("Robot missing 'name' attribute");
        }
        
        URDFX_INFO("Parsing robot '{}'", robot_name);
        auto robot = std::make_shared<Robot>(robot_name);
        
        // Parse all links
        for (pugi::xml_node link_node : robot_node.children("link")) {
            parseLink(link_node, robot, base_dir);
        }
        
        // Parse all joints
        for (pugi::xml_node joint_node : robot_node.children("joint")) {
            parseJoint(joint_node, robot);
        }
        
        // Determine root link (link with no parent joint)
        findRootLink(robot);
        
        // Validate robot structure
        if (!robot->validate()) {
            throw URDFParseException("Robot validation failed");
        }
        
        URDFX_INFO("Successfully parsed robot '{}' with {} links and {} joints",
                   robot_name, robot->getLinks().size(), robot->getJoints().size());
        
        return robot;
    }

private:
    void parseLink(const pugi::xml_node& link_node, std::shared_ptr<Robot> robot, const std::string& base_dir) {
        std::string link_name = getAttribute(link_node, "name");
        if (link_name.empty()) {
            throw URDFParseException("Link missing 'name' attribute");
        }
        
        URDFX_DEBUG("Parsing link '{}'", link_name);
        auto link = std::make_shared<Link>(link_name);
        
        // Parse inertial (optional)
        pugi::xml_node inertial_node = link_node.child("inertial");
        if (inertial_node) {
            try {
                link->setInertial(parseInertial(inertial_node));
            } catch (const std::exception& e) {
                throw URDFParseException("Error parsing inertial for link '" + link_name + "': " + e.what());
            }
        }
        
        // Parse visual elements (multiple allowed)
        for (pugi::xml_node visual_node : link_node.children("visual")) {
            try {
                link->addVisual(parseVisual(visual_node, base_dir));
            } catch (const std::exception& e) {
                throw URDFParseException("Error parsing visual for link '" + link_name + "': " + e.what());
            }
        }
        
        // Parse collision elements (multiple allowed)
        for (pugi::xml_node collision_node : link_node.children("collision")) {
            try {
                link->addCollision(parseCollision(collision_node, base_dir));
            } catch (const std::exception& e) {
                throw URDFParseException("Error parsing collision for link '" + link_name + "': " + e.what());
            }
        }
        
        robot->addLink(link);
    }
    
    void parseJoint(const pugi::xml_node& joint_node, std::shared_ptr<Robot> robot) {
        std::string joint_name = getAttribute(joint_node, "name");
        if (joint_name.empty()) {
            throw URDFParseException("Joint missing 'name' attribute");
        }
        
        std::string type_str = getAttribute(joint_node, "type");
        if (type_str.empty()) {
            throw URDFParseException("Joint '" + joint_name + "' missing 'type' attribute");
        }
        
        URDFX_DEBUG("Parsing joint '{}' of type '{}'", joint_name, type_str);
        
        JointType joint_type = parseJointType(type_str);
        auto joint = std::make_shared<Joint>(joint_name, joint_type);
        
        // Parse parent link
        pugi::xml_node parent_node = joint_node.child("parent");
        if (!parent_node) {
            throw URDFParseException("Joint '" + joint_name + "' missing <parent> element");
        }
        std::string parent_link = getAttribute(parent_node, "link");
        if (parent_link.empty()) {
            throw URDFParseException("Joint '" + joint_name + "' parent missing 'link' attribute");
        }
        joint->setParentLink(parent_link);
        
        // Parse child link
        pugi::xml_node child_node = joint_node.child("child");
        if (!child_node) {
            throw URDFParseException("Joint '" + joint_name + "' missing <child> element");
        }
        std::string child_link = getAttribute(child_node, "link");
        if (child_link.empty()) {
            throw URDFParseException("Joint '" + joint_name + "' child missing 'link' attribute");
        }
        joint->setChildLink(child_link);
        
        // Parse origin
        joint->setOrigin(parseOrigin(joint_node.child("origin")));
        
        // Parse axis (optional, defaults to Z axis)
        pugi::xml_node axis_node = joint_node.child("axis");
        if (axis_node) {
            std::string xyz_str = getAttribute(axis_node, "xyz");
            if (!xyz_str.empty()) {
                Eigen::Vector3d axis = parseVector3(xyz_str);
                axis.normalize();
                joint->setAxis(axis);
            }
        }
        
        // Parse limits (required for revolute and prismatic joints)
        pugi::xml_node limit_node = joint_node.child("limit");
        if (limit_node) {
            JointLimits limits;
            limits.lower = getDoubleAttribute(limit_node, "lower");
            limits.upper = getDoubleAttribute(limit_node, "upper");
            limits.effort = getDoubleAttribute(limit_node, "effort");
            limits.velocity = getDoubleAttribute(limit_node, "velocity");
            joint->setLimits(limits);
        } else if (joint_type == JointType::Revolute || joint_type == JointType::Prismatic) {
            URDFX_WARN("Joint '{}' of type '{}' missing <limit> element", joint_name, type_str);
        }
        
        // Parse dynamics (optional)
        pugi::xml_node dynamics_node = joint_node.child("dynamics");
        if (dynamics_node) {
            JointDynamics dynamics;
            dynamics.damping = getDoubleAttribute(dynamics_node, "damping");
            dynamics.friction = getDoubleAttribute(dynamics_node, "friction");
            joint->setDynamics(dynamics);
        }
        
        robot->addJoint(joint);
    }
    
    void findRootLink(std::shared_ptr<Robot> robot) {
        // Find link that is not a child of any joint
        std::unordered_set<std::string> child_links;
        for (const auto& joint : robot->getJoints()) {
            child_links.insert(joint->getChildLink());
        }
        
        for (const auto& link : robot->getLinks()) {
            if (child_links.find(link->getName()) == child_links.end()) {
                robot->setRootLink(link->getName());
                URDFX_INFO("Root link determined: '{}'", link->getName());
                return;
            }
        }
        
        // If no root found, use first link as fallback
        if (!robot->getLinks().empty()) {
            robot->setRootLink(robot->getLinks()[0]->getName());
            URDFX_WARN("Could not determine root link, using first link: '{}'", 
                      robot->getRootLink());
        }
    }
};

URDFParser::URDFParser() = default;
URDFParser::~URDFParser() = default;

std::shared_ptr<Robot> URDFParser::parseFile(const std::string& filepath) {
    URDFX_INFO("Parsing URDF file: {}", filepath);
    
    // Read file content
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw URDFParseException("Could not open file: " + filepath);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    
    // Set base directory for mesh resolution
    std::filesystem::path file_path(filepath);
    if (base_dir_.empty()) {
        base_dir_ = file_path.parent_path().string();
    }
    
    return parseDocument(content, filepath);
}

std::shared_ptr<Robot> URDFParser::parseString(const std::string& urdf_string) {
    URDFX_INFO("Parsing URDF from string ({} bytes)", urdf_string.size());
    return parseDocument(urdf_string, "<string>");
}

std::shared_ptr<Robot> URDFParser::parseDocument(const std::string& content, const std::string& source) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    
    try {
        return impl_->parse(content, source, base_dir_);
    } catch (const URDFParseException&) {
        throw;
    } catch (const std::exception& e) {
        throw URDFParseException(std::string("Unexpected error: ") + e.what());
    }
}

} // namespace urdfx
