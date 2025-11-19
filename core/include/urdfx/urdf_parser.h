#pragma once

#include "urdfx/export.h"
#include "urdfx/robot_model.h"
#include <string>
#include <memory>
#include <stdexcept>

namespace urdfx {

/**
 * @brief Exception thrown during URDF parsing
 */
class URDFX_API URDFParseException : public std::runtime_error {
public:
    URDFParseException(const std::string& message, int line = -1)
        : std::runtime_error(formatMessage(message, line))
        , line_(line) {}
    
    int getLine() const { return line_; }

private:
    int line_;
    
    static std::string formatMessage(const std::string& message, int line) {
        if (line >= 0) {
            return "URDF parse error at line " + std::to_string(line) + ": " + message;
        }
        return "URDF parse error: " + message;
    }
};

/**
 * @brief Parser for URDF (Unified Robot Description Format) files
 * 
 * This class uses pugixml to parse URDF XML files and construct
 * a Robot model with links, joints, and their properties.
 */
class URDFX_API URDFParser {
public:
    URDFParser();
    ~URDFParser();
    
    /**
     * @brief Parse URDF from file
     * @param filepath Path to URDF file
     * @return Parsed robot model
     * @throws URDFParseException on parse error
     */
    std::shared_ptr<Robot> parseFile(const std::string& filepath);
    
    /**
     * @brief Parse URDF from string
     * @param urdf_string URDF XML content
     * @return Parsed robot model
     * @throws URDFParseException on parse error
     */
    std::shared_ptr<Robot> parseString(const std::string& urdf_string);
    
    /**
     * @brief Set base directory for resolving relative mesh paths
     * @param base_dir Base directory path
     */
    void setBaseDirectory(const std::string& base_dir) { 
        base_dir_ = base_dir; 
    }
    
    const std::string& getBaseDirectory() const { 
        return base_dir_; 
    }

private:
    std::string base_dir_;  // Base directory for mesh path resolution
    
    // Internal parsing methods
    class Impl;
    std::unique_ptr<Impl> impl_;
    
    std::shared_ptr<Robot> parseDocument(const std::string& content, const std::string& source);
};

} // namespace urdfx
