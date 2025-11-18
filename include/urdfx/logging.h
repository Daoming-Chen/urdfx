#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <memory>
#include <string>

namespace urdfx {

/**
 * @brief Get the global urdfx logger instance
 * 
 * This logger is created on first access with a console sink.
 * Default log level is INFO.
 */
inline std::shared_ptr<spdlog::logger> getLogger() {
    static auto logger = []() {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto logger = std::make_shared<spdlog::logger>("urdfx", console_sink);
        logger->set_level(spdlog::level::info);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        return logger;
    }();
    return logger;
}

/**
 * @brief Set the log level for the urdfx logger
 * 
 * @param level Log level (trace, debug, info, warn, error, critical, off)
 */
inline void setLogLevel(spdlog::level::level_enum level) {
    getLogger()->set_level(level);
}

/**
 * @brief Set the log level from a string
 * 
 * @param level_str Log level string (trace, debug, info, warn, error, critical, off)
 */
inline void setLogLevel(const std::string& level_str) {
    auto level = spdlog::level::from_str(level_str);
    setLogLevel(level);
}

/**
 * @brief Add a file sink to the logger
 * 
 * @param filepath Path to the log file
 * @param truncate If true, truncate the file; otherwise append
 */
inline void setLogFile(const std::string& filepath, bool truncate = false) {
    try {
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filepath, truncate);
        file_sink->set_level(getLogger()->level());
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        getLogger()->sinks().push_back(file_sink);
    } catch (const spdlog::spdlog_ex& ex) {
        getLogger()->error("Failed to create log file {}: {}", filepath, ex.what());
    }
}

/**
 * @brief Remove all file sinks from the logger
 */
inline void clearLogFiles() {
    auto& sinks = getLogger()->sinks();
    sinks.erase(
        std::remove_if(sinks.begin(), sinks.end(),
            [](const spdlog::sink_ptr& sink) {
                return dynamic_cast<spdlog::sinks::basic_file_sink_mt*>(sink.get()) != nullptr;
            }),
        sinks.end()
    );
}

} // namespace urdfx

// Convenience macros for logging
#define URDFX_LOG_TRACE(...) urdfx::getLogger()->trace(__VA_ARGS__)
#define URDFX_LOG_DEBUG(...) urdfx::getLogger()->debug(__VA_ARGS__)
#define URDFX_LOG_INFO(...) urdfx::getLogger()->info(__VA_ARGS__)
#define URDFX_LOG_WARN(...) urdfx::getLogger()->warn(__VA_ARGS__)
#define URDFX_LOG_ERROR(...) urdfx::getLogger()->error(__VA_ARGS__)
#define URDFX_LOG_CRITICAL(...) urdfx::getLogger()->critical(__VA_ARGS__)

// Short-form macros
#define URDFX_TRACE(...) URDFX_LOG_TRACE(__VA_ARGS__)
#define URDFX_DEBUG(...) URDFX_LOG_DEBUG(__VA_ARGS__)
#define URDFX_INFO(...) URDFX_LOG_INFO(__VA_ARGS__)
#define URDFX_WARN(...) URDFX_LOG_WARN(__VA_ARGS__)
#define URDFX_ERROR(...) URDFX_LOG_ERROR(__VA_ARGS__)
#define URDFX_CRITICAL(...) URDFX_LOG_CRITICAL(__VA_ARGS__)
