#pragma once

/**
 * @file export.h
 * @brief Export macros for building urdfx as a shared library (DLL) on Windows
 */

// Define URDFX_API for proper symbol export/import on Windows
#if defined(_WIN32) || defined(_WIN64)
    // Windows DLL export/import
    #ifdef URDFX_BUILD
        // Building the library - export symbols
        #ifdef URDFX_SHARED
            #define URDFX_API __declspec(dllexport)
        #else
            #define URDFX_API
        #endif
    #else
        // Using the library - import symbols
        #ifdef URDFX_SHARED
            #define URDFX_API __declspec(dllimport)
        #else
            #define URDFX_API
        #endif
    #endif
#else
    // Non-Windows platforms
    #if defined(__GNUC__) && __GNUC__ >= 4
        #define URDFX_API __attribute__((visibility("default")))
    #else
        #define URDFX_API
    #endif
#endif

// Disable warnings about DLL interface for STL types on MSVC
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4251) // class needs to have dll-interface
    #pragma warning(disable: 4275) // non dll-interface class used as base
#endif
