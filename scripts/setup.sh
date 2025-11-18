#!/bin/bash
# setup.sh - Setup script for urdfx development environment
# This script installs required system dependencies and Emscripten SDK
# Usage: ./scripts/setup.sh

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
EMSDK_DIR="$PROJECT_ROOT/third_party/emsdk"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Linux or macOS
OS="$(uname -s)"
case "$OS" in
    Linux*)     PLATFORM=Linux;;
    Darwin*)    PLATFORM=macOS;;
    *)          echo_error "Unsupported operating system: $OS"; exit 1;;
esac

echo_info "Detected platform: $PLATFORM"

# Check and install CMake
check_cmake() {
    if command -v cmake &> /dev/null; then
        CMAKE_VERSION=$(cmake --version | head -n1 | awk '{print $3}')
        echo_info "CMake already installed: version $CMAKE_VERSION"
        
        # Check if version is at least 3.20
        REQUIRED_VERSION="3.20"
        if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$CMAKE_VERSION" | sort -V | head -n1)" = "$REQUIRED_VERSION" ]; then
            return 0
        else
            echo_warn "CMake version $CMAKE_VERSION is too old (need >= 3.20)"
            return 1
        fi
    else
        return 1
    fi
}

install_cmake() {
    echo_info "Installing CMake..."
    
    if [ "$PLATFORM" = "Linux" ]; then
        if command -v apt-get &> /dev/null; then
            sudo apt-get update
            sudo apt-get install -y cmake
        elif command -v yum &> /dev/null; then
            sudo yum install -y cmake
        else
            echo_error "Could not find package manager (apt-get or yum)"
            echo_info "Please install CMake 3.20+ manually"
            exit 1
        fi
    elif [ "$PLATFORM" = "macOS" ]; then
        if command -v brew &> /dev/null; then
            brew install cmake
        else
            echo_error "Homebrew not found. Please install Homebrew first: https://brew.sh"
            exit 1
        fi
    fi
}

# Check and install Python 3
check_python() {
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version | awk '{print $2}')
        echo_info "Python already installed: version $PYTHON_VERSION"
        return 0
    else
        return 1
    fi
}

install_python() {
    echo_info "Installing Python 3..."
    
    if [ "$PLATFORM" = "Linux" ]; then
        if command -v apt-get &> /dev/null; then
            sudo apt-get install -y python3 python3-pip python3-dev
        elif command -v yum &> /dev/null; then
            sudo yum install -y python3 python3-pip python3-devel
        fi
    elif [ "$PLATFORM" = "macOS" ]; then
        if command -v brew &> /dev/null; then
            brew install python3
        fi
    fi
}

# Check and install Node.js
check_nodejs() {
    if command -v node &> /dev/null; then
        NODE_VERSION=$(node --version)
        echo_info "Node.js already installed: version $NODE_VERSION"
        return 0
    else
        return 1
    fi
}

install_nodejs() {
    echo_info "Installing Node.js..."
    
    if [ "$PLATFORM" = "Linux" ]; then
        if command -v apt-get &> /dev/null; then
            curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
            sudo apt-get install -y nodejs
        elif command -v yum &> /dev/null; then
            curl -fsSL https://rpm.nodesource.com/setup_lts.x | sudo bash -
            sudo yum install -y nodejs
        fi
    elif [ "$PLATFORM" = "macOS" ]; then
        if command -v brew &> /dev/null; then
            brew install node
        fi
    fi
}

# Install Emscripten SDK
install_emsdk() {
    if [ -d "$EMSDK_DIR" ] && [ -f "$EMSDK_DIR/emsdk" ]; then
        echo_info "Emscripten SDK directory already exists at $EMSDK_DIR"
        
        # Check if emsdk is activated
        if [ -f "$EMSDK_DIR/.emscripten" ]; then
            echo_info "Emscripten SDK appears to be configured"
            return 0
        fi
    fi
    
    echo_info "Installing Emscripten SDK to $EMSDK_DIR..."
    
    # Create third_party directory if it doesn't exist
    mkdir -p "$(dirname "$EMSDK_DIR")"
    
    # Clone emsdk if not present
    if [ ! -d "$EMSDK_DIR" ]; then
        git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR"
    fi
    
    # Install and activate latest emsdk
    cd "$EMSDK_DIR"
    ./emsdk install latest
    ./emsdk activate latest
    
    echo_info "Emscripten SDK installed successfully"
    echo_info "To use Emscripten, run: source $EMSDK_DIR/emsdk_env.sh"
}

# Main installation flow
main() {
    echo_info "Starting urdfx development environment setup..."
    echo ""
    
    # Check and install CMake
    if ! check_cmake; then
        install_cmake
        if ! check_cmake; then
            echo_error "CMake installation failed or version is too old"
            exit 1
        fi
    fi
    echo ""
    
    # Check and install Python
    if ! check_python; then
        install_python
        if ! check_python; then
            echo_error "Python installation failed"
            exit 1
        fi
    fi
    echo ""
    
    # Check and install Node.js
    if ! check_nodejs; then
        install_nodejs
        if ! check_nodejs; then
            echo_error "Node.js installation failed"
            exit 1
        fi
    fi
    echo ""
    
    # Install Emscripten SDK
    install_emsdk
    echo ""
    
    echo_info "Setup complete!"
    echo ""
    echo_info "Next steps:"
    echo "  1. Initialize git submodules:"
    echo "     git submodule update --init --recursive"
    echo ""
    echo "  2. Configure and build the project:"
    echo "     cmake -B build -DCMAKE_BUILD_TYPE=Release"
    echo "     cmake --build build -j\$(nproc)"
    echo ""
    echo "  3. To build WebAssembly bindings:"
    echo "     source $EMSDK_DIR/emsdk_env.sh"
    echo "     emcmake cmake -B build-wasm -DBUILD_WASM=ON"
    echo "     cmake --build build-wasm"
    echo ""
}

# Run main
main
