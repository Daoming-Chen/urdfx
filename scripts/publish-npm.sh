#!/bin/bash
# Publish urdfx WASM bindings to npm
# This script builds the WASM module and publishes it to npm

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
VERSION=""
DRY_RUN=false
SKIP_BUILD=false
SKIP_TESTS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --version)
            VERSION="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --skip-tests)
            SKIP_TESTS=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --version VERSION    Set package version"
            echo "  --dry-run           Perform dry run without publishing"
            echo "  --skip-build        Skip WASM build step"
            echo "  --skip-tests        Skip test execution"
            echo "  -h, --help          Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

function write_step {
    echo -e "\n${CYAN}===> $1${NC}"
}

function write_success {
    echo -e "${GREEN}✓ $1${NC}"
}

function write_error {
    echo -e "${RED}✗ $1${NC}"
}

# Get script directory and project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
WASM_DIR="$PROJECT_ROOT/bindings/wasm"
DIST_DIR="$WASM_DIR/dist"

echo -e "${CYAN}╔════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   urdfx npm Publisher for Linux/Mac   ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════╝${NC}"

# Step 1: Check npm login status
write_step "Checking npm authentication..."
if ! NPM_USER=$(npm whoami 2>/dev/null); then
    write_error "Not logged in to npm. Please run 'npm login' first."
    exit 1
fi
write_success "Logged in as: $NPM_USER"

# Step 2: Build WASM module (unless skipped)
if [ "$SKIP_BUILD" = false ]; then
    write_step "Building WASM module..."
    
    BUILD_SCRIPT="$SCRIPT_DIR/build-wasm.sh"
    if [ ! -f "$BUILD_SCRIPT" ]; then
        write_error "Build script not found: $BUILD_SCRIPT"
        exit 1
    fi
    
    bash "$BUILD_SCRIPT"
    write_success "WASM build completed"
else
    write_step "Skipping WASM build (--skip-build specified)"
fi

# Step 3: Verify build artifacts
write_step "Verifying build artifacts..."
BUILD_WASM_DIR="$PROJECT_ROOT/build-wasm/wasm"
WASM_JS="$BUILD_WASM_DIR/urdfx.js"
WASM_BINARY="$BUILD_WASM_DIR/urdfx.wasm"

if [ ! -f "$WASM_JS" ]; then
    write_error "WASM JavaScript file not found: $WASM_JS"
    exit 1
fi
if [ ! -f "$WASM_BINARY" ]; then
    write_error "WASM binary file not found: $WASM_BINARY"
    exit 1
fi
write_success "Build artifacts verified"

# Step 4: Prepare distribution directory
write_step "Preparing distribution directory..."
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"

# Copy build artifacts
cp "$WASM_JS" "$DIST_DIR/"
cp "$WASM_BINARY" "$DIST_DIR/"

# Copy additional files
TYPE_DEFS="$WASM_DIR/urdfx.d.ts"
README_SOURCE="$WASM_DIR/dist/README.md"
LICENSE="$PROJECT_ROOT/LICENSE"

if [ -f "$TYPE_DEFS" ]; then
    cp "$TYPE_DEFS" "$DIST_DIR/"
else
    write_error "TypeScript definitions not found: $TYPE_DEFS"
    exit 1
fi

if [ -f "$LICENSE" ]; then
    cp "$LICENSE" "$DIST_DIR/"
else
    write_error "LICENSE file not found: $LICENSE"
    exit 1
fi

# Create or verify README.md
if [ ! -f "$README_SOURCE" ]; then
    echo -e "${YELLOW}README.md not found, creating default...${NC}"
    cat > "$DIST_DIR/README.md" << 'EOF'
# urdfx

A modern WebAssembly robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving.

## Installation

```bash
npm install urdfx
```

## Quick Start

```javascript
const createUrdfxModule = require('urdfx');

(async () => {
  const urdfx = await createUrdfxModule();
  
  const urdfXml = `<?xml version="1.0"?>
  <robot name="my_robot">
    <!-- Your URDF content -->
  </robot>`;
  
  const robot = urdfx.Robot.fromURDFString(urdfXml);
  console.log('Robot:', robot.getName());
  
  robot.dispose();
})();
```

## Documentation

For full documentation, visit: https://github.com/Daoming-Chen/urdfx

## License

MIT
EOF
else
    cp "$README_SOURCE" "$DIST_DIR/"
fi

write_success "Distribution directory prepared"

# Step 5: Create/update package.json
write_step "Preparing package.json..."

PACKAGE_JSON_PATH="$DIST_DIR/package.json"

# Create package.json
cat > "$PACKAGE_JSON_PATH" << EOF
{
  "name": "urdfx",
  "version": "${VERSION:-1.0.0}",
  "description": "A modern WebAssembly robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving",
  "main": "urdfx.js",
  "types": "urdfx.d.ts",
  "type": "module",
  "files": [
    "urdfx.js",
    "urdfx.wasm",
    "urdfx.d.ts",
    "README.md",
    "LICENSE"
  ],
  "keywords": [
    "robotics",
    "kinematics",
    "urdf",
    "inverse-kinematics",
    "forward-kinematics",
    "jacobian",
    "webassembly",
    "wasm",
    "robot",
    "3d"
  ],
  "author": "urdfx contributors",
  "license": "MIT",
  "repository": {
    "type": "git",
    "url": "https://github.com/Daoming-Chen/urdfx.git"
  },
  "bugs": {
    "url": "https://github.com/Daoming-Chen/urdfx/issues"
  },
  "homepage": "https://github.com/Daoming-Chen/urdfx#readme",
  "engines": {
    "node": ">=14.0.0"
  }
}
EOF

PACKAGE_VERSION=$(grep -o '"version": "[^"]*"' "$PACKAGE_JSON_PATH" | cut -d'"' -f4)
write_success "package.json prepared (version: $PACKAGE_VERSION)"

# Step 6: Create .npmignore
cat > "$DIST_DIR/.npmignore" << 'EOF'
# Test files
*.test.js
*.test.cjs
test.js
test.cjs
check.cjs

# Build files
CMakeLists.txt
cmake/
src/

# Development files
.gitignore
.git/
.vscode/
.cursor/

# Documentation source
docs/

# Node modules
node_modules/
EOF

write_success "Distribution files prepared"

# Step 7: Run npm pack (dry run preview)
write_step "Previewing package contents..."
cd "$DIST_DIR"
npm pack --dry-run

# Step 8: Publish to npm
if [ "$DRY_RUN" = true ]; then
    write_step "Performing dry run publish..."
    cd "$DIST_DIR"
    npm publish --dry-run
    write_success "Dry run completed successfully"
    echo -e "\n${YELLOW}To publish for real, run without --dry-run flag${NC}"
else
    echo -e "\n${YELLOW}⚠️  Ready to publish urdfx@$PACKAGE_VERSION to npm${NC}"
    echo -e "${YELLOW}Press Enter to continue or Ctrl+C to cancel...${NC}"
    read
    
    write_step "Publishing to npm..."
    cd "$DIST_DIR"
    npm publish
    
    echo -e "\n${GREEN}╔════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║   Successfully published to npm! 🎉   ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
    echo -e "\n${GREEN}Package: urdfx@$PACKAGE_VERSION${NC}"
    echo -e "${CYAN}URL: https://www.npmjs.com/package/urdfx${NC}"
    echo -e "\n${CYAN}Install with: npm install urdfx${NC}"
fi

# Step 9: Clean up info
echo -e "\n${CYAN}Distribution files are in: $DIST_DIR${NC}"
echo -e "${NC}You can safely delete this directory after publishing.${NC}"
