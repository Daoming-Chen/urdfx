# build-viz.ps1 - Build and deploy visualization app for urdfx
# This script builds the WASM module and copies it to the visualization public directory,
# then optionally builds or runs the visualization app
# Usage: 
#   .\scripts\build-viz.ps1              # Build WASM and copy to viz
#   .\scripts\build-viz.ps1 -Dev         # Build WASM, copy, and start dev server
#   .\scripts\build-viz.ps1 -Build       # Build WASM, copy, and build viz for production
#   .\scripts\build-viz.ps1 -SkipWasm    # Skip WASM build, only copy existing files

#Requires -Version 5.1

param(
    [switch]$Dev = $false,
    [switch]$Build = $false,
    [switch]$SkipWasm = $false,
    [switch]$Clean = $false
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$WasmBuildDir = Join-Path $ProjectRoot "build-wasm"
$WasmOutputDir = Join-Path $WasmBuildDir "wasm"
$VizDir = Join-Path $ProjectRoot "visualization"
$VizPublicDir = Join-Path $VizDir "public"

# Colors for output
function Write-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Green
}

function Write-Warn {
    param([string]$Message)
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Error-Custom {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "===> $Message" -ForegroundColor Cyan
    Write-Host ""
}

# Build WASM module
function Build-Wasm {
    Write-Step "Building WASM module"
    
    if ($Clean -and (Test-Path $WasmBuildDir)) {
        Write-Info "Cleaning previous WASM build directory..."
        Remove-Item -Recurse -Force $WasmBuildDir
    }
    
    # Find Visual Studio
    $vsPath = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
    if (-not $vsPath) {
        Write-Error-Custom "Visual Studio not found. Please install Visual Studio 2019+ with C++ workload"
        exit 1
    }
    
    Write-Info "Using Visual Studio at: $vsPath"
    
    # Set up Emscripten
    $emsdkPath = Join-Path $ProjectRoot "third_party\emsdk"
    $emsdkEnvBat = Join-Path $emsdkPath "emsdk_env.bat"
    
    if (-not (Test-Path $emsdkEnvBat)) {
        Write-Error-Custom "Emscripten SDK not found at $emsdkPath"
        Write-Error-Custom "Please run .\scripts\setup.ps1 first to set up the development environment"
        exit 1
    }
    
    # Prepare build script
    $vcvarsPath = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
    
    $buildScript = @"
@echo off
call "$vcvarsPath"
if errorlevel 1 exit /b 1
call "$emsdkEnvBat"
if errorlevel 1 exit /b 1
emcmake cmake -B "$WasmBuildDir" -G "NMake Makefiles" -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cd "$WasmBuildDir"
if errorlevel 1 exit /b 1
nmake
if errorlevel 1 exit /b 1
"@
    
    $tempScript = Join-Path $env:TEMP "build-wasm-temp.bat"
    $buildScript | Out-File -FilePath $tempScript -Encoding ASCII
    
    Write-Info "Running WASM build..."
    & cmd.exe /c $tempScript
    
    Remove-Item $tempScript -ErrorAction SilentlyContinue
    
    if ($LASTEXITCODE -ne 0) {
        Write-Error-Custom "WASM build failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }
    
    Write-Info "WASM build completed successfully!"
}

# Copy WASM files to visualization public directory
function Copy-WasmFiles {
    Write-Step "Copying WASM files to visualization directory"
    
    # Check if WASM files exist
    $wasmFile = Join-Path $WasmOutputDir "urdfx.wasm"
    $jsFile = Join-Path $WasmOutputDir "urdfx.js"
    
    if (-not (Test-Path $wasmFile)) {
        Write-Error-Custom "WASM file not found at $wasmFile"
        Write-Error-Custom "Please build WASM first or check build output"
        exit 1
    }
    
    if (-not (Test-Path $jsFile)) {
        Write-Error-Custom "JavaScript glue code not found at $jsFile"
        Write-Error-Custom "Please build WASM first or check build output"
        exit 1
    }
    
    # Ensure public directory exists
    if (-not (Test-Path $VizPublicDir)) {
        Write-Info "Creating visualization public directory..."
        New-Item -ItemType Directory -Path $VizPublicDir | Out-Null
    }
    
    # Copy files
    Write-Info "Copying urdfx.wasm..."
    Copy-Item $wasmFile -Destination (Join-Path $VizPublicDir "urdfx.wasm") -Force
    
    Write-Info "Copying urdfx.js..."
    Copy-Item $jsFile -Destination (Join-Path $VizPublicDir "urdfx.js") -Force
    
    Write-Info "WASM files copied successfully to $VizPublicDir"
}

# Install npm dependencies
function Install-Dependencies {
    Write-Step "Installing visualization app dependencies"
    
    Push-Location $VizDir
    try {
        if (-not (Test-Path (Join-Path $VizDir "node_modules"))) {
            Write-Info "Installing npm dependencies..."
            npm install
            if ($LASTEXITCODE -ne 0) {
                Write-Error-Custom "npm install failed"
                exit $LASTEXITCODE
            }
        } else {
            Write-Info "Dependencies already installed"
        }
    } finally {
        Pop-Location
    }
}

# Start development server
function Start-DevServer {
    Write-Step "Starting development server"
    
    Push-Location $VizDir
    try {
        Write-Info "Starting Vite dev server at http://localhost:5173/"
        Write-Host ""
        Write-Host "Press Ctrl+C to stop the server" -ForegroundColor Yellow
        Write-Host ""
        
        npm run dev
    } finally {
        Pop-Location
    }
}

# Build for production
function Build-Production {
    Write-Step "Building visualization app for production"
    
    Push-Location $VizDir
    try {
        Write-Info "Running production build..."
        npm run build
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "Production build failed"
            exit $LASTEXITCODE
        }
        
        Write-Info "Production build completed successfully!"
        Write-Info "Output directory: $(Join-Path $VizDir 'dist')"
    } finally {
        Pop-Location
    }
}

# Main execution flow
function Main {
    Write-Host ""
    Write-Info "urdfx Visualization Build Script"
    Write-Host ""
    
    # Step 1: Build WASM (unless skipped)
    if (-not $SkipWasm) {
        Build-Wasm
    } else {
        Write-Warn "Skipping WASM build (--SkipWasm flag provided)"
    }
    
    # Step 2: Copy WASM files
    Copy-WasmFiles
    
    # Step 3: Install dependencies
    Install-Dependencies
    
    # Step 4: Run dev server or production build
    if ($Dev) {
        Start-DevServer
    } elseif ($Build) {
        Build-Production
    } else {
        Write-Info "WASM files are ready for visualization app"
        Write-Host ""
        Write-Info "Next steps:"
        Write-Host "  To start development server:" -ForegroundColor White
        Write-Host "    .\scripts\build-viz.ps1 -Dev" -ForegroundColor Yellow
        Write-Host "  Or manually:" -ForegroundColor White
        Write-Host "    cd visualization" -ForegroundColor Yellow
        Write-Host "    npm run dev" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "  To build for production:" -ForegroundColor White
        Write-Host "    .\scripts\build-viz.ps1 -Build" -ForegroundColor Yellow
        Write-Host "  Or manually:" -ForegroundColor White
        Write-Host "    cd visualization" -ForegroundColor Yellow
        Write-Host "    npm run build" -ForegroundColor Yellow
        Write-Host ""
    }
    
    Write-Host ""
    Write-Info "Done!"
}

# Run main
Main
