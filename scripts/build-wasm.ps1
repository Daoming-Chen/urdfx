# Build WASM for urdfx
# This script sets up the Visual Studio environment and Emscripten, then builds WASM

$ErrorActionPreference = "Stop"

# Find Visual Studio
$vsPath = & "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
if (-not $vsPath) {
    Write-Error "Visual Studio not found"
    exit 1
}

Write-Host "Using Visual Studio at: $vsPath" -ForegroundColor Green

# Import Visual Studio environment
$vcvarsPath = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvarsPath)) {
    Write-Error "vcvars64.bat not found at $vcvarsPath"
    exit 1
}

# Set up Emscripten
$emsdkPath = Join-Path (Split-Path $PSScriptRoot -Parent) "third_party\emsdk"
$emsdkEnvBat = Join-Path $emsdkPath "emsdk_env.bat"

if (-not (Test-Path $emsdkEnvBat)) {
    Write-Error "Emscripten SDK not found at $emsdkPath"
    exit 1
}

Write-Host "Setting up Emscripten environment..." -ForegroundColor Green

# Clean previous build
if (Test-Path "build-wasm") {
    Write-Host "Cleaning previous build-wasm directory..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force "build-wasm"
}

# Build using cmd.exe with proper environment setup
$buildScript = @"
@echo off
call "$vcvarsPath"
if errorlevel 1 exit /b 1
call "$emsdkEnvBat"
if errorlevel 1 exit /b 1
call emcmake cmake -B build-wasm -G "NMake Makefiles" -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cd build-wasm
if errorlevel 1 exit /b 1
nmake
if errorlevel 1 exit /b 1
cd ..
echo Build completed successfully
"@

$tempScript = Join-Path $env:TEMP "build-wasm-temp.bat"
$buildScript | Out-File -FilePath $tempScript -Encoding ASCII

Write-Host "Running build..." -ForegroundColor Green
& cmd.exe /c $tempScript

Remove-Item $tempScript

if ($LASTEXITCODE -eq 0) {
    Write-Host "WASM build completed successfully!" -ForegroundColor Green
    Write-Host "Output files are in: build-wasm\wasm\" -ForegroundColor Cyan
} else {
    Write-Error "WASM build failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}
