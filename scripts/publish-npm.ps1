# Publish urdfx WASM bindings to npm
# This script builds the WASM module and publishes it to npm

param(
    [string]$Version = "",
    [switch]$DryRun = $false,
    [switch]$SkipBuild = $false,
    [switch]$SkipTests = $false
)

$ErrorActionPreference = "Stop"

function Write-Step {
    param([string]$Message)
    Write-Host "`n===> $Message" -ForegroundColor Cyan
}

function Write-Success {
    param([string]$Message)
    Write-Host "✓ $Message" -ForegroundColor Green
}

function Write-Error-Custom {
    param([string]$Message)
    Write-Host "✗ $Message" -ForegroundColor Red
}

# Get script directory and project root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$WasmDir = Join-Path $ProjectRoot "bindings\wasm"
$DistDir = Join-Path $WasmDir "dist"

Write-Host "╔════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║   urdfx npm Publisher for Windows     ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════╝" -ForegroundColor Cyan

# Step 1: Check npm login status
Write-Step "Checking npm authentication..."
$npmUser = npm whoami 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Error-Custom "Not logged in to npm. Please run 'npm login' first."
    exit 1
}
Write-Success "Logged in as: $npmUser"

# Step 2: Build WASM module (unless skipped)
if (-not $SkipBuild) {
    Write-Step "Building WASM module..."
    
    $BuildScript = Join-Path $ScriptDir "build-wasm.ps1"
    if (-not (Test-Path $BuildScript)) {
        Write-Error-Custom "Build script not found: $BuildScript"
        exit 1
    }
    
    & $BuildScript
    if ($LASTEXITCODE -ne 0) {
        Write-Error-Custom "WASM build failed"
        exit 1
    }
    Write-Success "WASM build completed"
} else {
    Write-Step "Skipping WASM build (--SkipBuild specified)"
}

# Step 3: Verify build artifacts
Write-Step "Verifying build artifacts..."
$BuildWasmDir = Join-Path $ProjectRoot "build-wasm\wasm"
$WasmJs = Join-Path $BuildWasmDir "urdfx.js"
$WasmBinary = Join-Path $BuildWasmDir "urdfx.wasm"

if (-not (Test-Path $WasmJs)) {
    Write-Error-Custom "WASM JavaScript file not found: $WasmJs"
    exit 1
}
if (-not (Test-Path $WasmBinary)) {
    Write-Error-Custom "WASM binary file not found: $WasmBinary"
    exit 1
}
Write-Success "Build artifacts verified"

# Step 4: Prepare distribution directory
Write-Step "Preparing distribution directory..."
if (Test-Path $DistDir) {
    Remove-Item -Recurse -Force $DistDir
}
New-Item -ItemType Directory -Path $DistDir -Force | Out-Null

# Copy build artifacts
Copy-Item $WasmJs $DistDir
Copy-Item $WasmBinary $DistDir

# Copy additional files
$TypeDefs = Join-Path $WasmDir "urdfx.d.ts"
$ReadmeSource = Join-Path $WasmDir "dist\README.md"
$License = Join-Path $ProjectRoot "LICENSE"

if (Test-Path $TypeDefs) {
    Copy-Item $TypeDefs $DistDir
} else {
    Write-Error-Custom "TypeScript definitions not found: $TypeDefs"
    exit 1
}

if (Test-Path $License) {
    Copy-Item $License $DistDir
} else {
    Write-Error-Custom "LICENSE file not found: $License"
    exit 1
}

# Create or verify README.md
if (-not (Test-Path $ReadmeSource)) {
    Write-Host "README.md not found, creating default..." -ForegroundColor Yellow
    $ReadmeContent = @"
# urdfx

A modern WebAssembly robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving.

## Installation

``````bash
npm install urdfx
``````

## Quick Start

``````javascript
const createUrdfxModule = require('urdfx');

(async () => {
  const urdfx = await createUrdfxModule();
  
  const urdfXml = ``<?xml version="1.0"?>
  <robot name="my_robot">
    <!-- Your URDF content -->
  </robot>``;
  
  const robot = urdfx.Robot.fromURDFString(urdfXml);
  console.log('Robot:', robot.getName());
  
  robot.dispose();
})();
``````

## Documentation

For full documentation, visit: https://github.com/Daoming-Chen/urdfx

## License

MIT
"@
    $ReadmeContent | Out-File -FilePath (Join-Path $DistDir "README.md") -Encoding utf8
} else {
    Copy-Item $ReadmeSource $DistDir
}

Write-Success "Distribution directory prepared"

# Step 5: Create/update package.json
Write-Step "Preparing package.json..."

$PackageJsonPath = Join-Path $DistDir "package.json"

# Read existing package.json or create new
if (Test-Path $PackageJsonPath) {
    $PackageJson = Get-Content $PackageJsonPath -Raw | ConvertFrom-Json
} else {
    $PackageJson = @{
        name = "urdfx"
        version = "1.0.0"
        description = "A modern WebAssembly robotics kinematics library"
        main = "urdfx.js"
        types = "urdfx.d.ts"
        type = "module"
        files = @("urdfx.js", "urdfx.wasm", "urdfx.d.ts", "README.md", "LICENSE")
        keywords = @("robotics", "kinematics", "urdf", "inverse-kinematics", "forward-kinematics", "jacobian", "webassembly", "wasm", "robot", "3d")
        author = "urdfx contributors"
        license = "MIT"
        repository = @{
            type = "git"
            url = "https://github.com/Daoming-Chen/urdfx.git"
        }
        bugs = @{
            url = "https://github.com/Daoming-Chen/urdfx/issues"
        }
        homepage = "https://github.com/Daoming-Chen/urdfx#readme"
        engines = @{
            node = ">=14.0.0"
        }
    }
}

# Update version if specified
if ($Version) {
    Write-Host "Updating version to: $Version" -ForegroundColor Yellow
    $PackageJson.version = $Version
}

# Save package.json
$PackageJson | ConvertTo-Json -Depth 10 | Out-File -FilePath $PackageJsonPath -Encoding utf8
Write-Success "package.json prepared (version: $($PackageJson.version))"

# Step 6: Create .npmignore
$NpmIgnorePath = Join-Path $DistDir ".npmignore"
$NpmIgnoreContent = @"
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
"@
$NpmIgnoreContent | Out-File -FilePath $NpmIgnorePath -Encoding utf8

Write-Success "Distribution files prepared"

# Step 7: Run npm pack (dry run preview)
Write-Step "Previewing package contents..."
Push-Location $DistDir
try {
    npm pack --dry-run
    if ($LASTEXITCODE -ne 0) {
        Write-Error-Custom "npm pack preview failed"
        exit 1
    }
} finally {
    Pop-Location
}

# Step 8: Publish to npm
if ($DryRun) {
    Write-Step "Performing dry run publish..."
    Push-Location $DistDir
    try {
        npm publish --dry-run
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "npm publish dry run failed"
            exit 1
        }
        Write-Success "Dry run completed successfully"
        Write-Host "`nTo publish for real, run without --DryRun flag" -ForegroundColor Yellow
    } finally {
        Pop-Location
    }
} else {
    Write-Host "`n⚠️  Ready to publish urdfx@$($PackageJson.version) to npm" -ForegroundColor Yellow
    Write-Host "Press Enter to continue or Ctrl+C to cancel..." -ForegroundColor Yellow
    Read-Host
    
    Write-Step "Publishing to npm..."
    Push-Location $DistDir
    try {
        npm publish
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Custom "npm publish failed"
            exit 1
        }
        
        Write-Host "`n╔════════════════════════════════════════╗" -ForegroundColor Green
        Write-Host "║   Successfully published to npm! 🎉   ║" -ForegroundColor Green
        Write-Host "╚════════════════════════════════════════╝" -ForegroundColor Green
        Write-Host "`nPackage: urdfx@$($PackageJson.version)" -ForegroundColor Green
        Write-Host "URL: https://www.npmjs.com/package/urdfx" -ForegroundColor Cyan
        Write-Host "`nInstall with: npm install urdfx" -ForegroundColor Cyan
    } finally {
        Pop-Location
    }
}

# Step 9: Clean up (optional)
Write-Host "`nDistribution files are in: $DistDir" -ForegroundColor Cyan
Write-Host "You can safely delete this directory after publishing." -ForegroundColor Gray
