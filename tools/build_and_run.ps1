param(
    [string]$ConfigFile = "user\\config\\missions\\default.json",
    [switch]$BuildOnly,
    [switch]$ListComponents,
    [string]$BuildType = "Release"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot

Push-Location $ProjectRoot

try {
    if (-not (Test-Path "build")) {
        New-Item -ItemType Directory -Path "build" | Out-Null
    }

    Push-Location build
    cmake .. -DCMAKE_BUILD_TYPE=$BuildType
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }

    cmake --build . --config $BuildType
    if ($LASTEXITCODE -ne 0) { throw "Build failed" }
    Pop-Location

    if ($BuildOnly) {
        return
    }

    if ($ListComponents) {
        & ".\\build\\bin\\gnc_sim.exe" --list-components
        return
    }

    & ".\\build\\bin\\gnc_sim.exe" $ConfigFile
} finally {
    Pop-Location
}
