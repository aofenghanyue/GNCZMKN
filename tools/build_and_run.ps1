param(
    [string]$ConfigFile = "",
    [string]$ActiveProject = "",
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
    $configureArgs = @("..", "-DCMAKE_BUILD_TYPE=$BuildType")
    if ($ActiveProject -ne "") {
        $configureArgs += "-DGNC_ACTIVE_PROJECT=$ActiveProject"
    }
    cmake @configureArgs
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

    if ($ConfigFile -ne "") {
        & ".\\build\\bin\\gnc_sim.exe" --config $ConfigFile
    } else {
        & ".\\build\\bin\\gnc_sim.exe"
    }
} finally {
    Pop-Location
}
