[CmdletBinding()]
param(
    [int]$ReplayRuns = 100,
    [string]$PythonExecutable = "python"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$PreviousPythonPath = $env:PYTHONPATH
Push-Location $ProjectRoot
try {
    $env:PYTHONPATH = Join-Path $ProjectRoot "src"
    & $PythonExecutable -m compileall -q src tests ros_ws\src
    if ($LASTEXITCODE -ne 0) { throw "compileall failed" }

    & $PythonExecutable -m pytest
    if ($LASTEXITCODE -ne 0) { throw "pytest failed" }

    & $PythonExecutable .\scripts\validate_project.py
    if ($LASTEXITCODE -ne 0) { throw "structural validation failed" }

    & $PythonExecutable -m edgegrasp replay --runs $ReplayRuns
    if ($LASTEXITCODE -ne 0) { throw "deterministic replay failed" }
}
finally {
    $env:PYTHONPATH = $PreviousPythonPath
    Pop-Location
}
