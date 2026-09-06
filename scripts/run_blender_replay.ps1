[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Run,
    [Parameter(Mandatory)][string]$Output,
    [Parameter(Mandatory)][string]$World,
    [string]$Distro = 'Ubuntu-24.04',
    [string]$Blender = "$env:USERPROFILE\Documents\edgegrasp-tools\blender-4.5.13-windows-x64\blender.exe",
    [switch]$RenderVideo
)
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $Blender)) { throw 'Run scripts/install_blender_portable.ps1 first, or specify -Blender.' }
if (Test-Path -LiteralPath $Output) { throw 'Output must be a new directory; existing results are preserved.' }
$absoluteOutput = [IO.Path]::GetFullPath($Output)
$repoRoot = Split-Path -Parent $PSScriptRoot
$wslRoot = & wsl.exe -d $Distro --exec wslpath -a -u $repoRoot.Replace('\', '/')
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve WSL repository path' }
$wslRoot = $wslRoot.Trim()
$wslOutput = & wsl.exe -d $Distro --exec wslpath -a -u $absoluteOutput.Replace('\', '/')
if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve WSL output path' }
$wslOutput = $wslOutput.Trim()
& wsl.exe -d $Distro --exec bash "$wslRoot/scripts/export_blender_replay.sh" $Run $wslOutput --world $World
if ($LASTEXITCODE -ne 0) { throw 'Recorded-data export failed; partial output preserved for diagnosis' }
& $Blender --background --factory-startup --python-exit-code 1 --python "$PSScriptRoot/build_blender_replay.py" -- $absoluteOutput --render-previews
if ($LASTEXITCODE -ne 0) { throw 'Blender scene generation failed' }
& $Blender --background --factory-startup --python-exit-code 1 --python "$PSScriptRoot/verify_blender_replay.py" -- $absoluteOutput
if ($LASTEXITCODE -ne 0) { throw 'Baked replay verification failed' }
if ($RenderVideo) {
    & $Blender --background --factory-startup --python-exit-code 1 --python "$PSScriptRoot/render_blender_replay.py" -- $absoluteOutput
    if ($LASTEXITCODE -ne 0) { throw 'Video render failed' }
}
Write-Output "Replay ready: $absoluteOutput\replay.blend"
