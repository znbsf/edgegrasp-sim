[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BagDirectory
)

$ErrorActionPreference = "Stop"
$ros2Command = Get-Command ros2 -ErrorAction SilentlyContinue
if ($null -eq $ros2Command) {
    throw "ros2 is not on PATH; source the intended ROS workspace first"
}

$clockInfo = @(& ros2 topic info /clock 2>$null)
$publisherCount = $null
foreach ($line in $clockInfo) {
    if ($line -match '^\s*Publisher count:\s*(\d+)\s*$') {
        $publisherCount = [int]$Matches[1]
        break
    }
}
if ($null -eq $publisherCount) {
    $clockText = ($clockInfo -join "`n")
    if ($clockInfo.Count -eq 0 -or $clockText -match '(?i)not found|unknown topic') {
        $publisherCount = 0
    } else {
        throw "could not determine the existing /clock publisher count; refusing replay"
    }
}
if ($publisherCount -gt 0) {
    [Console]::Error.WriteLine("refusing replay; /clock already has $publisherCount publisher(s)")
    [Console]::Error.WriteLine("stop Gazebo/other clock publishers; rosbag --clock must be the sole /clock authority")
    exit 3
}

Write-Host "Replay contract: this script does not launch edgegrasp_replay; start it separately."
Write-Host "rosbag --clock is the sole /clock authority; do not run Gazebo concurrently."

$RawReplayTopics = @(
    "/joint_states",
    "/edgegrasp/target_3d",
    "/edgegrasp/tracked_target",
    "/camera_head/color/image_raw",
    "/camera_head/depth/image_rect_raw",
    "/camera_head/depth/camera_info"
)

& ros2 bag play "$BagDirectory" --clock --topics @RawReplayTopics
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
