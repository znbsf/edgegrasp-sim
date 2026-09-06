[CmdletBinding()]
param(
    [string]$OutputDirectory = (Join-Path $env:USERPROFILE ".ros\edgegrasp_bags\edgegrasp_$(Get-Date -Format yyyyMMdd_HHmmss)"),
    [ValidateSet("ros_sim", "system", "gazebo", "mock")]
    [string]$Profile = "ros_sim"
)

$ErrorActionPreference = "Stop"
$ros2Command = Get-Command ros2 -ErrorAction SilentlyContinue
if ($null -eq $ros2Command) {
    throw "ros2 is not on PATH; source the intended ROS workspace first"
}

$simProfile = $Profile -in @("ros_sim", "gazebo")
$parent = Split-Path -Parent $OutputDirectory
if ([string]::IsNullOrWhiteSpace($parent)) {
    $parent = (Get-Location).Path
}
New-Item -ItemType Directory -Force -Path $parent | Out-Null

$topics = @(
    "/clock",
    "/joint_states",
    "/gripper_controller/controller_state",
    "/edgegrasp/target_3d",
    "/edgegrasp/tracked_target",
    "/edgegrasp/permission_evidence",
    "/edgegrasp/measured_cube",
    "/edgegrasp/perception_status",
    "/tf",
    "/tf_static",
    "/edgegrasp/motion_allowed",
    "/edgegrasp/safety_status",
    "/edgegrasp/arm_joint_trajectory_request",
    "/edgegrasp/gripper_joint_trajectory_request",
    "/edgegrasp/trajectory_gate_status",
    "/edgegrasp/interface_ready",
    "/edgegrasp/interface_status",
    "/edgegrasp/target_cube_pose",
    "/edgegrasp/target_cube_contacts",
    "/edgegrasp/table_contacts",
    "/edgegrasp/planning_scene_status",
    "/edgegrasp/grasp_timeline",
    "/edgegrasp/grasp_sequence_status",
    "/edgegrasp/grasp_sequence_terminal",
    "/edgegrasp/grasp_physics_status",
    "/camera_head/color/image_raw",
    "/camera_head/depth/image_rect_raw",
    "/camera_head/depth/camera_info"
)

if ($simProfile) {
    Write-Host "Waiting for one /clock sample before ros_sim recording..."
    & ros2 topic echo /clock --once | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "could not observe a /clock sample; ros_sim recording was not started"
    }
    & ros2 bag record -s mcap --use-sim-time --output "$OutputDirectory" --topics @topics
} else {
    & ros2 bag record -s mcap --output "$OutputDirectory" --topics @topics
}
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
