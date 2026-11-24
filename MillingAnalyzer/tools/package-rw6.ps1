param([string]$Build = "build-bezier-viewer", [string]$Config = "Release")
$ErrorActionPreference = "Stop"
$destination = Join-Path $Build "RobotStudio-RW6"
New-Item -ItemType Directory -Force $destination | Out-Null
Copy-Item (Join-Path $Build "$Config/MillingAnalyzer.exe") $destination
Copy-Item (Join-Path $Build "$Config/*.dll") $destination
Copy-Item (Join-Path $Build "$Config/licenses") $destination -Recurse -Force
Copy-Item "MillingAnalyzer/tools/Start-Milling.cmd" $destination
Copy-Item "experiments/robotstudio_pose_force/MillingPose6510.mod" $destination
Copy-Item "docs/20-cam-egm-rw6.md" (Join-Path $destination "README-RobotStudio.md")
$examples = Join-Path $destination "examples"
New-Item -ItemType Directory -Force $examples | Out-Null
Copy-Item "MillingAnalyzer/data/automatic-box.step" $examples
Copy-Item "MillingAnalyzer/data/stepped-solid.step" $examples
Copy-Item "experiments/robotstudio_pose_force/smoke.poses.csv" $examples
Write-Host "RobotStudio RW6 package: $destination"
