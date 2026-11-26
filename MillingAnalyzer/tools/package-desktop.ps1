param([string]$Build = "build-bezier-viewer")
$ErrorActionPreference = "Stop"
$dest = Join-Path $Build "MillingCAM-Desktop"
dotnet build MillingAnalyzer/desktop/MillingCAM.csproj -c Release -o $dest
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$engine = New-Item -ItemType Directory -Force (Join-Path $dest "engine")
Copy-Item "$Build/Release/MillingAnalyzer.exe","$Build/Release/*.dll","$Build/Release/licenses" $engine -Recurse -Force
$examples = New-Item -ItemType Directory -Force (Join-Path $dest "examples")
Copy-Item "MillingAnalyzer/data/automatic-box.step","MillingAnalyzer/data/stepped-solid.step" $examples
Copy-Item "docs/23-desktop-wobj.md" (Join-Path $dest "START-RU.md")
