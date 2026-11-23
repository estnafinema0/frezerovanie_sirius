# Pinned, separate MSVC x64 dependency build. Does not touch nanopb or Freza sources.
param(
    [string]$Root = "$PSScriptRoot/../out/deps",
    [ValidateSet('Release','Debug')][string]$Configuration = 'Release'
)
$ErrorActionPreference = 'Stop'
function Checked([string]$Program, [string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
function Find-CMake {
    $command = Get-Command cmake -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere) {
        $vsRoot = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($LASTEXITCODE -eq 0 -and $vsRoot) {
            $bundled = Join-Path $vsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
            if (Test-Path $bundled) { return $bundled }
        }
    }
    throw 'cmake not found. Install the Visual Studio C++ CMake tools, or add cmake to PATH.'
}
$CMake = Find-CMake
$Root = [IO.Path]::GetFullPath($Root)
New-Item -ItemType Directory -Force -Path $Root | Out-Null
$revision = '1a450b7c3d3b669224a2a5d09aab7f319794574c'
$source = Join-Path $Root 'opencamlib-src'
if (!(Test-Path $source)) {
    Checked 'git' @('clone', 'https://github.com/aewallin/opencamlib.git', $source)
    Checked 'git' @('-C', $source, 'checkout', '--detach', $revision)
}
$head = & git -C $source rev-parse HEAD
if ($LASTEXITCODE -ne 0 -or $head -ne $revision) { throw "Dependency checkout must be exactly $revision; use a fresh -Root." }
$dirty = & git -C $source status --porcelain
if ($LASTEXITCODE -ne 0 -or $dirty) { throw 'Dependency has local changes; use a fresh -Root (nothing overwritten).' }
$archive = Join-Path $Root 'boost_1_87_0.tar.gz'
if (!(Test-Path $archive)) {
    $url = 'https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz'
    try {
        Invoke-WebRequest -UseBasicParsing $url -OutFile $archive
    } catch {
        # Some Windows hosts fail Schannel certificate delegation for PowerShell only;
        # curl usually succeeds on the same machine.
        Write-Host "PowerShell download failed ($($_.Exception.Message)); retrying with curl."
        if (Test-Path $archive) { Remove-Item $archive -Force }
        Checked 'curl.exe' @('-L', '--fail', '--output', $archive, $url)
    }
}
$sha = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
if ($sha -ne 'f55c340aa49763b1925ccf02b2e83f35fdcf634c9d5164a2acb87540173c741d') { throw 'Boost archive SHA256 mismatch.' }
$boost = Join-Path $Root 'boost_1_87_0'
if (!(Test-Path $boost)) { Checked 'tar' @('-xzf', $archive, '-C', $Root) }
$build = Join-Path $Root "ocl-build-$Configuration"
$prefix = Join-Path $Root "ocl-$Configuration"
Checked $CMake @('-S', $source, '-B', $build, '-G', 'Visual Studio 17 2022', '-A', 'x64',
    '-DBUILD_CXX_LIB=ON', '-DBUILD_PY_LIB=OFF', '-DBUILD_NODEJS_LIB=OFF', '-DBUILD_DOC=OFF',
    '-DUSE_OPENMP=OFF', '-DVERSION_STRING=1a450b7', "-DBOOST_ROOT=$boost",
    "-DCMAKE_BUILD_TYPE=$Configuration", "-DCMAKE_INSTALL_PREFIX=$prefix")
Checked $CMake @('--build', $build, '--config', $Configuration, '--parallel', '4')
Checked $CMake @('--install', $build, '--config', $Configuration)
Write-Host "OpenCAMLib installed: $prefix"
Write-Host "Re-run the FrezaEgm configure step; CAM is detected from out/deps automatically."
