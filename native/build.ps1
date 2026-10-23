#requires -Version 5.1
<#
.SYNOPSIS
    Builds the native planegcs_native DLL (and the smoke test) with MSVC.

.DESCRIPTION
    The script bootstraps the pieces that are missing on a typical Windows
    machine:
      * Eigen 3.4.0 headers are downloaded into third_party/eigen;
      * Visual Studio is located with vswhere;
      * CMake is taken from Visual Studio, from PATH, or installed with winget.

    Boost is not required: the two tiny Boost usages in the upstream sources
    were replaced with standard C++ (see native/README.md).

.PARAMETER Configuration
    Debug or Release. Release by default.

.PARAMETER Clean
    Deletes native/build before configuring.

.PARAMETER SkipSmoke
    Does not run the native smoke test after building.

.PARAMETER CopyTo
    Optional directory to copy planegcs_native.dll (and its PDB) into.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\native\build.ps1

.EXAMPLE
    .\native\build.ps1 -Configuration Debug -Clean -CopyTo ..\..\topic\RE-4-regulatization\src\Planegcs.Native\runtimes\win-x64\native
#>
[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Configuration = 'Release',

    [switch]$Clean,

    [switch]$SkipSmoke,

    [string]$CopyTo = '',

    [string]$EigenVersion = '3.4.0'
)

$ErrorActionPreference = 'Stop'

$NativeDir = $PSScriptRoot
$RepoRoot = Split-Path -Parent $NativeDir
$ThirdPartyDir = Join-Path $RepoRoot 'third_party'
$EigenDir = Join-Path $ThirdPartyDir 'eigen'
$BuildDir = Join-Path $NativeDir 'build'
$OutDir = Join-Path $NativeDir 'out'
$PlanegcsSourceDir = Join-Path $RepoRoot 'planegcs'

function Write-Step([string]$Message)
{
    Write-Host ''
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Get-VisualStudio
{
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'

    if (-not (Test-Path -LiteralPath $vswhere)) {
        return $null
    }

    $instances = & $vswhere -all -prerelease -products * -format json | ConvertFrom-Json

    foreach ($instance in $instances) {
        $msvcDir = Join-Path $instance.installationPath 'VC\Tools\MSVC'

        if (Test-Path -LiteralPath $msvcDir) {
            return $instance
        }
    }

    return $null
}

function Install-BuildTools
{
    Write-Step 'Visual Studio with the C++ toolset was not found'
    Write-Host 'Trying to install Visual Studio 2022 Build Tools with the VCTools workload...'

    $arguments = @(
        'install',
        '--id', 'Microsoft.VisualStudio.2022.BuildTools',
        '-e',
        '--accept-source-agreements',
        '--accept-package-agreements',
        '--override', '--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended'
    )

    & winget @arguments

    if ($LASTEXITCODE -ne 0) {
        throw 'winget could not install Visual Studio Build Tools. Install the "Desktop development with C++" workload manually and run this script again.'
    }
}

function Get-CMakePath([object]$VisualStudio)
{
    if ($null -ne $VisualStudio) {
        $bundled = Join-Path $VisualStudio.installationPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'

        if (Test-Path -LiteralPath $bundled) {
            return $bundled
        }
    }

    $command = Get-Command cmake.exe -ErrorAction SilentlyContinue

    if ($null -ne $command) {
        return $command.Source
    }

    return $null
}

function Install-CMake
{
    Write-Step 'CMake was not found'
    Write-Host 'Trying to install CMake with winget...'

    & winget install --id Kitware.CMake -e --accept-source-agreements --accept-package-agreements

    if ($LASTEXITCODE -ne 0) {
        throw 'winget could not install CMake. Install it manually from https://cmake.org/download/ and run this script again.'
    }

    $candidates = @(
        (Join-Path $env:ProgramFiles 'CMake\bin\cmake.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'CMake\bin\cmake.exe')
    )

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    $command = Get-Command cmake.exe -ErrorAction SilentlyContinue

    if ($null -ne $command) {
        return $command.Source
    }

    throw 'CMake was installed but is still not visible in PATH. Open a new terminal and run this script again.'
}

function Ensure-Eigen
{
    $denseHeader = Join-Path $EigenDir 'Eigen\Dense'

    if (Test-Path -LiteralPath $denseHeader) {
        Write-Host "Eigen: $EigenDir"
        return
    }

    Write-Step "Downloading Eigen $EigenVersion"
    New-Item -ItemType Directory -Force -Path $ThirdPartyDir | Out-Null

    $archive = Join-Path $ThirdPartyDir "eigen-$EigenVersion.zip"
    $url = "https://gitlab.com/libeigen/eigen/-/archive/$EigenVersion/eigen-$EigenVersion.zip"

    if (-not (Test-Path -LiteralPath $archive)) {
        $curl = Get-Command curl.exe -ErrorAction SilentlyContinue

        if ($null -ne $curl) {
            & $curl.Source -L --fail --retry 3 -o $archive $url

            if ($LASTEXITCODE -ne 0) {
                throw "Failed to download $url"
            }
        }
        else {
            Invoke-WebRequest -Uri $url -OutFile $archive -UseBasicParsing
        }
    }

    $extractDir = Join-Path $ThirdPartyDir 'eigen.extract'

    if (Test-Path -LiteralPath $extractDir) {
        Remove-Item -LiteralPath $extractDir -Recurse -Force
    }

    New-Item -ItemType Directory -Force -Path $extractDir | Out-Null

    $tar = Get-Command tar.exe -ErrorAction SilentlyContinue

    if ($null -ne $tar) {
        & $tar.Source -xf $archive -C $extractDir

        if ($LASTEXITCODE -ne 0) {
            throw "Failed to extract $archive"
        }
    }
    else {
        Expand-Archive -LiteralPath $archive -DestinationPath $extractDir -Force
    }

    $extractedRoot = Join-Path $extractDir "eigen-$EigenVersion"

    if (-not (Test-Path -LiteralPath (Join-Path $extractedRoot 'Eigen\Dense'))) {
        throw "Unexpected Eigen archive layout: 'Eigen\Dense' was not found in $extractedRoot"
    }

    # The native build needs the headers and the license files, not the tests,
    # benchmarks, docs or CMake glue that come with the upstream archive.
    $keep = @(
        'Eigen',
        'COPYING.APACHE',
        'COPYING.BSD',
        'COPYING.GPL',
        'COPYING.LGPL',
        'COPYING.MINPACK',
        'COPYING.MPL2',
        'COPYING.README',
        'README.md'
    )

    Get-ChildItem -LiteralPath $extractedRoot -Force |
        Where-Object { $keep -notcontains $_.Name } |
        Remove-Item -Recurse -Force

    if (Test-Path -LiteralPath $EigenDir) {
        Remove-Item -LiteralPath $EigenDir -Recurse -Force
    }

    Move-Item -LiteralPath $extractedRoot -Destination $EigenDir
    Remove-Item -LiteralPath $extractDir -Recurse -Force
    Remove-Item -LiteralPath $archive -Force

    Write-Host "Eigen: $EigenDir"
}

function Get-GeneratorName([object]$VisualStudio, [string]$CMakePath)
{
    $major = [int]($VisualStudio.installationVersion.Split('.')[0])

    $generator = switch ($major)
    {
        18 { 'Visual Studio 18 2026' }
        17 { 'Visual Studio 17 2022' }
        16 { 'Visual Studio 16 2019' }
        default { throw "Unsupported Visual Studio version: $($VisualStudio.installationVersion)" }
    }

    $help = (& $CMakePath --help) -join "`n"

    if (-not ($help -match [regex]::Escape($generator))) {
        throw "CMake '$CMakePath' does not support the '$generator' generator. Update CMake or install the version bundled with Visual Studio."
    }

    return $generator
}

Write-Step 'Locating Visual Studio'
$visualStudio = Get-VisualStudio

if ($null -eq $visualStudio) {
    Install-BuildTools
    $visualStudio = Get-VisualStudio

    if ($null -eq $visualStudio) {
        throw 'Visual Studio is still not detected after installation.'
    }
}

Write-Host "Visual Studio: $($visualStudio.displayName) ($($visualStudio.installationVersion))"
Write-Host "Path: $($visualStudio.installationPath)"

Write-Step 'Locating CMake'
$cmake = Get-CMakePath $visualStudio

if ($null -eq $cmake) {
    $cmake = Install-CMake
}

Write-Host "CMake: $cmake"
Write-Host (& $cmake --version | Select-Object -First 1)

Ensure-Eigen

if (-not (Test-Path -LiteralPath (Join-Path $PlanegcsSourceDir 'GCS.cpp'))) {
    throw "PlaneGCS sources were not found in '$PlanegcsSourceDir'."
}

$generator = Get-GeneratorName $visualStudio $cmake

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Step 'Cleaning native/build'
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}

Write-Step "Configuring ($generator, $Configuration)"
& $cmake -S $NativeDir -B $BuildDir -G $generator -A x64 `
    -DPLANEGCS_SOURCE_DIR="$PlanegcsSourceDir" `
    -DPLANEGCS_EIGEN_DIR="$EigenDir" `
    -DPLANEGCS_BUILD_SMOKE=ON

if ($LASTEXITCODE -ne 0) {
    throw 'CMake configuration failed.'
}

Write-Step "Building ($Configuration)"
& $cmake --build $BuildDir --config $Configuration --parallel

if ($LASTEXITCODE -ne 0) {
    throw 'Build failed.'
}

$dll = Join-Path $OutDir "$Configuration\planegcs_native.dll"
$pdb = Join-Path $OutDir "$Configuration\planegcs_native.pdb"

if (-not (Test-Path -LiteralPath $dll)) {
    throw "Build succeeded but '$dll' was not produced."
}

Write-Host ''
Write-Host "DLL: $dll" -ForegroundColor Green

if ($CopyTo.Length -gt 0) {
    Write-Step "Copying to $CopyTo"
    New-Item -ItemType Directory -Force -Path $CopyTo | Out-Null
    Copy-Item -LiteralPath $dll -Destination $CopyTo -Force

    if (Test-Path -LiteralPath $pdb) {
        Copy-Item -LiteralPath $pdb -Destination $CopyTo -Force
    }
}

if (-not $SkipSmoke) {
    $smoke = Join-Path $OutDir "$Configuration\planegcs_native_smoke.exe"

    if (Test-Path -LiteralPath $smoke) {
        Write-Step 'Running native smoke test'
        & $smoke

        if ($LASTEXITCODE -ne 0) {
            throw 'Native smoke test failed.'
        }
    }
    else {
        Write-Warning "Smoke test executable was not found: $smoke"
    }
}

Write-Host ''
Write-Host 'Done.' -ForegroundColor Green
