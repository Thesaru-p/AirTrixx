param(
    [string]$PythonBin = $env:PYTHON_BIN
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Resolve-Path (Join-Path $ScriptDir "..")
$VenvDir = Join-Path $RootDir ".venv-build-windows"
$DistDir = Join-Path $RootDir "dist"
$OutDir = Join-Path $RootDir ".dist"
$ZipPath = Join-Path $OutDir "AirTrixx-windows-x64.zip"

$RunningOnWindows = $true
if (Get-Variable -Name IsWindows -Scope Global -ErrorAction SilentlyContinue) {
    $RunningOnWindows = $IsWindows
} else {
    $RunningOnWindows = [System.Environment]::OSVersion.Platform -eq [System.PlatformID]::Win32NT
}

if (-not $RunningOnWindows) {
    throw "Windows installer builds must run on Windows x64."
}

if (-not $PythonBin) {
    $PythonBin = "py"
    $PythonArgs = $null
    foreach ($Version in @("-3.11", "-3.12", "-3.13")) {
        & $PythonBin $Version --version *> $null
        if ($LASTEXITCODE -eq 0) {
            $PythonArgs = @($Version)
            break
        }
    }
    if (-not $PythonArgs) {
        throw "Python 3.11, 3.12, or 3.13 is required. Set PYTHON_BIN to a Python executable if needed."
    }
} else {
    $PythonArgs = @()
}

Set-Location $RootDir
& $PythonBin @PythonArgs -m venv $VenvDir
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"
& $VenvPython -m pip install --upgrade pip setuptools wheel
& $VenvPython -m pip install -r "python_app\requirements.txt" -r "packaging\requirements-build.txt"
& $VenvPython "packaging\make_icons.py"
& $VenvPython "packaging\download_models.py"
& $VenvPython -m PyInstaller --noconfirm --clean "packaging\AirTrixx.spec"
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller failed. Close any running AirTrixx.exe process and run the build again."
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Iscc = Get-Command "ISCC.exe" -ErrorAction SilentlyContinue
if ($Iscc) {
    & $Iscc.Source "packaging\windows\AirTrixx.iss"
    Write-Host "Built installer in $OutDir"
} else {
    if (Test-Path $ZipPath) {
        Remove-Item $ZipPath
    }
    Compress-Archive -Path (Join-Path $DistDir "AirTrixx\*") -DestinationPath $ZipPath
    Write-Host "Inno Setup not found; built portable zip instead: $ZipPath"
}
