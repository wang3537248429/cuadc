param(
    [string]$OutputDirectory = ""
)

$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
$ProjectDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$WorkspaceDirectory = Split-Path -Parent $ProjectDirectory

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $ProjectDirectory "dist\windows"
}
$OutputDirectory = [System.IO.Path]::GetFullPath($OutputDirectory)
$VirtualEnvironment = Join-Path $ProjectDirectory ".venv-windows"
$VenvPython = Join-Path $VirtualEnvironment "Scripts\python.exe"

if (-not (Test-Path $VenvPython)) {
    $PythonLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($null -ne $PythonLauncher) {
        & $PythonLauncher.Source -3.10 -m venv $VirtualEnvironment
    } else {
        $Python = Get-Command python -ErrorAction SilentlyContinue
        if ($null -eq $Python) {
            throw "未找到 Python。请安装 64 位 Python 3.10，并勾选 py launcher。"
        }
        & $Python.Source -m venv $VirtualEnvironment
    }
}

& $VenvPython -m pip install --disable-pip-version-check --upgrade pip wheel
& $VenvPython -m pip install `
    -r (Join-Path $ProjectDirectory "requirements.txt") `
    -r (Join-Path $ProjectDirectory "requirements-build.txt")

$WorkDirectory = Join-Path $ProjectDirectory "build\windows"
$BundleDirectory = Join-Path $OutputDirectory "CUADC中文靶标监控"
$ZipPath = Join-Path $OutputDirectory "CUADC-ground-station-win-x64.zip"
$ChecksumPath = Join-Path $OutputDirectory "SHA256SUMS.txt"

if (Test-Path $WorkDirectory) {
    Remove-Item -Recurse -Force $WorkDirectory
}
if (Test-Path $BundleDirectory) {
    Remove-Item -Recurse -Force $BundleDirectory
}
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null

Push-Location $ProjectDirectory
try {
    & $VenvPython -m PyInstaller `
        --noconfirm `
        --clean `
        --workpath $WorkDirectory `
        --distpath $OutputDirectory `
        (Join-Path $ProjectDirectory "CUADCGroundStation.spec")
} finally {
    Pop-Location
}

$GuidePath = Join-Path $WorkspaceDirectory "README.md"
if (-not (Test-Path $GuidePath)) { $GuidePath = Join-Path $ProjectDirectory "README.md" }
Copy-Item $GuidePath `
    (Join-Path $BundleDirectory "Windows使用说明.md")
Copy-Item (Join-Path $WorkspaceDirectory "LICENSE") $BundleDirectory
Copy-Item (Join-Path $WorkspaceDirectory "THIRD_PARTY_NOTICES.md") $BundleDirectory

if (Test-Path $ZipPath) {
    Remove-Item -Force $ZipPath
}
Compress-Archive -Path (Join-Path $BundleDirectory "*") -DestinationPath $ZipPath -CompressionLevel Optimal
$Hash = (Get-FileHash -Algorithm SHA256 $ZipPath).Hash.ToLowerInvariant()
"$Hash  $([System.IO.Path]::GetFileName($ZipPath))" | Set-Content -Encoding ascii $ChecksumPath

Write-Host "Windows 便携包已生成：$ZipPath"
Write-Host "SHA-256：$Hash"
