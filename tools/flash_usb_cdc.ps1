param(
    [string]$Port = "COM8",
    [int]$Baud = 460800,
    [switch]$SkipAppBootloaderReboot
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$firmwareDir = Join-Path $repoRoot "firmware"
$buildDir = Join-Path $firmwareDir "build"
$flashArgs = Join-Path $buildDir "flash_args"
$cliRoot = Join-Path $repoRoot "tools\esp_drone_cli"

if ($env:PYTHONPATH) {
    $env:PYTHONPATH = "$cliRoot;$env:PYTHONPATH"
} else {
    $env:PYTHONPATH = $cliRoot
}

if (-not (Test-Path -LiteralPath $flashArgs)) {
    throw "Missing firmware/build/flash_args. Run '.\tools\idf.ps1 build' first."
}

if (-not $SkipAppBootloaderReboot) {
    & python -m esp_drone_cli.cli.main --serial $Port reboot-bootloader
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "App bootloader reboot failed. If this board still runs older firmware without reboot-bootloader support,"
        Write-Host "hold BOOT, tap RESET, release BOOT, then rerun with -SkipAppBootloaderReboot."
        exit $LASTEXITCODE
    }
    Start-Sleep -Milliseconds 1500
}

. (Join-Path $scriptDir "esp-idf-env.ps1")

Push-Location $buildDir
try {
    & python -m esptool --chip esp32s3 --port $Port --baud $Baud --before no_reset --after hard_reset write_flash "@flash_args"
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "Flash failed. If this board still runs older firmware without reboot-bootloader support,"
        Write-Host "hold BOOT, tap RESET, release BOOT, then rerun with -SkipAppBootloaderReboot."
    }
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
