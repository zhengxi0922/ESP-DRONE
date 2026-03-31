param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Args
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $scriptDir "esp-idf-env.ps1")

$firmwareDir = Join-Path (Split-Path -Parent $scriptDir) "firmware"
Push-Location $firmwareDir
try {
    & idf.py @Args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
