param()

$ErrorActionPreference = "Stop"

$idfPath = "D:\Espressif\v5.5.1\esp-idf"
$exportBat = Join-Path $idfPath "export.bat"

if (-not (Test-Path $exportBat)) {
    throw "ESP-IDF export script not found: $exportBat"
}

$envDump = & cmd /c "`"$exportBat`" >nul && set"
if ($LASTEXITCODE -ne 0) {
    throw "Failed to activate ESP-IDF environment from $exportBat"
}

foreach ($line in $envDump) {
    $index = $line.IndexOf("=")
    if ($index -lt 1) {
        continue
    }
    $name = $line.Substring(0, $index)
    $value = $line.Substring($index + 1)
    Set-Item -Path "Env:$name" -Value $value
}

Write-Host "ESP-IDF environment loaded: $env:IDF_PATH"
