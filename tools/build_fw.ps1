$ErrorActionPreference = "Stop"

# Clear MSys/MinGW env vars that confuse ESP-IDF
Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue
Remove-Item Env:MINGW_PREFIX -ErrorAction SilentlyContinue
Remove-Item Env:MSYSCON -ErrorAction SilentlyContinue
Remove-Item Env:SHELL -ErrorAction SilentlyContinue
Remove-Item Env:TERM -ErrorAction SilentlyContinue

$idfPath = "D:\Espressif\v5.5.1\esp-idf"
$exportBat = Join-Path $idfPath "export.bat"

# Run export.bat in clean cmd
$envDump = & cmd /c "`"$exportBat`" >nul 2>&1 && set"
if ($LASTEXITCODE -ne 0) {
    # Try without the .bat check
    Write-Host "Standard export failed, trying direct env setup..."
    $env:PATH = "D:\Espressif\v5.5.1\esp-idf\tools;D:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;C:\Users\zheng\.espressif\python_env\idf5.5_py3.12_env\Scripts;$env:PATH"
    $env:IDF_PATH = "D:\Espressif\v5.5.1\esp-idf"
    $env:IDF_PYTHON_ENV_PATH = "C:\Users\zheng\.espressif\python_env\idf5.5_py3.12_env"
} else {
    foreach ($line in $envDump) {
        $index = $line.IndexOf("=")
        if ($index -lt 1) { continue }
        $name = $line.Substring(0, $index)
        $value = $line.Substring($index + 1)
        [Environment]::SetEnvironmentVariable($name, $value, "Process")
    }
}

Write-Host "IDF_PATH: $env:IDF_PATH"

Push-Location D:\0Work\Codex\ESP-drone-main-merge\firmware
try {
    $pythonPath = "C:\Users\zheng\.espressif\python_env\idf5.5_py3.12_env\Scripts\python.exe"
    $idfPyPath = "$env:IDF_PATH\tools\idf.py"
    & $pythonPath $idfPyPath build
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
