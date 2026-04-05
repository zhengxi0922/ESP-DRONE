# ESP-IDF Environment

## 中文摘要

- 本仓库固定使用 `ESP-IDF v5.5.1`。
- Windows 环境下推荐通过 `tools/esp-idf-env.ps1` 和 `tools/idf.ps1` 进入和调用构建环境。
- 不要混用别的 IDF 主次版本，否则构建结果不可保证。

This repository is locked to `ESP-IDF v5.5.1`.

Verified local installation on this workstation:

- `IDF_PATH = D:\Espressif\v5.5.1\esp-idf`
- Python env created by `idf_tools.py`
- Target used for this firmware: `esp32s3`

Verified build command:

```powershell
cmd /c "call D:\Espressif\v5.5.1\esp-idf\export.bat >nul && cd /d D:\0Work\Codex\ESP-drone\firmware && idf.py set-target esp32s3 && idf.py build"
```

Repository helpers:

- `tools/esp-idf-env.ps1`
  Loads the ESP-IDF v5.5.1 environment into the current PowerShell session.
- `tools/idf.ps1`
  Runs `idf.py` against `firmware/` after loading the same environment.

Notes:

- The local `idf-env.json` was reduced to `esp32s3` so tool export does not pull unrelated chip targets.
- The firmware build has been verified against `ESP-IDF v5.5.1`; do not mix this repository with a different IDF major or minor version.
