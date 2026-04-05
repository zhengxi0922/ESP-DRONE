# ESP-IDF 环境

**语言 / Language：** [English](./esp_idf_env.md) | **简体中文**

本仓库固定使用 `ESP-IDF v5.5.1`。

## 已验证的本机安装

- `IDF_PATH = D:\Espressif\v5.5.1\esp-idf`
- Python 环境由 `idf_tools.py` 创建
- 本固件使用的目标芯片为 `esp32s3`

## 已验证的构建命令

```powershell
cmd /c "call D:\Espressif\v5.5.1\esp-idf\export.bat >nul && cd /d D:\0Work\Codex\ESP-drone\firmware && idf.py set-target esp32s3 && idf.py build"
```

## 仓库辅助脚本

- `tools/esp-idf-env.ps1`
  将 ESP-IDF v5.5.1 环境加载到当前 PowerShell 会话。
- `tools/idf.ps1`
  在加载同一环境后，对 `firmware/` 目录执行 `idf.py`。

## 说明

- 本地 `idf-env.json` 已精简为 `esp32s3`，避免工具导出无关芯片目标。
- 固件构建已经按 `ESP-IDF v5.5.1` 验证，不要与其它 IDF 主版本或次版本混用。
