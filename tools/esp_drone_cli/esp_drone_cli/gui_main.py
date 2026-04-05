"""GUI 启动入口。"""

from __future__ import annotations

import os
import sys


def main(argv: list[str] | None = None) -> int:
    """启动桌面调试界面。

    Args:
        argv: 预留的命令行参数列表。当前实现不解析该参数，只用于保持入口签名稳定。

    Returns:
        GUI 退出码。依赖缺失时返回 `1`，正常退出时返回 Qt 事件循环的返回值。

    Raises:
        ModuleNotFoundError: 缺失的依赖不是 `PyQt5` 或 `pyqtgraph` 时向上抛出。

    注意:
        调用前需要安装 GUI 依赖；当前入口会强制将 `PYQTGRAPH_QT_LIB` 设为 `PyQt5`。
    """

    os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")
    try:
        from .gui.main_window import run_gui
    except ModuleNotFoundError as exc:
        missing_name = exc.name or str(exc)
        if "PyQt5" in missing_name or "pyqtgraph" in missing_name:
            print(
                "PyQt5 and pyqtgraph are required for esp-drone-gui. Install them with: pip install -e .[gui]",
                file=sys.stderr,
            )
            return 1
        raise
    return run_gui(argv)


if __name__ == "__main__":
    raise SystemExit(main())
