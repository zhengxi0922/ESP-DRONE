from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
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
