from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    try:
        from .gui.main_window import run_gui
    except ModuleNotFoundError as exc:
        missing_name = exc.name or str(exc)
        if "PySide6" in missing_name:
            print(
                "PySide6 is required for esp-drone-gui. Install it with: pip install -e .[gui]",
                file=sys.stderr,
            )
            return 1
        raise
    return run_gui(argv)


if __name__ == "__main__":
    raise SystemExit(main())
