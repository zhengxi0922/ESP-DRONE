"""`python -m esp_drone_cli` 的命令行入口。"""

from .cli.main import main


if __name__ == "__main__":
    raise SystemExit(main())
