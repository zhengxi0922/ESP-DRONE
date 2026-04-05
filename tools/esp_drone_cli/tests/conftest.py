# ============================================================
# @file conftest.py
# @brief ESP-DRONE Python ????????
# @details ??????????????????
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
