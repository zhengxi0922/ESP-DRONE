from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_docs_sync() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    script = repo_root / "tools" / "check_docs_sync.py"
    result = subprocess.run(
        [sys.executable, str(script), "--repo-root", str(repo_root)],
        cwd=repo_root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert result.returncode == 0, result.stdout
