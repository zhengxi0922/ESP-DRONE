# ============================================================
# @file csv_log.py
# @brief ESP-DRONE CSV ????
# @details ????? TelemetrySample ???? CSV???? CLI ? GUI ????????
# @author Codex
# @date 2026-04-05
# @version 1.0
# ============================================================

from __future__ import annotations

import csv
import threading
from pathlib import Path

from .models import TELEMETRY_CSV_FIELDS, TelemetrySample


class CsvTelemetryLogger:
    def __init__(self, output_path: Path) -> None:
        self._output_path = output_path
        self._handle = output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(TELEMETRY_CSV_FIELDS)
        self._lock = threading.Lock()
        self._rows = 0

    @property
    def output_path(self) -> Path:
        return self._output_path

    @property
    def rows_written(self) -> int:
        return self._rows

    def write(self, sample: TelemetrySample) -> None:
        with self._lock:
            self._writer.writerow(sample.to_csv_row())
            self._handle.flush()
            self._rows += 1

    def close(self) -> None:
        with self._lock:
            self._handle.close()
