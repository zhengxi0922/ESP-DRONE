"""遥测 CSV 日志写入器。"""

from __future__ import annotations

import csv
import threading
from pathlib import Path
from typing import Callable

from .models import TELEMETRY_CSV_FIELDS, TelemetrySample


class CsvTelemetryLogger:
    """将遥测样本持续写入 CSV 文件。

    该类只负责文件写入与线程安全，不负责启动或停止设备侧遥测流。
    """

    def __init__(
        self,
        output_path: Path,
        *,
        fieldnames: list[str] | None = None,
        extra_row_fn: Callable[[TelemetrySample], dict[str, object]] | None = None,
    ) -> None:
        """创建日志写入器并立即写入表头。

        Args:
            output_path: 输出 CSV 文件路径。父目录需要已存在。
            fieldnames: 可选 CSV 列名列表；默认使用标准遥测列。
            extra_row_fn: 可选附加列提供器；会在每行写入前合并到显示映射。

        Raises:
            OSError: 目标文件无法创建时抛出。
        """

        self._output_path = output_path
        self._fieldnames = list(fieldnames or TELEMETRY_CSV_FIELDS)
        self._extra_row_fn = extra_row_fn
        self._handle = output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(self._fieldnames)
        self._lock = threading.Lock()
        self._rows = 0

    @property
    def output_path(self) -> Path:
        """当前日志文件路径。"""

        return self._output_path

    @property
    def rows_written(self) -> int:
        """已成功写入的数据行数，不含表头。"""

        return self._rows

    def write(self, sample: TelemetrySample) -> None:
        """写入一行遥测数据。

        Args:
            sample: 待写入的遥测样本。

        Raises:
            ValueError: 文件已关闭时底层写入失败可能抛出。
            OSError: 文件系统写入失败时抛出。
        """

        with self._lock:
            row_map = sample.to_display_map()
            if self._extra_row_fn is not None:
                row_map.update(self._extra_row_fn(sample))
            self._writer.writerow([row_map[name] for name in self._fieldnames])
            self._handle.flush()
            self._rows += 1

    def close(self) -> None:
        """关闭日志文件。

        Returns:
            None.

        注意:
            该方法可重复调用；底层文件已关闭时由 Python 自身维持幂等行为。
        """

        with self._lock:
            self._handle.close()
