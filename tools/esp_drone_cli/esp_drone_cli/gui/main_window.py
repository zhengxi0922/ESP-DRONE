from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import pyqtgraph as pg
from PyQt5.QtCore import QByteArray, QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QKeySequence
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QDoubleSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from esp_drone_cli.core import DeviceSession, ParamValue, TelemetrySample


APP_ORG = "ESP-DRONE"
APP_NAME = "ESP-DRONE GUI"

STATUS_STYLE = {
    "neutral": "background:#ECEFF1;color:#263238;border:1px solid #B0BEC5;border-radius:10px;padding:2px 8px;font-weight:600;",
    "ok": "background:#E8F5E9;color:#1B5E20;border:1px solid #81C784;border-radius:10px;padding:2px 8px;font-weight:600;",
    "warn": "background:#FFF8E1;color:#E65100;border:1px solid #FFB74D;border-radius:10px;padding:2px 8px;font-weight:600;",
    "danger": "background:#FFEBEE;color:#B71C1C;border:1px solid #E57373;border-radius:10px;padding:2px 8px;font-weight:700;",
    "active": "background:#E3F2FD;color:#0D47A1;border:1px solid #64B5F6;border-radius:10px;padding:2px 8px;font-weight:600;",
}

ROW_STYLE = {
    "neutral": (QBrush(QColor("#263238")), QBrush(QColor("#FFFFFF"))),
    "ok": (QBrush(QColor("#1B5E20")), QBrush(QColor("#E8F5E9"))),
    "warn": (QBrush(QColor("#E65100")), QBrush(QColor("#FFF8E1"))),
    "danger": (QBrush(QColor("#B71C1C")), QBrush(QColor("#FFEBEE"))),
    "active": (QBrush(QColor("#0D47A1")), QBrush(QColor("#E3F2FD"))),
}

TYPE_NAMES = {
    0: "bool",
    1: "u8",
    2: "u32",
    3: "i32",
    4: "float",
}

ARM_STATE_TEXT = {
    0: ("DISARMED", "neutral"),
    1: ("ARMED", "ok"),
    2: ("FAILSAFE", "warn"),
    3: ("FAULT_LOCK", "danger"),
}

FAILSAFE_TEXT = {
    0: ("NONE", "neutral"),
    1: ("KILL", "danger"),
    2: ("RC_TIMEOUT", "warn"),
    3: ("IMU_TIMEOUT", "warn"),
    4: ("IMU_PARSE", "warn"),
    5: ("BAT_CRITICAL", "danger"),
    6: ("LOOP_OVERRUN", "warn"),
}

CONTROL_MODE_TEXT = {
    0: ("IDLE", "neutral"),
    1: ("AXIS_TEST", "active"),
    2: ("RATE_TEST", "active"),
}

IMU_MODE_TEXT = {
    0: ("RAW", "warn"),
    1: ("DIRECT", "ok"),
}


def _format_float(value: float) -> str:
    return f"{value:.3f}"


def _format_value(name: str, value: object) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        if "battery_voltage" in name:
            return f"{value:.3f}"
        if "quat_" in name:
            return f"{value:.4f}"
        return _format_float(value)
    return str(value)


def _set_badge(label: QLabel, text: str, role: str) -> None:
    label.setText(text)
    label.setStyleSheet(STATUS_STYLE.get(role, STATUS_STYLE["neutral"]))


def _status_from_map(mapping: dict[int, tuple[str, str]], value: int) -> tuple[str, str]:
    return mapping.get(value, (str(value), "neutral"))


def _device_info_text(info: object | None) -> str:
    if info is None:
        return "No active device session."
    if hasattr(info, "protocol_version"):
        protocol = getattr(info, "protocol_version", "?")
        imu_mode = getattr(info, "imu_mode", "?")
        arm_state = getattr(info, "arm_state", "?")
        stream_enabled = getattr(info, "stream_enabled", "?")
        return (
            f"protocol={protocol} | imu_mode={imu_mode} | "
            f"arm_state={arm_state} | stream={stream_enabled}"
        )
    return str(info)


def _row_role_for_field(name: str, raw_value: object) -> tuple[str, str] | None:
    if not isinstance(raw_value, int):
        return None
    if name == "arm_state":
        return _status_from_map(ARM_STATE_TEXT, raw_value)
    if name == "failsafe_reason":
        return _status_from_map(FAILSAFE_TEXT, raw_value)
    if name == "control_mode":
        return _status_from_map(CONTROL_MODE_TEXT, raw_value)
    if name == "imu_mode":
        return _status_from_map(IMU_MODE_TEXT, raw_value)
    return None


class CopyableTableWidget(QTableWidget):
    def keyPressEvent(self, event) -> None:  # pragma: no cover - UI handling
        if event.matches(QKeySequence.Copy):
            self.copy_selection()
            return
        super().keyPressEvent(event)

    def copy_selection(self) -> None:
        ranges = self.selectedRanges()
        if not ranges:
            return
        selected_range = ranges[0]
        rows = []
        for row in range(selected_range.topRow(), selected_range.bottomRow() + 1):
            fields = []
            for col in range(selected_range.leftColumn(), selected_range.rightColumn() + 1):
                item = self.item(row, col)
                fields.append("" if item is None else item.text())
            rows.append("\t".join(fields))
        QApplication.clipboard().setText("\n".join(rows))


class QtSessionBridge(QObject):
    connection_changed = pyqtSignal(object)
    telemetry_received = pyqtSignal(object)
    event_received = pyqtSignal(str)
    command_finished = pyqtSignal(str, object)
    error_raised = pyqtSignal(str)

    def __init__(self, session: DeviceSession) -> None:
        super().__init__()
        self._session = session
        self._connection_token = self._session.subscribe_connection_state(
            lambda payload: self.connection_changed.emit(payload)
        )
        self._telemetry_token = self._session.subscribe_telemetry(
            lambda sample: self.telemetry_received.emit(sample)
        )
        self._event_token = self._session.subscribe_event_log(
            lambda message: self.event_received.emit(message)
        )

    def dispose(self) -> None:
        self._session.unsubscribe(self._connection_token)
        self._session.unsubscribe(self._telemetry_token)
        self._session.unsubscribe(self._event_token)

    def run_async(self, label: str, callback) -> None:
        def worker() -> None:
            try:
                result = callback()
                self.command_finished.emit(label, result)
            except Exception as exc:  # pragma: no cover - depends on runtime/device state
                self.error_raised.emit(f"{label}: {exc}")

        threading.Thread(target=worker, daemon=True, name=f"esp-drone-gui-{label}").start()


class TelemetryHistory:
    def __init__(self, max_samples: int = 12000) -> None:
        self._series: dict[str, deque[tuple[float, float]]] = {}
        self._max_samples = max_samples

    def append(self, sample: TelemetrySample) -> None:
        values = sample.to_display_map()
        timestamp = time.monotonic()
        for key, value in values.items():
            if not isinstance(value, (int, float)):
                continue
            if key not in self._series:
                self._series[key] = deque(maxlen=self._max_samples)
            self._series[key].append((timestamp, float(value)))

    def clear(self) -> None:
        self._series.clear()

    def slice(self, name: str, window_s: float) -> tuple[list[float], list[float]]:
        samples = list(self._series.get(name, ()))
        if not samples:
            return [], []
        latest = samples[-1][0]
        cutoff = latest - window_s
        xs: list[float] = []
        ys: list[float] = []
        for sample_time, value in samples:
            if sample_time < cutoff:
                continue
            xs.append(sample_time - latest)
            ys.append(value)
        return xs, ys


@dataclass(slots=True)
class ChartSpec:
    title: str
    fields: list[tuple[str, str]]
    y_label: str


class MainWindow(QMainWindow):
    TELEMETRY_FIELDS = [
        "gyro_x", "gyro_y", "gyro_z",
        "roll_deg", "pitch_deg", "yaw_deg",
        "rate_setpoint_roll", "rate_setpoint_pitch", "rate_setpoint_yaw",
        "motor1", "motor2", "motor3", "motor4",
        "battery_voltage", "battery_adc_raw",
        "imu_age_us", "loop_dt_us",
        "imu_mode", "imu_health", "arm_state", "failsafe_reason", "control_mode",
    ]

    PARAM_HELP = {
        "telemetry_usb_hz": "USB CDC telemetry target rate in Hz. Device-side bounds still apply.",
        "telemetry_udp_hz": "UDP telemetry target rate in Hz. Device-side bounds still apply.",
        "imu_mode": "0 = RAW, 1 = DIRECT. Bench validation is currently centered on DIRECT mode.",
        "imu_return_rate_code": "ATK-MS901M return-rate code. 0x00 is 250Hz, 0x01 is 200Hz.",
        "motor_idle_duty": "Brushed motor armed idle floor, normalized 0..1.",
        "motor_max_duty": "Brushed motor output ceiling, normalized 0..1.",
        "bringup_test_base_duty": "Base duty used in axis/rate bench tests. Keep low on a restrained frame.",
    }

    CHART_SPECS = [
        ChartSpec("Gyro", [("gyro_x", "gyro_x"), ("gyro_y", "gyro_y"), ("gyro_z", "gyro_z")], "deg/s"),
        ChartSpec("Attitude", [("roll_deg", "roll_deg"), ("pitch_deg", "pitch_deg"), ("yaw_deg", "yaw_deg")], "deg"),
        ChartSpec("Motors", [("motor1", "motor1"), ("motor2", "motor2"), ("motor3", "motor3"), ("motor4", "motor4")], "duty"),
        ChartSpec("Battery", [("battery_voltage", "battery_voltage")], "V"),
    ]

    CHART_COLORS = ["#D32F2F", "#1976D2", "#388E3C", "#7B1FA2"]

    def __init__(
        self,
        session: DeviceSession | None = None,
        bridge_cls: type[QtSessionBridge] = QtSessionBridge,
        serial_port_provider: Callable[[], Iterable] | None = None,
        settings: QSettings | None = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("ESP-DRONE Bench Debugger")
        self.resize(1660, 980)

        self._session = session or DeviceSession()
        self._bridge = bridge_cls(self._session)
        self._serial_port_provider = serial_port_provider or list_ports.comports
        self._settings = settings or QSettings(APP_ORG, APP_NAME)
        self._params: list[ParamValue] = []
        self._selected_param: ParamValue | None = None
        self._last_telemetry: TelemetrySample | None = None
        self._chart_bundles: dict[str, dict[str, object]] = {}
        self._history = TelemetryHistory()
        self._charts_running = True
        self._stream_enabled = False
        self._last_result = "-"
        self._closing = False

        self._build_ui()
        self._wire_signals()
        self._load_settings()
        self._refresh_serial_ports()
        self._refresh_enabled_state()

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(100)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start()

    def _build_ui(self) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(8, 8, 8, 8)

        self.vertical_splitter = QSplitter(Qt.Vertical)
        self.main_splitter = QSplitter(Qt.Horizontal)

        self.left_panel = self._build_left_panel()
        self.center_panel = self._build_center_panel()
        self.right_panel = self._build_right_panel()
        self.bottom_panel = self._build_bottom_panel()

        self.main_splitter.addWidget(self.left_panel)
        self.main_splitter.addWidget(self.center_panel)
        self.main_splitter.addWidget(self.right_panel)
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setStretchFactor(2, 0)
        self.main_splitter.setSizes([340, 850, 420])

        self.vertical_splitter.addWidget(self.main_splitter)
        self.vertical_splitter.addWidget(self.bottom_panel)
        self.vertical_splitter.setStretchFactor(0, 1)
        self.vertical_splitter.setStretchFactor(1, 0)
        self.vertical_splitter.setSizes([760, 220])

        layout.addWidget(self.vertical_splitter)
        self.setCentralWidget(root)

    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_safety_group())
        layout.addWidget(self._build_debug_group())
        layout.addStretch(1)
        return panel

    def _build_center_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(self._build_telemetry_group())
        layout.addWidget(self._build_charts_group())
        return panel

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(self._build_params_group())
        return panel

    def _build_bottom_panel(self) -> QWidget:
        group = QGroupBox("Event Log / Errors / Recent Result")
        layout = QVBoxLayout(group)

        bar = QHBoxLayout()
        self.clear_log_button = QPushButton("Clear Log")
        self.copy_log_button = QPushButton("Copy Log")
        self.save_log_button = QPushButton("Save Log")
        self.last_result_label = QLabel("-")
        self.last_result_label.setWordWrap(True)
        bar.addWidget(self.clear_log_button)
        bar.addWidget(self.copy_log_button)
        bar.addWidget(self.save_log_button)
        bar.addWidget(QLabel("Last Result"))
        bar.addWidget(self.last_result_label, 1)
        layout.addLayout(bar)

        self.event_log_edit = QPlainTextEdit()
        self.event_log_edit.setReadOnly(True)
        self.event_log_edit.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.event_log_edit.setPlaceholderText("Connection events, command results, and errors will appear here.")
        layout.addWidget(self.event_log_edit, 1)

        summary = QGridLayout()
        self.last_log_path_label = QLabel("-")
        self.last_log_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.last_error_label = QLabel("-")
        self.last_error_label.setWordWrap(True)
        self.last_error_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        summary.addWidget(QLabel("Last Log"), 0, 0)
        summary.addWidget(self.last_log_path_label, 0, 1)
        summary.addWidget(QLabel("Last Error"), 1, 0)
        summary.addWidget(self.last_error_label, 1, 1)
        layout.addLayout(summary)
        return group

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        self.link_type_combo = QComboBox()
        self.link_type_combo.addItems(["serial", "udp"])

        self.serial_port_combo = QComboBox()
        self.serial_port_combo.setEditable(True)
        self.serial_port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.refresh_ports_button = QPushButton("Refresh")

        self.baudrate_spin = QSpinBox()
        self.baudrate_spin.setRange(9600, 2000000)
        self.baudrate_spin.setValue(115200)

        self.udp_host_edit = QLineEdit("192.168.4.1")
        self.udp_port_spin = QSpinBox()
        self.udp_port_spin.setRange(1, 65535)
        self.udp_port_spin.setValue(2391)

        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")

        self.connection_status_chip = QLabel("Disconnected")
        _set_badge(self.connection_status_chip, "Disconnected", "neutral")
        self.connection_info_label = QLabel("No active device session.")
        self.connection_info_label.setWordWrap(True)
        self.connection_error_detail = QLabel("No connection error.")
        self.connection_error_detail.setWordWrap(True)
        self.connection_error_detail.setTextInteractionFlags(Qt.TextSelectableByMouse)

        layout.addWidget(QLabel("Link"), 0, 0)
        layout.addWidget(self.link_type_combo, 0, 1)
        layout.addWidget(self.connection_status_chip, 0, 2)
        layout.addWidget(QLabel("Serial Port"), 1, 0)
        layout.addWidget(self.serial_port_combo, 1, 1)
        layout.addWidget(self.refresh_ports_button, 1, 2)
        layout.addWidget(QLabel("Baud"), 2, 0)
        layout.addWidget(self.baudrate_spin, 2, 1)
        layout.addWidget(QLabel("UDP Host"), 3, 0)
        layout.addWidget(self.udp_host_edit, 3, 1, 1, 2)
        layout.addWidget(QLabel("UDP Port"), 4, 0)
        layout.addWidget(self.udp_port_spin, 4, 1)
        layout.addWidget(self.connect_button, 5, 1)
        layout.addWidget(self.disconnect_button, 5, 2)
        layout.addWidget(QLabel("Session"), 6, 0)
        layout.addWidget(self.connection_info_label, 6, 1, 1, 2)
        layout.addWidget(QLabel("Last Connection Error"), 7, 0)
        layout.addWidget(self.connection_error_detail, 7, 1, 1, 2)
        return group

    def _build_safety_group(self) -> QGroupBox:
        group = QGroupBox("Safety Control")
        layout = QGridLayout(group)

        self.arm_button = QPushButton("Arm")
        self.disarm_button = QPushButton("Disarm")
        self.kill_button = QPushButton("Kill")
        self.kill_button.setStyleSheet("background:#C62828;color:white;font-weight:700;")
        self.reboot_button = QPushButton("Reboot")

        self.arm_state_chip = QLabel("-")
        self.failsafe_chip = QLabel("-")
        self.control_mode_chip = QLabel("-")
        self.stream_chip = QLabel("-")
        self.imu_mode_chip = QLabel("-")

        layout.addWidget(self.arm_button, 0, 0)
        layout.addWidget(self.disarm_button, 0, 1)
        layout.addWidget(self.kill_button, 0, 2)
        layout.addWidget(self.reboot_button, 0, 3)
        layout.addWidget(QLabel("arm_state"), 1, 0)
        layout.addWidget(self.arm_state_chip, 1, 1)
        layout.addWidget(QLabel("failsafe_reason"), 1, 2)
        layout.addWidget(self.failsafe_chip, 1, 3)
        layout.addWidget(QLabel("control_mode"), 2, 0)
        layout.addWidget(self.control_mode_chip, 2, 1)
        layout.addWidget(QLabel("imu_mode"), 2, 2)
        layout.addWidget(self.imu_mode_chip, 2, 3)
        layout.addWidget(QLabel("stream"), 3, 0)
        layout.addWidget(self.stream_chip, 3, 1)
        return group

    def _build_telemetry_group(self) -> QGroupBox:
        group = QGroupBox("Realtime Telemetry")
        layout = QVBoxLayout(group)

        bar = QHBoxLayout()
        self.stream_on_button = QPushButton("Stream On")
        self.stream_off_button = QPushButton("Stream Off")
        self.stream_rate_spin = QSpinBox()
        self.stream_rate_spin.setRange(1, 200)
        self.stream_rate_spin.setValue(200)
        self.apply_stream_rate_button = QPushButton("Apply Hz")
        self.copy_telemetry_button = QPushButton("Copy Selected")
        bar.addWidget(self.stream_on_button)
        bar.addWidget(self.stream_off_button)
        bar.addWidget(QLabel("Target Hz"))
        bar.addWidget(self.stream_rate_spin)
        bar.addWidget(self.apply_stream_rate_button)
        bar.addStretch(1)
        bar.addWidget(self.copy_telemetry_button)
        layout.addLayout(bar)

        self.telemetry_table = CopyableTableWidget(len(self.TELEMETRY_FIELDS), 2)
        self.telemetry_table.setHorizontalHeaderLabels(["Field", "Value"])
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.telemetry_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.telemetry_table.setAlternatingRowColors(True)
        self.telemetry_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.telemetry_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            field_item = QTableWidgetItem(name)
            value_item = QTableWidgetItem("-")
            self.telemetry_table.setItem(row, 0, field_item)
            self.telemetry_table.setItem(row, 1, value_item)
        layout.addWidget(self.telemetry_table)
        return group

    def _build_charts_group(self) -> QGroupBox:
        group = QGroupBox("Charts")
        layout = QVBoxLayout(group)

        control_bar = QHBoxLayout()
        self.chart_toggle_button = QPushButton("Pause Charts")
        self.clear_charts_button = QPushButton("Clear Charts")
        self.chart_window_combo = QComboBox()
        self.chart_window_combo.addItem("5 s", 5.0)
        self.chart_window_combo.addItem("10 s", 10.0)
        self.chart_window_combo.addItem("30 s", 30.0)
        self.chart_window_combo.setCurrentIndex(1)
        control_bar.addWidget(self.chart_toggle_button)
        control_bar.addWidget(self.clear_charts_button)
        control_bar.addWidget(QLabel("Window"))
        control_bar.addWidget(self.chart_window_combo)
        control_bar.addStretch(1)
        layout.addLayout(control_bar)

        grid = QGridLayout()
        for index, spec in enumerate(self.CHART_SPECS):
            box = QGroupBox(spec.title)
            box_layout = QVBoxLayout(box)

            checkbox_bar = QHBoxLayout()
            plot = pg.PlotWidget()
            plot.setBackground("w")
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", spec.y_label)
            plot.setLabel("bottom", "time", units="s")
            plot.addLegend(offset=(10, 10))
            plot.setMenuEnabled(False)

            curves: dict[str, object] = {}
            checks: dict[str, QCheckBox] = {}
            for field_index, (field_name, field_label) in enumerate(spec.fields):
                checkbox = QCheckBox(field_label)
                checkbox.setChecked(True)
                checks[field_name] = checkbox
                checkbox_bar.addWidget(checkbox)
                pen = pg.mkPen(self.CHART_COLORS[field_index % len(self.CHART_COLORS)], width=2)
                curves[field_name] = plot.plot([], [], pen=pen, name=field_label)
            checkbox_bar.addStretch(1)
            box_layout.addLayout(checkbox_bar)
            box_layout.addWidget(plot, 1)

            self._chart_bundles[spec.title] = {
                "spec": spec,
                "plot": plot,
                "curves": curves,
                "checks": checks,
            }
            grid.addWidget(box, index // 2, index % 2)

        layout.addLayout(grid)
        return group

    def _build_params_group(self) -> QGroupBox:
        group = QGroupBox("Parameters")
        layout = QVBoxLayout(group)

        top_bar = QHBoxLayout()
        self.param_search_edit = QLineEdit()
        self.param_search_edit.setPlaceholderText("Search parameter...")
        self.refresh_params_button = QPushButton("Refresh")
        self.save_params_button = QPushButton("Save")
        self.reset_params_button = QPushButton("Reset")
        self.export_params_button = QPushButton("Export JSON")
        self.import_params_button = QPushButton("Import JSON")
        top_bar.addWidget(self.param_search_edit, 1)
        top_bar.addWidget(self.refresh_params_button)
        top_bar.addWidget(self.save_params_button)
        top_bar.addWidget(self.reset_params_button)
        top_bar.addWidget(self.export_params_button)
        top_bar.addWidget(self.import_params_button)
        layout.addLayout(top_bar)

        self.params_splitter = QSplitter(Qt.Vertical)

        self.params_table = CopyableTableWidget(0, 3)
        self.params_table.setHorizontalHeaderLabels(["Name", "Type", "Current Value"])
        self.params_table.verticalHeader().setVisible(False)
        self.params_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.params_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.params_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.params_table.setAlternatingRowColors(True)
        self.params_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.params_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.params_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.params_splitter.addWidget(self.params_table)

        detail = QWidget()
        detail_layout = QVBoxLayout(detail)
        form = QFormLayout()
        self.param_name_label = QLabel("-")
        self.param_type_label = QLabel("-")
        self.param_current_value_label = QLabel("-")
        self.param_new_value_edit = QLineEdit()
        self.param_new_value_edit.setPlaceholderText("New value for selected parameter")
        self.param_hint_label = QLabel("Select a parameter to see a local hint. Device-side validation remains authoritative.")
        self.param_hint_label.setWordWrap(True)
        self.param_help_text = QPlainTextEdit()
        self.param_help_text.setReadOnly(True)
        self.param_help_text.setPlaceholderText("Parameter description placeholder")
        self.param_help_text.setMaximumHeight(120)
        form.addRow("Name", self.param_name_label)
        form.addRow("Type", self.param_type_label)
        form.addRow("Current", self.param_current_value_label)
        form.addRow("New Value", self.param_new_value_edit)
        detail_layout.addLayout(form)
        detail_layout.addWidget(self.param_hint_label)
        detail_layout.addWidget(QLabel("Description"))
        detail_layout.addWidget(self.param_help_text)
        self.set_param_button = QPushButton("Set Selected")
        detail_layout.addWidget(self.set_param_button)
        self.params_splitter.addWidget(detail)
        self.params_splitter.setStretchFactor(0, 1)
        self.params_splitter.setStretchFactor(1, 0)
        layout.addWidget(self.params_splitter, 1)
        return group

    def _build_debug_group(self) -> QGroupBox:
        group = QGroupBox("Debug Actions")
        layout = QVBoxLayout(group)

        warning = QLabel("Bench only: keep props removed or the frame restrained. Motor/rate tests default to conservative values.")
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#E65100;font-weight:600;")
        layout.addWidget(warning)

        motor_box = QGroupBox("motor_test")
        motor_layout = QGridLayout(motor_box)
        self.motor_combo = QComboBox()
        self.motor_combo.addItems(["m1", "m2", "m3", "m4"])
        self.motor_duty_spin = QDoubleSpinBox()
        self.motor_duty_spin.setRange(0.0, 1.0)
        self.motor_duty_spin.setDecimals(3)
        self.motor_duty_spin.setSingleStep(0.01)
        self.motor_duty_spin.setValue(0.05)
        self.motor_start_button = QPushButton("Start")
        self.motor_stop_button = QPushButton("Stop")
        motor_layout.addWidget(QLabel("Motor"), 0, 0)
        motor_layout.addWidget(self.motor_combo, 0, 1)
        motor_layout.addWidget(QLabel("Duty"), 0, 2)
        motor_layout.addWidget(self.motor_duty_spin, 0, 3)
        motor_layout.addWidget(self.motor_start_button, 0, 4)
        motor_layout.addWidget(self.motor_stop_button, 0, 5)
        layout.addWidget(motor_box)

        calib_box = QGroupBox("Calibration")
        calib_layout = QHBoxLayout(calib_box)
        self.calib_gyro_button = QPushButton("Calib Gyro")
        self.calib_level_button = QPushButton("Calib Level")
        calib_layout.addWidget(self.calib_gyro_button)
        calib_layout.addWidget(self.calib_level_button)
        calib_layout.addStretch(1)
        layout.addWidget(calib_box)

        rate_box = QGroupBox("rate_test")
        rate_layout = QGridLayout(rate_box)
        self.rate_axis_combo = QComboBox()
        self.rate_axis_combo.addItems(["roll", "pitch", "yaw"])
        self.rate_value_spin = QDoubleSpinBox()
        self.rate_value_spin.setRange(-720.0, 720.0)
        self.rate_value_spin.setDecimals(1)
        self.rate_value_spin.setSingleStep(5.0)
        self.rate_value_spin.setValue(20.0)
        self.rate_start_button = QPushButton("Start")
        self.rate_stop_button = QPushButton("Stop")
        rate_layout.addWidget(QLabel("Axis"), 0, 0)
        rate_layout.addWidget(self.rate_axis_combo, 0, 1)
        rate_layout.addWidget(QLabel("Rate dps"), 0, 2)
        rate_layout.addWidget(self.rate_value_spin, 0, 3)
        rate_layout.addWidget(self.rate_start_button, 0, 4)
        rate_layout.addWidget(self.rate_stop_button, 0, 5)
        layout.addWidget(rate_box)

        log_box = QGroupBox("CSV / Log")
        log_layout = QGridLayout(log_box)
        self.log_path_edit = QLineEdit(str(Path.cwd() / "telemetry.csv"))
        self.log_browse_button = QPushButton("Browse")
        self.start_log_button = QPushButton("Start Log")
        self.stop_log_button = QPushButton("Stop Log")
        self.dump_duration_spin = QDoubleSpinBox()
        self.dump_duration_spin.setRange(0.5, 300.0)
        self.dump_duration_spin.setDecimals(1)
        self.dump_duration_spin.setValue(5.0)
        self.dump_csv_button = QPushButton("Dump CSV")
        log_layout.addWidget(QLabel("Output"), 0, 0)
        log_layout.addWidget(self.log_path_edit, 0, 1, 1, 4)
        log_layout.addWidget(self.log_browse_button, 0, 5)
        log_layout.addWidget(self.start_log_button, 1, 0)
        log_layout.addWidget(self.stop_log_button, 1, 1)
        log_layout.addWidget(QLabel("Dump s"), 1, 2)
        log_layout.addWidget(self.dump_duration_spin, 1, 3)
        log_layout.addWidget(self.dump_csv_button, 1, 4)
        layout.addWidget(log_box)

        return group

    def _wire_signals(self) -> None:
        self.link_type_combo.currentTextChanged.connect(self._update_link_inputs)
        self.refresh_ports_button.clicked.connect(self._refresh_serial_ports)
        self.connect_button.clicked.connect(self._connect_requested)
        self.disconnect_button.clicked.connect(self._disconnect_requested)

        self.arm_button.clicked.connect(lambda: self._run_session_action("arm", self._session.arm))
        self.disarm_button.clicked.connect(lambda: self._run_session_action("disarm", self._session.disarm))
        self.kill_button.clicked.connect(lambda: self._run_session_action("kill", self._session.kill))
        self.reboot_button.clicked.connect(lambda: self._run_session_action("reboot", self._session.reboot))

        self.stream_on_button.clicked.connect(lambda: self._run_session_action("stream_on", self._session.start_stream))
        self.stream_off_button.clicked.connect(lambda: self._run_session_action("stream_off", self._session.stop_stream))
        self.apply_stream_rate_button.clicked.connect(self._apply_stream_rate)
        self.copy_telemetry_button.clicked.connect(self.telemetry_table.copy_selection)

        self.chart_toggle_button.clicked.connect(self._toggle_charts)
        self.clear_charts_button.clicked.connect(self._clear_charts)
        self.chart_window_combo.currentIndexChanged.connect(self._save_settings)

        self.refresh_params_button.clicked.connect(self._refresh_params)
        self.param_search_edit.textChanged.connect(self._filter_params_table)
        self.params_table.itemSelectionChanged.connect(self._handle_param_selection)
        self.param_new_value_edit.textChanged.connect(self._update_param_hint)
        self.set_param_button.clicked.connect(self._set_selected_param)
        self.save_params_button.clicked.connect(lambda: self._run_session_action("save_params", self._session.save_params))
        self.reset_params_button.clicked.connect(lambda: self._run_session_action("reset_params", self._session.reset_params))
        self.export_params_button.clicked.connect(self._export_params)
        self.import_params_button.clicked.connect(self._import_params)

        self.motor_start_button.clicked.connect(self._start_motor_test)
        self.motor_stop_button.clicked.connect(self._stop_motor_test)
        self.calib_gyro_button.clicked.connect(lambda: self._run_session_action("calib_gyro", self._session.calib_gyro))
        self.calib_level_button.clicked.connect(lambda: self._run_session_action("calib_level", self._session.calib_level))
        self.rate_start_button.clicked.connect(self._start_rate_test)
        self.rate_stop_button.clicked.connect(self._stop_rate_test)
        self.log_browse_button.clicked.connect(self._browse_log_path)
        self.start_log_button.clicked.connect(self._start_log)
        self.stop_log_button.clicked.connect(self._stop_log)
        self.dump_csv_button.clicked.connect(self._dump_csv)

        self.clear_log_button.clicked.connect(self.event_log_edit.clear)
        self.copy_log_button.clicked.connect(lambda: QApplication.clipboard().setText(self.event_log_edit.toPlainText()))
        self.save_log_button.clicked.connect(self._save_event_log)

        self._bridge.connection_changed.connect(self._on_connection_changed)
        self._bridge.telemetry_received.connect(self._on_telemetry_received)
        self._bridge.event_received.connect(self._on_event_received)
        self._bridge.command_finished.connect(self._on_command_finished)
        self._bridge.error_raised.connect(self._on_error)

    def _run_session_action(self, label: str, callback) -> None:
        self._set_last_result(f"{label} ...")
        self._bridge.run_async(label, callback)

    def _set_last_result(self, text: str) -> None:
        self._last_result = text
        self.last_result_label.setText(text)

    def _append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.event_log_edit.appendPlainText(f"[{stamp}] {text}")

    def _save_event_log(self) -> None:
        output, _ = QFileDialog.getSaveFileName(self, "Save Event Log", "esp-drone-gui.log", "Log Files (*.log *.txt)")
        if not output:
            return
        Path(output).write_text(self.event_log_edit.toPlainText(), encoding="utf-8")
        self._append_log(f"saved event log to {output}")

    def _update_link_inputs(self) -> None:
        is_serial = self.link_type_combo.currentText() == "serial"
        self.serial_port_combo.setEnabled(is_serial)
        self.refresh_ports_button.setEnabled(is_serial)
        self.baudrate_spin.setEnabled(is_serial)
        self.udp_host_edit.setEnabled(not is_serial)
        self.udp_port_spin.setEnabled(not is_serial)
        self._save_settings()

    def _refresh_serial_ports(self) -> None:
        current = self.serial_port_combo.currentText().strip()
        ports: list[str] = []
        for entry in self._serial_port_provider():
            device = getattr(entry, "device", None)
            description = getattr(entry, "description", None)
            if device is not None:
                ports.append(device)
            elif isinstance(entry, str):
                ports.append(entry)
            elif description is not None:
                ports.append(str(description))
        self.serial_port_combo.blockSignals(True)
        self.serial_port_combo.clear()
        self.serial_port_combo.addItems(ports)
        if current:
            if current not in ports:
                self.serial_port_combo.addItem(current)
            self.serial_port_combo.setCurrentText(current)
        self.serial_port_combo.blockSignals(False)

    def _connect_requested(self) -> None:
        if self.link_type_combo.currentText() == "serial":
            port = self.serial_port_combo.currentText().strip()
            if not port:
                self._on_error("connect_serial: no serial port selected")
                return
            baudrate = int(self.baudrate_spin.value())
            self._run_session_action(
                "connect_serial",
                lambda: self._session.connect_serial(port, baudrate=baudrate, timeout=0.2),
            )
            return

        host = self.udp_host_edit.text().strip()
        if not host:
            self._on_error("connect_udp: no UDP host provided")
            return
        port = int(self.udp_port_spin.value())
        self._run_session_action(
            "connect_udp",
            lambda: self._session.connect_udp(host, port=port, timeout=1.0),
        )

    def _disconnect_requested(self) -> None:
        self._run_session_action("disconnect", lambda: self._session.disconnect())

    def _apply_stream_rate(self) -> None:
        name = "telemetry_usb_hz" if self.link_type_combo.currentText() == "serial" else "telemetry_udp_hz"
        value = str(int(self.stream_rate_spin.value()))
        self._run_session_action(
            f"set_param:{name}",
            lambda: self._session.set_param(name, 2, value),
        )

    def _refresh_params(self) -> None:
        self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))

    def _populate_params_table(self, params: list[ParamValue]) -> None:
        self._params = sorted(params, key=lambda item: item.name)
        self.params_table.setRowCount(len(self._params))
        for row, param in enumerate(self._params):
            self.params_table.setItem(row, 0, QTableWidgetItem(param.name))
            self.params_table.setItem(row, 1, QTableWidgetItem(TYPE_NAMES.get(param.type_id, str(param.type_id))))
            self.params_table.setItem(row, 2, QTableWidgetItem(_format_value(param.name, param.value)))
        self._filter_params_table()

    def _filter_params_table(self) -> None:
        text = self.param_search_edit.text().strip().lower()
        for row, param in enumerate(self._params):
            hidden = bool(text) and text not in param.name.lower()
            self.params_table.setRowHidden(row, hidden)

    def _handle_param_selection(self) -> None:
        row = self.params_table.currentRow()
        if row < 0 or row >= len(self._params):
            self._selected_param = None
            self.param_name_label.setText("-")
            self.param_type_label.setText("-")
            self.param_current_value_label.setText("-")
            self.param_new_value_edit.clear()
            self.param_help_text.setPlainText("")
            self.param_hint_label.setText("Select a parameter to see a local hint. Device-side validation remains authoritative.")
            return
        self._selected_param = self._params[row]
        self.param_name_label.setText(self._selected_param.name)
        self.param_type_label.setText(TYPE_NAMES.get(self._selected_param.type_id, str(self._selected_param.type_id)))
        self.param_current_value_label.setText(_format_value(self._selected_param.name, self._selected_param.value))
        self.param_new_value_edit.setText(str(self._selected_param.value))
        self.param_help_text.setPlainText(self.PARAM_HELP.get(self._selected_param.name, "No local note yet. Device-side validation remains authoritative."))
        self._update_param_hint()

    def _local_param_hint(self, param: ParamValue | None, value_text: str) -> str:
        if param is None:
            return "Select a parameter to see a local hint. Device-side validation remains authoritative."
        if not value_text:
            return "Enter a new value. Device-side validation remains authoritative."
        name = param.name
        try:
            if name.startswith("motor_") or name == "bringup_test_base_duty":
                value = float(value_text)
                if not 0.0 <= value <= 1.0:
                    return "Expected normalized duty in the range 0..1."
                return "Normalized duty looks locally valid. Device-side validation still applies."
            if name == "imu_return_rate_code":
                value = int(value_text, 0)
                if not 0x00 <= value <= 0x09:
                    return "Expected IMU return-rate code in the range 0x00..0x09."
                return "IMU return-rate code looks locally valid."
            if name in {"telemetry_usb_hz", "telemetry_udp_hz"}:
                value = int(value_text, 0)
                if name == "telemetry_usb_hz" and not 1 <= value <= 200:
                    return "telemetry_usb_hz should stay within 1..200."
                if name == "telemetry_udp_hz" and not 1 <= value <= 100:
                    return "telemetry_udp_hz should stay within 1..100."
                return "Telemetry rate looks locally valid."
        except ValueError:
            return "Could not parse the value locally. Device-side validation will reject malformed data."
        return self.PARAM_HELP.get(name, "No local note yet. Device-side validation remains authoritative.")

    def _update_param_hint(self) -> None:
        self.param_hint_label.setText(self._local_param_hint(self._selected_param, self.param_new_value_edit.text().strip()))

    def _set_selected_param(self) -> None:
        if self._selected_param is None:
            self._on_error("set_param: no parameter selected")
            return
        value_text = self.param_new_value_edit.text().strip()
        if not value_text:
            self._on_error("set_param: no new value provided")
            return
        name = self._selected_param.name
        type_id = self._selected_param.type_id
        self._run_session_action(
            f"set_param:{name}",
            lambda: self._session.set_param(name, type_id, value_text),
        )

    def _export_params(self) -> None:
        output, _ = QFileDialog.getSaveFileName(self, "Export Parameters", "params.json", "JSON Files (*.json)")
        if not output:
            return
        output_path = Path(output)
        self._run_session_action("export_params", lambda: self._session.export_params(output_path))

    def _import_params(self) -> None:
        input_path, _ = QFileDialog.getOpenFileName(self, "Import Parameters", "", "JSON Files (*.json)")
        if not input_path:
            return
        path_obj = Path(input_path)
        self._run_session_action("import_params", lambda: self._session.import_params(path_obj, save_after=False))

    def _start_motor_test(self) -> None:
        index = self.motor_combo.currentIndex()
        duty = float(self.motor_duty_spin.value())
        self._run_session_action("motor_test_start", lambda: self._session.motor_test(index, duty))

    def _stop_motor_test(self) -> None:
        index = self.motor_combo.currentIndex()
        self._run_session_action("motor_test_stop", lambda: self._session.motor_test(index, 0.0))

    def _start_rate_test(self) -> None:
        index = self.rate_axis_combo.currentIndex()
        rate = float(self.rate_value_spin.value())
        self._run_session_action("rate_test_start", lambda: self._session.rate_test(index, rate))

    def _stop_rate_test(self) -> None:
        index = self.rate_axis_combo.currentIndex()
        self._run_session_action("rate_test_stop", lambda: self._session.rate_test(index, 0.0))

    def _browse_log_path(self) -> None:
        output, _ = QFileDialog.getSaveFileName(self, "Select CSV Output", self.log_path_edit.text(), "CSV Files (*.csv)")
        if output:
            self.log_path_edit.setText(output)

    def _start_log(self) -> None:
        path = Path(self.log_path_edit.text().strip())
        self._run_session_action("start_csv_log", lambda: (self._session.start_csv_log(path), path)[1])

    def _stop_log(self) -> None:
        self._run_session_action("stop_csv_log", self._session.stop_csv_log)

    def _dump_csv(self) -> None:
        path = Path(self.log_path_edit.text().strip())
        duration = float(self.dump_duration_spin.value())
        self._run_session_action(
            "dump_csv",
            lambda: {"rows": self._session.dump_csv(path, duration_s=duration), "path": path},
        )

    def _toggle_charts(self) -> None:
        self._charts_running = not self._charts_running
        self.chart_toggle_button.setText("Pause Charts" if self._charts_running else "Resume Charts")
        self._append_log("chart updates resumed" if self._charts_running else "chart updates paused")

    def _clear_charts(self) -> None:
        self._history.clear()
        for bundle in self._chart_bundles.values():
            curves = bundle["curves"]
            for curve in curves.values():
                curve.setData([], [])
        self._append_log("chart history cleared")

    def _current_chart_window(self) -> float:
        return float(self.chart_window_combo.currentData() or 10.0)

    def _refresh_plots(self) -> None:
        if not self._charts_running:
            return
        window = self._current_chart_window()
        for bundle in self._chart_bundles.values():
            plot = bundle["plot"]
            checks = bundle["checks"]
            curves = bundle["curves"]
            spec = bundle["spec"]
            for field_name, _field_label in spec.fields:
                xs, ys = self._history.slice(field_name, window)
                curve = curves[field_name]
                if checks[field_name].isChecked():
                    curve.setData(xs, ys)
                else:
                    curve.setData([], [])
            plot.setXRange(-window, 0.0, padding=0.01)

    def _refresh_enabled_state(self) -> None:
        connected = bool(getattr(self._session, "is_connected", False))
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)

        for button in (
            self.arm_button,
            self.disarm_button,
            self.kill_button,
            self.reboot_button,
            self.stream_on_button,
            self.stream_off_button,
            self.apply_stream_rate_button,
            self.refresh_params_button,
            self.save_params_button,
            self.reset_params_button,
            self.export_params_button,
            self.import_params_button,
            self.set_param_button,
            self.motor_start_button,
            self.motor_stop_button,
            self.calib_gyro_button,
            self.calib_level_button,
            self.rate_start_button,
            self.rate_stop_button,
            self.start_log_button,
            self.stop_log_button,
            self.dump_csv_button,
        ):
            button.setEnabled(connected)

        is_serial = self.link_type_combo.currentText() == "serial"
        self.link_type_combo.setEnabled(not connected)
        self.serial_port_combo.setEnabled(not connected and is_serial)
        self.refresh_ports_button.setEnabled(not connected and is_serial)
        self.baudrate_spin.setEnabled(not connected and is_serial)
        self.udp_host_edit.setEnabled(not connected and not is_serial)
        self.udp_port_spin.setEnabled(not connected and not is_serial)

    def _apply_status_to_row(self, row: int, role: str) -> None:
        foreground, background = ROW_STYLE.get(role, ROW_STYLE["neutral"])
        for col in range(2):
            item = self.telemetry_table.item(row, col)
            if item is not None:
                item.setForeground(foreground)
                item.setBackground(background)

    def _on_connection_changed(self, payload: dict[str, object]) -> None:
        connected = bool(payload.get("connected"))
        error = payload.get("error")
        info = payload.get("device_info")
        if connected:
            _set_badge(self.connection_status_chip, "Connected", "ok")
            self.connection_info_label.setText(_device_info_text(info))
            self.connection_error_detail.setText("No connection error.")
            self.last_error_label.setText("-")
            if hasattr(info, "stream_enabled"):
                self._stream_enabled = bool(getattr(info, "stream_enabled"))
            self._append_log(f"connected: {_device_info_text(info)}")
            self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))
        else:
            _set_badge(self.connection_status_chip, "Disconnected", "neutral" if not error else "warn")
            self.connection_info_label.setText("No active device session.")
            self._stream_enabled = False
            if error:
                message = str(error)
                self.connection_error_detail.setText(message)
                self.last_error_label.setText(message)
                self._append_log(f"connection error: {message}")
            elif not self._closing:
                self._append_log("disconnected")
        self._update_stream_chip()
        self._refresh_enabled_state()

    def _update_stream_chip(self) -> None:
        if self._stream_enabled:
            _set_badge(self.stream_chip, "STREAM ON", "active")
        else:
            _set_badge(self.stream_chip, "STREAM OFF", "neutral")

    def _on_telemetry_received(self, sample: TelemetrySample) -> None:
        self._last_telemetry = sample
        self._history.append(sample)
        values = sample.to_display_map()
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            value = values.get(name)
            text = _format_value(name, value)
            value_item = self.telemetry_table.item(row, 1)
            if value_item is not None:
                value_item.setText(text)
            role_info = _row_role_for_field(name, value)
            if role_info is None:
                self._apply_status_to_row(row, "neutral")
            else:
                label, role = role_info
                if value_item is not None:
                    value_item.setText(f"{label} [{value}]")
                self._apply_status_to_row(row, role)

        arm_text, arm_role = _status_from_map(ARM_STATE_TEXT, sample.arm_state)
        failsafe_text, failsafe_role = _status_from_map(FAILSAFE_TEXT, sample.failsafe_reason)
        control_text, control_role = _status_from_map(CONTROL_MODE_TEXT, sample.control_mode)
        imu_text, imu_role = _status_from_map(IMU_MODE_TEXT, sample.imu_mode)
        _set_badge(self.arm_state_chip, arm_text, arm_role)
        _set_badge(self.failsafe_chip, failsafe_text, failsafe_role)
        _set_badge(self.control_mode_chip, control_text, control_role)
        _set_badge(self.imu_mode_chip, imu_text, imu_role)

    def _on_event_received(self, message: str) -> None:
        self._append_log(message)

    def _on_error(self, message: str) -> None:
        self.last_error_label.setText(message)
        self._set_last_result(message)
        self._append_log(message)

    def _on_command_finished(self, label: str, result: object) -> None:
        if label == "stream_on":
            self._stream_enabled = True
            self._update_stream_chip()
            self._set_last_result("stream enabled")
            self._append_log("stream enabled")
            return
        if label == "stream_off":
            self._stream_enabled = False
            self._update_stream_chip()
            self._set_last_result("stream disabled")
            self._append_log("stream disabled")
            return
        if label == "list_params" and isinstance(result, list):
            self._populate_params_table(result)
            self._set_last_result(f"loaded {len(result)} params")
            self._append_log(f"loaded {len(result)} params")
            return
        if label.startswith("set_param:") and isinstance(result, ParamValue):
            updated = False
            for index, param in enumerate(self._params):
                if param.name == result.name:
                    self._params[index] = result
                    updated = True
                    break
            if not updated:
                self._params.append(result)
            self._populate_params_table(self._params)
            for row, param in enumerate(self._params):
                if param.name == result.name:
                    self.params_table.selectRow(row)
                    break
            self._set_last_result(f"{result.name} updated")
            self._append_log(f"set {result.name} -> {result.value}")
            return
        if label == "export_params":
            self._set_last_result("parameter snapshot exported")
            self._append_log("parameter snapshot exported")
            return
        if label == "import_params":
            applied = len(result or [])
            self._set_last_result(f"imported {applied} params")
            self._append_log(f"imported {applied} params from JSON snapshot")
            self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))
            return
        if label == "start_csv_log":
            path = Path(result) if result else Path(self.log_path_edit.text().strip())
            self.last_log_path_label.setText(str(path))
            self._set_last_result(f"logging to {path}")
            self._append_log(f"started CSV log at {path}")
            return
        if label == "stop_csv_log":
            path_text = "-" if result is None else str(result)
            self.last_log_path_label.setText(path_text)
            self._set_last_result(f"stopped CSV log ({path_text})")
            self._append_log(f"stopped CSV log ({path_text})")
            return
        if label == "dump_csv" and isinstance(result, dict):
            path = result.get("path")
            rows = result.get("rows")
            self.last_log_path_label.setText(str(path))
            self._set_last_result(f"dumped {rows} telemetry rows to {path}")
            self._append_log(f"dumped {rows} telemetry rows to {path}")
            return
        if label in {"connect_serial", "connect_udp", "disconnect", "arm", "disarm", "kill", "reboot", "save_params", "reset_params", "motor_test_start", "motor_test_stop", "calib_gyro", "calib_level", "rate_test_start", "rate_test_stop"}:
            summary = label.replace("_", " ")
            if label == "connect_serial":
                summary = f"connected via serial: {_device_info_text(result)}"
            elif label == "connect_udp":
                summary = f"connected via udp: {_device_info_text(result)}"
            elif label == "disconnect":
                summary = "disconnect complete"
            elif label == "arm":
                summary = f"arm -> status {result}"
            elif label == "disarm":
                summary = f"disarm -> status {result}"
            elif label == "kill":
                summary = f"kill -> status {result}"
            elif label == "reboot":
                summary = f"reboot -> status {result}"
            elif label == "save_params":
                summary = "params saved to device"
            elif label == "reset_params":
                summary = "params reset on device"
            elif label == "motor_test_start":
                summary = f"motor_test start -> status {result}"
            elif label == "motor_test_stop":
                summary = f"motor_test stop -> status {result}"
            elif label == "calib_gyro":
                summary = f"calib gyro -> status {result}"
            elif label == "calib_level":
                summary = f"calib level -> status {result}"
            elif label == "rate_test_start":
                summary = f"rate_test start -> status {result}"
            elif label == "rate_test_stop":
                summary = f"rate_test stop -> status {result}"
            self._set_last_result(summary)
            self._append_log(summary)
            return

        self._set_last_result(f"{label} complete")
        self._append_log(f"{label} complete")

    def _save_settings(self) -> None:
        self._settings.setValue("window/geometry", self.saveGeometry())
        self._settings.setValue("window/state", self.saveState())
        self._settings.setValue("splitter/main", self.main_splitter.saveState())
        self._settings.setValue("splitter/vertical", self.vertical_splitter.saveState())
        self._settings.setValue("splitter/params", self.params_splitter.saveState())
        self._settings.setValue("link/type", self.link_type_combo.currentText())
        self._settings.setValue("serial/port", self.serial_port_combo.currentText())
        self._settings.setValue("serial/baudrate", self.baudrate_spin.value())
        self._settings.setValue("udp/host", self.udp_host_edit.text())
        self._settings.setValue("udp/port", self.udp_port_spin.value())
        self._settings.setValue("chart/window_index", self.chart_window_combo.currentIndex())
        self._settings.setValue("params/search", self.param_search_edit.text())
        self._settings.setValue("log/path", self.log_path_edit.text())

    def _load_settings(self) -> None:
        geometry = self._settings.value("window/geometry", type=QByteArray)
        state = self._settings.value("window/state", type=QByteArray)
        main_splitter_state = self._settings.value("splitter/main", type=QByteArray)
        vertical_splitter_state = self._settings.value("splitter/vertical", type=QByteArray)
        params_splitter_state = self._settings.value("splitter/params", type=QByteArray)
        if geometry:
            self.restoreGeometry(geometry)
        if state:
            self.restoreState(state)
        if main_splitter_state:
            self.main_splitter.restoreState(main_splitter_state)
        if vertical_splitter_state:
            self.vertical_splitter.restoreState(vertical_splitter_state)
        if params_splitter_state:
            self.params_splitter.restoreState(params_splitter_state)

        link_type = self._settings.value("link/type", "serial")
        serial_port = self._settings.value("serial/port", "")
        baudrate = int(self._settings.value("serial/baudrate", 115200))
        udp_host = self._settings.value("udp/host", "192.168.4.1")
        udp_port = int(self._settings.value("udp/port", 2391))
        chart_index = int(self._settings.value("chart/window_index", 1))
        param_search = self._settings.value("params/search", "")
        log_path = self._settings.value("log/path", self.log_path_edit.text())

        self.link_type_combo.setCurrentText(str(link_type))
        self.serial_port_combo.setCurrentText(str(serial_port))
        self.baudrate_spin.setValue(baudrate)
        self.udp_host_edit.setText(str(udp_host))
        self.udp_port_spin.setValue(udp_port)
        self.chart_window_combo.setCurrentIndex(max(0, min(chart_index, self.chart_window_combo.count() - 1)))
        self.param_search_edit.setText(str(param_search))
        self.log_path_edit.setText(str(log_path))
        self._update_link_inputs()

    def closeEvent(self, event) -> None:  # pragma: no cover - exercised by smoke tests
        self._closing = True
        self._plot_timer.stop()
        self._save_settings()
        try:
            self._session.disconnect()
        finally:
            self._bridge.dispose()
        super().closeEvent(event)


def run_gui(_argv: list[str] | None = None) -> int:
    app = QApplication.instance() or QApplication([])
    pg.setConfigOptions(antialias=True, foreground="#263238")
    window = MainWindow()
    window.show()
    return int(app.exec())
