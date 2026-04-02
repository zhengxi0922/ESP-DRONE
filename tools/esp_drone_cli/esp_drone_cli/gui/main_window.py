from __future__ import annotations

import threading
from pathlib import Path
from typing import Callable, Iterable

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QDoubleSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from esp_drone_cli.core import DeviceSession, ParamValue, TelemetrySample


class QtSessionBridge(QObject):
    connection_changed = Signal(object)
    telemetry_received = Signal(object)
    event_received = Signal(str)
    params_loaded = Signal(object)
    command_finished = Signal(str)
    error_raised = Signal(str)

    def __init__(self, session: DeviceSession) -> None:
        super().__init__()
        self._session = session
        self._session.subscribe_connection_state(lambda payload: self.connection_changed.emit(payload))
        self._session.subscribe_telemetry(lambda sample: self.telemetry_received.emit(sample))
        self._session.subscribe_event_log(lambda message: self.event_received.emit(message))

    def run_async(self, label: str, callback) -> None:
        def worker() -> None:
            try:
                callback()
                self.command_finished.emit(label)
            except Exception as exc:  # pragma: no cover - UI path
                self.error_raised.emit(f"{label}: {exc}")

        threading.Thread(target=worker, daemon=True).start()


class MainWindow(QMainWindow):
    TELEMETRY_FIELDS = [
        "gyro_x", "gyro_y", "gyro_z",
        "roll_deg", "pitch_deg", "yaw_deg",
        "rate_setpoint_roll", "rate_setpoint_pitch", "rate_setpoint_yaw",
        "motor1", "motor2", "motor3", "motor4",
        "battery_voltage", "battery_adc_raw", "imu_age_us", "loop_dt_us",
        "imu_mode", "arm_state", "failsafe_reason", "control_mode",
    ]

    def __init__(
        self,
        session: DeviceSession | None = None,
        bridge_cls: type[QtSessionBridge] = QtSessionBridge,
        serial_port_provider: Callable[[], Iterable] | None = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("ESP-DRONE Debug GUI")
        self.resize(1300, 900)

        self._session = session or DeviceSession()
        self._bridge = bridge_cls(self._session)
        self._serial_port_provider = serial_port_provider or list_ports.comports
        self._params: list[ParamValue] = []
        self._selected_param: ParamValue | None = None
        self._last_telemetry: TelemetrySample | None = None

        self._build_ui()
        self._wire_signals()
        self._refresh_serial_ports()

    def _build_ui(self) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_status_group())
        layout.addWidget(self._build_telemetry_group())
        layout.addWidget(self._build_params_group())
        layout.addWidget(self._build_debug_group())
        layout.addWidget(self._build_log_group())
        self.setCentralWidget(root)

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        self.link_type_combo = QComboBox()
        self.link_type_combo.addItems(["serial", "udp"])
        self.serial_port_combo = QComboBox()
        self.serial_port_combo.setEditable(True)
        self.refresh_ports_button = QPushButton("Refresh Ports")
        self.baudrate_spin = QSpinBox()
        self.baudrate_spin.setRange(9600, 2000000)
        self.baudrate_spin.setValue(115200)
        self.udp_host_edit = QLineEdit("192.168.4.1")
        self.udp_port_spin = QSpinBox()
        self.udp_port_spin.setRange(1, 65535)
        self.udp_port_spin.setValue(2391)
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.connection_status_label = QLabel("Disconnected")

        layout.addWidget(QLabel("Link"), 0, 0)
        layout.addWidget(self.link_type_combo, 0, 1)
        layout.addWidget(QLabel("Serial Port"), 0, 2)
        layout.addWidget(self.serial_port_combo, 0, 3)
        layout.addWidget(self.refresh_ports_button, 0, 4)
        layout.addWidget(QLabel("Baud"), 0, 5)
        layout.addWidget(self.baudrate_spin, 0, 6)
        layout.addWidget(QLabel("UDP Host"), 1, 0)
        layout.addWidget(self.udp_host_edit, 1, 1, 1, 2)
        layout.addWidget(QLabel("UDP Port"), 1, 3)
        layout.addWidget(self.udp_port_spin, 1, 4)
        layout.addWidget(self.connect_button, 1, 5)
        layout.addWidget(self.disconnect_button, 1, 6)
        layout.addWidget(QLabel("Status"), 2, 0)
        layout.addWidget(self.connection_status_label, 2, 1, 1, 6)
        return group

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("Safety Control")
        layout = QGridLayout(group)
        self.arm_button = QPushButton("Arm")
        self.disarm_button = QPushButton("Disarm")
        self.kill_button = QPushButton("Kill")
        self.kill_button.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")
        self.reboot_button = QPushButton("Reboot")
        self.arm_state_label = QLabel("-")
        self.failsafe_label = QLabel("-")
        self.control_mode_label = QLabel("-")

        layout.addWidget(self.arm_button, 0, 0)
        layout.addWidget(self.disarm_button, 0, 1)
        layout.addWidget(self.kill_button, 0, 2)
        layout.addWidget(self.reboot_button, 0, 3)
        layout.addWidget(QLabel("arm_state"), 1, 0)
        layout.addWidget(self.arm_state_label, 1, 1)
        layout.addWidget(QLabel("failsafe_reason"), 1, 2)
        layout.addWidget(self.failsafe_label, 1, 3)
        layout.addWidget(QLabel("control_mode"), 2, 0)
        layout.addWidget(self.control_mode_label, 2, 1)
        return group

    def _build_telemetry_group(self) -> QGroupBox:
        group = QGroupBox("Realtime Telemetry")
        layout = QVBoxLayout(group)
        top_bar = QHBoxLayout()
        self.stream_on_button = QPushButton("Stream On")
        self.stream_off_button = QPushButton("Stream Off")
        self.stream_rate_spin = QSpinBox()
        self.stream_rate_spin.setRange(1, 200)
        self.stream_rate_spin.setValue(200)
        self.apply_stream_rate_button = QPushButton("Apply Stream Hz")
        top_bar.addWidget(self.stream_on_button)
        top_bar.addWidget(self.stream_off_button)
        top_bar.addWidget(QLabel("Target Hz"))
        top_bar.addWidget(self.stream_rate_spin)
        top_bar.addWidget(self.apply_stream_rate_button)
        top_bar.addStretch(1)
        layout.addLayout(top_bar)

        self.telemetry_table = QTableWidget(len(self.TELEMETRY_FIELDS), 2)
        self.telemetry_table.setHorizontalHeaderLabels(["Field", "Value"])
        self.telemetry_table.verticalHeader().setVisible(False)
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            self.telemetry_table.setItem(row, 0, QTableWidgetItem(name))
            self.telemetry_table.setItem(row, 1, QTableWidgetItem("-"))
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.telemetry_table)
        return group

    def _build_params_group(self) -> QGroupBox:
        group = QGroupBox("Parameter Debug")
        layout = QVBoxLayout(group)
        search_bar = QHBoxLayout()
        self.param_search_edit = QLineEdit()
        self.param_search_edit.setPlaceholderText("Search parameter...")
        self.refresh_params_button = QPushButton("Refresh Params")
        self.save_params_button = QPushButton("Save")
        self.reset_params_button = QPushButton("Reset")
        self.export_params_button = QPushButton("Export JSON")
        self.import_params_button = QPushButton("Import JSON")
        search_bar.addWidget(self.param_search_edit)
        search_bar.addWidget(self.refresh_params_button)
        search_bar.addWidget(self.save_params_button)
        search_bar.addWidget(self.reset_params_button)
        search_bar.addWidget(self.export_params_button)
        search_bar.addWidget(self.import_params_button)
        layout.addLayout(search_bar)

        self.params_table = QTableWidget(0, 3)
        self.params_table.setHorizontalHeaderLabels(["Name", "Type", "Value"])
        self.params_table.verticalHeader().setVisible(False)
        self.params_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.params_table.setSelectionMode(QTableWidget.SingleSelection)
        layout.addWidget(self.params_table)

        edit_bar = QHBoxLayout()
        self.param_value_edit = QLineEdit()
        self.param_value_edit.setPlaceholderText("Selected param new value")
        self.set_param_button = QPushButton("Set Selected")
        edit_bar.addWidget(self.param_value_edit)
        edit_bar.addWidget(self.set_param_button)
        layout.addLayout(edit_bar)
        return group

    def _build_debug_group(self) -> QGroupBox:
        group = QGroupBox("Debug Actions")
        layout = QGridLayout(group)

        self.motor_combo = QComboBox()
        self.motor_combo.addItems(["m1", "m2", "m3", "m4"])
        self.motor_duty_spin = QDoubleSpinBox()
        self.motor_duty_spin.setRange(0.0, 1.0)
        self.motor_duty_spin.setSingleStep(0.01)
        self.motor_duty_spin.setValue(0.12)
        self.motor_start_button = QPushButton("Start Motor Test")
        self.motor_stop_button = QPushButton("Stop Motor Test")

        self.calib_gyro_button = QPushButton("Calib Gyro")
        self.calib_level_button = QPushButton("Calib Level")

        self.rate_axis_combo = QComboBox()
        self.rate_axis_combo.addItems(["roll", "pitch", "yaw"])
        self.rate_value_spin = QDoubleSpinBox()
        self.rate_value_spin.setRange(-720.0, 720.0)
        self.rate_value_spin.setSingleStep(5.0)
        self.rate_value_spin.setValue(30.0)
        self.rate_start_button = QPushButton("Start Rate Test")
        self.rate_stop_button = QPushButton("Stop Rate Test")

        self.rc_status_label = QLabel("RC send reserved in core; firmware protocol not implemented yet")

        layout.addWidget(QLabel("Motor"), 0, 0)
        layout.addWidget(self.motor_combo, 0, 1)
        layout.addWidget(QLabel("Duty"), 0, 2)
        layout.addWidget(self.motor_duty_spin, 0, 3)
        layout.addWidget(self.motor_start_button, 0, 4)
        layout.addWidget(self.motor_stop_button, 0, 5)
        layout.addWidget(self.calib_gyro_button, 1, 0)
        layout.addWidget(self.calib_level_button, 1, 1)
        layout.addWidget(QLabel("Rate Axis"), 2, 0)
        layout.addWidget(self.rate_axis_combo, 2, 1)
        layout.addWidget(QLabel("Value dps"), 2, 2)
        layout.addWidget(self.rate_value_spin, 2, 3)
        layout.addWidget(self.rate_start_button, 2, 4)
        layout.addWidget(self.rate_stop_button, 2, 5)
        layout.addWidget(self.rc_status_label, 3, 0, 1, 6)
        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("Log Export")
        layout = QGridLayout(group)
        self.log_path_edit = QLineEdit(str(Path.cwd() / "telemetry.csv"))
        self.log_browse_button = QPushButton("Browse")
        self.start_log_button = QPushButton("Start Log")
        self.stop_log_button = QPushButton("Stop Log")
        self.dump_duration_spin = QDoubleSpinBox()
        self.dump_duration_spin.setRange(0.5, 300.0)
        self.dump_duration_spin.setValue(5.0)
        self.dump_csv_button = QPushButton("Dump CSV")
        self.last_log_path_label = QLabel("-")
        self.last_error_label = QLabel("-")
        self.event_log_label = QLabel("-")
        self.event_log_label.setWordWrap(True)

        layout.addWidget(QLabel("Output"), 0, 0)
        layout.addWidget(self.log_path_edit, 0, 1, 1, 4)
        layout.addWidget(self.log_browse_button, 0, 5)
        layout.addWidget(self.start_log_button, 1, 0)
        layout.addWidget(self.stop_log_button, 1, 1)
        layout.addWidget(QLabel("Dump duration s"), 1, 2)
        layout.addWidget(self.dump_duration_spin, 1, 3)
        layout.addWidget(self.dump_csv_button, 1, 4)
        layout.addWidget(QLabel("Last log"), 2, 0)
        layout.addWidget(self.last_log_path_label, 2, 1, 1, 5)
        layout.addWidget(QLabel("Last error"), 3, 0)
        layout.addWidget(self.last_error_label, 3, 1, 1, 5)
        layout.addWidget(QLabel("Latest event"), 4, 0)
        layout.addWidget(self.event_log_label, 4, 1, 1, 5)
        return group

    def _wire_signals(self) -> None:
        self.refresh_ports_button.clicked.connect(self._refresh_serial_ports)
        self.connect_button.clicked.connect(self._connect_requested)
        self.disconnect_button.clicked.connect(lambda: self._bridge.run_async("disconnect", self._session.disconnect))

        self.arm_button.clicked.connect(lambda: self._bridge.run_async("arm", self._session.arm))
        self.disarm_button.clicked.connect(lambda: self._bridge.run_async("disarm", self._session.disarm))
        self.kill_button.clicked.connect(lambda: self._bridge.run_async("kill", self._session.kill))
        self.reboot_button.clicked.connect(lambda: self._bridge.run_async("reboot", self._session.reboot))

        self.stream_on_button.clicked.connect(lambda: self._bridge.run_async("stream on", self._session.start_stream))
        self.stream_off_button.clicked.connect(lambda: self._bridge.run_async("stream off", self._session.stop_stream))
        self.apply_stream_rate_button.clicked.connect(self._apply_stream_rate)

        self.refresh_params_button.clicked.connect(self._refresh_params)
        self.save_params_button.clicked.connect(lambda: self._bridge.run_async("save params", self._session.save_params))
        self.reset_params_button.clicked.connect(lambda: self._bridge.run_async("reset params", self._session.reset_params))
        self.export_params_button.clicked.connect(self._export_params)
        self.import_params_button.clicked.connect(self._import_params)
        self.set_param_button.clicked.connect(self._set_selected_param)
        self.param_search_edit.textChanged.connect(self._filter_params_table)
        self.params_table.itemSelectionChanged.connect(self._handle_param_selection)

        self.motor_start_button.clicked.connect(self._start_motor_test)
        self.motor_stop_button.clicked.connect(lambda: self._bridge.run_async("motor stop", lambda: self._session.motor_test(self.motor_combo.currentIndex(), 0.0)))
        self.calib_gyro_button.clicked.connect(lambda: self._bridge.run_async("calib gyro", self._session.calib_gyro))
        self.calib_level_button.clicked.connect(lambda: self._bridge.run_async("calib level", self._session.calib_level))
        self.rate_start_button.clicked.connect(self._start_rate_test)
        self.rate_stop_button.clicked.connect(lambda: self._bridge.run_async("rate stop", lambda: self._session.rate_test(self.rate_axis_combo.currentIndex(), 0.0)))

        self.log_browse_button.clicked.connect(self._browse_log_path)
        self.start_log_button.clicked.connect(self._start_log)
        self.stop_log_button.clicked.connect(self._stop_log)
        self.dump_csv_button.clicked.connect(self._dump_csv)

        self._bridge.connection_changed.connect(self._on_connection_changed)
        self._bridge.telemetry_received.connect(self._on_telemetry_received)
        self._bridge.event_received.connect(self._on_event_received)
        self._bridge.params_loaded.connect(self._populate_params_table)
        self._bridge.error_raised.connect(self._on_error)
        self._bridge.command_finished.connect(self._on_command_finished)

    def _refresh_serial_ports(self) -> None:
        current_text = self.serial_port_combo.currentText()
        self.serial_port_combo.clear()
        for port in self._serial_port_provider():
            device = getattr(port, "device", str(port))
            self.serial_port_combo.addItem(device)
        if current_text:
            self.serial_port_combo.setCurrentText(current_text)

    def _connect_requested(self) -> None:
        if self.link_type_combo.currentText() == "serial":
            port = self.serial_port_combo.currentText().strip()
            if not port:
                self._on_error("connect: no serial port selected")
                return
            self._bridge.run_async(
                "connect serial",
                lambda: self._session.connect_serial(port, baudrate=self.baudrate_spin.value()),
            )
            return

        host = self.udp_host_edit.text().strip()
        self._bridge.run_async(
            "connect udp",
            lambda: self._session.connect_udp(host, port=self.udp_port_spin.value()),
        )

    def _apply_stream_rate(self) -> None:
        param_name = "telemetry_usb_hz" if self.link_type_combo.currentText() == "serial" else "telemetry_udp_hz"
        self._bridge.run_async(
            "set stream rate",
            lambda: self._session.set_param(param_name, 2, str(self.stream_rate_spin.value())),
        )

    def _refresh_params(self) -> None:
        def load() -> None:
            self._params = self._session.list_params(timeout=3.0)
            self._bridge.params_loaded.emit(self._params)

        self._bridge.run_async("refresh params", load)

    def _set_selected_param(self) -> None:
        if self._selected_param is None:
            self._on_error("set param: no parameter selected")
            return
        self._bridge.run_async(
            f"set {self._selected_param.name}",
            lambda: self._session.set_param(self._selected_param.name, self._selected_param.type_id, self.param_value_edit.text()),
        )

    def _populate_params_table(self, params: list[ParamValue]) -> None:
        self.params_table.setRowCount(0)
        for row, item in enumerate(params):
            self.params_table.insertRow(row)
            self.params_table.setItem(row, 0, QTableWidgetItem(item.name))
            self.params_table.setItem(row, 1, QTableWidgetItem(str(item.type_id)))
            self.params_table.setItem(row, 2, QTableWidgetItem(str(item.value)))
        self._filter_params_table(self.param_search_edit.text())

    def _filter_params_table(self, text: str) -> None:
        needle = text.strip().lower()
        for row in range(self.params_table.rowCount()):
            visible = needle in self.params_table.item(row, 0).text().lower()
            self.params_table.setRowHidden(row, not visible)

    def _handle_param_selection(self) -> None:
        row = self.params_table.currentRow()
        if row < 0 or row >= len(self._params):
            return
        name = self.params_table.item(row, 0).text()
        for item in self._params:
            if item.name == name:
                self._selected_param = item
                self.param_value_edit.setText(str(item.value))
                return

    def _export_params(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export Params", "", "JSON Files (*.json)")
        if not path:
            return
        self._bridge.run_async("export params", lambda: self._session.export_params(Path(path)))

    def _import_params(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import Params", "", "JSON Files (*.json)")
        if not path:
            return
        self._bridge.run_async("import params", lambda: self._session.import_params(Path(path), save_after=False))

    def _start_motor_test(self) -> None:
        index = self.motor_combo.currentIndex()
        duty = float(self.motor_duty_spin.value())
        self._bridge.run_async("motor test", lambda: self._session.motor_test(index, duty))

    def _start_rate_test(self) -> None:
        index = self.rate_axis_combo.currentIndex()
        value = float(self.rate_value_spin.value())
        self._bridge.run_async("rate test", lambda: self._session.rate_test(index, value))

    def _browse_log_path(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Telemetry CSV", self.log_path_edit.text(), "CSV Files (*.csv)")
        if path:
            self.log_path_edit.setText(path)

    def _dump_csv(self) -> None:
        self._bridge.run_async(
            "dump csv",
            lambda: self._session.dump_csv(Path(self.log_path_edit.text()), duration_s=float(self.dump_duration_spin.value())),
        )

    def _on_connection_changed(self, payload: dict[str, object]) -> None:
        connected = bool(payload.get("connected"))
        if connected:
            self.connection_status_label.setText(f"Connected: {payload.get('device_info')}")
        else:
            self.connection_status_label.setText("Disconnected")
        if payload.get("error"):
            self.last_error_label.setText(str(payload["error"]))

    def _on_telemetry_received(self, sample: TelemetrySample) -> None:
        self._last_telemetry = sample
        values = sample.to_display_map()
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            self.telemetry_table.item(row, 1).setText(str(values.get(name, "-")))
        self.arm_state_label.setText(str(sample.arm_state))
        self.failsafe_label.setText(str(sample.failsafe_reason))
        self.control_mode_label.setText(str(sample.control_mode))

    def _on_event_received(self, message: str) -> None:
        self.event_log_label.setText(message)

    def _on_error(self, message: str) -> None:
        self.last_error_label.setText(message)
        QMessageBox.critical(self, "ESP-DRONE GUI", message)

    def _on_command_finished(self, label: str) -> None:
        self.last_log_path_label.setText(str(self._session.last_log_path or "-"))
        if label in {"reset params", "import params", "set stream rate"} or label.startswith("set "):
            if self._session.is_connected:
                self._refresh_params()
        if label.startswith("connect ") and self._session.is_connected:
            self._refresh_params()

    def _start_log(self) -> None:
        def action() -> None:
            self._session.start_csv_log(Path(self.log_path_edit.text()))
            self._session.start_stream()

        self._bridge.run_async("start log", action)

    def _stop_log(self) -> None:
        self._bridge.run_async("stop log", self._session.stop_csv_log)

    def closeEvent(self, event) -> None:  # pragma: no cover - UI lifecycle
        try:
            self._session.disconnect()
        finally:
            super().closeEvent(event)


def run_gui(argv: list[str] | None = None) -> int:
    app = QApplication.instance() or QApplication(argv or [])
    window = MainWindow()
    window.show()
    return app.exec()
