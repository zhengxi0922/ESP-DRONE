"""ESP-DRONE 图形调试工作台。

本模块提供基于 PyQt5 的桌面调试界面，负责布局、图表和交互编排。
所有设备通信与协议细节仍统一委托给 `esp_drone_cli.core.DeviceSession`。
"""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")

from PyQt5.QtCore import QByteArray, QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QKeySequence
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
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
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QDoubleSpinBox,
    QSplitter,
    QStackedWidget,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QToolButton,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg
from serial.tools import list_ports

if __package__ in (None, ""):
    package_root = Path(__file__).resolve().parents[2]
    package_root_text = str(package_root)
    if package_root_text not in sys.path:
        sys.path.insert(0, package_root_text)

from esp_drone_cli.core import DeviceSession, ParamValue, TelemetrySample
from esp_drone_cli.core.ground_bench import run_ground_bench_round
from esp_drone_cli.core.protocol.messages import CmdId, ensure_command_ok


APP_ORG = "ESP-DRONE"
APP_NAME = "ESP-DRONE GUI"
UDP_MANUAL_WATCHDOG_MS = 1000
UDP_MANUAL_SETPOINT_TIMEOUT_S = 0.2

TRANSLATIONS = {
    "zh": {
        "window.title": "ESP-DRONE 调试工作台",
        "toolbar.language": "语言",
        "lang.zh": "中文",
        "lang.en": "English",
        "group.connection": "设备连接",
        "group.safety": "安全控制",
        "group.debug": "调试动作",
        "group.chart": "主图表工作区",
        "group.realtime": "实时数值",
        "group.status": "关键状态",
        "group.params": "参数调试",
        "group.log": "事件日志",
        "group.motor": "电机测试",
        "group.calib": "校准",
        "group.rate": "速率测试",
        "group.csv": "日志导出",
        "label.link": "连接方式",
        "label.serial_port": "串口",
        "label.baud": "波特率",
        "label.udp_host": "UDP 主机",
        "label.udp_port": "UDP 端口",
        "label.session": "会话信息",
        "label.last_conn_error": "最近连接错误",
        "label.target_hz": "目标频率",
        "label.chart_group": "图表组",
        "label.window": "时间窗",
        "label.motor": "电机",
        "label.duty": "占空比",
        "label.axis": "轴",
        "label.rate_dps": "角速度(dps)",
        "label.output": "输出文件",
        "label.dump_s": "导出时长(s)",
        "label.name": "名称",
        "label.type": "类型",
        "label.current": "当前值",
        "label.new_value": "新值",
        "label.description": "说明",
        "label.last_result": "最近结果",
        "label.last_log": "最近日志",
        "label.last_error": "最近错误",
        "placeholder.search": "搜索参数...",
        "placeholder.new_value": "输入新的参数值",
        "placeholder.desc": "参数说明占位区",
        "placeholder.event_log": "连接事件、命令结果和错误会显示在这里。",
        "hint.param_default": "选择参数后可查看本地提示。最终校验仍以设备端为准。",
        "warn.bench": "台架调试专用：请拆桨或固定机体。motor_test / rate_test 默认使用保守值。",
        "button.connect": "连接",
        "button.disconnect": "断开",
        "button.arm": "解锁",
        "button.disarm": "上锁",
        "button.kill": "急停",
        "button.reboot": "重启",
        "button.stream_on": "开始流",
        "button.stream_off": "停止流",
        "button.apply_hz": "应用频率",
        "button.copy_selected": "复制选中",
        "button.pause_charts": "暂停图表",
        "button.resume_charts": "继续图表",
        "button.clear_charts": "清空图表",
        "button.auto_scale": "自动缩放",
        "button.reset_view": "重置视图",
        "button.refresh": "刷新",
        "button.save": "保存",
        "button.reset": "恢复默认",
        "button.export_json": "导出 JSON",
        "button.import_json": "导入 JSON",
        "button.set_selected": "设置当前参数",
        "button.start": "开始",
        "button.stop": "停止",
        "button.calib_gyro": "校准陀螺仪",
        "button.calib_level": "校准水平",
        "button.calib_gyro_short": "校陀螺",
        "button.calib_level_short": "校水平",
        "button.start_log": "开始记录",
        "button.stop_log": "停止记录",
        "button.dump_csv": "导出 CSV",
        "button.browse": "浏览",
        "button.clear_log": "清空日志",
        "button.copy_log": "复制日志",
        "button.save_log": "保存日志",
        "button.collapse_log": "折叠日志",
        "button.expand_log": "展开日志",
        "status.connected": "已连接",
        "status.disconnected": "未连接",
        "status.no_session": "当前无会话",
        "status.no_conn_error": "当前无连接错误",
        "status.stream_on": "流已开启",
        "status.stream_off": "流未开启",
        "status.no_error": "-",
        "status.no_result": "-",
        "status.no_log": "-",
        "status.no_value": "-",
        "arm.disarmed": "已上锁",
        "arm.armed": "已解锁",
        "arm.failsafe": "失效保护",
        "arm.fault_lock": "故障锁定",
        "failsafe.none": "无",
        "failsafe.kill": "急停",
        "failsafe.rc_timeout": "遥控超时",
        "failsafe.imu_timeout": "IMU 超时",
        "failsafe.imu_parse": "IMU 解析错误",
        "failsafe.bat_critical": "电池严重欠压",
        "failsafe.loop_overrun": "控制循环超时",
        "control.idle": "空闲",
        "control.axis_test": "轴测试",
        "control.rate_test": "速率测试",
        "imu.raw": "RAW",
        "imu.direct": "DIRECT",
        "msg.saved_log": "已保存事件日志到 {path}",
        "msg.chart_paused": "图表已暂停",
        "msg.chart_resumed": "图表已继续",
        "msg.chart_cleared": "图表历史已清空",
        "msg.connect_no_port": "未选择串口",
        "msg.connect_no_host": "未填写 UDP 主机",
        "msg.set_param_no_select": "未选择参数",
        "msg.set_param_no_value": "未输入新的参数值",
        "msg.loaded_params": "已加载 {count} 个参数",
        "msg.param_updated": "{name} 已更新为 {value}",
        "msg.export_done": "参数快照已导出",
        "msg.import_done": "已导入 {count} 个参数",
        "msg.logging_to": "正在记录到 {path}",
        "msg.log_stopped": "CSV 日志已停止 ({path})",
        "msg.dump_csv_done": "已导出 {rows} 行遥测到 {path}",
        "msg.disconnect_done": "已断开连接",
        "msg.command_ok": "{label} 完成",
        "chart.gyro": "陀螺",
        "chart.attitude": "姿态",
        "chart.motors": "电机输出",
        "chart.battery": "电池电压",
        "chart.y_deg_s": "角速度 (deg/s)",
        "chart.y_deg": "角度 (deg)",
        "chart.y_duty": "占空比",
        "chart.y_volt": "电压 (V)",
        "chart.time": "时间 (s)",
        "field.gyro_x": "陀螺 X (gyro_x)",
        "field.gyro_y": "陀螺 Y (gyro_y)",
        "field.gyro_z": "陀螺 Z (gyro_z)",
        "field.roll_deg": "横滚角 (roll_deg)",
        "field.pitch_deg": "俯仰角 (pitch_deg)",
        "field.yaw_deg": "航向角 (yaw_deg)",
        "field.rate_setpoint_roll": "横滚速率目标",
        "field.rate_setpoint_pitch": "俯仰速率目标",
        "field.rate_setpoint_yaw": "航向速率目标",
        "field.motor1": "电机1",
        "field.motor2": "电机2",
        "field.motor3": "电机3",
        "field.motor4": "电机4",
        "field.battery_voltage": "电池电压",
        "field.battery_adc_raw": "电池 ADC 原始值",
        "field.imu_age_us": "IMU 样本龄",
        "field.loop_dt_us": "控制循环周期",
        "field.imu_mode": "IMU 模式",
        "field.imu_health": "IMU 健康",
        "field.arm_state": "解锁状态",
        "field.failsafe_reason": "失效保护原因",
        "field.control_mode": "控制模式",
        "param.help.default": "暂无本地说明。最终校验仍以设备端为准。",
    },
    "en": {
        "window.title": "ESP-DRONE Debug Workbench",
        "toolbar.language": "Language",
        "lang.zh": "中文",
        "lang.en": "English",
        "group.connection": "Connection",
        "group.safety": "Safety Control",
        "group.debug": "Debug Actions",
        "group.chart": "Main Chart Workspace",
        "group.realtime": "Realtime Values",
        "group.status": "Key Status",
        "group.params": "Parameters",
        "group.log": "Event Log",
        "tab.params": "Parameters",
        "tab.events": "Events",
        "group.motor": "Motor Test",
        "group.calib": "Calibration",
        "group.rate": "Rate Test",
        "group.csv": "Log Export",
        "label.link": "Link",
        "label.serial_port": "Serial Port",
        "label.baud": "Baud",
        "label.udp_host": "UDP Host",
        "label.udp_port": "UDP Port",
        "label.session": "Session",
        "label.last_conn_error": "Last Connection Error",
        "label.target_hz": "Target Hz",
        "label.chart_group": "Chart Group",
        "label.window": "Window",
        "label.motor": "Motor",
        "label.duty": "Duty",
        "label.axis": "Axis",
        "label.rate_dps": "Rate dps",
        "label.output": "Output",
        "label.dump_s": "Dump s",
        "label.name": "Name",
        "label.type": "Type",
        "label.current": "Current",
        "label.new_value": "New Value",
        "label.description": "Description",
        "label.last_result": "Last Result",
        "label.last_log": "Last Log",
        "label.last_error": "Last Error",
        "placeholder.search": "Search parameter...",
        "placeholder.new_value": "Enter a new parameter value",
        "placeholder.desc": "Parameter description placeholder",
        "placeholder.event_log": "Connection events, command results, and errors will appear here.",
        "hint.param_default": "Select a parameter to see a local hint. Device-side validation remains authoritative.",
        "warn.bench": "Bench only: keep props removed or the frame restrained. motor_test / rate_test use conservative defaults.",
        "button.connect": "Connect",
        "button.disconnect": "Disconnect",
        "button.arm": "Arm",
        "button.disarm": "Disarm",
        "button.kill": "Kill",
        "button.reboot": "Reboot",
        "button.stream_on": "Stream On",
        "button.stream_off": "Stream Off",
        "button.apply_hz": "Apply Hz",
        "button.copy_selected": "Copy Selected",
        "button.pause_charts": "Pause",
        "button.resume_charts": "Resume",
        "button.clear_charts": "Clear",
        "button.auto_scale": "Auto Scale",
        "button.reset_view": "Reset View",
        "button.refresh": "Refresh",
        "button.save": "Save",
        "button.reset": "Reset",
        "button.export_json": "Export JSON",
        "button.import_json": "Import JSON",
        "button.set_selected": "Set Selected",
        "button.start": "Start",
        "button.stop": "Stop",
        "button.browse": "Browse",
        "button.clear_log": "Clear Log",
        "button.copy_log": "Copy Log",
        "button.save_log": "Save Log",
        "button.collapse_log": "Collapse Log",
        "button.expand_log": "Expand Log",
        "status.connected": "Connected",
        "status.disconnected": "Disconnected",
        "status.no_session": "No active session.",
        "status.no_conn_error": "No connection error.",
        "status.stream_on": "STREAM ON",
        "status.stream_off": "STREAM OFF",
        "status.no_error": "-",
        "status.no_result": "-",
        "status.no_log": "-",
        "status.no_value": "-",
        "arm.disarmed": "DISARMED",
        "arm.armed": "ARMED",
        "arm.failsafe": "FAILSAFE",
        "arm.fault_lock": "FAULT_LOCK",
        "failsafe.none": "NONE",
        "failsafe.kill": "KILL",
        "failsafe.rc_timeout": "RC_TIMEOUT",
        "failsafe.imu_timeout": "IMU_TIMEOUT",
        "failsafe.imu_parse": "IMU_PARSE",
        "failsafe.bat_critical": "BAT_CRITICAL",
        "failsafe.loop_overrun": "LOOP_OVERRUN",
        "control.idle": "IDLE",
        "control.axis_test": "AXIS_TEST",
        "control.rate_test": "RATE_TEST",
        "imu.raw": "RAW",
        "imu.direct": "DIRECT",
        "msg.saved_log": "Saved event log to {path}",
        "msg.chart_paused": "Charts paused",
        "msg.chart_resumed": "Charts resumed",
        "msg.chart_cleared": "Chart history cleared",
        "msg.connect_no_port": "No serial port selected",
        "msg.connect_no_host": "No UDP host provided",
        "msg.set_param_no_select": "No parameter selected",
        "msg.set_param_no_value": "No new parameter value provided",
        "msg.loaded_params": "Loaded {count} params",
        "msg.param_updated": "{name} updated to {value}",
        "msg.export_done": "Parameter snapshot exported",
        "msg.import_done": "Imported {count} params",
        "msg.logging_to": "Logging to {path}",
        "msg.log_stopped": "CSV log stopped ({path})",
        "msg.dump_csv_done": "Dumped {rows} telemetry rows to {path}",
        "msg.disconnect_done": "Disconnect complete",
        "msg.command_ok": "{label} complete",
        "chart.gyro": "Gyro",
        "chart.attitude": "Attitude",
        "chart.motors": "Motors",
        "chart.battery": "Battery",
        "chart.y_deg_s": "deg/s",
        "chart.y_deg": "deg",
        "chart.y_duty": "duty",
        "chart.y_volt": "V",
        "chart.time": "time (s)",
        "field.gyro_x": "gyro_x",
        "field.gyro_y": "gyro_y",
        "field.gyro_z": "gyro_z",
        "field.roll_deg": "roll_deg",
        "field.pitch_deg": "pitch_deg",
        "field.yaw_deg": "yaw_deg",
        "field.rate_setpoint_roll": "rate_setpoint_roll",
        "field.rate_setpoint_pitch": "rate_setpoint_pitch",
        "field.rate_setpoint_yaw": "rate_setpoint_yaw",
        "field.motor1": "motor1",
        "field.motor2": "motor2",
        "field.motor3": "motor3",
        "field.motor4": "motor4",
        "field.battery_voltage": "battery_voltage",
        "field.battery_adc_raw": "battery_adc_raw",
        "field.imu_age_us": "imu_age_us",
        "field.loop_dt_us": "loop_dt_us",
        "field.imu_mode": "imu_mode",
        "field.imu_health": "imu_health",
        "field.arm_state": "arm_state",
        "field.failsafe_reason": "failsafe_reason",
        "field.control_mode": "control_mode",
        "param.help.default": "No local note yet. Device-side validation remains authoritative.",
    },
}

TRANSLATIONS = {
    "zh": {
        "window.title": "ESP-DRONE 调试工作台",
        "toolbar.language": "语言",
        "lang.zh": "中文",
        "lang.en": "English",
        "group.connection": "设备连接",
        "group.safety": "安全控制",
        "group.debug": "调试动作",
        "group.chart": "主图表工作区",
        "group.realtime": "实时数值",
        "group.status": "关键状态",
        "group.params": "参数调试",
        "group.log": "事件日志",
        "group.tools": "工具",
        "group.session_info": "连接摘要",
        "tab.params": "参数",
        "tab.events": "日志",
        "tab.tools": "工具",
        "tab.motor": "电机测试",
        "tab.rate": "速率测试",
        "group.motor": "电机测试",
        "group.calib": "校准",
        "group.rate": "速率测试",
        "group.csv": "日志导出",
        "label.link": "连接方式",
        "label.serial_port": "串口",
        "label.baud": "波特率",
        "label.udp_host": "UDP 主机",
        "label.udp_port": "UDP 端口",
        "label.session": "会话信息",
        "label.last_conn_error": "最近连接错误",
        "label.target_hz": "目标频率",
        "label.chart_group": "通道组",
        "label.window": "时间窗",
        "label.motor": "电机",
        "label.duty": "占空比",
        "label.axis": "轴",
        "label.rate_dps": "角速度(dps)",
        "label.output": "输出文件",
        "label.dump_s": "导出时长(s)",
        "label.name": "名称",
        "label.type": "类型",
        "label.current": "当前值",
        "label.new_value": "新值",
        "label.description": "说明",
        "label.last_result": "最近结果",
        "label.last_log": "最近日志",
        "label.last_error": "最近错误",
        "placeholder.search": "搜索参数...",
        "placeholder.new_value": "输入新的参数值",
        "placeholder.desc": "参数说明占位区",
        "placeholder.event_log": "连接事件、命令结果和错误会显示在这里。",
        "hint.param_default": "选择参数后可查看本地提示。最终校验仍以设备端为准。",
        "warn.bench": "仅限台架调试：请拆桨或固定机体。motor_test / rate_test 默认使用保守值。",
        "button.connect": "连接",
        "button.disconnect": "断开",
        "button.arm": "解锁",
        "button.disarm": "上锁",
        "button.kill": "急停",
        "button.reboot": "重启",
        "button.stream_on": "开始流",
        "button.stream_off": "停止流",
        "button.apply_hz": "应用频率",
        "button.copy_selected": "复制选中",
        "button.pause_charts": "暂停图表",
        "button.resume_charts": "继续图表",
        "button.clear_charts": "清空图表",
        "button.auto_scale": "自动缩放",
        "button.reset_view": "重置视图",
        "button.refresh": "刷新",
        "button.save": "保存",
        "button.reset": "恢复默认",
        "button.export_json": "导出 JSON",
        "button.import_json": "导入 JSON",
        "button.set_selected": "设置当前参数",
        "button.start": "开始",
        "button.stop": "停止",
        "button.calib_gyro": "校准陀螺仪",
        "button.calib_level": "校准水平",
        "button.calib_gyro_short": "校陀螺",
        "button.calib_level_short": "校水平",
        "button.start_log": "开始记录",
        "button.stop_log": "停止记录",
        "button.dump_csv": "导出 CSV",
        "button.browse": "浏览",
        "button.clear_log": "清空日志",
        "button.copy_log": "复制日志",
        "button.save_log": "保存日志",
        "button.collapse_log": "折叠日志",
        "button.expand_log": "展开日志",
        "status.connected": "已连接",
        "status.disconnected": "未连接",
        "status.no_session": "当前无活动会话。",
        "status.no_conn_error": "当前无连接错误。",
        "status.stream_on": "流已开启",
        "status.stream_off": "流未开启",
        "status.no_error": "-",
        "status.no_result": "-",
        "status.no_log": "-",
        "status.no_value": "-",
        "arm.disarmed": "已上锁",
        "arm.armed": "已解锁",
        "arm.failsafe": "失效保护",
        "arm.fault_lock": "故障锁定",
        "failsafe.none": "无",
        "failsafe.kill": "急停",
        "failsafe.rc_timeout": "遥控超时",
        "failsafe.imu_timeout": "IMU 超时",
        "failsafe.imu_parse": "IMU 解析错误",
        "failsafe.bat_critical": "电池严重欠压",
        "failsafe.loop_overrun": "控制循环超时",
        "control.idle": "空闲",
        "control.axis_test": "轴测试",
        "control.rate_test": "速率测试",
        "imu.raw": "RAW",
        "imu.direct": "DIRECT",
        "msg.saved_log": "已保存事件日志到 {path}",
        "msg.chart_paused": "图表已暂停",
        "msg.chart_resumed": "图表已继续",
        "msg.chart_cleared": "图表历史已清空",
        "msg.connect_no_port": "未选择串口",
        "msg.connect_no_host": "未填写 UDP 主机",
        "msg.set_param_no_select": "未选择参数",
        "msg.set_param_no_value": "未输入新的参数值",
        "msg.loaded_params": "已加载 {count} 个参数",
        "msg.param_updated": "{name} 已更新为 {value}",
        "msg.export_done": "参数快照已导出",
        "msg.import_done": "已导入 {count} 个参数",
        "msg.logging_to": "正在记录到 {path}",
        "msg.log_stopped": "CSV 日志已停止 ({path})",
        "msg.dump_csv_done": "已导出 {rows} 行遥测到 {path}",
        "msg.disconnect_done": "已断开连接",
        "msg.command_ok": "{label} 完成",
        "msg.command_running": "{label} 执行中...",
        "msg.connected": "已连接：{info}",
        "msg.disconnected": "连接已断开",
        "chart.gyro": "陀螺仪",
        "chart.attitude": "姿态",
        "chart.motors": "电机输出",
        "chart.battery": "电池电压",
        "chart.y_deg_s": "角速度 (deg/s)",
        "chart.y_deg": "角度 (deg)",
        "chart.y_duty": "占空比",
        "chart.y_volt": "电压 (V)",
        "chart.time": "时间 (s)",
        "transport.serial": "串口",
        "axis.roll": "横滚",
        "axis.pitch": "俯仰",
        "axis.yaw": "偏航",
        "field.gyro_x": "陀螺仪 X (gyro_x)",
        "field.gyro_y": "陀螺仪 Y (gyro_y)",
        "field.gyro_z": "陀螺仪 Z (gyro_z)",
        "field.roll_deg": "横滚角 (roll_deg)",
        "field.pitch_deg": "俯仰角 (pitch_deg)",
        "field.yaw_deg": "偏航角 (yaw_deg)",
        "field.rate_setpoint_roll": "横滚速率目标",
        "field.rate_setpoint_pitch": "俯仰速率目标",
        "field.rate_setpoint_yaw": "偏航速率目标",
        "field.motor1": "电机1",
        "field.motor2": "电机2",
        "field.motor3": "电机3",
        "field.motor4": "电机4",
        "field.battery_voltage": "电池电压",
        "field.battery_adc_raw": "电池 ADC 原始值",
        "field.imu_age_us": "IMU 样本龄",
        "field.loop_dt_us": "控制循环周期",
        "field.imu_mode": "IMU 模式",
        "field.imu_health": "IMU 健康",
        "field.arm_state": "解锁状态",
        "field.failsafe_reason": "失效保护原因",
        "field.control_mode": "控制模式",
        "field.stream": "流状态",
        "param.help.default": "暂无本地说明。最终校验仍以设备端为准。",
    },
    "en": {
        "window.title": "ESP-DRONE Debug Workbench",
        "toolbar.language": "Language",
        "lang.zh": "中文",
        "lang.en": "English",
        "group.connection": "Connection",
        "group.safety": "Safety Control",
        "group.debug": "Debug Actions",
        "group.chart": "Main Chart Workspace",
        "group.realtime": "Realtime Values",
        "group.status": "Key Status",
        "group.params": "Parameters",
        "group.log": "Event Log",
        "group.tools": "Tools",
        "group.session_info": "Session",
        "group.motor": "Motor Test",
        "group.calib": "Calibration",
        "group.rate": "Rate Test",
        "group.csv": "Log Export",
        "tab.params": "Parameters",
        "tab.events": "Events",
        "tab.tools": "Tools",
        "tab.motor": "Motor Test",
        "tab.rate": "Rate Test",
        "label.link": "Link",
        "label.serial_port": "Serial Port",
        "label.baud": "Baud",
        "label.udp_host": "UDP Host",
        "label.udp_port": "UDP Port",
        "label.session": "Session",
        "label.last_conn_error": "Last Connection Error",
        "label.target_hz": "Target Hz",
        "label.chart_group": "Chart Group",
        "label.window": "Window",
        "label.motor": "Motor",
        "label.duty": "Duty",
        "label.axis": "Axis",
        "label.rate_dps": "Rate dps",
        "label.output": "Output File",
        "label.dump_s": "Dump Duration (s)",
        "label.name": "Name",
        "label.type": "Type",
        "label.current": "Current",
        "label.new_value": "New Value",
        "label.description": "Description",
        "label.last_result": "Last Result",
        "label.last_log": "Last Log",
        "label.last_error": "Last Error",
        "placeholder.search": "Search parameter...",
        "placeholder.new_value": "Enter a new parameter value",
        "placeholder.desc": "Parameter description placeholder",
        "placeholder.event_log": "Connection events, command results, and errors appear here.",
        "hint.param_default": "Select a parameter to see a local hint. Device-side validation remains authoritative.",
        "warn.bench": "Bench only: keep props removed or the frame restrained. motor_test / rate_test use conservative defaults.",
        "button.connect": "Connect",
        "button.disconnect": "Disconnect",
        "button.arm": "Arm",
        "button.disarm": "Disarm",
        "button.kill": "Kill",
        "button.reboot": "Reboot",
        "button.stream_on": "Stream On",
        "button.stream_off": "Stream Off",
        "button.apply_hz": "Apply Hz",
        "button.copy_selected": "Copy Selected",
        "button.pause_charts": "Pause Charts",
        "button.resume_charts": "Resume Charts",
        "button.clear_charts": "Clear Charts",
        "button.auto_scale": "Auto Scale",
        "button.reset_view": "Reset View",
        "button.refresh": "Refresh",
        "button.save": "Save",
        "button.reset": "Reset",
        "button.export_json": "Export JSON",
        "button.import_json": "Import JSON",
        "button.set_selected": "Set Selected",
        "button.start": "Start",
        "button.stop": "Stop",
        "button.calib_gyro": "Calib Gyro",
        "button.calib_level": "Calib Level",
        "button.calib_gyro_short": "Gyro",
        "button.calib_level_short": "Level",
        "button.start_log": "Start Log",
        "button.stop_log": "Stop Log",
        "button.dump_csv": "Dump CSV",
        "button.browse": "Browse",
        "button.clear_log": "Clear Log",
        "button.copy_log": "Copy Log",
        "button.save_log": "Save Log",
        "button.collapse_log": "Collapse Log",
        "button.expand_log": "Expand Log",
        "status.connected": "Connected",
        "status.disconnected": "Disconnected",
        "status.no_session": "No active session.",
        "status.no_conn_error": "No connection error.",
        "status.stream_on": "STREAM ON",
        "status.stream_off": "STREAM OFF",
        "status.no_error": "-",
        "status.no_result": "-",
        "status.no_log": "-",
        "status.no_value": "-",
        "arm.disarmed": "DISARMED",
        "arm.armed": "ARMED",
        "arm.failsafe": "FAILSAFE",
        "arm.fault_lock": "FAULT_LOCK",
        "failsafe.none": "NONE",
        "failsafe.kill": "KILL",
        "failsafe.rc_timeout": "RC_TIMEOUT",
        "failsafe.imu_timeout": "IMU_TIMEOUT",
        "failsafe.imu_parse": "IMU_PARSE",
        "failsafe.bat_critical": "BAT_CRITICAL",
        "failsafe.loop_overrun": "LOOP_OVERRUN",
        "control.idle": "IDLE",
        "control.axis_test": "AXIS_TEST",
        "control.rate_test": "RATE_TEST",
        "imu.raw": "RAW",
        "imu.direct": "DIRECT",
        "msg.saved_log": "Saved event log to {path}",
        "msg.chart_paused": "Charts paused",
        "msg.chart_resumed": "Charts resumed",
        "msg.chart_cleared": "Chart history cleared",
        "msg.connect_no_port": "No serial port selected",
        "msg.connect_no_host": "No UDP host provided",
        "msg.set_param_no_select": "No parameter selected",
        "msg.set_param_no_value": "No new parameter value provided",
        "msg.loaded_params": "Loaded {count} params",
        "msg.param_updated": "{name} updated to {value}",
        "msg.export_done": "Parameter snapshot exported",
        "msg.import_done": "Imported {count} params",
        "msg.logging_to": "Logging to {path}",
        "msg.log_stopped": "CSV log stopped ({path})",
        "msg.dump_csv_done": "Dumped {rows} telemetry rows to {path}",
        "msg.disconnect_done": "Disconnect complete",
        "msg.command_ok": "{label} complete",
        "msg.command_running": "{label} running...",
        "msg.connected": "Connected: {info}",
        "msg.disconnected": "Disconnected",
        "chart.gyro": "Gyro",
        "chart.attitude": "Attitude",
        "chart.motors": "Motors",
        "chart.battery": "Battery",
        "chart.y_deg_s": "deg/s",
        "chart.y_deg": "deg",
        "chart.y_duty": "duty",
        "chart.y_volt": "V",
        "chart.time": "time (s)",
        "transport.serial": "Serial",
        "axis.roll": "Roll",
        "axis.pitch": "Pitch",
        "axis.yaw": "Yaw",
        "field.gyro_x": "gyro_x",
        "field.gyro_y": "gyro_y",
        "field.gyro_z": "gyro_z",
        "field.roll_deg": "roll_deg",
        "field.pitch_deg": "pitch_deg",
        "field.yaw_deg": "yaw_deg",
        "field.rate_setpoint_roll": "rate_setpoint_roll",
        "field.rate_setpoint_pitch": "rate_setpoint_pitch",
        "field.rate_setpoint_yaw": "rate_setpoint_yaw",
        "field.motor1": "motor1",
        "field.motor2": "motor2",
        "field.motor3": "motor3",
        "field.motor4": "motor4",
        "field.battery_voltage": "battery_voltage",
        "field.battery_adc_raw": "battery_adc_raw",
        "field.imu_age_us": "imu_age_us",
        "field.loop_dt_us": "loop_dt_us",
        "field.imu_mode": "imu_mode",
        "field.imu_health": "imu_health",
        "field.arm_state": "arm_state",
        "field.failsafe_reason": "failsafe_reason",
        "field.control_mode": "control_mode",
        "field.stream": "stream",
        "param.help.default": "No local note yet. Device-side validation remains authoritative.",
    },
}

EXTRA_TRANSLATIONS = {
    "zh": {
        "status.connecting": "连接中...",
        "msg.connect_failed": "连接失败：{error}",
        "chart.baro": "气压计",
        "chart.y_baro": "气压 / 高度 / 温度",
        "chart.rate_roll": "横滚速率",
        "chart.rate_pitch": "俯仰速率",
        "chart.rate_yaw": "偏航速率",
        "chart.hang_roll": "姿态台架横滚",
        "chart.hang_pitch": "姿态台架俯仰",
        "chart.y_rate_mix": "角速度 / 占空比",
        "chart.y_attitude_mix": "角度 / 角速度 / 占空比",
        "field.baro_pressure_pa": "气压 (Pa)",
        "field.baro_temperature_c": "温度 (C)",
        "field.baro_altitude_m": "气压高度 (m)",
        "field.baro_vspeed_mps": "垂直速度 (m/s)",
        "field.baro_valid": "气压数据有效",
        "field.baro_health": "气压计健康",
        "field.baro_update_age_us": "气压计样本龄",
        "field.rate_pid_p_roll": "横滚 PID P",
        "field.rate_pid_p_pitch": "俯仰 PID P",
        "field.rate_pid_p_yaw": "偏航 PID P",
        "field.rate_pid_i_roll": "横滚 PID I",
        "field.rate_pid_i_pitch": "俯仰 PID I",
        "field.rate_pid_i_yaw": "偏航 PID I",
        "field.rate_pid_d_roll": "横滚 PID D",
        "field.rate_pid_d_pitch": "俯仰 PID D",
        "field.rate_pid_d_yaw": "偏航 PID D",
        "field.pid_out_roll": "横滚 PID 输出",
        "field.pid_out_pitch": "俯仰 PID 输出",
        "field.pid_out_yaw": "偏航 PID 输出",
        "baro.init": "初始化",
        "baro.ok": "正常",
        "baro.stale": "陈旧",
        "baro.invalid": "无效",
        "baro.valid": "有效",
        "baro.invalid_data": "无效",
        "control.height_hold_reserved": "定高保留",
        "control.udp_manual": "UDP 手动",
        "tab.hang_attitude": "姿态台架",
        "tab.udp_control": "UDP 控制",
        "hang.note": "仅限台架约束测试。带桨自由飞行时不要使用。",
        "hang.capture_ref": "捕获参考",
        "hang.start": "启动姿态测试",
        "hang.stop": "停止姿态测试",
        "hang.base_duty": "基础占空比",
        "hang.kp_roll": "横滚 Kp",
        "hang.kp_pitch": "俯仰 Kp",
        "hang.rate_limit_roll": "横滚速率限幅",
        "hang.rate_limit_pitch": "俯仰速率限幅",
        "hang.deadband_deg": "死区角度",
        "hang.trip_deg": "保护角度",
        "udp.warn": "实验性 UDP 手动控制：油门是基础占空比目标，横滚/俯仰/偏航经 rate PID 闭环后进入混控。请限制最大 PWM 并处理好桨叶安全；尚不适合自由飞行。",
        "udp.max_pwm": "最大 PWM (%)",
        "udp.throttle": "基础油门 (%)",
        "udp.axis_step": "速率指令步进 (%)",
        "udp.pitch": "俯仰速率 (%)",
        "udp.roll": "横滚速率 (%)",
        "udp.yaw": "偏航速率 (%)",
        "udp.enable": "启用 UDP 手动",
        "udp.disable": "禁用",
        "udp.stop": "停止 / 清零",
        "udp.takeoff": "起飞",
        "udp.land": "降落",
        "udp.forward": "前进",
        "udp.backward": "后退",
        "udp.up": "加油门",
        "udp.down": "减油门",
        "udp.yaw_left": "左偏航",
        "udp.yaw_right": "右偏航",
        "udp.send": "发送设定",
        "udp.status_watchdog": "看门狗",
        "udp.status_mode": "模式",
        "udp.status_armed": "解锁状态",
        "udp.status_battery": "电池",
        "udp.enabled": "UDP 手动已启用",
        "udp.disabled": "UDP 手动已禁用",
        "udp.stopped": "UDP 手动停止命令已发送",
        "udp.setpoint_sent": "UDP 设定已发送：基础油门={throttle:.3f} 俯仰={pitch:.3f} 横滚={roll:.3f} 偏航={yaw:.3f}",
        "udp.watchdog_age": "距上一帧设定 {age_ms:.0f} ms",
        "udp.transport_hint": "请先将电脑连接到 ESP-DRONE SoftAP。默认 AP IP 为 192.168.4.1，默认 UDP 端口为 2391。",
        "udp.ap_info": "默认 SoftAP SSID：ESP-DRONE | 密码：12345678",
        "msg.udp_host_required": "需要填写 UDP 主机。",
    },
    "en": {
        "status.connecting": "Connecting...",
        "msg.connect_failed": "Connect failed: {error}",
        "chart.baro": "Barometer",
        "chart.y_baro": "baro / altitude / temp",
        "chart.rate_roll": "Rate Roll",
        "chart.rate_pitch": "Rate Pitch",
        "chart.rate_yaw": "Rate Yaw",
        "chart.y_rate_mix": "deg/s / duty",
        "field.baro_pressure_pa": "baro_pressure_pa",
        "field.baro_temperature_c": "baro_temperature_c",
        "field.baro_altitude_m": "baro_altitude_m",
        "field.baro_vspeed_mps": "baro_vspeed_mps",
        "field.baro_valid": "baro_valid",
        "field.baro_health": "baro_health",
        "field.baro_update_age_us": "baro_update_age_us",
        "field.rate_pid_p_roll": "rate_pid_p_roll",
        "field.rate_pid_p_pitch": "rate_pid_p_pitch",
        "field.rate_pid_p_yaw": "rate_pid_p_yaw",
        "field.rate_pid_i_roll": "rate_pid_i_roll",
        "field.rate_pid_i_pitch": "rate_pid_i_pitch",
        "field.rate_pid_i_yaw": "rate_pid_i_yaw",
        "field.rate_pid_d_roll": "rate_pid_d_roll",
        "field.rate_pid_d_pitch": "rate_pid_d_pitch",
        "field.rate_pid_d_yaw": "rate_pid_d_yaw",
        "field.pid_out_roll": "pid_out_roll",
        "field.pid_out_pitch": "pid_out_pitch",
        "field.pid_out_yaw": "pid_out_yaw",
        "field.attitude_ref_valid": "attitude_ref_valid",
        "field.attitude_err_roll_deg": "attitude_err_roll_deg",
        "field.attitude_err_pitch_deg": "attitude_err_pitch_deg",
        "field.attitude_rate_sp_roll": "attitude_rate_sp_roll",
        "field.attitude_rate_sp_pitch": "attitude_rate_sp_pitch",
        "field.attitude_ref_qw": "attitude_ref_qw",
        "field.attitude_ref_qx": "attitude_ref_qx",
        "field.attitude_ref_qy": "attitude_ref_qy",
        "field.attitude_ref_qz": "attitude_ref_qz",
        "field.base_duty_active": "base_duty_active",
        "baro.init": "INIT",
        "baro.ok": "OK",
        "baro.stale": "STALE",
        "baro.invalid": "INVALID",
        "baro.valid": "VALID",
        "baro.invalid_data": "INVALID",
        "control.height_hold_reserved": "HEIGHT_HOLD_RESERVED",
        "control.attitude_hang_test": "ATTITUDE_HANG_TEST",
        "control.udp_manual": "UDP_MANUAL",
        "tab.hang_attitude": "Hang Attitude",
        "tab.udp_control": "UDP Control",
        "hang.note": "Bench-only constrained rig. Never use with prop-on free flight.",
        "hang.capture_ref": "Capture Ref",
        "hang.start": "Attitude Test Start",
        "hang.stop": "Attitude Test Stop",
        "hang.base_duty": "Base Duty",
        "hang.kp_roll": "Kp Roll",
        "hang.kp_pitch": "Kp Pitch",
        "hang.rate_limit_roll": "Rate Limit Roll",
        "hang.rate_limit_pitch": "Rate Limit Pitch",
        "hang.deadband_deg": "Deadband Deg",
        "hang.trip_deg": "Trip Deg",
        "udp.warn": "Experimental UDP manual control. Throttle is the base duty target; roll/pitch use attitude hold and yaw uses rate PID. Respect Max PWM and keep prop safety in mind. Not free-flight ready.",
        "udp.max_pwm": "Max PWM (%)",
        "udp.throttle": "Throttle (%)",
        "udp.axis_step": "Axis Step (%)",
        "udp.pitch": "Pitch (%)",
        "udp.roll": "Roll (%)",
        "udp.yaw": "Yaw (%)",
        "udp.enable": "Enable UDP Manual",
        "udp.disable": "Disable",
        "udp.stop": "Stop / Zero",
        "udp.takeoff": "Takeoff",
        "udp.land": "Land",
        "udp.forward": "Forward",
        "udp.backward": "Backward",
        "udp.up": "Up",
        "udp.down": "Down",
        "udp.yaw_left": "Yaw Left",
        "udp.yaw_right": "Yaw Right",
        "udp.send": "Send Setpoint",
        "udp.status_watchdog": "Watchdog",
        "udp.status_mode": "Mode",
        "udp.status_armed": "Armed",
        "udp.status_battery": "Battery",
        "udp.enabled": "udp manual enabled",
        "udp.disabled": "udp manual disabled",
        "udp.stopped": "udp manual stop sent",
        "udp.setpoint_sent": "udp setpoint sent base={throttle:.3f} pitch={pitch:.3f} roll={roll:.3f} yaw={yaw:.3f}",
        "udp.watchdog_age": "{age_ms:.0f} ms since setpoint",
        "udp.transport_hint": "Connect your PC to the ESP-DRONE SoftAP first. Default AP IP is 192.168.4.1. Default UDP port is 2391.",
        "udp.ap_info": "Default SoftAP SSID: ESP-DRONE | Password: 12345678",
        "msg.udp_host_required": "UDP Host is required.",
    },
}

TELEMETRY_FIELD_KEYS = {
    "gyro_x": "field.gyro_x",
    "gyro_y": "field.gyro_y",
    "gyro_z": "field.gyro_z",
    "roll_deg": "field.roll_deg",
    "pitch_deg": "field.pitch_deg",
    "yaw_deg": "field.yaw_deg",
    "rate_setpoint_roll": "field.rate_setpoint_roll",
    "rate_setpoint_pitch": "field.rate_setpoint_pitch",
    "rate_setpoint_yaw": "field.rate_setpoint_yaw",
    "motor1": "field.motor1",
    "motor2": "field.motor2",
    "motor3": "field.motor3",
    "motor4": "field.motor4",
    "battery_voltage": "field.battery_voltage",
    "battery_adc_raw": "field.battery_adc_raw",
    "baro_pressure_pa": "field.baro_pressure_pa",
    "baro_temperature_c": "field.baro_temperature_c",
    "baro_altitude_m": "field.baro_altitude_m",
    "baro_vspeed_mps": "field.baro_vspeed_mps",
    "baro_valid": "field.baro_valid",
    "baro_health": "field.baro_health",
    "baro_update_age_us": "field.baro_update_age_us",
    "imu_age_us": "field.imu_age_us",
    "loop_dt_us": "field.loop_dt_us",
    "imu_mode": "field.imu_mode",
    "imu_health": "field.imu_health",
    "arm_state": "field.arm_state",
    "failsafe_reason": "field.failsafe_reason",
    "control_mode": "field.control_mode",
    "attitude_ref_valid": "field.attitude_ref_valid",
    "attitude_err_roll_deg": "field.attitude_err_roll_deg",
    "attitude_err_pitch_deg": "field.attitude_err_pitch_deg",
    "attitude_rate_sp_roll": "field.attitude_rate_sp_roll",
    "attitude_rate_sp_pitch": "field.attitude_rate_sp_pitch",
    "attitude_ref_qw": "field.attitude_ref_qw",
    "attitude_ref_qx": "field.attitude_ref_qx",
    "attitude_ref_qy": "field.attitude_ref_qy",
    "attitude_ref_qz": "field.attitude_ref_qz",
    "base_duty_active": "field.base_duty_active",
    "filtered_gyro_x": "filtered_gyro_x",
    "filtered_gyro_y": "filtered_gyro_y",
    "filtered_gyro_z": "filtered_gyro_z",
    "filtered_acc_x": "filtered_acc_x",
    "filtered_acc_y": "filtered_acc_y",
    "filtered_acc_z": "filtered_acc_z",
    "kalman_roll_deg": "kalman_roll_deg",
    "kalman_pitch_deg": "kalman_pitch_deg",
    "rate_meas_roll_raw": "rate_meas_roll_raw",
    "rate_meas_pitch_raw": "rate_meas_pitch_raw",
    "rate_meas_yaw_raw": "rate_meas_yaw_raw",
    "rate_meas_roll_filtered": "rate_meas_roll_filtered",
    "rate_meas_pitch_filtered": "rate_meas_pitch_filtered",
    "rate_meas_yaw_filtered": "rate_meas_yaw_filtered",
    "rate_err_roll": "rate_err_roll",
    "rate_err_pitch": "rate_err_pitch",
    "rate_err_yaw": "rate_err_yaw",
    "motor_saturation_flag": "motor_saturation_flag",
    "integrator_freeze_flag": "integrator_freeze_flag",
    "ground_ref_valid": "ground_ref_valid",
    "reference_valid": "reference_valid",
    "ground_trip_reason": "ground_trip_reason",
}

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
    0: ("arm.disarmed", "neutral"),
    1: ("arm.armed", "ok"),
    2: ("arm.failsafe", "warn"),
    3: ("arm.fault_lock", "danger"),
}

FAILSAFE_TEXT = {
    0: ("failsafe.none", "neutral"),
    1: ("failsafe.kill", "danger"),
    2: ("failsafe.rc_timeout", "warn"),
    3: ("failsafe.imu_timeout", "warn"),
    4: ("failsafe.imu_parse", "warn"),
    5: ("failsafe.bat_critical", "danger"),
    6: ("failsafe.loop_overrun", "warn"),
}

CONTROL_MODE_TEXT = {
    0: ("control.idle", "neutral"),
    1: ("control.axis_test", "active"),
    2: ("control.rate_test", "active"),
    3: ("control.height_hold_reserved", "warn"),
    4: ("control.attitude_hang_test", "active"),
    5: ("control.udp_manual", "warn"),
    6: ("control.ground_tune", "active"),
}

IMU_MODE_TEXT = {
    0: ("imu.raw", "warn"),
    1: ("imu.direct", "ok"),
}

BARO_HEALTH_TEXT = {
    0: ("baro.init", "neutral"),
    1: ("baro.ok", "ok"),
    2: ("baro.stale", "warn"),
    3: ("baro.invalid", "danger"),
}

BARO_VALID_TEXT = {
    0: ("baro.invalid_data", "danger"),
    1: ("baro.valid", "ok"),
}


def _format_float(value: float) -> str:
    return f"{value:.3f}"


def _format_value(name: str, value: object) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        if "battery_voltage" in name:
            return f"{value:.3f}"
        if "baro_pressure_pa" in name:
            return f"{value:.1f}"
        if "baro_temperature_c" in name:
            return f"{value:.2f}"
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
    if name == "baro_health":
        return _status_from_map(BARO_HEALTH_TEXT, raw_value)
    if name == "baro_valid":
        return _status_from_map(BARO_VALID_TEXT, raw_value)
    return None


class CopyableTableWidget(QTableWidget):
    """支持将当前表格选区复制为制表符文本。"""

    def keyPressEvent(self, event) -> None:  # pragma: no cover - UI handling
        """拦截复制快捷键并转发到 `copy_selection()`。"""

        if event.matches(QKeySequence.Copy):
            self.copy_selection()
            return
        super().keyPressEvent(event)

    def copy_selection(self) -> None:
        """将首个连续选区复制到系统剪贴板。

        Note:
            当前实现只处理 `selectedRanges()` 返回的第一个矩形区域。
        """

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
    """将 `DeviceSession` 的订阅回调桥接为 Qt 信号。"""

    connection_changed = pyqtSignal(object)
    telemetry_received = pyqtSignal(object)
    event_received = pyqtSignal(str)
    command_finished = pyqtSignal(str, object)
    error_raised = pyqtSignal(str)

    def __init__(self, session: DeviceSession) -> None:
        """创建桥接对象并注册会话订阅。

        Args:
            session: 需要桥接到 Qt 事件系统的设备会话。
        """

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
        """移除已注册的会话订阅。

        Note:
            窗口关闭前应调用本方法，避免会话继续向已销毁对象分发事件。
        """

        self._session.unsubscribe(self._connection_token)
        self._session.unsubscribe(self._telemetry_token)
        self._session.unsubscribe(self._event_token)

    def run_async(self, label: str, callback) -> None:
        """在后台线程执行会话操作并回传结果。

        Args:
            label: 操作标识，会随结果或错误一起发回。
            callback: 需要异步执行的无参回调。

        Note:
            回调异常不会向调用栈继续抛出，而是转成 `error_raised` 信号。
        """

        def worker() -> None:
            try:
                result = callback()
                self.command_finished.emit(label, result)
            except Exception as exc:  # pragma: no cover - depends on runtime/device state
                if label in {"connect_serial", "connect_udp"}:
                    self.error_raised.emit(f"Connect failed: {exc}")
                else:
                    self.error_raised.emit(f"{label}: {exc}")

        threading.Thread(target=worker, daemon=True, name=f"esp-drone-gui-{label}").start()


class TelemetryHistory:
    """按字段缓存近期遥测样本，供图表窗口切片显示。"""

    def __init__(self, max_samples: int = 12000) -> None:
        """初始化历史缓存。

        Args:
            max_samples: 每个字段最多保留的样本数。
        """

        self._series: dict[str, deque[tuple[float, float]]] = {}
        self._max_samples = max_samples

    def append(self, sample: TelemetrySample) -> None:
        """追加一帧遥测样本中的数值字段。

        Args:
            sample: 待写入缓存的遥测样本。
        """

        values = sample.to_display_map()
        timestamp = time.monotonic()
        for key, value in values.items():
            if not isinstance(value, (int, float)):
                continue
            if key not in self._series:
                self._series[key] = deque(maxlen=self._max_samples)
            self._series[key].append((timestamp, float(value)))

    def clear(self) -> None:
        """清空全部历史样本。"""

        self._series.clear()

    def slice(self, name: str, window_s: float) -> tuple[list[float], list[float]]:
        """返回指定字段在时间窗口内的绘图数据。

        Args:
            name: 遥测字段名。
            window_s: 回看窗口，单位为秒。

        Returns:
            `(xs, ys)` 二元组，其中 `xs` 以最新样本时间为 `0`，向过去为负值。
            字段不存在或窗口内无数据时返回两个空列表。
        """

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


class CollapsibleSection(QWidget):
    """带标题栏的可折叠内容容器。"""

    toggled = pyqtSignal(bool)

    def __init__(self, title: str = "", expanded: bool = True, parent: QWidget | None = None) -> None:
        """初始化折叠面板。

        Args:
            title: 标题文本。
            expanded: 初始展开状态。
            parent: Qt 父对象。
        """

        super().__init__(parent)
        self._expanded = expanded

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.toggle_button = QToolButton()
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(expanded)
        self.toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toggle_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.toggle_button.setStyleSheet(
            "QToolButton {text-align:left; font-weight:600; padding:4px 6px; border:none; color:#e5eef9;}"
        )
        layout.addWidget(self.toggle_button)

        self.body = QWidget()
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        self.body_layout.setSpacing(0)
        layout.addWidget(self.body)

        self.toggle_button.toggled.connect(self.set_expanded)
        self.set_title(title)
        self.set_expanded(expanded)

    def set_title(self, title: str) -> None:
        """更新标题栏文本。

        Args:
            title: 新标题。
        """

        self.toggle_button.setText(title)

    def set_content(self, widget: QWidget) -> None:
        """替换折叠体中的内容控件。

        Args:
            widget: 需要嵌入面板主体的新控件。
        """

        while self.body_layout.count():
            item = self.body_layout.takeAt(0)
            old = item.widget()
            if old is not None:
                old.setParent(None)
        self.body_layout.addWidget(widget)

    def is_expanded(self) -> bool:
        """返回当前是否处于展开状态。"""

        return self._expanded

    def set_expanded(self, expanded: bool) -> None:
        """设置展开状态并同步箭头与可见性。

        Args:
            expanded: 目标展开状态。
        """

        expanded = bool(expanded)
        self._expanded = expanded
        self.toggle_button.blockSignals(True)
        self.toggle_button.setChecked(expanded)
        self.toggle_button.blockSignals(False)
        self.toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.body.setVisible(expanded)
        self.toggled.emit(expanded)


@dataclass(slots=True)
class ChartSpec:
    """描述图表分组的标题、字段集合和纵轴标签。"""

    title: str
    fields: list[tuple[str, str]]
    y_label: str


class MainWindow(QMainWindow):
    """ESP-DRONE 桌面调试主窗口。

    窗口负责组织连接、遥测图表、参数调试和日志导出等界面元素，
    但设备通信仍通过 `DeviceSession` 与 `QtSessionBridge` 完成。
    """

    TELEMETRY_FIELDS = [
        "gyro_x", "gyro_y", "gyro_z",
        "roll_deg", "pitch_deg", "yaw_deg",
        "rate_setpoint_roll", "rate_setpoint_pitch", "rate_setpoint_yaw",
        "rate_pid_p_roll", "rate_pid_p_pitch", "rate_pid_p_yaw",
        "rate_pid_i_roll", "rate_pid_i_pitch", "rate_pid_i_yaw",
        "rate_pid_d_roll", "rate_pid_d_pitch", "rate_pid_d_yaw",
        "pid_out_roll", "pid_out_pitch", "pid_out_yaw",
        "motor1", "motor2", "motor3", "motor4",
        "battery_voltage", "battery_adc_raw",
        "baro_pressure_pa", "baro_temperature_c", "baro_altitude_m", "baro_vspeed_mps",
        "baro_valid", "baro_health", "baro_update_age_us",
        "attitude_ref_valid",
        "attitude_err_roll_deg", "attitude_err_pitch_deg",
        "attitude_rate_sp_roll", "attitude_rate_sp_pitch",
        "attitude_ref_qw", "attitude_ref_qx", "attitude_ref_qy", "attitude_ref_qz",
        "base_duty_active",
        "filtered_gyro_x", "filtered_gyro_y", "filtered_gyro_z",
        "filtered_acc_x", "filtered_acc_y", "filtered_acc_z",
        "kalman_roll_deg", "kalman_pitch_deg", "kalman_valid",
        "rate_meas_roll_raw", "rate_meas_roll_filtered",
        "rate_meas_pitch_raw", "rate_meas_pitch_filtered",
        "rate_meas_yaw_raw", "rate_meas_yaw_filtered",
        "rate_err_roll", "rate_err_pitch", "rate_err_yaw",
        "motor_saturation_flag", "integrator_freeze_flag",
        "ground_ref_valid", "reference_valid", "ground_trip_reason",
        "imu_age_us", "loop_dt_us",
        "imu_mode", "imu_health", "arm_state", "failsafe_reason", "control_mode",
    ]

    PARAM_HELP = {
        "telemetry_usb_hz": "USB CDC telemetry target rate in Hz. Device-side bounds still apply.",
        "telemetry_udp_hz": "UDP telemetry target rate in Hz. Device-side bounds still apply.",
        "wifi_ap_enable": "Enable ESP32 SoftAP on boot. USB CDC remains available if Wi-Fi fails.",
        "wifi_ap_channel": "ESP32 SoftAP channel, valid range 1..13. Default is 6.",
        "wifi_udp_port": "Binary CLI/GUI UDP protocol listening port. Default is 2391.",
        "imu_mode": "0 = RAW, 1 = DIRECT. Bench validation is currently centered on DIRECT mode.",
        "imu_return_rate_code": "ATK-MS901M return-rate code. 0x00 is 250Hz, 0x01 is 200Hz.",
        "motor_idle_duty": "Brushed motor armed idle floor, normalized 0..1.",
        "motor_max_duty": "Brushed motor output ceiling, normalized 0..1.",
        "bringup_test_base_duty": "Base duty used in axis/rate bench tests. Keep low on a restrained frame.",
        "attitude_kp_roll": "Bench-only hang attitude outer-loop P gain for roll. Do not use as a free-flight angle gain.",
        "attitude_kp_pitch": "Bench-only hang attitude outer-loop P gain for pitch. Do not use as a free-flight angle gain.",
        "attitude_rate_limit_roll": "Bench-only roll rate-setpoint clamp in deg/s for the hang-attitude outer loop.",
        "attitude_rate_limit_pitch": "Bench-only pitch rate-setpoint clamp in deg/s for the hang-attitude outer loop.",
        "attitude_error_deadband_deg": "Small-angle deadband around the captured hanging reference, in degrees.",
        "attitude_trip_deg": "Safety trip threshold for roll/pitch attitude error on the constrained bench.",
        "attitude_test_base_duty": "Base duty for the bench-only hang attitude mode. Keep conservative on the constrained rig.",
        "attitude_ref_valid": "Runtime-only flag. True after attitude-capture-ref succeeds; not stored in flash.",
        "rate_kp_roll": "Roll rate-loop proportional gain. Start with small steps such as +0.0005.",
        "rate_ki_roll": "Roll rate-loop integral gain. Keep zero until P is stable on the bench.",
        "rate_kd_roll": "Roll rate-loop derivative gain. Add only after P direction and sign are verified.",
        "rate_kp_pitch": "Pitch rate-loop proportional gain. Start with small steps such as +0.0005.",
        "rate_ki_pitch": "Pitch rate-loop integral gain. Keep zero until P is stable on the bench.",
        "rate_kd_pitch": "Pitch rate-loop derivative gain. Add only after P direction and sign are verified.",
        "rate_kp_yaw": "Yaw rate-loop proportional gain. Keep conservative because yaw authority is weaker.",
        "rate_ki_yaw": "Yaw rate-loop integral gain. Keep zero until yaw sign and damping are verified.",
        "rate_kd_yaw": "Yaw rate-loop derivative gain. Usually smaller than roll/pitch for bench bring-up.",
        "rate_integral_limit": "Per-axis rate-loop integral clamp in deg/s·s equivalent state units.",
        "rate_output_limit": "Per-axis PID output clamp sent into the mixer around bring-up_test_base_duty.",
        "udp_manual_max_pwm": "Max UDP manual base duty clamp, normalized 0..1.",
        "udp_takeoff_pwm": "UDP takeoff base-duty ramp target, normalized 0..1.",
        "udp_land_min_pwm": "UDP landing and watchdog base-duty floor before auto-disarm.",
        "udp_manual_timeout_ms": "UDP manual setpoint watchdog timeout in milliseconds.",
        "udp_manual_axis_limit": "UDP manual normalized command clamp. Yaw maps to rate setpoints; roll/pitch are held by attitude control.",
    }

    PARAM_HELP_ZH = {
        "telemetry_usb_hz": "USB CDC 遥测目标频率，单位 Hz；最终范围以设备端校验为准。",
        "telemetry_udp_hz": "UDP 遥测目标频率，单位 Hz；最终范围以设备端校验为准。",
        "wifi_ap_enable": "开机启用 ESP32 SoftAP；Wi-Fi 失败时 USB CDC 仍可使用。",
        "wifi_ap_channel": "ESP32 SoftAP 信道，合法范围 1..13，默认 6。",
        "wifi_udp_port": "二进制 CLI/GUI UDP 协议监听端口，默认 2391。",
        "imu_mode": "0 = RAW，1 = DIRECT；当前台架验证主要围绕 DIRECT 模式。",
        "imu_return_rate_code": "ATK-MS901M 回传频率代码；0x00 为 250Hz，0x01 为 200Hz。",
        "motor_idle_duty": "有刷电机解锁后的怠速下限，归一化 0..1。",
        "motor_max_duty": "有刷电机输出上限，归一化 0..1。",
        "bringup_test_base_duty": "轴向/速率台架测试使用的基础占空比；约束机体上应保持较低。",
        "attitude_kp_roll": "仅限台架的姿态外环横滚 P 增益；不要当作自由飞行角度增益使用。",
        "attitude_kp_pitch": "仅限台架的姿态外环俯仰 P 增益；不要当作自由飞行角度增益使用。",
        "attitude_rate_limit_roll": "姿态台架外环输出的横滚速率设定限幅，单位 deg/s。",
        "attitude_rate_limit_pitch": "姿态台架外环输出的俯仰速率设定限幅，单位 deg/s。",
        "attitude_error_deadband_deg": "捕获姿态参考附近的小角度死区，单位度。",
        "attitude_trip_deg": "约束台架上横滚/俯仰姿态误差的保护触发阈值。",
        "attitude_test_base_duty": "姿态台架模式使用的基础占空比；约束机体上应保守设置。",
        "attitude_ref_valid": "运行时标志；姿态参考捕获成功后为 True，不保存到 flash。",
        "rate_kp_roll": "横滚速率环比例增益；建议用 +0.0005 这类小步进开始。",
        "rate_ki_roll": "横滚速率环积分增益；台架上 P 稳定前保持为 0。",
        "rate_kd_roll": "横滚速率环微分增益；确认 P 方向和符号后再加入。",
        "rate_kp_pitch": "俯仰速率环比例增益；建议用 +0.0005 这类小步进开始。",
        "rate_ki_pitch": "俯仰速率环积分增益；台架上 P 稳定前保持为 0。",
        "rate_kd_pitch": "俯仰速率环微分增益；确认 P 方向和符号后再加入。",
        "rate_kp_yaw": "偏航速率环比例增益；偏航控制余量较弱，应保守设置。",
        "rate_ki_yaw": "偏航速率环积分增益；确认偏航符号和阻尼前保持为 0。",
        "rate_kd_yaw": "偏航速率环微分增益；台架调试时通常小于横滚/俯仰。",
        "rate_integral_limit": "每轴速率环积分状态限幅，等效单位为 deg/s*s。",
        "rate_output_limit": "每轴 PID 输出限幅，输出进入 mixer 并叠加在基础占空比附近。",
        "udp_manual_max_pwm": "UDP 手动模式基础占空比最大限幅，归一化 0..1。",
        "udp_takeoff_pwm": "UDP 起飞基础占空比斜坡目标，归一化 0..1。",
        "udp_land_min_pwm": "UDP 降落和看门狗降油门时的安全基础占空比下限。",
        "udp_manual_timeout_ms": "UDP 手动 setpoint 看门狗超时时间，单位毫秒。",
        "udp_manual_axis_limit": "UDP 手动横滚/俯仰/偏航归一化指令限幅，之后会映射为速率设定。",
    }

    CHART_GROUPS = {
        "gyro": ChartSpec("chart.gyro", [("gyro_x", "field.gyro_x"), ("gyro_y", "field.gyro_y"), ("gyro_z", "field.gyro_z")], "chart.y_deg_s"),
        "attitude": ChartSpec("chart.attitude", [("roll_deg", "field.roll_deg"), ("pitch_deg", "field.pitch_deg"), ("yaw_deg", "field.yaw_deg")], "chart.y_deg"),
        "motors": ChartSpec("chart.motors", [("motor1", "field.motor1"), ("motor2", "field.motor2"), ("motor3", "field.motor3"), ("motor4", "field.motor4")], "chart.y_duty"),
        "rate_roll": ChartSpec(
            "chart.rate_roll",
            [
                ("gyro_y", "field.gyro_y"),
                ("rate_setpoint_roll", "field.rate_setpoint_roll"),
                ("pid_out_roll", "field.pid_out_roll"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_rate_mix",
        ),
        "rate_pitch": ChartSpec(
            "chart.rate_pitch",
            [
                ("gyro_x", "field.gyro_x"),
                ("rate_setpoint_pitch", "field.rate_setpoint_pitch"),
                ("pid_out_pitch", "field.pid_out_pitch"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_rate_mix",
        ),
        "rate_yaw": ChartSpec(
            "chart.rate_yaw",
            [
                ("gyro_z", "field.gyro_z"),
                ("rate_setpoint_yaw", "field.rate_setpoint_yaw"),
                ("pid_out_yaw", "field.pid_out_yaw"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_rate_mix",
        ),
        "hang_roll": ChartSpec(
            "chart.hang_roll",
            [
                ("attitude_err_roll_deg", "field.attitude_err_roll_deg"),
                ("attitude_rate_sp_roll", "field.attitude_rate_sp_roll"),
                ("pid_out_roll", "field.pid_out_roll"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_attitude_mix",
        ),
        "hang_pitch": ChartSpec(
            "chart.hang_pitch",
            [
                ("attitude_err_pitch_deg", "field.attitude_err_pitch_deg"),
                ("attitude_rate_sp_pitch", "field.attitude_rate_sp_pitch"),
                ("pid_out_pitch", "field.pid_out_pitch"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_attitude_mix",
        ),
        "ground_roll": ChartSpec(
            "Ground Roll",
            [
                ("attitude_err_roll_deg", "attitude_err_roll_deg"),
                ("attitude_rate_sp_roll", "attitude_rate_sp_roll"),
                ("rate_meas_roll_raw", "rate_meas_roll_raw"),
                ("rate_meas_roll_filtered", "rate_meas_roll_filtered"),
                ("pid_out_roll", "field.pid_out_roll"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_attitude_mix",
        ),
        "ground_pitch": ChartSpec(
            "Ground Pitch",
            [
                ("attitude_err_pitch_deg", "attitude_err_pitch_deg"),
                ("attitude_rate_sp_pitch", "attitude_rate_sp_pitch"),
                ("rate_meas_pitch_raw", "rate_meas_pitch_raw"),
                ("rate_meas_pitch_filtered", "rate_meas_pitch_filtered"),
                ("pid_out_pitch", "field.pid_out_pitch"),
                ("motor1", "field.motor1"),
                ("motor2", "field.motor2"),
                ("motor3", "field.motor3"),
                ("motor4", "field.motor4"),
            ],
            "chart.y_attitude_mix",
        ),
        "rate_roll_filtered": ChartSpec(
            "Rate Roll Filtered",
            [
                ("rate_meas_roll_raw", "rate_meas_roll_raw"),
                ("rate_meas_roll_filtered", "rate_meas_roll_filtered"),
                ("rate_setpoint_roll", "field.rate_setpoint_roll"),
                ("pid_out_roll", "field.pid_out_roll"),
            ],
            "chart.y_rate_mix",
        ),
        "rate_pitch_filtered": ChartSpec(
            "Rate Pitch Filtered",
            [
                ("rate_meas_pitch_raw", "rate_meas_pitch_raw"),
                ("rate_meas_pitch_filtered", "rate_meas_pitch_filtered"),
                ("rate_setpoint_pitch", "field.rate_setpoint_pitch"),
                ("pid_out_pitch", "field.pid_out_pitch"),
            ],
            "chart.y_rate_mix",
        ),
        "rate_yaw_filtered": ChartSpec(
            "Rate Yaw Filtered",
            [
                ("rate_meas_yaw_raw", "rate_meas_yaw_raw"),
                ("rate_meas_yaw_filtered", "rate_meas_yaw_filtered"),
                ("rate_setpoint_yaw", "field.rate_setpoint_yaw"),
                ("pid_out_yaw", "field.pid_out_yaw"),
            ],
            "chart.y_rate_mix",
        ),
        "battery": ChartSpec("chart.battery", [("battery_voltage", "field.battery_voltage")], "chart.y_volt"),
        "baro": ChartSpec(
            "chart.baro",
            [
                ("baro_altitude_m", "field.baro_altitude_m"),
                ("baro_pressure_pa", "field.baro_pressure_pa"),
                ("baro_temperature_c", "field.baro_temperature_c"),
            ],
            "chart.y_baro",
        ),
    }

    CHART_COLORS = ["#D32F2F", "#1976D2", "#388E3C", "#7B1FA2"]

    def __init__(
        self,
        session: DeviceSession | None = None,
        bridge_cls: type[QtSessionBridge] = QtSessionBridge,
        serial_port_provider: Callable[[], Iterable] | None = None,
        settings: QSettings | None = None,
    ) -> None:
        """初始化主窗口。

        Args:
            session: 可选的设备会话实例。未提供时会创建默认会话。
            bridge_cls: 会话桥接类型，测试或替换桥接策略时可注入。
            serial_port_provider: 串口枚举函数，默认使用 `serial.tools.list_ports.comports`。
            settings: 可选的 `QSettings` 对象，用于持久化窗口与表单状态。
        """

        super().__init__()
        self.setWindowTitle("ESP-DRONE Bench Debugger")
        self.resize(1660, 980)
        self._apply_workbench_style()

        self._session = session or DeviceSession()
        self._bridge = bridge_cls(self._session)
        self._serial_port_provider = serial_port_provider or list_ports.comports
        self._settings = settings or QSettings(APP_ORG, APP_NAME)
        self._params: list[ParamValue] = []
        self._selected_param: ParamValue | None = None
        self._last_telemetry: TelemetrySample | None = None
        self._history = TelemetryHistory()
        self._charts_running = True
        self._stream_enabled = False
        self._last_result = "-"
        self._closing = False
        self._connecting = False
        self._connecting_detail = ""
        self._connection_target_detail = ""
        self._last_connect_error_message: str | None = None
        self._language = "zh"
        self._current_chart_group = "gyro"
        self._chart_curves: dict[str, object] = {}
        self._chart_channel_checks: dict[str, QCheckBox] = {}
        self._udp_manual_enabled = False
        self._udp_manual_last_send_monotonic: float | None = None
        self._udp_manual_send_inflight = False

        self._build_ui()
        self._wire_signals()
        self._load_settings()
        self._apply_language()
        self._rebuild_chart_channels()
        self._refresh_serial_ports()
        self._refresh_enabled_state()

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(100)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start()

        self._udp_control_timer = QTimer(self)
        self._udp_control_timer.setInterval(100)
        self._udp_control_timer.timeout.connect(self._send_udp_manual_timer_setpoint)

        self._connect_watchdog_timer = QTimer(self)
        self._connect_watchdog_timer.setSingleShot(True)
        self._connect_watchdog_timer.setInterval(12000)
        self._connect_watchdog_timer.timeout.connect(self._handle_connect_watchdog_timeout)

    def _t(self, key: str, **kwargs) -> str:
        table = TRANSLATIONS.get(self._language, TRANSLATIONS["zh"])
        extra_table = EXTRA_TRANSLATIONS.get(self._language, EXTRA_TRANSLATIONS["zh"])
        text = extra_table.get(key, table.get(key, key))
        return text.format(**kwargs)

    def _combo_data(self, combo: QComboBox) -> object:
        value = combo.currentData()
        return combo.currentText() if value is None else value

    def _apply_workbench_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background-color: #111827;
                color: #e5eefb;
                font-size: 13px;
                font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
            }
            QMainWindow {
                background-color: #0b1220;
            }
            QGroupBox {
                border: 1px solid #253447;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 12px;
                font-weight: 600;
                background-color: #101826;
            }
            QHeaderView::section {
                background-color: #162131;
                color: #dbeafe;
                border: 0;
                padding: 6px;
            }
            QTableWidget {
                gridline-color: #223145;
                background-color: #0f1722;
                alternate-background-color: #142030;
                selection-background-color: #1d4ed8;
                selection-color: white;
            }
            QPlainTextEdit {
                background-color: #0f1722;
                border: 1px solid #253447;
                border-radius: 8px;
                selection-background-color: #1d4ed8;
                selection-color: white;
            }
            QPushButton {
                background-color: #1d4ed8;
                border: 0;
                border-radius: 6px;
                padding: 6px 12px;
                color: white;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #2563eb;
            }
            QPushButton:disabled {
                background-color: #415266;
                color: #a8b4c4;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #0f1722;
                border: 1px solid #2a3a51;
                border-radius: 6px;
                padding: 4px 6px;
                min-height: 26px;
            }
            QTabWidget::pane {
                border: 1px solid #253447;
                border-radius: 8px;
                top: -1px;
                background-color: #101826;
            }
            QTabBar::tab {
                background-color: #162131;
                border: 1px solid #253447;
                padding: 8px 14px;
                min-width: 60px;
                min-height: 24px;
                color: #dbeafe;
            }
            QTabBar::tab:selected {
                background-color: #1d4ed8;
                color: white;
            }
            QScrollArea {
                border: 0;
                background: transparent;
            }
            QScrollBar:vertical {
                background: #101826;
                width: 12px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #2a3a51;
                border-radius: 6px;
                min-height: 24px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
            """
        )

    def _build_ui(self) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        top_bar = QHBoxLayout()
        self.window_title_label = QLabel()
        self.window_title_label.setStyleSheet("font-size:18px;font-weight:700;color:#f8fafc;")
        top_bar.addWidget(self.window_title_label)
        top_bar.addStretch(1)
        self.language_label = QLabel()
        self.language_combo = QComboBox()
        self.language_combo.addItems(["中文", "English"])
        top_bar.addWidget(self.language_label)
        top_bar.addWidget(self.language_combo)
        layout.addLayout(top_bar)

        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.setOpaqueResize(False)
        self.main_splitter.setHandleWidth(8)

        self.left_panel = self._build_left_panel()
        self.center_panel = self._build_center_workbench()
        self.bottom_panel = self._build_bottom_panel()
        self.right_panel = self._build_right_workbench()

        self.main_splitter.addWidget(self.left_panel)
        self.main_splitter.addWidget(self.center_panel)
        self.main_splitter.addWidget(self.right_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 4)
        self.main_splitter.setStretchFactor(2, 1)
        self._apply_default_main_splitter_sizes()

        layout.addWidget(self.main_splitter, 1)
        self.setCentralWidget(root)

    def _build_left_panel(self) -> QWidget:
        content = QWidget()
        content.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.connection_group = self._build_connection_group()
        self.safety_group = self._build_safety_group()
        self.debug_group = self._build_debug_group()
        self.connection_section = CollapsibleSection(expanded=True)
        self.connection_section.set_content(self.connection_group)
        self.safety_section = CollapsibleSection(expanded=True)
        self.safety_section.set_content(self.safety_group)
        layout.addWidget(self.connection_section)
        layout.addWidget(self.safety_section)
        layout.addWidget(self.debug_group)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setMinimumWidth(360)
        scroll.setMaximumWidth(520)
        scroll.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        scroll.setWidget(content)
        return scroll

    def _build_center_workbench(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(780)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.center_splitter = QSplitter(Qt.Vertical)
        self.center_splitter.setChildrenCollapsible(False)
        self.center_splitter.setOpaqueResize(False)
        self.chart_group = self._build_charts_group()
        self.realtime_group = self._build_telemetry_group()
        self.center_splitter.addWidget(self.chart_group)
        self.center_splitter.addWidget(self.realtime_group)
        self.center_splitter.setStretchFactor(0, 8)
        self.center_splitter.setStretchFactor(1, 2)
        self.center_splitter.setSizes([760, 170])
        layout.addWidget(self.center_splitter)
        return panel

    def _build_right_workbench(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        panel.setMinimumWidth(360)
        panel.setMaximumWidth(420)
        self.right_splitter = QSplitter(Qt.Vertical)
        self.right_splitter.setChildrenCollapsible(False)
        self.right_splitter.setOpaqueResize(False)
        self.status_group = self._build_status_group()
        self.params_group = self._build_params_group()
        self.tools_panel = self._build_tools_panel()
        self.right_tabs = QTabWidget()
        self.right_tabs.setDocumentMode(True)
        self.right_tabs.addTab(self.params_group, "")
        self.right_tabs.addTab(self.bottom_panel, "")
        self.right_tabs.addTab(self.tools_panel, "")
        self.right_tabs.setCurrentIndex(0)
        self.right_splitter.addWidget(self.status_group)
        self.right_splitter.addWidget(self.right_tabs)
        self.right_splitter.setStretchFactor(0, 0)
        self.right_splitter.setStretchFactor(1, 1)
        self.right_splitter.setSizes([152, 698])
        layout.addWidget(self.right_splitter)
        return panel

    def _build_tools_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.session_info_group = QGroupBox()
        session_layout = QGridLayout(self.session_info_group)
        session_layout.setColumnStretch(1, 1)
        self.tools_session_title_label = QLabel()
        self.tools_last_conn_error_title_label = QLabel()
        self.tools_connection_info_label = QLabel(self.connection_info_label.text())
        self.tools_connection_info_label.setWordWrap(True)
        self.tools_connection_error_detail = QLabel(self.connection_error_detail.text())
        self.tools_connection_error_detail.setWordWrap(True)
        self.tools_connection_error_detail.setTextInteractionFlags(Qt.TextSelectableByMouse)
        session_layout.addWidget(self.tools_session_title_label, 0, 0)
        session_layout.addWidget(self.tools_connection_info_label, 0, 1)
        session_layout.addWidget(self.tools_last_conn_error_title_label, 1, 0)
        session_layout.addWidget(self.tools_connection_error_detail, 1, 1)

        self.calib_group = self._build_calibration_panel()
        self.csv_group = self._build_log_export_panel()

        layout.addWidget(self.session_info_group)
        layout.addWidget(self.calib_group)
        layout.addWidget(self.csv_group)
        layout.addStretch(1)
        return panel

    def _apply_default_main_splitter_sizes(self) -> None:
        self.main_splitter.setSizes([480, 980, 380])

    def _normalize_main_splitter_sizes(self) -> None:
        sizes = self.main_splitter.sizes()
        if len(sizes) != 3:
            self._apply_default_main_splitter_sizes()
            return
        min_left = self.left_panel.minimumWidth()
        min_center = self.center_panel.minimumWidth()
        min_right = self.right_panel.minimumWidth()
        if (
            sizes[0] < min_left
            or sizes[1] < min_center
            or sizes[2] < min_right
            or sizes[1] <= max(sizes[0], sizes[2])
        ):
            self._apply_default_main_splitter_sizes()

    def _build_bottom_panel(self) -> QWidget:
        group = QGroupBox()
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        bar = QHBoxLayout()
        self.clear_log_button = QPushButton()
        self.copy_log_button = QPushButton()
        self.save_log_button = QPushButton()
        self.last_result_title_label = QLabel()
        self.last_result_label = QLabel("-")
        self.last_result_label.setWordWrap(True)
        bar.addWidget(self.clear_log_button)
        bar.addWidget(self.copy_log_button)
        bar.addWidget(self.save_log_button)
        bar.addWidget(self.last_result_title_label)
        bar.addWidget(self.last_result_label, 1)
        layout.addLayout(bar)

        self.event_log_edit = QPlainTextEdit()
        self.event_log_edit.setReadOnly(True)
        self.event_log_edit.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.event_log_edit.document().setMaximumBlockCount(300)
        layout.addWidget(self.event_log_edit, 1)

        summary = QGridLayout()
        self.last_log_path_label = QLabel("-")
        self.last_log_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.last_error_label = QLabel("-")
        self.last_error_label.setWordWrap(True)
        self.last_error_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.last_log_title_label = QLabel()
        self.last_error_title_label = QLabel()
        summary.addWidget(self.last_log_title_label, 0, 0)
        summary.addWidget(self.last_log_path_label, 0, 1)
        summary.addWidget(self.last_error_title_label, 1, 0)
        summary.addWidget(self.last_error_label, 1, 1)
        layout.addLayout(summary)
        return group

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)

        self.link_type_combo = QComboBox()
        self.link_type_combo.addItems(["serial", "udp"])

        self.link_type_label = QLabel()
        self.serial_port_label = QLabel()
        self.baud_label = QLabel()
        self.udp_host_label = QLabel()
        self.udp_port_label = QLabel()
        self.session_title_label = QLabel()
        self.last_conn_error_title_label = QLabel()

        self.serial_port_combo = QComboBox()
        self.serial_port_combo.setEditable(True)
        self.serial_port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.refresh_ports_button = QPushButton()

        self.baudrate_spin = QSpinBox()
        self.baudrate_spin.setRange(9600, 2000000)
        self.baudrate_spin.setValue(115200)

        self.transport_stack = QStackedWidget()

        serial_page = QWidget()
        serial_form = QGridLayout(serial_page)
        serial_form.setContentsMargins(0, 0, 0, 0)
        self.serial_port_row = QWidget()
        serial_port_row_layout = QHBoxLayout(self.serial_port_row)
        serial_port_row_layout.setContentsMargins(0, 0, 0, 0)
        serial_port_row_layout.setSpacing(6)
        serial_port_row_layout.addWidget(self.serial_port_combo, 1)
        serial_port_row_layout.addWidget(self.refresh_ports_button)
        self.refresh_ports_button.setMaximumWidth(72)
        serial_form.setColumnStretch(1, 1)
        serial_form.addWidget(self.serial_port_label, 0, 0)
        serial_form.addWidget(self.serial_port_row, 0, 1)
        serial_form.addWidget(self.baud_label, 1, 0)
        serial_form.addWidget(self.baudrate_spin, 1, 1)
        self.transport_stack.addWidget(serial_page)

        udp_page = QWidget()
        udp_form = QGridLayout(udp_page)
        udp_form.setContentsMargins(0, 0, 0, 0)
        self.udp_host_edit = QLineEdit("192.168.4.1")
        self.udp_port_spin = QSpinBox()
        self.udp_port_spin.setRange(1, 65535)
        self.udp_port_spin.setValue(2391)
        self.udp_transport_hint_label = QLabel()
        self.udp_transport_hint_label.setWordWrap(True)
        self.udp_transport_hint_label.setStyleSheet("color:#bfdbfe;")
        self.udp_ap_info_label = QLabel()
        self.udp_ap_info_label.setWordWrap(True)
        self.udp_ap_info_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        udp_form.setColumnStretch(1, 1)
        udp_form.addWidget(self.udp_host_label, 0, 0)
        udp_form.addWidget(self.udp_host_edit, 0, 1)
        udp_form.addWidget(self.udp_port_label, 1, 0)
        udp_form.addWidget(self.udp_port_spin, 1, 1)
        udp_form.addWidget(self.udp_transport_hint_label, 2, 0, 1, 2)
        udp_form.addWidget(self.udp_ap_info_label, 3, 0, 1, 2)
        self.transport_stack.addWidget(udp_page)

        self.connect_button = QPushButton()
        self.disconnect_button = QPushButton()
        self.connect_button.setMaximumWidth(96)
        self.disconnect_button.setMaximumWidth(96)

        self.connection_status_chip = QLabel()
        self.connection_info_label = QLabel()
        self.connection_info_label.setWordWrap(True)
        self.connection_error_detail = QLabel()
        self.connection_error_detail.setWordWrap(True)
        self.connection_error_detail.setTextInteractionFlags(Qt.TextSelectableByMouse)

        layout.addWidget(self.link_type_label, 0, 0)
        layout.addWidget(self.link_type_combo, 0, 1)
        layout.addWidget(self.connection_status_chip, 0, 2)
        layout.addWidget(self.transport_stack, 1, 0, 1, 3)
        layout.addWidget(self.connect_button, 2, 1)
        layout.addWidget(self.disconnect_button, 2, 2)
        layout.addWidget(self.session_title_label, 3, 0)
        layout.addWidget(self.connection_info_label, 3, 1, 1, 2)
        layout.addWidget(self.last_conn_error_title_label, 4, 0)
        layout.addWidget(self.connection_error_detail, 4, 1, 1, 2)
        return group

    def _build_safety_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        self.arm_button = QPushButton()
        self.disarm_button = QPushButton()
        self.kill_button = QPushButton()
        self.kill_button.setStyleSheet("background:#C62828;color:white;font-weight:700;")
        self.reboot_button = QPushButton()
        self.arm_button.setMaximumWidth(112)
        self.disarm_button.setMaximumWidth(112)
        self.reboot_button.setMaximumWidth(92)
        self.kill_button.setMinimumWidth(110)

        layout.addWidget(self.arm_button, 0, 0)
        layout.addWidget(self.disarm_button, 0, 1)
        layout.addWidget(self.kill_button, 1, 0)
        layout.addWidget(self.reboot_button, 1, 1)
        return group

    def _build_telemetry_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QVBoxLayout(group)
        layout.setSpacing(4)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        self.stream_on_button = QPushButton()
        self.stream_off_button = QPushButton()
        self.stream_rate_spin = QSpinBox()
        self.stream_rate_spin.setRange(1, 200)
        self.stream_rate_spin.setValue(200)
        self.apply_stream_rate_button = QPushButton()
        self.copy_telemetry_button = QPushButton()
        self.target_hz_label = QLabel()
        bar.addWidget(self.stream_on_button)
        bar.addWidget(self.stream_off_button)
        bar.addWidget(self.target_hz_label)
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
        self.telemetry_table.setMinimumHeight(150)
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            field_item = QTableWidgetItem(name)
            value_item = QTableWidgetItem("-")
            self.telemetry_table.setItem(row, 0, field_item)
            self.telemetry_table.setItem(row, 1, value_item)
        layout.addWidget(self.telemetry_table)
        return group

    def _build_charts_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QVBoxLayout(group)
        layout.setSpacing(4)

        control_bar = QHBoxLayout()
        control_bar.setSpacing(6)
        self.chart_group_label = QLabel()
        self.chart_group_combo = QComboBox()
        self.chart_group_combo.setMaximumWidth(140)
        self.chart_toggle_button = QPushButton()
        self.clear_charts_button = QPushButton()
        self.auto_scale_button = QPushButton()
        self.reset_view_button = QPushButton()
        self.chart_window_label = QLabel()
        self.chart_window_combo = QComboBox()
        self.chart_window_combo.setMaximumWidth(92)
        self.chart_window_combo.addItem("5 s", 5.0)
        self.chart_window_combo.addItem("10 s", 10.0)
        self.chart_window_combo.addItem("30 s", 30.0)
        self.chart_window_combo.setCurrentIndex(1)
        self.chart_group_combo.addItems(["gyro", "attitude", "motors", "battery"])
        for button in (
            self.chart_toggle_button,
            self.clear_charts_button,
            self.auto_scale_button,
            self.reset_view_button,
        ):
            button.setMaximumWidth(96)
        control_bar.addWidget(self.chart_group_label)
        control_bar.addWidget(self.chart_group_combo)
        control_bar.addWidget(self.chart_toggle_button)
        control_bar.addWidget(self.clear_charts_button)
        control_bar.addWidget(self.auto_scale_button)
        control_bar.addWidget(self.reset_view_button)
        control_bar.addWidget(self.chart_window_label)
        control_bar.addWidget(self.chart_window_combo)
        control_bar.addStretch(1)
        layout.addLayout(control_bar)

        self.chart_channel_bar = QHBoxLayout()
        self.chart_channel_bar.addStretch(1)
        layout.addLayout(self.chart_channel_bar)

        self.main_plot = pg.PlotWidget()
        self.main_plot.setMinimumHeight(520)
        self.main_plot.setBackground("#0f1722")
        self.main_plot.showGrid(x=True, y=True, alpha=0.25)
        self.main_plot.addLegend(offset=(10, 10))
        self.main_plot.setMenuEnabled(False)
        self.main_plot.setClipToView(True)
        self.main_plot.getAxis("left").setTextPen("#dbeafe")
        self.main_plot.getAxis("bottom").setTextPen("#dbeafe")
        self.main_plot.getAxis("left").setPen(pg.mkPen("#4b5d75"))
        self.main_plot.getAxis("bottom").setPen(pg.mkPen("#4b5d75"))
        layout.addWidget(self.main_plot, 1)
        return group

    def _build_params_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QVBoxLayout(group)
        layout.setSpacing(4)

        top_bar = QHBoxLayout()
        self.param_search_edit = QLineEdit()
        self.refresh_params_button = QPushButton()
        self.save_params_button = QPushButton()
        self.reset_params_button = QPushButton()
        self.export_params_button = QPushButton()
        self.import_params_button = QPushButton()
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
        detail_layout.setSpacing(4)
        form = QFormLayout()
        form.setVerticalSpacing(4)
        self.param_name_title_label = QLabel()
        self.param_type_title_label = QLabel()
        self.param_current_title_label = QLabel()
        self.param_name_label = QLabel("-")
        self.param_type_label = QLabel("-")
        self.param_current_value_label = QLabel("-")
        self.param_new_value_edit = QLineEdit()
        self.param_hint_label = QLabel()
        self.param_hint_label.setWordWrap(True)
        self.param_hint_label.setMaximumHeight(38)
        self.param_help_text = QPlainTextEdit()
        self.param_help_text.setReadOnly(True)
        self.param_help_text.setMaximumHeight(54)
        form.addRow(self.param_name_title_label, self.param_name_label)
        form.addRow(self.param_type_title_label, self.param_type_label)
        form.addRow(self.param_current_title_label, self.param_current_value_label)
        self.param_new_value_title_label = QLabel()
        form.addRow(self.param_new_value_title_label, self.param_new_value_edit)
        detail_layout.addLayout(form)
        detail_layout.addWidget(self.param_hint_label)
        self.param_desc_title_label = QLabel()
        detail_layout.addWidget(self.param_desc_title_label)
        detail_layout.addWidget(self.param_help_text)
        self.set_param_button = QPushButton()
        detail_layout.addWidget(self.set_param_button)
        self.params_splitter.addWidget(detail)
        self.params_splitter.setStretchFactor(0, 7)
        self.params_splitter.setStretchFactor(1, 1)
        self.params_splitter.setSizes([620, 150])
        layout.addWidget(self.params_splitter, 1)
        return group

    def _build_debug_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        self.debug_warning_label = QLabel()
        warning = self.debug_warning_label
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#E65100;font-weight:600;")
        layout.addWidget(warning)

        self.debug_action_tabs = QTabWidget()
        self.debug_action_tabs.setDocumentMode(True)
        self.debug_action_tabs.setUsesScrollButtons(True)

        motor_box = QWidget()
        motor_layout = QGridLayout(motor_box)
        motor_layout.setHorizontalSpacing(6)
        motor_layout.setVerticalSpacing(6)
        motor_layout.setColumnStretch(1, 1)
        motor_layout.setColumnStretch(3, 1)
        motor_layout.setRowStretch(2, 1)
        self.motor_combo = QComboBox()
        self.motor_combo.addItems(["m1", "m2", "m3", "m4"])
        self.motor_duty_spin = QDoubleSpinBox()
        self.motor_duty_spin.setRange(0.0, 1.0)
        self.motor_duty_spin.setDecimals(3)
        self.motor_duty_spin.setSingleStep(0.01)
        self.motor_duty_spin.setValue(0.05)
        self.motor_group = motor_box
        self.motor_start_button = QPushButton()
        self.motor_stop_button = QPushButton()
        self.motor_label = QLabel()
        self.duty_label = QLabel()
        self.motor_start_button.setMaximumWidth(68)
        self.motor_stop_button.setMaximumWidth(68)
        motor_layout.addWidget(self.motor_label, 0, 0)
        motor_layout.addWidget(self.motor_combo, 0, 1)
        motor_layout.addWidget(self.duty_label, 0, 2)
        motor_layout.addWidget(self.motor_duty_spin, 0, 3)
        motor_layout.addWidget(self.motor_start_button, 1, 1)
        motor_layout.addWidget(self.motor_stop_button, 1, 2)
        self.motor_group = motor_box

        rate_box = QWidget()
        rate_layout = QGridLayout(rate_box)
        rate_layout.setHorizontalSpacing(6)
        rate_layout.setVerticalSpacing(6)
        rate_layout.setColumnStretch(1, 1)
        rate_layout.setColumnStretch(3, 1)
        rate_layout.setRowStretch(2, 1)
        self.rate_axis_combo = QComboBox()
        self.rate_axis_combo.addItems(["roll", "pitch", "yaw"])
        self.rate_value_spin = QDoubleSpinBox()
        self.rate_value_spin.setRange(-720.0, 720.0)
        self.rate_value_spin.setDecimals(1)
        self.rate_value_spin.setSingleStep(5.0)
        self.rate_value_spin.setValue(20.0)
        self.rate_group = rate_box
        self.rate_start_button = QPushButton()
        self.rate_stop_button = QPushButton()
        self.rate_axis_label = QLabel()
        self.rate_dps_label = QLabel()
        self.rate_start_button.setMaximumWidth(68)
        self.rate_stop_button.setMaximumWidth(68)
        rate_layout.addWidget(self.rate_axis_label, 0, 0)
        rate_layout.addWidget(self.rate_axis_combo, 0, 1)
        rate_layout.addWidget(self.rate_dps_label, 0, 2)
        rate_layout.addWidget(self.rate_value_spin, 0, 3)
        rate_layout.addWidget(self.rate_start_button, 1, 1)
        rate_layout.addWidget(self.rate_stop_button, 1, 2)
        self.rate_group = rate_box

        hang_box = QWidget()
        hang_layout = QGridLayout(hang_box)
        hang_layout.setHorizontalSpacing(6)
        hang_layout.setVerticalSpacing(6)
        hang_layout.setColumnStretch(1, 1)
        hang_layout.setRowStretch(10, 1)
        self.hang_note_label = QLabel()
        self.hang_note_label.setWordWrap(True)
        self.hang_capture_button = QPushButton()
        self.hang_start_button = QPushButton()
        self.hang_stop_button = QPushButton()
        self.hang_base_duty_label = QLabel()
        self.hang_base_duty_spin = QDoubleSpinBox()
        self.hang_base_duty_spin.setRange(0.0, 1.0)
        self.hang_base_duty_spin.setDecimals(3)
        self.hang_base_duty_spin.setSingleStep(0.01)
        self.hang_base_duty_spin.setValue(0.05)
        self.hang_kp_roll_label = QLabel()
        self.hang_kp_roll_spin = QDoubleSpinBox()
        self.hang_kp_roll_spin.setRange(0.0, 10.0)
        self.hang_kp_roll_spin.setDecimals(2)
        self.hang_kp_roll_spin.setSingleStep(0.1)
        self.hang_kp_roll_spin.setValue(2.0)
        self.hang_kp_pitch_label = QLabel()
        self.hang_kp_pitch_spin = QDoubleSpinBox()
        self.hang_kp_pitch_spin.setRange(0.0, 10.0)
        self.hang_kp_pitch_spin.setDecimals(2)
        self.hang_kp_pitch_spin.setSingleStep(0.1)
        self.hang_kp_pitch_spin.setValue(2.0)
        self.hang_rate_limit_roll_label = QLabel()
        self.hang_rate_limit_roll_spin = QDoubleSpinBox()
        self.hang_rate_limit_roll_spin.setRange(0.0, 180.0)
        self.hang_rate_limit_roll_spin.setDecimals(1)
        self.hang_rate_limit_roll_spin.setSingleStep(1.0)
        self.hang_rate_limit_roll_spin.setValue(25.0)
        self.hang_rate_limit_pitch_label = QLabel()
        self.hang_rate_limit_pitch_spin = QDoubleSpinBox()
        self.hang_rate_limit_pitch_spin.setRange(0.0, 180.0)
        self.hang_rate_limit_pitch_spin.setDecimals(1)
        self.hang_rate_limit_pitch_spin.setSingleStep(1.0)
        self.hang_rate_limit_pitch_spin.setValue(25.0)
        self.hang_deadband_label = QLabel()
        self.hang_deadband_spin = QDoubleSpinBox()
        self.hang_deadband_spin.setRange(0.0, 10.0)
        self.hang_deadband_spin.setDecimals(1)
        self.hang_deadband_spin.setSingleStep(0.1)
        self.hang_deadband_spin.setValue(1.0)
        self.hang_trip_label = QLabel()
        self.hang_trip_spin = QDoubleSpinBox()
        self.hang_trip_spin.setRange(1.0, 90.0)
        self.hang_trip_spin.setDecimals(1)
        self.hang_trip_spin.setSingleStep(1.0)
        self.hang_trip_spin.setValue(30.0)
        self.hang_group = hang_box
        hang_layout.addWidget(self.hang_note_label, 0, 0, 1, 2)
        hang_layout.addWidget(self.hang_capture_button, 1, 0)
        hang_layout.addWidget(self.hang_start_button, 1, 1)
        hang_layout.addWidget(self.hang_stop_button, 2, 0, 1, 2)
        hang_layout.addWidget(self.hang_base_duty_label, 3, 0)
        hang_layout.addWidget(self.hang_base_duty_spin, 3, 1)
        hang_layout.addWidget(self.hang_kp_roll_label, 4, 0)
        hang_layout.addWidget(self.hang_kp_roll_spin, 4, 1)
        hang_layout.addWidget(self.hang_kp_pitch_label, 5, 0)
        hang_layout.addWidget(self.hang_kp_pitch_spin, 5, 1)
        hang_layout.addWidget(self.hang_rate_limit_roll_label, 6, 0)
        hang_layout.addWidget(self.hang_rate_limit_roll_spin, 6, 1)
        hang_layout.addWidget(self.hang_rate_limit_pitch_label, 7, 0)
        hang_layout.addWidget(self.hang_rate_limit_pitch_spin, 7, 1)
        hang_layout.addWidget(self.hang_deadband_label, 8, 0)
        hang_layout.addWidget(self.hang_deadband_spin, 8, 1)
        hang_layout.addWidget(self.hang_trip_label, 9, 0)
        hang_layout.addWidget(self.hang_trip_spin, 9, 1)

        ground_box = QWidget()
        self.ground_group = ground_box
        ground_layout = QGridLayout(ground_box)
        ground_layout.setHorizontalSpacing(6)
        ground_layout.setVerticalSpacing(6)
        ground_layout.setColumnStretch(1, 1)
        ground_layout.setColumnStretch(3, 1)
        self.ground_capture_button = QPushButton("Capture Ground Ref")
        self.ground_start_button = QPushButton("Ground Test Start")
        self.ground_stop_button = QPushButton("Ground Test Stop")
        self.ground_record_5_button = QPushButton("Record 5s")
        self.ground_record_10_button = QPushButton("Record 10s")
        self.ground_bench_roll_button = QPushButton("Run Ground Bench Roll")
        self.ground_bench_pitch_button = QPushButton("Run Ground Bench Pitch")
        self.ground_bench_yaw_button = QPushButton("Run Ground Bench Yaw")
        self.ground_bench_all_button = QPushButton("Run Ground Bench All")
        ground_layout.addWidget(self.ground_capture_button, 0, 0)
        ground_layout.addWidget(self.ground_start_button, 0, 1)
        ground_layout.addWidget(self.ground_stop_button, 0, 2)
        ground_layout.addWidget(self.ground_record_5_button, 1, 0)
        ground_layout.addWidget(self.ground_record_10_button, 1, 1)
        ground_layout.addWidget(self.ground_bench_roll_button, 2, 0)
        ground_layout.addWidget(self.ground_bench_pitch_button, 2, 1)
        ground_layout.addWidget(self.ground_bench_yaw_button, 3, 0)
        ground_layout.addWidget(self.ground_bench_all_button, 3, 1)
        self.ground_ref_status_label = QLabel("-")
        self.ground_kalman_status_label = QLabel("-")
        self.ground_trip_status_label = QLabel("-")
        ground_layout.addWidget(QLabel("Current Ref"), 4, 0)
        ground_layout.addWidget(self.ground_ref_status_label, 4, 1)
        ground_layout.addWidget(QLabel("Kalman"), 4, 2)
        ground_layout.addWidget(self.ground_kalman_status_label, 4, 3)
        ground_layout.addWidget(QLabel("Trip"), 5, 0)
        ground_layout.addWidget(self.ground_trip_status_label, 5, 1, 1, 3)
        self.ground_param_spins: dict[str, QDoubleSpinBox] = {}

        def add_ground_param(row: int, col: int, name: str, label: str, minimum: float, maximum: float, decimals: int, step: float, value: float) -> None:
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(decimals)
            spin.setSingleStep(step)
            spin.setValue(value)
            self.ground_param_spins[name] = spin
            ground_layout.addWidget(QLabel(label), row, col)
            ground_layout.addWidget(spin, row, col + 1)

        ground_row = 6
        ground_params = [
            ("rate_kp_roll", "rate_kp_roll", 0.0, 0.05, 5, 0.0005, 0.0026),
            ("rate_ki_roll", "rate_ki_roll", 0.0, 0.02, 5, 0.0001, 0.0),
            ("rate_kd_roll", "rate_kd_roll", 0.0, 0.01, 5, 0.0001, 0.0),
            ("rate_kp_pitch", "rate_kp_pitch", 0.0, 0.05, 5, 0.0005, 0.0024),
            ("rate_ki_pitch", "rate_ki_pitch", 0.0, 0.02, 5, 0.0001, 0.0),
            ("rate_kd_pitch", "rate_kd_pitch", 0.0, 0.01, 5, 0.0001, 0.0),
            ("rate_kp_yaw", "rate_kp_yaw", 0.0, 0.05, 5, 0.0005, 0.0026),
            ("rate_ki_yaw", "rate_ki_yaw", 0.0, 0.02, 5, 0.0001, 0.0),
            ("rate_kd_yaw", "rate_kd_yaw", 0.0, 0.01, 5, 0.0001, 0.0),
            ("ground_att_kp_roll", "ground_att_kp_roll", 0.0, 10.0, 2, 0.1, 1.2),
            ("ground_att_kp_pitch", "ground_att_kp_pitch", 0.0, 10.0, 2, 0.1, 1.2),
            ("ground_att_rate_limit_roll", "ground_att_rate_limit_roll", 0.0, 60.0, 1, 1.0, 12.0),
            ("ground_att_rate_limit_pitch", "ground_att_rate_limit_pitch", 0.0, 60.0, 1, 1.0, 12.0),
            ("ground_att_error_deadband_deg", "ground_deadband", 0.0, 5.0, 1, 0.1, 0.8),
            ("ground_att_trip_deg", "ground_trip", 5.0, 30.0, 1, 1.0, 12.0),
            ("ground_test_base_duty", "ground_base_duty", 0.0, 1.0, 3, 0.01, 0.08),
            ("gyro_lpf_hz", "gyro_lpf_hz", 0.0, 250.0, 1, 5.0, 40.0),
            ("accel_lpf_hz", "accel_lpf_hz", 0.0, 120.0, 1, 5.0, 20.0),
            ("rate_lpf_hz", "rate_lpf_hz", 0.0, 250.0, 1, 5.0, 30.0),
            ("kalman_q_angle", "kalman_q_angle", 0.000001, 1.0, 6, 0.0005, 0.0025),
            ("kalman_q_bias", "kalman_q_bias", 0.0000001, 0.1, 6, 0.0005, 0.003),
            ("kalman_r_measure", "kalman_r_measure", 0.00001, 10.0, 5, 0.01, 0.08),
        ]
        for idx, spec in enumerate(ground_params):
            col = 0 if idx % 2 == 0 else 2
            row = ground_row + idx // 2
            add_ground_param(row, col, *spec)

        udp_box = QWidget()
        self.udp_group = udp_box
        udp_layout = QGridLayout(udp_box)
        udp_layout.setHorizontalSpacing(6)
        udp_layout.setVerticalSpacing(6)
        udp_layout.setColumnStretch(1, 1)
        udp_layout.setRowStretch(18, 1)
        self.udp_warning_label = QLabel()
        self.udp_warning_label.setWordWrap(True)
        self.udp_warning_label.setStyleSheet(
            "color:#fecaca;background:#7f1d1d;border:1px solid #ef4444;border-radius:6px;padding:8px;font-weight:700;"
        )
        self.udp_enable_button = QPushButton()
        self.udp_disable_button = QPushButton()
        self.udp_stop_button = QPushButton()
        self.udp_takeoff_button = QPushButton()
        self.udp_land_button = QPushButton()
        self.udp_forward_button = QPushButton()
        self.udp_backward_button = QPushButton()
        self.udp_yaw_left_button = QPushButton()
        self.udp_yaw_right_button = QPushButton()
        self.udp_up_button = QPushButton()
        self.udp_down_button = QPushButton()
        self.udp_send_button = QPushButton()
        self.udp_max_pwm_label = QLabel()
        self.udp_throttle_label = QLabel()
        self.udp_axis_step_label = QLabel()
        self.udp_pitch_label = QLabel()
        self.udp_roll_label = QLabel()
        self.udp_yaw_label = QLabel()
        self.udp_max_pwm_spin = QDoubleSpinBox()
        self.udp_max_pwm_spin.setRange(1.0, 30.0)
        self.udp_max_pwm_spin.setDecimals(1)
        self.udp_max_pwm_spin.setSingleStep(1.0)
        self.udp_max_pwm_spin.setValue(12.0)
        self.udp_throttle_spin = QDoubleSpinBox()
        self.udp_throttle_spin.setRange(0.0, 30.0)
        self.udp_throttle_spin.setDecimals(1)
        self.udp_throttle_spin.setSingleStep(1.0)
        self.udp_pitch_spin = QDoubleSpinBox()
        self.udp_roll_spin = QDoubleSpinBox()
        self.udp_yaw_spin = QDoubleSpinBox()
        self.udp_axis_step_spin = QDoubleSpinBox()
        for spin in (self.udp_pitch_spin, self.udp_roll_spin, self.udp_yaw_spin):
            spin.setRange(-15.0, 15.0)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
        self.udp_axis_step_spin.setRange(0.5, 10.0)
        self.udp_axis_step_spin.setDecimals(1)
        self.udp_axis_step_spin.setSingleStep(0.5)
        self.udp_axis_step_spin.setValue(3.0)
        self.udp_watchdog_status_label = QLabel("-")
        self.udp_mode_status_label = QLabel("-")
        self.udp_armed_status_label = QLabel("-")
        self.udp_battery_status_label = QLabel("-")
        self.udp_watchdog_title_label = QLabel()
        self.udp_mode_title_label = QLabel()
        self.udp_armed_title_label = QLabel()
        self.udp_battery_title_label = QLabel()

        udp_layout.addWidget(self.udp_warning_label, 0, 0, 1, 2)
        udp_layout.addWidget(self.udp_enable_button, 1, 0)
        udp_layout.addWidget(self.udp_disable_button, 1, 1)
        udp_layout.addWidget(self.udp_stop_button, 2, 0, 1, 2)
        udp_layout.addWidget(self.udp_takeoff_button, 3, 0)
        udp_layout.addWidget(self.udp_land_button, 3, 1)
        udp_layout.addWidget(self.udp_max_pwm_label, 4, 0)
        udp_layout.addWidget(self.udp_max_pwm_spin, 4, 1)
        udp_layout.addWidget(self.udp_throttle_label, 5, 0)
        udp_layout.addWidget(self.udp_throttle_spin, 5, 1)
        udp_layout.addWidget(self.udp_axis_step_label, 6, 0)
        udp_layout.addWidget(self.udp_axis_step_spin, 6, 1)
        udp_layout.addWidget(self.udp_pitch_label, 7, 0)
        udp_layout.addWidget(self.udp_pitch_spin, 7, 1)
        udp_layout.addWidget(self.udp_roll_label, 8, 0)
        udp_layout.addWidget(self.udp_roll_spin, 8, 1)
        udp_layout.addWidget(self.udp_yaw_label, 9, 0)
        udp_layout.addWidget(self.udp_yaw_spin, 9, 1)
        udp_layout.addWidget(self.udp_forward_button, 10, 0)
        udp_layout.addWidget(self.udp_backward_button, 10, 1)
        udp_layout.addWidget(self.udp_yaw_left_button, 11, 0)
        udp_layout.addWidget(self.udp_yaw_right_button, 11, 1)
        udp_layout.addWidget(self.udp_up_button, 12, 0)
        udp_layout.addWidget(self.udp_down_button, 12, 1)
        udp_layout.addWidget(self.udp_send_button, 13, 0, 1, 2)
        udp_layout.addWidget(self.udp_watchdog_title_label, 14, 0)
        udp_layout.addWidget(self.udp_watchdog_status_label, 14, 1)
        udp_layout.addWidget(self.udp_mode_title_label, 15, 0)
        udp_layout.addWidget(self.udp_mode_status_label, 15, 1)
        udp_layout.addWidget(self.udp_armed_title_label, 16, 0)
        udp_layout.addWidget(self.udp_armed_status_label, 16, 1)
        udp_layout.addWidget(self.udp_battery_title_label, 17, 0)
        udp_layout.addWidget(self.udp_battery_status_label, 17, 1)

        self.debug_action_tabs.addTab(motor_box, "")
        self.debug_action_tabs.addTab(rate_box, "")
        self.debug_action_tabs.addTab(hang_box, "")
        self.debug_action_tabs.addTab(udp_box, "")
        layout.addWidget(self.debug_action_tabs)
        self.ground_title_label = QLabel("Ground Tune")
        self.ground_title_label.setStyleSheet("color:#93c5fd;font-weight:700;")
        layout.addWidget(self.ground_title_label)
        layout.addWidget(ground_box)

        return group

    def _build_calibration_panel(self) -> QGroupBox:
        group = QGroupBox()
        layout = QHBoxLayout(group)
        self.calib_group = group
        self.calib_gyro_button = QPushButton()
        self.calib_level_button = QPushButton()
        self.calib_gyro_button.setMaximumWidth(96)
        self.calib_level_button.setMaximumWidth(96)
        layout.addWidget(self.calib_gyro_button)
        layout.addWidget(self.calib_level_button)
        layout.addStretch(1)
        return group

    def _build_log_export_panel(self) -> QGroupBox:
        group = QGroupBox()
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        self.log_path_edit = QLineEdit(str(Path.cwd() / "telemetry.csv"))
        self.csv_group = group
        self.log_browse_button = QPushButton()
        self.start_log_button = QPushButton()
        self.stop_log_button = QPushButton()
        self.dump_duration_spin = QDoubleSpinBox()
        self.dump_duration_spin.setRange(0.5, 300.0)
        self.dump_duration_spin.setDecimals(1)
        self.dump_duration_spin.setValue(5.0)
        self.dump_csv_button = QPushButton()
        self.output_label = QLabel()
        self.dump_s_label = QLabel()
        self.log_browse_button.setMaximumWidth(64)
        self.start_log_button.setMaximumWidth(82)
        self.stop_log_button.setMaximumWidth(82)
        self.dump_csv_button.setMaximumWidth(82)
        layout.addWidget(self.output_label, 0, 0)
        layout.addWidget(self.log_path_edit, 0, 1, 1, 4)
        layout.addWidget(self.log_browse_button, 0, 5)
        layout.addWidget(self.start_log_button, 1, 0)
        layout.addWidget(self.stop_log_button, 1, 1)
        layout.addWidget(self.dump_s_label, 1, 2)
        layout.addWidget(self.dump_duration_spin, 1, 3)
        layout.addWidget(self.dump_csv_button, 1, 4)
        return group

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox()
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)
        self.status_cards: dict[str, tuple[QLabel, QLabel]] = {}
        cards = [
            ("arm_state", 0, 0),
            ("failsafe_reason", 0, 1),
            ("control_mode", 1, 0),
            ("imu_mode", 1, 1),
            ("stream", 2, 0),
            ("battery_voltage", 2, 1),
            ("imu_age_us", 3, 0),
            ("loop_dt_us", 3, 1),
            ("baro_health", 4, 0),
            ("baro_altitude_m", 4, 1),
        ]
        for key, row, col in cards:
            card = QFrame()
            card.setStyleSheet("QFrame {background:#0f1722;border:1px solid #253447;border-radius:8px;}")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(8, 6, 8, 6)
            title = QLabel(key)
            title.setStyleSheet("color:#93c5fd;font-size:10px;font-weight:600;")
            value = QLabel("-")
            value.setStyleSheet("font-size:12px;font-weight:700;color:#f8fafc;")
            value.setWordWrap(True)
            card_layout.addWidget(title)
            card_layout.addWidget(value)
            self.status_cards[key] = (title, value)
            layout.addWidget(card, row, col)
        return group

    def _wire_signals(self) -> None:
        self.language_combo.currentIndexChanged.connect(self._change_language)
        self.link_type_combo.currentTextChanged.connect(self._update_link_inputs)
        self.main_splitter.splitterMoved.connect(lambda *_args: self._save_settings())
        self.center_splitter.splitterMoved.connect(lambda *_args: self._save_settings())
        self.right_splitter.splitterMoved.connect(lambda *_args: self._save_settings())
        self.params_splitter.splitterMoved.connect(lambda *_args: self._save_settings())
        self.connection_section.toggled.connect(lambda *_args: self._save_settings())
        self.safety_section.toggled.connect(lambda *_args: self._save_settings())
        self.debug_action_tabs.currentChanged.connect(lambda *_args: self._save_settings())
        self.refresh_ports_button.clicked.connect(self._refresh_serial_ports)
        self.connect_button.clicked.connect(self._connect_requested)
        self.disconnect_button.clicked.connect(self._disconnect_requested)

        self.arm_button.clicked.connect(lambda: self._run_checked_command_action("arm", CmdId.ARM, self._session.arm))
        self.disarm_button.clicked.connect(lambda: self._run_checked_command_action("disarm", CmdId.DISARM, self._session.disarm))
        self.kill_button.clicked.connect(lambda: self._run_checked_command_action("kill", CmdId.KILL, self._session.kill))
        self.reboot_button.clicked.connect(lambda: self._run_checked_command_action("reboot", CmdId.REBOOT, self._session.reboot))

        self.stream_on_button.clicked.connect(lambda: self._run_session_action("stream_on", self._session.start_stream))
        self.stream_off_button.clicked.connect(lambda: self._run_session_action("stream_off", self._session.stop_stream))
        self.apply_stream_rate_button.clicked.connect(self._apply_stream_rate)
        self.copy_telemetry_button.clicked.connect(self.telemetry_table.copy_selection)

        self.chart_toggle_button.clicked.connect(self._toggle_charts)
        self.clear_charts_button.clicked.connect(self._clear_charts)
        self.auto_scale_button.clicked.connect(self._auto_scale_plot)
        self.reset_view_button.clicked.connect(self._reset_plot_view)
        self.chart_window_combo.currentIndexChanged.connect(self._save_settings)
        self.chart_group_combo.currentTextChanged.connect(self._change_chart_group)

        self.refresh_params_button.clicked.connect(self._refresh_params)
        self.param_search_edit.textChanged.connect(self._filter_params_table)
        self.params_table.itemSelectionChanged.connect(self._handle_param_selection)
        self.params_table.itemDoubleClicked.connect(lambda *_args: self.param_new_value_edit.setFocus())
        self.param_new_value_edit.textChanged.connect(self._update_param_hint)
        self.set_param_button.clicked.connect(self._set_selected_param)
        self.save_params_button.clicked.connect(lambda: self._run_session_action("save_params", self._session.save_params))
        self.reset_params_button.clicked.connect(lambda: self._run_session_action("reset_params", self._session.reset_params))
        self.export_params_button.clicked.connect(self._export_params)
        self.import_params_button.clicked.connect(self._import_params)

        self.motor_start_button.clicked.connect(self._start_motor_test)
        self.motor_stop_button.clicked.connect(self._stop_motor_test)
        self.calib_gyro_button.clicked.connect(
            lambda: self._run_checked_command_action("calib_gyro", CmdId.CALIB_GYRO, self._session.calib_gyro)
        )
        self.calib_level_button.clicked.connect(
            lambda: self._run_checked_command_action("calib_level", CmdId.CALIB_LEVEL, self._session.calib_level)
        )
        self.rate_start_button.clicked.connect(self._start_rate_test)
        self.rate_stop_button.clicked.connect(self._stop_rate_test)
        self.hang_capture_button.clicked.connect(self._capture_attitude_ref)
        self.hang_start_button.clicked.connect(self._start_attitude_test)
        self.hang_stop_button.clicked.connect(self._stop_attitude_test)
        self.ground_capture_button.clicked.connect(self._capture_ground_ref)
        self.ground_start_button.clicked.connect(self._start_ground_test)
        self.ground_stop_button.clicked.connect(self._stop_ground_test)
        self.ground_record_5_button.clicked.connect(lambda: self._record_ground(5.0))
        self.ground_record_10_button.clicked.connect(lambda: self._record_ground(10.0))
        self.ground_bench_roll_button.clicked.connect(lambda: self._run_ground_bench("roll"))
        self.ground_bench_pitch_button.clicked.connect(lambda: self._run_ground_bench("pitch"))
        self.ground_bench_yaw_button.clicked.connect(lambda: self._run_ground_bench("yaw"))
        self.ground_bench_all_button.clicked.connect(lambda: self._run_ground_bench("all"))
        self.udp_enable_button.clicked.connect(self._enable_udp_manual)
        self.udp_disable_button.clicked.connect(self._disable_udp_manual)
        self.udp_stop_button.clicked.connect(self._stop_udp_manual)
        self.udp_takeoff_button.clicked.connect(self._takeoff_udp_manual)
        self.udp_land_button.clicked.connect(self._land_udp_manual)
        self.udp_send_button.clicked.connect(self._send_udp_manual_once)
        self.udp_max_pwm_spin.valueChanged.connect(self._clamp_udp_throttle_spin)
        self.udp_forward_button.pressed.connect(lambda: self._set_udp_motion(pitch=-self.udp_axis_step_spin.value()))
        self.udp_forward_button.released.connect(self._zero_udp_axes)
        self.udp_backward_button.pressed.connect(lambda: self._set_udp_motion(pitch=self.udp_axis_step_spin.value()))
        self.udp_backward_button.released.connect(self._zero_udp_axes)
        self.udp_yaw_left_button.pressed.connect(lambda: self._set_udp_motion(yaw=-self.udp_axis_step_spin.value()))
        self.udp_yaw_left_button.released.connect(self._zero_udp_axes)
        self.udp_yaw_right_button.pressed.connect(lambda: self._set_udp_motion(yaw=self.udp_axis_step_spin.value()))
        self.udp_yaw_right_button.released.connect(self._zero_udp_axes)
        self.udp_up_button.clicked.connect(lambda: self._adjust_udp_throttle(+self.udp_axis_step_spin.value()))
        self.udp_down_button.clicked.connect(lambda: self._adjust_udp_throttle(-self.udp_axis_step_spin.value()))
        self.log_browse_button.clicked.connect(self._browse_log_path)
        self.log_path_edit.textChanged.connect(self._sync_log_path_tooltip)
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
        self._set_last_result(self._t("msg.command_running", label=label))
        self._bridge.run_async(label, callback)

    def _run_checked_command_action(self, label: str, cmd_id: int, callback) -> None:
        def wrapped():
            return ensure_command_ok(cmd_id, int(callback()))

        self._run_session_action(label, wrapped)

    def _set_last_result(self, text: str) -> None:
        self._last_result = text
        self.last_result_label.setText(text)

    def _sync_log_path_tooltip(self) -> None:
        self.log_path_edit.setToolTip(self.log_path_edit.text().strip())

    def _append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.event_log_edit.appendPlainText(f"[{stamp}] {text}")

    def _connect_failed_text(self, error: object) -> str:
        message = str(error)
        if message.startswith("Connect failed:"):
            return message
        return self._t("msg.connect_failed", error=message)

    def _set_connection_info_text(self, text: str) -> None:
        self.connection_info_label.setText(text)
        if hasattr(self, "tools_connection_info_label"):
            self.tools_connection_info_label.setText(text)

    def _set_connection_error_text(self, text: str) -> None:
        self.connection_error_detail.setText(text)
        if hasattr(self, "tools_connection_error_detail"):
            self.tools_connection_error_detail.setText(text)

    def _set_connecting_state(self, detail: str) -> None:
        self._connecting = True
        self._connecting_detail = detail
        self._connection_target_detail = detail
        self._last_connect_error_message = None
        self._connect_watchdog_timer.start()
        _set_badge(self.connection_status_chip, self._t("status.connecting"), "active")
        self._set_connection_info_text(detail)
        self._set_connection_error_text(self._t("status.no_conn_error"))
        self.last_error_label.setText(self._t("status.no_error"))
        self._set_last_result(self._t("status.connecting"))
        self._append_log(f"{self._t('status.connecting')}: {detail}")
        self._refresh_enabled_state()

    def _show_connect_failure(self, error: object) -> None:
        message = self._connect_failed_text(error)
        self._connecting = False
        self._connecting_detail = ""
        self._connect_watchdog_timer.stop()
        self._stream_enabled = False
        _set_badge(self.connection_status_chip, self._t("status.disconnected"), "warn")
        self._set_connection_info_text(self._t("status.no_session"))
        self._set_connection_error_text(message)
        self.last_error_label.setText(message)
        self._set_last_result(message)
        if self._last_connect_error_message != message:
            self._append_log(message)
            self._last_connect_error_message = message
        self._update_stream_chip()
        self._refresh_enabled_state()

    def _handle_connect_watchdog_timeout(self) -> None:
        if not self._connecting:
            return
        detail = self._connecting_detail or "device"
        self._show_connect_failure(
            f"connection attempt timed out in GUI while waiting for result from {detail}. "
            "If this is a serial link, close any other serial monitor or GUI that may be holding the port, "
            "then unplug/replug the device and retry."
        )

    def _save_event_log(self) -> None:
        output, _ = QFileDialog.getSaveFileName(
            self,
            self._t("button.save_log"),
            "esp-drone-gui.log",
            "Log Files (*.log *.txt)",
        )
        if not output:
            return
        Path(output).write_text(self.event_log_edit.toPlainText(), encoding="utf-8")
        self._append_log(self._t("msg.saved_log", path=output))

    def _update_link_inputs(self) -> None:
        is_serial = self._combo_data(self.link_type_combo) == "serial"
        self.transport_stack.setCurrentIndex(0 if is_serial else 1)
        self._save_settings()

    def _change_language(self) -> None:
        self._language = "zh" if self.language_combo.currentIndex() == 0 else "en"
        self._apply_language()
        self._save_settings()

    def _status_text(self, mapping: dict[int, tuple[str, str]], value: int) -> tuple[str, str]:
        key, role = _status_from_map(mapping, value)
        return self._t(key), role

    def _apply_language(self) -> None:
        self.setWindowTitle(self._t("window.title"))
        self.window_title_label.setText(self._t("window.title"))
        self.language_label.setText(self._t("toolbar.language"))
        self.language_combo.blockSignals(True)
        self.language_combo.clear()
        self.language_combo.addItems([self._t("lang.zh"), self._t("lang.en")])
        self.language_combo.setCurrentIndex(0 if self._language == "zh" else 1)
        self.language_combo.blockSignals(False)

        self.connection_group.setTitle("")
        self.safety_group.setTitle("")
        self.connection_section.set_title(self._t("group.connection"))
        self.safety_section.set_title(self._t("group.safety"))
        self.debug_group.setTitle(self._t("group.debug"))
        self.chart_group.setTitle(self._t("group.chart"))
        self.realtime_group.setTitle(self._t("group.realtime"))
        self.status_group.setTitle(self._t("group.status"))
        self.params_group.setTitle("")
        self.bottom_panel.setTitle("")
        self.session_info_group.setTitle(self._t("group.session_info"))
        self.calib_group.setTitle(self._t("group.calib"))
        self.csv_group.setTitle(self._t("group.csv"))

        self.link_type_label.setText(self._t("label.link"))
        self.serial_port_label.setText(self._t("label.serial_port"))
        self.baud_label.setText(self._t("label.baud"))
        self.udp_host_label.setText(self._t("label.udp_host"))
        self.udp_port_label.setText(self._t("label.udp_port"))
        self.udp_transport_hint_label.setText(self._t("udp.transport_hint"))
        self.udp_ap_info_label.setText(self._t("udp.ap_info"))
        self.session_title_label.setText(self._t("label.session"))
        self.last_conn_error_title_label.setText(self._t("label.last_conn_error"))
        if hasattr(self, "tools_session_title_label"):
            self.tools_session_title_label.setText(self._t("label.session"))
        if hasattr(self, "tools_last_conn_error_title_label"):
            self.tools_last_conn_error_title_label.setText(self._t("label.last_conn_error"))
        self.connect_button.setText(self._t("button.connect"))
        self.disconnect_button.setText(self._t("button.disconnect"))
        self.refresh_ports_button.setText(self._t("button.refresh"))

        self.arm_button.setText(self._t("button.arm"))
        self.disarm_button.setText(self._t("button.disarm"))
        self.kill_button.setText(self._t("button.kill"))
        self.reboot_button.setText(self._t("button.reboot"))

        self.debug_warning_label.setText(self._t("warn.bench"))
        self.motor_label.setText(self._t("label.motor"))
        self.duty_label.setText(self._t("label.duty"))
        self.motor_start_button.setText(self._t("button.start"))
        self.motor_stop_button.setText(self._t("button.stop"))
        self.calib_gyro_button.setText(self._t("button.calib_gyro_short"))
        self.calib_level_button.setText(self._t("button.calib_level_short"))
        self.calib_gyro_button.setToolTip(self._t("button.calib_gyro"))
        self.calib_level_button.setToolTip(self._t("button.calib_level"))
        self.rate_axis_label.setText(self._t("label.axis"))
        self.rate_dps_label.setText(self._t("label.rate_dps"))
        self.rate_start_button.setText(self._t("button.start"))
        self.rate_stop_button.setText(self._t("button.stop"))
        self.hang_note_label.setText(self._t("hang.note"))
        self.hang_capture_button.setText(self._t("hang.capture_ref"))
        self.hang_start_button.setText(self._t("hang.start"))
        self.hang_stop_button.setText(self._t("hang.stop"))
        self.hang_base_duty_label.setText(self._t("hang.base_duty"))
        self.hang_kp_roll_label.setText(self._t("hang.kp_roll"))
        self.hang_kp_pitch_label.setText(self._t("hang.kp_pitch"))
        self.hang_rate_limit_roll_label.setText(self._t("hang.rate_limit_roll"))
        self.hang_rate_limit_pitch_label.setText(self._t("hang.rate_limit_pitch"))
        self.hang_deadband_label.setText(self._t("hang.deadband_deg"))
        self.hang_trip_label.setText(self._t("hang.trip_deg"))
        self.ground_capture_button.setText("Capture Ground Ref")
        self.ground_start_button.setText("Ground Test Start")
        self.ground_stop_button.setText("Ground Test Stop")
        self.ground_record_5_button.setText("Record 5s")
        self.ground_record_10_button.setText("Record 10s")
        self.ground_bench_roll_button.setText("Run Ground Bench Roll")
        self.ground_bench_pitch_button.setText("Run Ground Bench Pitch")
        self.ground_bench_yaw_button.setText("Run Ground Bench Yaw")
        self.ground_bench_all_button.setText("Run Ground Bench All")
        self.udp_warning_label.setText(self._t("udp.warn"))
        self.udp_enable_button.setText(self._t("udp.enable"))
        self.udp_disable_button.setText(self._t("udp.disable"))
        self.udp_stop_button.setText(self._t("udp.stop"))
        self.udp_takeoff_button.setText(self._t("udp.takeoff"))
        self.udp_land_button.setText(self._t("udp.land"))
        self.udp_forward_button.setText(self._t("udp.forward"))
        self.udp_backward_button.setText(self._t("udp.backward"))
        self.udp_yaw_left_button.setText(self._t("udp.yaw_left"))
        self.udp_yaw_right_button.setText(self._t("udp.yaw_right"))
        self.udp_up_button.setText(self._t("udp.up"))
        self.udp_down_button.setText(self._t("udp.down"))
        self.udp_send_button.setText(self._t("udp.send"))
        self.udp_max_pwm_label.setText(self._t("udp.max_pwm"))
        self.udp_throttle_label.setText(self._t("udp.throttle"))
        self.udp_axis_step_label.setText(self._t("udp.axis_step"))
        self.udp_pitch_label.setText(self._t("udp.pitch"))
        self.udp_roll_label.setText(self._t("udp.roll"))
        self.udp_yaw_label.setText(self._t("udp.yaw"))
        self.udp_watchdog_title_label.setText(self._t("udp.status_watchdog"))
        self.udp_mode_title_label.setText(self._t("udp.status_mode"))
        self.udp_armed_title_label.setText(self._t("udp.status_armed"))
        self.udp_battery_title_label.setText(self._t("udp.status_battery"))
        self.output_label.setText(self._t("label.output"))
        self.dump_s_label.setText(self._t("label.dump_s"))
        self.log_browse_button.setText(self._t("button.browse"))
        self.start_log_button.setText(self._t("button.start_log"))
        self.stop_log_button.setText(self._t("button.stop_log"))
        self.dump_csv_button.setText(self._t("button.dump_csv"))
        self.debug_action_tabs.setTabText(0, self._t("tab.motor"))
        self.debug_action_tabs.setTabText(1, self._t("tab.rate"))
        self.debug_action_tabs.setTabText(2, self._t("tab.hang_attitude"))
        self.ground_title_label.setText("Ground Tune")
        self.debug_action_tabs.setTabText(3, self._t("tab.udp_control"))

        self.stream_on_button.setText(self._t("button.stream_on"))
        self.stream_off_button.setText(self._t("button.stream_off"))
        self.target_hz_label.setText(self._t("label.target_hz"))
        self.apply_stream_rate_button.setText(self._t("button.apply_hz"))
        self.copy_telemetry_button.setText(self._t("button.copy_selected"))
        self.telemetry_table.setHorizontalHeaderLabels([self._t("label.name"), self._t("label.current")])
        for row, name in enumerate(self.TELEMETRY_FIELDS):
            item = self.telemetry_table.item(row, 0)
            if item is not None:
                item.setText(self._t(TELEMETRY_FIELD_KEYS.get(name, name)))

        current_link = self._combo_data(self.link_type_combo) if self.link_type_combo.count() else "serial"
        self.link_type_combo.blockSignals(True)
        self.link_type_combo.clear()
        self.link_type_combo.addItem(self._t("transport.serial"), "serial")
        self.link_type_combo.addItem("UDP", "udp")
        self.link_type_combo.setCurrentIndex(0 if current_link == "serial" else 1)
        self.link_type_combo.blockSignals(False)

        current_axis = self.rate_axis_combo.currentIndex()
        self.rate_axis_combo.blockSignals(True)
        self.rate_axis_combo.clear()
        self.rate_axis_combo.addItem(self._t("axis.roll"), 0)
        self.rate_axis_combo.addItem(self._t("axis.pitch"), 1)
        self.rate_axis_combo.addItem(self._t("axis.yaw"), 2)
        self.rate_axis_combo.setCurrentIndex(max(0, min(current_axis, 2)))
        self.rate_axis_combo.blockSignals(False)

        self.chart_group_label.setText(self._t("label.chart_group"))
        current_group = self._current_chart_group
        self.chart_group_combo.blockSignals(True)
        self.chart_group_combo.clear()
        for group_key in self.CHART_GROUPS:
            self.chart_group_combo.addItem(self._t(self.CHART_GROUPS[group_key].title), group_key)
        chart_index = self.chart_group_combo.findData(current_group)
        self.chart_group_combo.setCurrentIndex(chart_index if chart_index >= 0 else 0)
        self.chart_group_combo.blockSignals(False)
        self.chart_toggle_button.setText(self._t("button.pause_charts") if self._charts_running else self._t("button.resume_charts"))
        self.clear_charts_button.setText(self._t("button.clear_charts"))
        self.auto_scale_button.setText(self._t("button.auto_scale"))
        self.reset_view_button.setText(self._t("button.reset_view"))
        self.chart_window_label.setText(self._t("label.window"))

        self.param_search_edit.setPlaceholderText(self._t("placeholder.search"))
        self.refresh_params_button.setText(self._t("button.refresh"))
        self.save_params_button.setText(self._t("button.save"))
        self.reset_params_button.setText(self._t("button.reset"))
        self.export_params_button.setText(self._t("button.export_json"))
        self.import_params_button.setText(self._t("button.import_json"))
        self.right_tabs.setTabText(0, self._t("tab.params"))
        self.right_tabs.setTabText(1, self._t("tab.events"))
        self.right_tabs.setTabText(2, self._t("tab.tools"))
        self.params_table.setHorizontalHeaderLabels([self._t("label.name"), self._t("label.type"), self._t("label.current")])
        self.param_name_title_label.setText(self._t("label.name"))
        self.param_type_title_label.setText(self._t("label.type"))
        self.param_current_title_label.setText(self._t("label.current"))
        self.param_new_value_title_label.setText(self._t("label.new_value"))
        self.param_new_value_edit.setPlaceholderText(self._t("placeholder.new_value"))
        self.param_desc_title_label.setText(self._t("label.description"))
        self.param_help_text.setPlaceholderText(self._t("placeholder.desc"))
        if self._selected_param is not None:
            self.param_help_text.setPlainText(self._param_help(self._selected_param.name))
            self._update_param_hint()
        self.set_param_button.setText(self._t("button.set_selected"))

        self.last_result_title_label.setText(self._t("label.last_result"))
        self.last_log_title_label.setText(self._t("label.last_log"))
        self.last_error_title_label.setText(self._t("label.last_error"))
        self.clear_log_button.setText(self._t("button.clear_log"))
        self.copy_log_button.setText(self._t("button.copy_log"))
        self.save_log_button.setText(self._t("button.save_log"))
        self.event_log_edit.setPlaceholderText(self._t("placeholder.event_log"))

        for key, (title, _value) in self.status_cards.items():
            title.setText(self._t(TELEMETRY_FIELD_KEYS.get(key, f"field.{key}")))

        self._rebuild_chart_channels()
        self._update_stream_chip()
        if self._last_telemetry is None:
            self._refresh_status_cards_from_none()

    def _rebuild_chart_channels(self) -> None:
        current_group = self.chart_group_combo.currentData() or self._current_chart_group
        if current_group not in self.CHART_GROUPS:
            current_group = "gyro"
        self._current_chart_group = str(current_group)
        spec = self.CHART_GROUPS[self._current_chart_group]

        while self.chart_channel_bar.count():
            item = self.chart_channel_bar.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        self.main_plot.clear()
        plot_item = self.main_plot.getPlotItem()
        if plot_item.legend is not None:
            plot_item.legend.scene().removeItem(plot_item.legend)
            plot_item.legend = None
        plot_item.addLegend(offset=(12, 12))
        plot_item.setLabel("left", self._t(spec.y_label))
        plot_item.setLabel("bottom", self._t("chart.time"))
        plot_item.setTitle(self._t(spec.title), color="#f8fafc", size="14pt")

        self._chart_curves.clear()
        self._chart_channel_checks.clear()
        for index, (field_name, label_key) in enumerate(spec.fields):
            checkbox = QCheckBox(self._t(label_key))
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(self._refresh_plots)
            self.chart_channel_bar.addWidget(checkbox)
            self._chart_channel_checks[field_name] = checkbox
            self._chart_curves[field_name] = self.main_plot.plot(
                [],
                [],
                name=self._t(label_key),
                pen=pg.mkPen(self.CHART_COLORS[index % len(self.CHART_COLORS)], width=2.2),
            )
        self.chart_channel_bar.addStretch(1)
        self._reset_plot_view()

    def _change_chart_group(self) -> None:
        self._rebuild_chart_channels()
        self._save_settings()

    def _toggle_log_panel(self) -> None:
        if hasattr(self, "right_tabs"):
            self.right_tabs.setCurrentIndex(1)

    def _auto_scale_plot(self) -> None:
        self.main_plot.enableAutoRange(axis="y", enable=True)
        self.main_plot.autoRange()

    def _reset_plot_view(self) -> None:
        window = self._current_chart_window()
        self.main_plot.setXRange(-window, 0.0, padding=0.01)
        self.main_plot.enableAutoRange(axis="y", enable=True)
        self.main_plot.autoRange()

    def _refresh_status_cards_from_none(self) -> None:
        for _key, (_title, value) in self.status_cards.items():
            value.setText(self._t("status.no_value"))
            value.setStyleSheet("font-size:14px;font-weight:700;color:#f8fafc;")

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
        if (self.link_type_combo.currentData() or self.link_type_combo.currentText()) == "serial":
            port = self.serial_port_combo.currentText().strip()
            if not port:
                self._show_connect_failure(self._t("msg.connect_no_port"))
                return
            baudrate = int(self.baudrate_spin.value())
            self._set_connecting_state(f"serial {port} @ {baudrate}")
            self._run_session_action(
                "connect_serial",
                lambda: self._session.connect_serial(
                    port,
                    baudrate=baudrate,
                    timeout=0.2,
                    open_retry_timeout_s=0.75,
                ),
            )
            return

        host = self.udp_host_edit.text().strip()
        if not host:
            self._show_connect_failure(self._t("msg.udp_host_required"))
            return
        port = int(self.udp_port_spin.value())
        self._set_connecting_state(f"udp {host}:{port}")
        self._run_session_action(
            "connect_udp",
            lambda: self._session.connect_udp(host, port=port, timeout=1.0),
        )

    def _disconnect_requested(self) -> None:
        self._udp_manual_enabled = False
        if hasattr(self, "_udp_control_timer"):
            self._udp_control_timer.stop()
        self._run_session_action("disconnect", lambda: self._session.disconnect())

    def _apply_stream_rate(self) -> None:
        link_value = self.link_type_combo.currentData() or self.link_type_combo.currentText()
        name = "telemetry_usb_hz" if link_value == "serial" else "telemetry_udp_hz"
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
        self._sync_hang_param_spins()
        self._sync_ground_param_spins()
        self._sync_udp_param_spins()
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
            self.param_hint_label.setText(self._t("hint.param_default"))
            return
        self._selected_param = self._params[row]
        self.param_name_label.setText(self._selected_param.name)
        self.param_type_label.setText(TYPE_NAMES.get(self._selected_param.type_id, str(self._selected_param.type_id)))
        self.param_current_value_label.setText(_format_value(self._selected_param.name, self._selected_param.value))
        self.param_new_value_edit.setText(str(self._selected_param.value))
        self.param_help_text.setPlainText(self._param_help(self._selected_param.name))
        self._update_param_hint()

    def _param_help(self, name: str) -> str:
        if self._language == "zh":
            return self.PARAM_HELP_ZH.get(name, self._t("param.help.default"))
        return self.PARAM_HELP.get(name, self._t("param.help.default"))

    def _local_param_hint(self, param: ParamValue | None, value_text: str) -> str:
        if param is None:
            return self._t("hint.param_default")
        if not value_text:
            return self._t("hint.param_default")
        name = param.name
        try:
            if name.startswith("motor_") or name == "bringup_test_base_duty":
                value = float(value_text)
                if not 0.0 <= value <= 1.0:
                    return "期望归一化 duty 范围为 0..1。" if self._language == "zh" else "Expected normalized duty in the range 0..1."
                return "本地看起来是合法 duty。" if self._language == "zh" else "Normalized duty looks locally valid."
            if name == "imu_return_rate_code":
                value = int(value_text, 0)
                if not 0x00 <= value <= 0x09:
                    return "期望 IMU 回传码范围 0x00..0x09。" if self._language == "zh" else "Expected IMU return-rate code in the range 0x00..0x09."
                return "本地看起来是合法 IMU 回传码。" if self._language == "zh" else "IMU return-rate code looks locally valid."
            if name in {"telemetry_usb_hz", "telemetry_udp_hz"}:
                value = int(value_text, 0)
                if name == "telemetry_usb_hz" and not 1 <= value <= 200:
                    return "telemetry_usb_hz 建议保持在 1..200。" if self._language == "zh" else "telemetry_usb_hz should stay within 1..200."
                if name == "telemetry_udp_hz" and not 1 <= value <= 100:
                    return "telemetry_udp_hz 建议保持在 1..100。" if self._language == "zh" else "telemetry_udp_hz should stay within 1..100."
                return "本地看起来是合法遥测频率。" if self._language == "zh" else "Telemetry rate looks locally valid."
        except ValueError:
            return "本地无法解析该值。" if self._language == "zh" else "Could not parse the value locally."
        return self._param_help(name)

    def _update_param_hint(self) -> None:
        self.param_hint_label.setText(self._local_param_hint(self._selected_param, self.param_new_value_edit.text().strip()))

    def _set_selected_param(self) -> None:
        if self._selected_param is None:
            self._on_error(self._t("msg.set_param_no_select"))
            return
        value_text = self.param_new_value_edit.text().strip()
        if not value_text:
            self._on_error(self._t("msg.set_param_no_value"))
            return
        name = self._selected_param.name
        type_id = self._selected_param.type_id
        self._run_session_action(
            f"set_param:{name}",
            lambda: self._session.set_param(name, type_id, value_text),
        )

    def _export_params(self) -> None:
        output, _ = QFileDialog.getSaveFileName(self, self._t("button.export_json"), "params.json", "JSON Files (*.json)")
        if not output:
            return
        output_path = Path(output)
        self._run_session_action("export_params", lambda: self._session.export_params(output_path))

    def _import_params(self) -> None:
        input_path, _ = QFileDialog.getOpenFileName(self, self._t("button.import_json"), "", "JSON Files (*.json)")
        if not input_path:
            return
        path_obj = Path(input_path)
        self._run_session_action("import_params", lambda: self._session.import_params(path_obj, save_after=False))

    def _start_motor_test(self) -> None:
        index = self.motor_combo.currentIndex()
        duty = float(self.motor_duty_spin.value())
        self._run_checked_command_action("motor_test_start", CmdId.MOTOR_TEST, lambda: self._session.motor_test(index, duty))

    def _stop_motor_test(self) -> None:
        index = self.motor_combo.currentIndex()
        self._run_checked_command_action("motor_test_stop", CmdId.MOTOR_TEST, lambda: self._session.motor_test(index, 0.0))

    def _start_rate_test(self) -> None:
        index = self.rate_axis_combo.currentIndex()
        rate = float(self.rate_value_spin.value())
        self._run_checked_command_action("rate_test_start", CmdId.RATE_TEST, lambda: self._session.rate_test(index, rate))

    def _stop_rate_test(self) -> None:
        index = self.rate_axis_combo.currentIndex()
        self._run_checked_command_action("rate_test_stop", CmdId.RATE_TEST, lambda: self._session.rate_test(index, 0.0))

    def _sync_hang_param_spins(self) -> None:
        if not getattr(self, "_params", None):
            return

        param_map = {item.name: item.value for item in self._params}
        updates = {
            "attitude_test_base_duty": self.hang_base_duty_spin,
            "attitude_kp_roll": self.hang_kp_roll_spin,
            "attitude_kp_pitch": self.hang_kp_pitch_spin,
            "attitude_rate_limit_roll": self.hang_rate_limit_roll_spin,
            "attitude_rate_limit_pitch": self.hang_rate_limit_pitch_spin,
            "attitude_error_deadband_deg": self.hang_deadband_spin,
            "attitude_trip_deg": self.hang_trip_spin,
        }
        for name, spin in updates.items():
            if name not in param_map:
                continue
            spin.blockSignals(True)
            spin.setValue(float(param_map[name]))
            spin.blockSignals(False)

    def _apply_hang_params(self) -> None:
        updates = [
            ("attitude_test_base_duty", self.hang_base_duty_spin.value()),
            ("attitude_kp_roll", self.hang_kp_roll_spin.value()),
            ("attitude_kp_pitch", self.hang_kp_pitch_spin.value()),
            ("attitude_rate_limit_roll", self.hang_rate_limit_roll_spin.value()),
            ("attitude_rate_limit_pitch", self.hang_rate_limit_pitch_spin.value()),
            ("attitude_error_deadband_deg", self.hang_deadband_spin.value()),
            ("attitude_trip_deg", self.hang_trip_spin.value()),
        ]
        for name, value in updates:
            self._session.set_param(name, 4, value)
        return None

    def _capture_attitude_ref(self) -> None:
        def action():
            self._apply_hang_params()
            return ensure_command_ok(CmdId.ATTITUDE_CAPTURE_REF, int(self._session.attitude_capture_ref()))

        self._run_session_action("attitude_capture_ref", action)

    def _start_attitude_test(self) -> None:
        def action():
            self._apply_hang_params()
            return ensure_command_ok(CmdId.ATTITUDE_TEST_START, int(self._session.attitude_test_start()))

        self._run_session_action("attitude_test_start", action)

    def _stop_attitude_test(self) -> None:
        self._run_checked_command_action("attitude_test_stop", CmdId.ATTITUDE_TEST_STOP, self._session.attitude_test_stop)

    def _sync_ground_param_spins(self) -> None:
        if not getattr(self, "_params", None) or not hasattr(self, "ground_param_spins"):
            return
        param_map = {item.name: item.value for item in self._params}
        for name, spin in self.ground_param_spins.items():
            if name not in param_map:
                continue
            spin.blockSignals(True)
            spin.setValue(float(param_map[name]))
            spin.blockSignals(False)

    def _apply_ground_params(self) -> None:
        for name, spin in self.ground_param_spins.items():
            self._session.set_param(name, 4, spin.value())

    def _capture_ground_ref(self) -> None:
        def action():
            return ensure_command_ok(CmdId.GROUND_CAPTURE_REF, int(self._session.ground_capture_ref()))

        self._run_session_action("ground_capture_ref", action)

    def _start_ground_test(self) -> None:
        def action():
            self._apply_ground_params()
            base_spin = self.ground_param_spins.get("ground_test_base_duty")
            base_duty = None if base_spin is None else float(base_spin.value())
            return ensure_command_ok(CmdId.GROUND_TEST_START, int(self._session.ground_test_start(base_duty=base_duty)))

        self._run_session_action("ground_test_start", action)

    def _stop_ground_test(self) -> None:
        self._run_checked_command_action("ground_test_stop", CmdId.GROUND_TEST_STOP, self._session.ground_test_stop)

    def _record_ground(self, seconds: float) -> None:
        output_dir = Path.cwd() / "logs"
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_ground_record.csv"

        def action():
            rows = self._session.dump_csv(path, duration_s=seconds)
            return {"path": str(path), "rows": rows}

        self._run_session_action("dump_csv", action)

    def _run_ground_bench(self, axis: str) -> None:
        def action():
            self._apply_ground_params()
            base_spin = self.ground_param_spins.get("ground_test_base_duty")
            result = run_ground_bench_round(
                self._session,
                Path.cwd() / "logs",
                axis=axis,
                duration_s=5.0,
                base_duty=None if base_spin is None else float(base_spin.value()),
                auto_arm=False,
            )
            return {
                "path": result.output_dir,
                "safe_to_continue": result.summary.safe_to_continue,
                "next_action_hint": result.summary.next_action_hint,
            }

        self._run_session_action(f"ground_bench_{axis}", action)

    def _sync_udp_param_spins(self) -> None:
        if not getattr(self, "_params", None):
            return
        param_map = {item.name: item.value for item in self._params}
        if "udp_manual_max_pwm" in param_map:
            self.udp_max_pwm_spin.blockSignals(True)
            self.udp_max_pwm_spin.setValue(float(param_map["udp_manual_max_pwm"]) * 100.0)
            self.udp_max_pwm_spin.blockSignals(False)
            self._clamp_udp_throttle_spin()

    def _clamp_udp_throttle_spin(self) -> None:
        max_pwm = float(self.udp_max_pwm_spin.value())
        self.udp_throttle_spin.setMaximum(max_pwm)
        if self.udp_throttle_spin.value() > max_pwm:
            self.udp_throttle_spin.setValue(max_pwm)

    def _udp_setpoint_values(self) -> tuple[float, float, float, float]:
        throttle = min(float(self.udp_throttle_spin.value()), float(self.udp_max_pwm_spin.value())) / 100.0
        pitch = float(self.udp_pitch_spin.value()) / 100.0
        roll = float(self.udp_roll_spin.value()) / 100.0
        yaw = float(self.udp_yaw_spin.value()) / 100.0
        return throttle, pitch, roll, yaw

    def _apply_udp_manual_params(self) -> None:
        self._session.set_param("udp_manual_max_pwm", 4, float(self.udp_max_pwm_spin.value()) / 100.0)
        self._session.set_param("udp_manual_timeout_ms", 2, UDP_MANUAL_WATCHDOG_MS)

    def _enable_udp_manual(self) -> None:
        def action():
            self._session.require_udp_manual_control()
            self._apply_udp_manual_params()
            return ensure_command_ok(CmdId.UDP_MANUAL_ENABLE, int(self._session.udp_manual_enable()))

        self._run_session_action("udp_manual_enable", action)

    def _disable_udp_manual(self) -> None:
        def action():
            return ensure_command_ok(CmdId.UDP_MANUAL_DISABLE, int(self._session.udp_manual_disable()))

        self._udp_manual_enabled = False
        self._udp_control_timer.stop()
        self._run_session_action("udp_manual_disable", action)

    def _stop_udp_manual(self) -> None:
        self.udp_throttle_spin.setValue(0.0)
        self._zero_udp_axes(send=False)

        def action():
            return ensure_command_ok(CmdId.UDP_MANUAL_STOP, int(self._session.udp_manual_stop()))

        self._udp_manual_enabled = False
        self._udp_control_timer.stop()
        self._run_session_action("udp_manual_stop", action)

    def _takeoff_udp_manual(self) -> None:
        takeoff_percent = min(float(self.udp_max_pwm_spin.value()), 10.0)
        self.udp_throttle_spin.setValue(takeoff_percent)

        def action():
            self._session.require_udp_manual_control()
            self._apply_udp_manual_params()
            return ensure_command_ok(CmdId.UDP_TAKEOFF, int(self._session.udp_takeoff()))

        self._run_session_action("udp_takeoff", action)

    def _land_udp_manual(self) -> None:
        self.udp_throttle_spin.setValue(0.0)
        self._zero_udp_axes(send=False)
        self._udp_manual_enabled = False
        self._udp_control_timer.stop()

        def action():
            return ensure_command_ok(CmdId.UDP_LAND, int(self._session.udp_land()))

        self._run_session_action("udp_land", action)

    def _set_udp_motion(self, *, pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0) -> None:
        self.udp_pitch_spin.setValue(pitch)
        self.udp_roll_spin.setValue(roll)
        self.udp_yaw_spin.setValue(yaw)
        self._send_udp_manual_once()

    def _zero_udp_axes(self, send: bool = True) -> None:
        self.udp_pitch_spin.setValue(0.0)
        self.udp_roll_spin.setValue(0.0)
        self.udp_yaw_spin.setValue(0.0)
        if send:
            self._send_udp_manual_once()

    def _adjust_udp_throttle(self, delta_percent: float) -> None:
        value = float(self.udp_throttle_spin.value()) + float(delta_percent)
        value = max(0.0, min(float(self.udp_max_pwm_spin.value()), value))
        self.udp_throttle_spin.setValue(value)
        self._send_udp_manual_once()

    def _send_udp_manual_timer_setpoint(self) -> None:
        if self._udp_manual_enabled:
            self._send_udp_manual_once(from_timer=True)
        self._refresh_udp_watchdog_status()

    def _send_udp_manual_once(self, from_timer: bool = False) -> None:
        if not getattr(self._session, "is_connected", False):
            return
        if self._udp_manual_send_inflight:
            return
        throttle, pitch, roll, yaw = self._udp_setpoint_values()

        def action():
            return ensure_command_ok(
                CmdId.UDP_MANUAL_SETPOINT,
                int(
                    self._session.udp_manual_setpoint(
                        throttle=throttle,
                        pitch=pitch,
                        roll=roll,
                        yaw=yaw,
                        timeout=UDP_MANUAL_SETPOINT_TIMEOUT_S,
                    )
                ),
            )

        self._udp_manual_send_inflight = True
        self._udp_manual_last_send_monotonic = time.monotonic()
        label = "udp_manual_setpoint_timer" if from_timer else "udp_manual_setpoint"
        self._run_session_action(label, action)
        if not from_timer:
            self._append_log(self._t("udp.setpoint_sent", throttle=throttle, pitch=pitch, roll=roll, yaw=yaw))

    def _refresh_udp_watchdog_status(self) -> None:
        if self._udp_manual_last_send_monotonic is None:
            self.udp_watchdog_status_label.setText("-")
            return
        age_ms = (time.monotonic() - self._udp_manual_last_send_monotonic) * 1000.0
        self.udp_watchdog_status_label.setText(self._t("udp.watchdog_age", age_ms=age_ms))

    def _browse_log_path(self) -> None:
        output, _ = QFileDialog.getSaveFileName(self, self._t("label.output"), self.log_path_edit.text(), "CSV Files (*.csv)")
        if output:
            self.log_path_edit.setText(output)
            self._sync_log_path_tooltip()

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
        self.chart_toggle_button.setText(self._t("button.pause_charts") if self._charts_running else self._t("button.resume_charts"))
        self._append_log(self._t("msg.chart_resumed") if self._charts_running else self._t("msg.chart_paused"))

    def _clear_charts(self) -> None:
        self._history.clear()
        for curve in self._chart_curves.values():
            curve.setData([], [])
        self._append_log(self._t("msg.chart_cleared"))

    def _current_chart_window(self) -> float:
        return float(self.chart_window_combo.currentData() or 10.0)

    def _refresh_plots(self) -> None:
        if not self._charts_running:
            return
        window = self._current_chart_window()
        spec = self.CHART_GROUPS[self._current_chart_group]
        for field_name, _field_label in spec.fields:
            xs, ys = self._history.slice(field_name, window)
            curve = self._chart_curves.get(field_name)
            checkbox = self._chart_channel_checks.get(field_name)
            if curve is None:
                continue
            if checkbox is not None and checkbox.isChecked():
                curve.setData(xs, ys)
            else:
                curve.setData([], [])
        self.main_plot.setXRange(-window, 0.0, padding=0.01)

    def _refresh_enabled_state(self) -> None:
        connected = bool(getattr(self._session, "is_connected", False))
        connecting = bool(self._connecting and not connected)
        self.connect_button.setEnabled(not connected and not connecting)
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
            self.hang_capture_button,
            self.hang_start_button,
            self.hang_stop_button,
            self.ground_capture_button,
            self.ground_start_button,
            self.ground_stop_button,
            self.ground_record_5_button,
            self.ground_record_10_button,
            self.ground_bench_roll_button,
            self.ground_bench_pitch_button,
            self.ground_bench_yaw_button,
            self.ground_bench_all_button,
            self.udp_enable_button,
            self.udp_disable_button,
            self.udp_stop_button,
            self.udp_takeoff_button,
            self.udp_land_button,
            self.udp_forward_button,
            self.udp_backward_button,
            self.udp_yaw_left_button,
            self.udp_yaw_right_button,
            self.udp_up_button,
            self.udp_down_button,
            self.udp_send_button,
            self.start_log_button,
            self.stop_log_button,
            self.dump_csv_button,
        ):
            button.setEnabled(connected)

        self.link_type_combo.setEnabled(not connected and not connecting)
        self.transport_stack.setEnabled(not connected and not connecting)

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
            target_detail = self._connection_target_detail
            self._connecting = False
            self._connecting_detail = ""
            self._connect_watchdog_timer.stop()
            self._last_connect_error_message = None
            _set_badge(self.connection_status_chip, self._t("status.connected"), "ok")
            info_text = _device_info_text(info)
            self._set_connection_info_text(f"{target_detail}\n{info_text}" if target_detail else info_text)
            self._set_connection_error_text(self._t("status.no_conn_error"))
            self.last_error_label.setText(self._t("status.no_error"))
            if hasattr(info, "stream_enabled"):
                self._stream_enabled = bool(getattr(info, "stream_enabled"))
            self._append_log(self._t("msg.connected", info=_device_info_text(info)))
            self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))
        else:
            if error:
                self._show_connect_failure(error)
                return
            self._connecting = False
            self._connecting_detail = ""
            self._connect_watchdog_timer.stop()
            self._connection_target_detail = ""
            self._udp_manual_enabled = False
            if hasattr(self, "_udp_control_timer"):
                self._udp_control_timer.stop()
            _set_badge(self.connection_status_chip, self._t("status.disconnected"), "neutral" if not error else "warn")
            self._set_connection_info_text(self._t("status.no_session"))
            self._stream_enabled = False
            if not self._closing:
                self._set_connection_error_text(self._t("status.no_conn_error"))
                self.last_error_label.setText(self._t("status.no_error"))
                self._append_log(self._t("msg.disconnected"))
        self._update_stream_chip()
        self._refresh_enabled_state()

    def _update_stream_chip(self) -> None:
        if self._stream_enabled:
            _set_badge(self.status_cards["stream"][1], self._t("status.stream_on"), "active")
        else:
            _set_badge(self.status_cards["stream"][1], self._t("status.stream_off"), "neutral")

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
                    value_item.setText(f"{self._t(label)} [{value}]")
                self._apply_status_to_row(row, role)

        arm_text, arm_role = self._status_text(ARM_STATE_TEXT, sample.arm_state)
        failsafe_text, failsafe_role = self._status_text(FAILSAFE_TEXT, sample.failsafe_reason)
        control_text, control_role = self._status_text(CONTROL_MODE_TEXT, sample.control_mode)
        imu_text, imu_role = self._status_text(IMU_MODE_TEXT, sample.imu_mode)
        baro_text, baro_role = self._status_text(BARO_HEALTH_TEXT, sample.baro_health)
        _set_badge(self.status_cards["arm_state"][1], arm_text, arm_role)
        _set_badge(self.status_cards["failsafe_reason"][1], failsafe_text, failsafe_role)
        _set_badge(self.status_cards["control_mode"][1], control_text, control_role)
        _set_badge(self.status_cards["imu_mode"][1], imu_text, imu_role)
        self.status_cards["battery_voltage"][1].setText(f"{sample.battery_voltage:.3f} V")
        self.status_cards["imu_age_us"][1].setText(f"{sample.imu_age_us} us")
        self.status_cards["loop_dt_us"][1].setText(f"{sample.loop_dt_us} us")
        _set_badge(self.status_cards["baro_health"][1], baro_text, baro_role)
        self.status_cards["baro_altitude_m"][1].setText(
            f"{sample.baro_altitude_m:.3f} m" if sample.baro_valid else self._t("status.no_value")
        )
        self.udp_mode_status_label.setText(control_text)
        self.udp_armed_status_label.setText(arm_text)
        self.udp_battery_status_label.setText(f"{sample.battery_voltage:.3f} V")
        if hasattr(self, "ground_ref_status_label"):
            self.ground_ref_status_label.setText("valid" if sample.ground_ref_valid else "missing")
            self.ground_kalman_status_label.setText("valid" if sample.kalman_valid else "invalid")
            self.ground_trip_status_label.setText(str(sample.ground_trip_reason))
        self._refresh_udp_watchdog_status()
        self._update_stream_chip()

    def _on_event_received(self, message: str) -> None:
        self._append_log(message)

    def _on_error(self, message: str) -> None:
        if "udp_manual_setpoint" in message:
            self._udp_manual_send_inflight = False
        if self._connecting or message.startswith("Connect failed:") or message.startswith("connect_serial:") or message.startswith("connect_udp:"):
            self._show_connect_failure(message)
            return
        self.last_error_label.setText(message)
        self._set_last_result(message)
        self._append_log(message)

    def _on_command_finished(self, label: str, result: object) -> None:
        if label == "stream_on":
            self._stream_enabled = True
            self._update_stream_chip()
            self._set_last_result(self._t("status.stream_on"))
            self._append_log(self._t("status.stream_on"))
            return
        if label == "stream_off":
            self._stream_enabled = False
            self._update_stream_chip()
            self._set_last_result(self._t("status.stream_off"))
            self._append_log(self._t("status.stream_off"))
            return
        if label == "list_params" and isinstance(result, list):
            self._populate_params_table(result)
            self._set_last_result(self._t("msg.loaded_params", count=len(result)))
            self._append_log(self._t("msg.loaded_params", count=len(result)))
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
            self._set_last_result(self._t("msg.param_updated", name=result.name, value=result.value))
            self._append_log(self._t("msg.param_updated", name=result.name, value=result.value))
            return
        if label == "export_params":
            self._set_last_result(self._t("msg.export_done"))
            self._append_log(self._t("msg.export_done"))
            return
        if label == "import_params":
            applied = len(result or [])
            self._set_last_result(self._t("msg.import_done", count=applied))
            self._append_log(self._t("msg.import_done", count=applied))
            self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))
            return
        if label == "start_csv_log":
            path = Path(result) if result else Path(self.log_path_edit.text().strip())
            self.last_log_path_label.setText(str(path))
            self._set_last_result(self._t("msg.logging_to", path=path))
            self._append_log(self._t("msg.logging_to", path=path))
            return
        if label == "stop_csv_log":
            path_text = self._t("status.no_log") if result is None else str(result)
            self.last_log_path_label.setText(path_text)
            self._set_last_result(self._t("msg.log_stopped", path=path_text))
            self._append_log(self._t("msg.log_stopped", path=path_text))
            return
        if label == "dump_csv" and isinstance(result, dict):
            path = result.get("path")
            rows = result.get("rows")
            self.last_log_path_label.setText(str(path))
            self._set_last_result(self._t("msg.dump_csv_done", rows=rows, path=path))
            self._append_log(self._t("msg.dump_csv_done", rows=rows, path=path))
            return
        if label.startswith("ground_bench_") and isinstance(result, dict):
            path = result.get("path")
            safe = result.get("safe_to_continue")
            hint = result.get("next_action_hint")
            self.last_log_path_label.setText(str(path))
            summary = f"ground bench saved={path} safe_to_continue={safe} next_action_hint={hint}"
            self._set_last_result(summary)
            self._append_log(summary)
            return
        if label == "udp_manual_enable":
            self._udp_manual_enabled = True
            self._udp_control_timer.start()
            self._set_last_result(self._t("udp.enabled"))
            self._append_log(self._t("udp.enabled"))
            self._send_udp_manual_once()
            return
        if label == "udp_manual_disable":
            self._udp_manual_enabled = False
            self._udp_control_timer.stop()
            self._set_last_result(self._t("udp.disabled"))
            self._append_log(self._t("udp.disabled"))
            return
        if label == "udp_manual_stop":
            self._udp_manual_enabled = False
            self._udp_control_timer.stop()
            self._set_last_result(self._t("udp.stopped"))
            self._append_log(self._t("udp.stopped"))
            return
        if label == "udp_takeoff":
            self._udp_manual_enabled = True
            self._udp_control_timer.start()
            summary = self._t("msg.command_ok", label=self._t("udp.takeoff"))
            self._set_last_result(summary)
            self._append_log(summary)
            self._send_udp_manual_once()
            return
        if label == "udp_land":
            self._udp_manual_enabled = False
            self._udp_control_timer.stop()
            summary = self._t("msg.command_ok", label=self._t("udp.land"))
            self._set_last_result(summary)
            self._append_log(summary)
            return
        if label in {"udp_manual_setpoint", "udp_manual_setpoint_timer"}:
            self._udp_manual_send_inflight = False
            if label == "udp_manual_setpoint":
                self._set_last_result(self._t("msg.command_ok", label=self._t("udp.send")))
            return
        if label in {"connect_serial", "connect_udp", "disconnect", "arm", "disarm", "kill", "reboot", "save_params", "reset_params", "motor_test_start", "motor_test_stop", "calib_gyro", "calib_level", "rate_test_start", "rate_test_stop", "attitude_capture_ref", "attitude_test_start", "attitude_test_stop", "ground_capture_ref", "ground_test_start", "ground_test_stop"}:
            summary = label.replace("_", " ")
            if label == "connect_serial":
                summary = f"serial: {_device_info_text(result)}"
            elif label == "connect_udp":
                summary = f"udp: {_device_info_text(result)}"
            elif label == "disconnect":
                summary = self._t("msg.disconnect_done")
            elif label == "arm":
                summary = self._t("msg.command_ok", label=self._t("button.arm"))
            elif label == "disarm":
                summary = self._t("msg.command_ok", label=self._t("button.disarm"))
            elif label == "kill":
                summary = self._t("msg.command_ok", label=self._t("button.kill"))
            elif label == "reboot":
                summary = self._t("msg.command_ok", label=self._t("button.reboot"))
            elif label == "save_params":
                summary = self._t("msg.command_ok", label=self._t("button.save"))
            elif label == "reset_params":
                summary = self._t("msg.command_ok", label=self._t("button.reset"))
            elif label == "motor_test_start":
                summary = self._t("msg.command_ok", label=f"{self._t('group.motor')} {self._t('button.start')}")
            elif label == "motor_test_stop":
                summary = self._t("msg.command_ok", label=f"{self._t('group.motor')} {self._t('button.stop')}")
            elif label == "calib_gyro":
                summary = self._t("msg.command_ok", label=self.calib_gyro_button.text())
            elif label == "calib_level":
                summary = self._t("msg.command_ok", label=self.calib_level_button.text())
            elif label == "rate_test_start":
                summary = self._t("msg.command_ok", label=f"{self._t('group.rate')} {self._t('button.start')}")
            elif label == "rate_test_stop":
                summary = self._t("msg.command_ok", label=f"{self._t('group.rate')} {self._t('button.stop')}")
            elif label == "attitude_capture_ref":
                summary = self._t("msg.command_ok", label=self.hang_capture_button.text())
            elif label == "attitude_test_start":
                summary = self._t("msg.command_ok", label=self.hang_start_button.text())
            elif label == "attitude_test_stop":
                summary = self._t("msg.command_ok", label=self.hang_stop_button.text())
            elif label == "ground_capture_ref":
                summary = self._t("msg.command_ok", label=self.ground_capture_button.text())
            elif label == "ground_test_start":
                summary = self._t("msg.command_ok", label=self.ground_start_button.text())
            elif label == "ground_test_stop":
                summary = self._t("msg.command_ok", label=self.ground_stop_button.text())
            self._set_last_result(summary)
            self._append_log(summary)
            return

        self._set_last_result(self._t("msg.command_ok", label=label))
        self._append_log(self._t("msg.command_ok", label=label))

    def _save_settings(self) -> None:
        self._settings.setValue("window/geometry", self.saveGeometry())
        self._settings.setValue("window/state", self.saveState())
        self._settings.setValue("splitter/main", self.main_splitter.saveState())
        self._settings.setValue("splitter/center", self.center_splitter.saveState())
        self._settings.setValue("splitter/right", self.right_splitter.saveState())
        self._settings.setValue("splitter/params", self.params_splitter.saveState())
        self._settings.setValue("ui/language", self._language)
        self._settings.setValue("link/type", self.link_type_combo.currentData() or self.link_type_combo.currentText())
        self._settings.setValue("serial/port", self.serial_port_combo.currentText())
        self._settings.setValue("serial/baudrate", self.baudrate_spin.value())
        self._settings.setValue("udp/host", self.udp_host_edit.text())
        self._settings.setValue("udp/port", self.udp_port_spin.value())
        self._settings.setValue("chart/window_index", self.chart_window_combo.currentIndex())
        self._settings.setValue("chart/group", self._current_chart_group)
        self._settings.setValue("params/search", self.param_search_edit.text())
        self._settings.setValue("log/path", self.log_path_edit.text())
        self._settings.setValue("section/connection", self.connection_section.is_expanded())
        self._settings.setValue("section/safety", self.safety_section.is_expanded())
        self._settings.setValue("debug/action_index", self.debug_action_tabs.currentIndex())

    def _load_settings(self) -> None:
        geometry = self._settings.value("window/geometry", type=QByteArray)
        state = self._settings.value("window/state", type=QByteArray)
        main_splitter_state = self._settings.value("splitter/main", type=QByteArray)
        center_splitter_state = self._settings.value("splitter/center", type=QByteArray)
        right_splitter_state = self._settings.value("splitter/right", type=QByteArray)
        params_splitter_state = self._settings.value("splitter/params", type=QByteArray)
        if geometry:
            self.restoreGeometry(geometry)
        if state:
            self.restoreState(state)
        language = str(self._settings.value("ui/language", self._language))
        if language in {"zh", "en"}:
            self._language = language
            self._apply_language()

        link_type = self._settings.value("link/type", "serial")
        serial_port = self._settings.value("serial/port", "")
        baudrate = int(self._settings.value("serial/baudrate", 115200))
        udp_host = self._settings.value("udp/host", "192.168.4.1")
        udp_port = int(self._settings.value("udp/port", 2391))
        chart_index = int(self._settings.value("chart/window_index", 1))
        chart_group = str(self._settings.value("chart/group", self._current_chart_group))
        param_search = self._settings.value("params/search", "")
        log_path = self._settings.value("log/path", self.log_path_edit.text())
        section_connection = bool(self._settings.value("section/connection", True, type=bool))
        section_safety = bool(self._settings.value("section/safety", True, type=bool))
        debug_action_index = int(self._settings.value("debug/action_index", 0))

        link_index = self.link_type_combo.findData(str(link_type))
        self.link_type_combo.setCurrentIndex(link_index if link_index >= 0 else 0)
        self.serial_port_combo.setCurrentText(str(serial_port))
        self.baudrate_spin.setValue(baudrate)
        self.udp_host_edit.setText(str(udp_host))
        self.udp_port_spin.setValue(udp_port)
        self.chart_window_combo.setCurrentIndex(max(0, min(chart_index, self.chart_window_combo.count() - 1)))
        chart_group_index = self.chart_group_combo.findData(chart_group)
        self.chart_group_combo.setCurrentIndex(chart_group_index if chart_group_index >= 0 else 0)
        self.param_search_edit.setText(str(param_search))
        self.log_path_edit.setText(str(log_path))
        self._sync_log_path_tooltip()
        self.connection_section.set_expanded(section_connection)
        self.safety_section.set_expanded(section_safety)
        self.debug_action_tabs.setCurrentIndex(max(0, min(debug_action_index, self.debug_action_tabs.count() - 1)))
        if main_splitter_state:
            self.main_splitter.restoreState(main_splitter_state)
        else:
            self._apply_default_main_splitter_sizes()
        self._normalize_main_splitter_sizes()
        if center_splitter_state:
            self.center_splitter.restoreState(center_splitter_state)
        if right_splitter_state:
            self.right_splitter.restoreState(right_splitter_state)
        if params_splitter_state:
            self.params_splitter.restoreState(params_splitter_state)
        self._update_link_inputs()
        self._rebuild_chart_channels()

    def closeEvent(self, event) -> None:  # pragma: no cover - exercised by smoke tests
        """关闭窗口前保存状态并释放设备会话。

        Args:
            event: Qt 关闭事件对象。
        """

        self._closing = True
        self._plot_timer.stop()
        if hasattr(self, "_connect_watchdog_timer"):
            self._connect_watchdog_timer.stop()
        if hasattr(self, "_udp_control_timer"):
            self._udp_control_timer.stop()
        self._save_settings()
        try:
            self._session.disconnect()
        finally:
            self._bridge.dispose()
        super().closeEvent(event)


def run_gui(_argv: list[str] | None = None) -> int:
    """启动 GUI 应用并进入 Qt 事件循环。

    Args:
        _argv: 保留参数，当前实现未使用。

    Returns:
        Qt 应用退出码。
    """

    app = QApplication.instance() or QApplication([])
    app.setFont(QFont("Microsoft YaHei", 10))
    pg.setConfigOptions(antialias=True, foreground="#dbeafe", background="#0f1722")
    window = MainWindow()
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(run_gui())
