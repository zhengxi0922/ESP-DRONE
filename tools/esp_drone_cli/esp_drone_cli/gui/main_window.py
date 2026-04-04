from __future__ import annotations

import os
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

from esp_drone_cli.core import DeviceSession, ParamValue, TelemetrySample


APP_ORG = "ESP-DRONE"
APP_NAME = "ESP-DRONE GUI"

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
        "tab.params": "参数",
        "tab.events": "日志",
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
    "imu_age_us": "field.imu_age_us",
    "loop_dt_us": "field.loop_dt_us",
    "imu_mode": "field.imu_mode",
    "imu_health": "field.imu_health",
    "arm_state": "field.arm_state",
    "failsafe_reason": "field.failsafe_reason",
    "control_mode": "field.control_mode",
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
}

IMU_MODE_TEXT = {
    0: ("imu.raw", "warn"),
    1: ("imu.direct", "ok"),
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


class CollapsibleSection(QWidget):
    toggled = pyqtSignal(bool)

    def __init__(self, title: str = "", expanded: bool = True, parent: QWidget | None = None) -> None:
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
        self.toggle_button.setText(title)

    def set_content(self, widget: QWidget) -> None:
        while self.body_layout.count():
            item = self.body_layout.takeAt(0)
            old = item.widget()
            if old is not None:
                old.setParent(None)
        self.body_layout.addWidget(widget)

    def is_expanded(self) -> bool:
        return self._expanded

    def set_expanded(self, expanded: bool) -> None:
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

    CHART_GROUPS = {
        "gyro": ChartSpec("chart.gyro", [("gyro_x", "field.gyro_x"), ("gyro_y", "field.gyro_y"), ("gyro_z", "field.gyro_z")], "chart.y_deg_s"),
        "attitude": ChartSpec("chart.attitude", [("roll_deg", "field.roll_deg"), ("pitch_deg", "field.pitch_deg"), ("yaw_deg", "field.yaw_deg")], "chart.y_deg"),
        "motors": ChartSpec("chart.motors", [("motor1", "field.motor1"), ("motor2", "field.motor2"), ("motor3", "field.motor3"), ("motor4", "field.motor4")], "chart.y_duty"),
        "battery": ChartSpec("chart.battery", [("battery_voltage", "field.battery_voltage")], "chart.y_volt"),
    }

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
        self._language = "zh"
        self._current_chart_group = "gyro"
        self._chart_curves: dict[str, object] = {}
        self._chart_channel_checks: dict[str, QCheckBox] = {}

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

    def _t(self, key: str, **kwargs) -> str:
        table = TRANSLATIONS.get(self._language, TRANSLATIONS["zh"])
        text = table.get(key, key)
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
                font-family: "Microsoft YaHei";
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
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setMinimumWidth(320)
        scroll.setMaximumWidth(420)
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
        self.right_tabs = QTabWidget()
        self.right_tabs.setDocumentMode(True)
        self.right_tabs.addTab(self.params_group, "")
        self.right_tabs.addTab(self.bottom_panel, "")
        self.right_tabs.setCurrentIndex(0)
        self.right_splitter.addWidget(self.status_group)
        self.right_splitter.addWidget(self.right_tabs)
        self.right_splitter.setStretchFactor(0, 0)
        self.right_splitter.setStretchFactor(1, 1)
        self.right_splitter.setSizes([152, 698])
        layout.addWidget(self.right_splitter)
        return panel

    def _apply_default_main_splitter_sizes(self) -> None:
        self.main_splitter.setSizes([340, 980, 360])

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
        udp_form.setColumnStretch(1, 1)
        udp_form.addWidget(self.udp_host_label, 0, 0)
        udp_form.addWidget(self.udp_host_edit, 0, 1)
        udp_form.addWidget(self.udp_port_label, 1, 0)
        udp_form.addWidget(self.udp_port_spin, 1, 1)
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

        motor_box = QWidget()
        motor_layout = QGridLayout(motor_box)
        motor_layout.setColumnStretch(1, 1)
        motor_layout.setColumnStretch(3, 1)
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
        motor_layout.addWidget(self.motor_start_button, 0, 4)
        motor_layout.addWidget(self.motor_stop_button, 0, 5)
        self.motor_section = CollapsibleSection(expanded=True)
        self.motor_section.set_content(motor_box)
        layout.addWidget(self.motor_section)

        calib_box = QWidget()
        calib_layout = QHBoxLayout(calib_box)
        self.calib_group = calib_box
        self.calib_gyro_button = QPushButton()
        self.calib_level_button = QPushButton()
        self.calib_gyro_button.setMaximumWidth(84)
        self.calib_level_button.setMaximumWidth(84)
        calib_layout.addWidget(self.calib_gyro_button)
        calib_layout.addWidget(self.calib_level_button)
        calib_layout.addStretch(1)
        self.calib_section = CollapsibleSection(expanded=False)
        self.calib_section.set_content(calib_box)
        layout.addWidget(self.calib_section)

        rate_box = QWidget()
        rate_layout = QGridLayout(rate_box)
        rate_layout.setColumnStretch(1, 1)
        rate_layout.setColumnStretch(3, 1)
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
        rate_layout.addWidget(self.rate_start_button, 0, 4)
        rate_layout.addWidget(self.rate_stop_button, 0, 5)
        self.rate_section = CollapsibleSection(expanded=False)
        self.rate_section.set_content(rate_box)
        layout.addWidget(self.rate_section)

        log_box = QWidget()
        log_layout = QGridLayout(log_box)
        log_layout.setColumnStretch(1, 1)
        self.log_path_edit = QLineEdit(str(Path.cwd() / "telemetry.csv"))
        self.csv_group = log_box
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
        log_layout.addWidget(self.output_label, 0, 0)
        log_layout.addWidget(self.log_path_edit, 0, 1, 1, 4)
        log_layout.addWidget(self.log_browse_button, 0, 5)
        log_layout.addWidget(self.start_log_button, 1, 0)
        log_layout.addWidget(self.stop_log_button, 1, 1)
        log_layout.addWidget(self.dump_s_label, 1, 2)
        log_layout.addWidget(self.dump_duration_spin, 1, 3)
        log_layout.addWidget(self.dump_csv_button, 1, 4)
        self.csv_section = CollapsibleSection(expanded=False)
        self.csv_section.set_content(log_box)
        layout.addWidget(self.csv_section)

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
        self.motor_section.toggled.connect(lambda *_args: self._save_settings())
        self.calib_section.toggled.connect(lambda *_args: self._save_settings())
        self.rate_section.toggled.connect(lambda *_args: self._save_settings())
        self.csv_section.toggled.connect(lambda *_args: self._save_settings())
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
        self.calib_gyro_button.clicked.connect(lambda: self._run_session_action("calib_gyro", self._session.calib_gyro))
        self.calib_level_button.clicked.connect(lambda: self._run_session_action("calib_level", self._session.calib_level))
        self.rate_start_button.clicked.connect(self._start_rate_test)
        self.rate_stop_button.clicked.connect(self._stop_rate_test)
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

    def _set_last_result(self, text: str) -> None:
        self._last_result = text
        self.last_result_label.setText(text)

    def _sync_log_path_tooltip(self) -> None:
        self.log_path_edit.setToolTip(self.log_path_edit.text().strip())

    def _append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.event_log_edit.appendPlainText(f"[{stamp}] {text}")

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
        self.motor_section.set_title(self._t("group.motor"))
        self.calib_section.set_title(self._t("group.calib"))
        self.rate_section.set_title(self._t("group.rate"))
        self.csv_section.set_title(self._t("group.csv"))

        self.link_type_label.setText(self._t("label.link"))
        self.serial_port_label.setText(self._t("label.serial_port"))
        self.baud_label.setText(self._t("label.baud"))
        self.udp_host_label.setText(self._t("label.udp_host"))
        self.udp_port_label.setText(self._t("label.udp_port"))
        self.session_title_label.setText(self._t("label.session"))
        self.last_conn_error_title_label.setText(self._t("label.last_conn_error"))
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
        self.output_label.setText(self._t("label.output"))
        self.dump_s_label.setText(self._t("label.dump_s"))
        self.log_browse_button.setText(self._t("button.browse"))
        self.start_log_button.setText(self._t("button.start_log"))
        self.stop_log_button.setText(self._t("button.stop_log"))
        self.dump_csv_button.setText(self._t("button.dump_csv"))

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
        self.params_table.setHorizontalHeaderLabels([self._t("label.name"), self._t("label.type"), self._t("label.current")])
        self.param_name_title_label.setText(self._t("label.name"))
        self.param_type_title_label.setText(self._t("label.type"))
        self.param_current_title_label.setText(self._t("label.current"))
        self.param_new_value_title_label.setText(self._t("label.new_value"))
        self.param_new_value_edit.setPlaceholderText(self._t("placeholder.new_value"))
        self.param_desc_title_label.setText(self._t("label.description"))
        self.param_help_text.setPlaceholderText(self._t("placeholder.desc"))
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
                self._on_error(self._t("msg.connect_no_port"))
                return
            baudrate = int(self.baudrate_spin.value())
            self._run_session_action(
                "connect_serial",
                lambda: self._session.connect_serial(port, baudrate=baudrate, timeout=0.2),
            )
            return

        host = self.udp_host_edit.text().strip()
        if not host:
            self._on_error(self._t("msg.connect_no_host"))
            return
        port = int(self.udp_port_spin.value())
        self._run_session_action(
            "connect_udp",
            lambda: self._session.connect_udp(host, port=port, timeout=1.0),
        )

    def _disconnect_requested(self) -> None:
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
        self.param_help_text.setPlainText(self.PARAM_HELP.get(self._selected_param.name, self._t("param.help.default")))
        self._update_param_hint()

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
        return self.PARAM_HELP.get(name, self._t("param.help.default"))

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

        self.link_type_combo.setEnabled(not connected)
        self.transport_stack.setEnabled(not connected)

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
            _set_badge(self.connection_status_chip, self._t("status.connected"), "ok")
            self.connection_info_label.setText(_device_info_text(info))
            self.connection_error_detail.setText(self._t("status.no_conn_error"))
            self.last_error_label.setText(self._t("status.no_error"))
            if hasattr(info, "stream_enabled"):
                self._stream_enabled = bool(getattr(info, "stream_enabled"))
            self._append_log(self._t("msg.connected", info=_device_info_text(info)))
            self._run_session_action("list_params", lambda: self._session.list_params(timeout=3.0))
        else:
            _set_badge(self.connection_status_chip, self._t("status.disconnected"), "neutral" if not error else "warn")
            self.connection_info_label.setText(self._t("status.no_session"))
            self._stream_enabled = False
            if error:
                message = str(error)
                self.connection_error_detail.setText(message)
                self.last_error_label.setText(message)
                self._append_log(message)
            elif not self._closing:
                self.connection_error_detail.setText(self._t("status.no_conn_error"))
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
        _set_badge(self.status_cards["arm_state"][1], arm_text, arm_role)
        _set_badge(self.status_cards["failsafe_reason"][1], failsafe_text, failsafe_role)
        _set_badge(self.status_cards["control_mode"][1], control_text, control_role)
        _set_badge(self.status_cards["imu_mode"][1], imu_text, imu_role)
        self.status_cards["battery_voltage"][1].setText(f"{sample.battery_voltage:.3f} V")
        self.status_cards["imu_age_us"][1].setText(f"{sample.imu_age_us} us")
        self.status_cards["loop_dt_us"][1].setText(f"{sample.loop_dt_us} us")
        self._update_stream_chip()

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
        if label in {"connect_serial", "connect_udp", "disconnect", "arm", "disarm", "kill", "reboot", "save_params", "reset_params", "motor_test_start", "motor_test_stop", "calib_gyro", "calib_level", "rate_test_start", "rate_test_stop"}:
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
        self._settings.setValue("section/motor", self.motor_section.is_expanded())
        self._settings.setValue("section/calib", self.calib_section.is_expanded())
        self._settings.setValue("section/rate", self.rate_section.is_expanded())
        self._settings.setValue("section/csv", self.csv_section.is_expanded())

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
        section_motor = bool(self._settings.value("section/motor", True, type=bool))
        section_calib = bool(self._settings.value("section/calib", False, type=bool))
        section_rate = bool(self._settings.value("section/rate", False, type=bool))
        section_csv = bool(self._settings.value("section/csv", False, type=bool))

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
        self.motor_section.set_expanded(section_motor)
        self.calib_section.set_expanded(section_calib)
        self.rate_section.set_expanded(section_rate)
        self.csv_section.set_expanded(section_csv)
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
    app.setFont(QFont("Microsoft YaHei", 10))
    pg.setConfigOptions(antialias=True, foreground="#dbeafe", background="#0f1722")
    window = MainWindow()
    window.show()
    return int(app.exec())
