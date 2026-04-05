# 气压计框架

**语言 / Language：** [English](./barometer_framework.md) | **简体中文**

## 当前范围

当前阶段只接入气压计数据链路，并预留未来定高能力的骨架。

已经实现：

- 解析 `ATK-MS901M` 的气压计帧
- 在固件中保存气压计状态
- 通过 telemetry 输出气压计字段
- 在 CLI 和 GUI 中查看并导出这些字段
- 预留后续定高模式所需的数据结构

当前未实现：

- 定高控制闭环
- 油门闭环
- 自动起飞或降落
- 将气压计数据接入现有姿态、速率、mixer 或电机输出路径

## 模块边界

当前固件中的边界是：

- `firmware/main/barometer/barometer.h`
- `firmware/main/barometer/barometer.c`

当前 API 只负责：

- `barometer_init()`
- `barometer_update_from_module_frame()`
- `barometer_get_latest()`
- `barometer_get_altitude_hold_reserved_state()`

也就是说，barometer 模块目前只是状态保存、简单垂向速度估计和 telemetry 出口，还不是控制器的一部分。

## 设备 Telemetry 字段

当前追加到 telemetry 末尾的字段包括：

- `baro_pressure_pa`
- `baro_temperature_c`
- `baro_altitude_m`
- `baro_vspeed_mps`
- `baro_update_age_us`
- `baro_valid`
- `baro_health`

当前行为为：

- `baro_altitude_m` 优先使用模块气压计帧直接给出的高度值，并从 `cm` 转成 `m`
- `baro_vspeed_mps` 当前是相邻高度样本的一阶差分估计
- `baro_health` 当前仅区分 `INIT`、`OK`、`STALE` 和 `INVALID`

## Future Altitude Hold 预留

`firmware/main/esp_drone_types.h` 已经增加：

- `barometer_state_t`
- `altitude_hold_reserved_state_t`
- `CONTROL_MODE_HEIGHT_HOLD_RESERVED`

这些定义本身并不会启用控制闭环。

如果后续要真正实现定高，推荐按以下位置扩展：

1. `barometer.c`
   继续完善滤波和高度 / 垂向速度估计。
2. `estimator/`
   增加高度估计器，用于融合气压计与其它高度源。
3. `controller/`
   增加高度外环。
4. `safety/`
   增加高度模式切换条件和异常保护策略。
5. `console`、Python core 和 GUI
   增加定高设定值与状态字段的观测能力。

## CLI 查看方式

常规 telemetry：

```powershell
esp-drone-cli --serial COM4 log --telemetry --timeout 5
```

直接查看 barometer：

```powershell
esp-drone-cli --serial COM4 baro --timeout 5
```

或者：

```powershell
esp-drone-cli --serial COM4 watch-baro --timeout 5
```

CSV 导出：

```powershell
esp-drone-cli --serial COM4 dump-csv telemetry.csv --duration 5
```

## GUI 展示位置

GUI 当前会在以下位置展示气压计数据：

- 实时 telemetry 表中的 `baro_*` 字段
- 关键状态卡片中的 `baro_health` 和 `baro_altitude_m`
- `Barometer` 图表组
