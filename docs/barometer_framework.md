# Barometer Framework

## 中文说明

### 当前阶段目标

当前仓库只实现以下内容：

- 读取 `ATK-MS901M` 的气压计帧
- 在 firmware 内保存并输出气压计状态
- 通过 USB CDC telemetry 把气压计字段发到主机
- 让 Python CLI / GUI 能实时查看和导出这些字段
- 预留后续“基于气压计的定高闭环”所需的数据结构和模块边界

当前**没有**实现：

- 定高 PID
- 油门闭环
- 自动起降
- 气压计参与现有姿态 / 速率 / mixer / 电机输出逻辑

### 模块边界

当前新增的 firmware 边界如下：

- `firmware/main/barometer/barometer.h`
- `firmware/main/barometer/barometer.c`

当前 API 只负责：

- `barometer_init()`
- `barometer_update_from_module_frame()`
- `barometer_get_latest()`
- `barometer_get_altitude_hold_reserved_state()`

也就是说，barometer 模块现在只是“状态保存 + 简单速度估计 + telemetry 出口”，不是控制器的一部分。

### 设备 telemetry 字段

当前追加到 telemetry 末尾的字段如下：

- `baro_pressure_pa`
- `baro_temperature_c`
- `baro_altitude_m`
- `baro_vspeed_mps`
- `baro_update_age_us`
- `baro_valid`
- `baro_health`

其中：

- `baro_altitude_m` 当前优先使用模块 baro 帧直接提供的高度值（由 `cm` 转成 `m`）
- `baro_vspeed_mps` 当前是基于相邻高度样本的简单差分估计
- `baro_health` 当前只表示 `INIT / OK / STALE / INVALID`

### Future altitude hold 预留

`firmware/main/esp_drone_types.h` 里已经新增：

- `barometer_state_t`
- `altitude_hold_reserved_state_t`
- `CONTROL_MODE_HEIGHT_HOLD_RESERVED`

但这些预留目前**不启用闭环**。

后续若要继续做定高，建议从这些位置扩展：

1. `barometer.c`
   - 继续完善滤波和高度 / 速度估计
2. `estimator/`
   - 增加高度估计器，统一融合 baro 与其他高度源
3. `controller/`
   - 增加 altitude outer loop
4. `safety/`
   - 增加高度模式切换条件与异常保护
5. `console / python core / GUI`
   - 增加 altitude hold setpoint / status 的可观测字段

### CLI 查看方法

常规 telemetry：

```powershell
esp-drone-cli --serial COM4 log --telemetry --timeout 5
```

直接查看 baro：

```powershell
esp-drone-cli --serial COM4 baro --timeout 5
```

或：

```powershell
esp-drone-cli --serial COM4 watch-baro --timeout 5
```

CSV 导出：

```powershell
esp-drone-cli --serial COM4 dump-csv telemetry.csv --duration 5
```

### GUI 查看位置

GUI 中当前可以在这些位置看到气压计数据：

- 实时数值表：`baro_*` 字段
- 关键状态卡片：`baro_health`、`baro_altitude_m`
- 图表组：`Barometer`

## English Notes

This stage only adds a barometer data path and a future altitude-hold scaffold.

Implemented now:

- decode ATK-MS901M barometer frames
- store filtered barometer state in firmware
- expose barometer fields through telemetry
- display/export those fields in CLI and GUI
- reserve future altitude-hold data structures

Not implemented now:

- altitude hold control loop
- throttle closed loop
- auto takeoff / landing
- barometer influence on the current attitude / rate control path
