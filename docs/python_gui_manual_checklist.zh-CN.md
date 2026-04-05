# Python GUI 人工检查清单

**语言 / Language：** [English](./python_gui_manual_checklist.md) | **简体中文**

请在完成以下安装后使用这份清单验收 `esp-drone-gui`：

```powershell
cd tools\esp_drone_cli
pip install -e .[gui]
```

台架前提：

- 已拆桨或机体受限固定
- 固件已刷写
- USB CDC 或 UDP 端点可用

## 启动

- [ ] `esp-drone-gui` 能正常启动
- [ ] 关闭窗口时不会卡死
- [ ] 如果缺少 `PyQt5` 或 `pyqtgraph`，GUI 会给出明确安装错误，而不是含糊崩溃
- [ ] 主窗口使用新的三列布局，而不是旧的堆叠 tab
- [ ] 中央图表区域明显大于控制侧栏和参数编辑区
- [ ] 底部日志默认以紧凑形态启动，并可展开
- [ ] 语言切换默认是中文，并能切到英文

## 连接

- [ ] serial 模式可以列出 COM 端口，或接受手动输入
- [ ] serial 模式只显示 serial 控件
- [ ] UDP 模式只显示 host 和 port 控件
- [ ] serial 连接成功
- [ ] UDP 模式可以连接到 `host:port`
- [ ] disconnect 后 GUI 回到 `Disconnected`
- [ ] 连接失败时，最近连接错误字段可读且有意义

## Stream 与 Telemetry

- [ ] `Stream On` 会启动 telemetry 更新
- [ ] `Stream Off` 会停止 telemetry 更新，或能明确停止新样本进入
- [ ] telemetry 表会刷新 gyro、姿态、电机、电池和循环时序字段
- [ ] telemetry 表选区支持 `Ctrl+C` 复制
- [ ] 修改目标 stream 频率不会破坏当前会话
- [ ] 图表区和 telemetry 表可同时可见

## 图表

- [ ] streaming 时 gyro 图会更新
- [ ] streaming 时 attitude 图会更新
- [ ] streaming 时电机图会更新
- [ ] streaming 时电池图会更新
- [ ] 主图足够大，不需要切 tab 就能看波形
- [ ] pause / resume 生效
- [ ] 清空图表历史生效
- [ ] 图表组切换生效
- [ ] 图表时间窗切换生效
- [ ] 通道可见性勾选框生效
- [ ] auto scale 生效
- [ ] reset view 生效

## 参数

- [ ] `Refresh` 可以加载参数列表
- [ ] 搜索框能过滤列表
- [ ] 参数表可显示足够多的行，满足实际台架调参
- [ ] 选中单个参数后会填充紧凑详情区
- [ ] 编辑值时，本地 hint 文本会更新
- [ ] 设置单个参数时不会出现 GUI 错误
- [ ] `Save` 成功
- [ ] `Reset` 成功
- [ ] `Export JSON` 会写出快照文件
- [ ] `Import JSON` 会应用快照文件
- [ ] 参数表选区支持复制

## 安全命令

- [ ] `Arm` 能到达 session 层并返回结果
- [ ] `Disarm` 能到达 session 层并返回结果
- [ ] `Kill` 能到达 session 层并返回结果
- [ ] `Kill` 仍是最醒目的危险按钮
- [ ] `Reboot` 能到达 session 层并返回结果

## 调试动作

- [ ] `motor_test` 能启动所选电机命令
- [ ] 停止 `motor_test` 会发送零 duty
- [ ] `calib gyro` 成功
- [ ] `calib level` 成功
- [ ] `rate_test` 会发送所选轴和值
- [ ] 停止 `rate_test` 会发送零 rate

## 日志

- [ ] `Start Log` 会创建或打开所选 CSV 路径
- [ ] `Stop Log` 会停止当前 CSV logging
- [ ] `Dump CSV` 会按要求时长写出 CSV 文件
- [ ] `Last log` 会更新为最近输出路径
- [ ] 在正常会话中 `Last error` 保持清晰
- [ ] 事件日志支持清空、复制和保存为文本
- [ ] 事件日志支持展开和折叠，且不会破坏布局

## 状态持久化

- [ ] 重启后会恢复上次 serial 端口
- [ ] 重启后会恢复上次 UDP 目标
- [ ] 重启后会恢复窗口几何信息
- [ ] 重启后会恢复图表组选择
- [ ] 重启后会恢复图表时间窗选择
- [ ] 重启后会恢复上次参数搜索文本
- [ ] 重启后会恢复上次语言选择

## 关闭

- [ ] 关闭主窗口时能干净释放 transport
- [ ] 关闭后不会残留串口占用
- [ ] 关闭后可以立即再次启动 GUI
