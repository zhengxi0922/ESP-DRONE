# ESP-Drone UDP 兼容说明

**语言 / Language：** [English](./esp_drone_udp_compat.md) | **简体中文**

## 兼容目标

当前兼容范围仅限于 legacy 控制入口路径。本次重写不承诺与 stock App 完整特性对等。

## Legacy 端点

- 端口：`2390`
- 校验模型：`sum(payload) & 0xFF`
- 接受的 legacy 数据：
  - 12 字节旧 App 控制包
  - CRTP-over-UDP 数据路径

## 已确认兼容项

- `roll`、`pitch`、`yaw`、`thrust` 四通道的基础语义兼容
- 通过旧 UDP / CRTP 兼容机制提供基础下行链路
- legacy 控制端口号 `2390`

## V1 不承诺的内容

- 与 stock App 的完整功能对等
- 通过 legacy 传输支持新的参数 API
- 通过 legacy 传输支持批量日志导出
- 除非后续抓包验证，否则不承诺 stock App 独立 `arm`、`disarm` 或 `mode` 包兼容

## 替代策略

- `2390`：用于 legacy 控制入口与基础下行兼容层
- `2391`：新的二进制 CLI UDP 协议
- `USB CDC`：与 `2391` 共用同一套新二进制协议
- 显式 `arm`、`disarm`、`kill`：统一通过新 CLI 提供
- legacy 控制路径仍可通过摇杆手势和安全状态机完成解锁或上锁
