# 鐢垫満鏄犲皠

璇█ / Language: 绠€浣撲腑鏂?| [English](./motor_map.md)

## 鏈轰綋绯?
- `+Y` = 鏈哄ご / 鍓嶆柟
- `+X` = 鍙充晶
- `+Z` = 鍚戜笂
- `+pitch` = 鎶ご
- `+roll` = 鍙充晶涓嬫矇
- `+yaw` = 鏈哄ご鍙宠浆

## 鏈熸湜閫昏緫鐢垫満椤哄簭

椋炴帶鏂囨。浣跨敤浠ヤ笅閫昏緫椤哄簭锛?
- `M1` = 宸﹀墠
- `M2` = 鍙冲墠
- `M3` = 鍙冲悗
- `M4` = 宸﹀悗

鎵€鏈夋柟鍚戞晱鎰熼€昏緫閮藉繀椤诲拰 `axis_truth_table.zh-CN.md` 淇濇寔涓€鑷淬€?
## 宸插疄鐜扮殑鏄犲皠鍔熻兘

鍥轰欢閫氳繃鍙傛暟鏆撮湶 `motor_output_map[4]`锛?
- `motor_output_map0`
- `motor_output_map1`
- `motor_output_map2`
- `motor_output_map3`

`motor.c` 鍦ㄨ皟鐢?`ledc_set_duty()` 涔嬪墠锛屼細閫氳繃杩欎釜鏄犲皠鎶婇€昏緫鐢垫満鏄犲皠鍒板疄闄?LEDC 閫氶亾銆?
杩欐槸閫氶亾鏄犲皠鍔熻兘锛屽彲浠ュ湪杞欢灞傚惛鏀剁數鏈洪€氶亾/鎺ョ嚎椤哄簭宸紓銆?
## 褰撳墠鐢垫満杈撳嚭鎺у埗

褰撳墠宸茬粡瀹炵幇鐨勮緭鍑烘帶鍒跺寘鎷細

- `motor_output_map[4]`
- 鍏ㄥ眬 `motor_idle_duty`
- 鍏ㄥ眬 `motor_max_duty`
- 鍏ㄥ眬 `motor_startup_boost_duty`
- 鍏ㄥ眬 `motor_slew_limit_per_tick`
- 鍙傛暟鍖?`motor_pwm_freq_hz`

`motor.c` 使用参数化 PWM 分辨率（`motor_pwm_resolution_bits`，默认 10-bit）。支持 8/10/12 bit。PWM 频率也已参数化。
## 每电机推力补偿

每电机推力补偿已在 `motor_apply_compensation()` 中实现：

- 每电机 `motor_trim` — gamma 前的加性 trim
- 每电机 `motor_gamma` — 幂律 gamma 校正
- 每电机 `motor_scale` — 乘性缩放
- 每电机 `motor_offset` — 加性偏移
- 每电机 `motor_deadband` — 死区阈值（低于此值为零输出）
- 每电机 `motor_min_start` — 最小起转占空比

`motor_output_map` 是通道映射，不是每电机推力补偿。

涓嶈鎶?`motor_output_map` 鎻忚堪涓烘瘡鐢垫満鎺ㄥ姏琛ュ伩銆傞€氶亾鏄犲皠鍜屾帹鍔涜ˉ鍋挎槸涓嶅悓灞傜骇銆?
## Current motor output implementation note

## Current motor output implementation note

`motor.c` uses parameterized PWM resolution (`motor_pwm_resolution_bits`, default 10-bit). Both PWM frequency and resolution are parameterized.

`motor_output_map` maps logical motor order to physical outputs. Per-motor thrust compensation (`motor_trim`, `motor_gamma`, `motor_scale`, `motor_offset`, `motor_deadband`, `motor_min_start`) is implemented in `motor_apply_compensation()`.
