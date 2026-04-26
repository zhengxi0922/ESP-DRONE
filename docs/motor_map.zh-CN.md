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

`motor.c` 褰撳墠浣跨敤鍥哄畾 `LEDC_TIMER_8_BIT` PWM 鍒嗚鲸鐜囥€侾WM 棰戠巼宸茬粡鍙傛暟鍖栵紝浣?PWM 鍒嗚鲸鐜囨病鏈夊弬鏁板寲銆?
## 灏氭湭瀹炵幇

褰撳墠浠ｇ爜灏氭湭瀹炵幇姣忕數鏈烘帹鍔涜ˉ鍋匡細

- 姣忕數鏈?`scale`
- 姣忕數鏈?`offset`
- 姣忕數鏈?`min_start`
- 姣忕數鏈?`deadband`
- 姣忕數鏈?`gamma`

涓嶈鎶?`motor_output_map` 鎻忚堪涓烘瘡鐢垫満鎺ㄥ姏琛ュ伩銆傞€氶亾鏄犲皠鍜屾帹鍔涜ˉ鍋挎槸涓嶅悓灞傜骇銆?
## Current motor output implementation note

Current `motor.c` uses fixed 8-bit LEDC PWM resolution (`LEDC_TIMER_8_BIT`). `motor_pwm_freq_hz` parameterizes PWM frequency only; PWM resolution is not parameterized.

`motor_output_map` maps logical motor order to physical outputs. It is not per-motor thrust compensation. The current firmware does not yet implement per-motor `scale`, `offset`, `min_start`, `deadband`, or `gamma` compensation.
