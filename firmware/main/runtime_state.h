/**
 * @file runtime_state.h
 * @brief 跨任务运行时状态接口。
 */

#pragma once

#include "esp_drone_types.h"

/**
 * @brief 初始化跨任务共享状态。
 */
void runtime_state_init(void);

/**
 * @brief 设置当前 arm 状态快照。
 *
 * @param[in] state arm 状态枚举值。
 */
void runtime_state_set_arm_state(arm_state_t state);

/**
 * @brief 获取当前 arm 状态快照。
 *
 * @return 当前 arm 状态。
 */
arm_state_t runtime_state_get_arm_state(void);

/**
 * @brief 设置当前 failsafe 原因快照。
 *
 * @param[in] reason failsafe 原因枚举值。
 */
void runtime_state_set_failsafe_reason(failsafe_reason_t reason);

/**
 * @brief 获取当前 failsafe 原因快照。
 *
 * @return 当前 failsafe 原因。
 */
failsafe_reason_t runtime_state_get_failsafe_reason(void);

/**
 * @brief 设置控制循环统计信息。
 *
 * @param[in] stats 控制循环统计快照，时间单位为 us。
 */
void runtime_state_set_loop_stats(loop_stats_t stats);

/**
 * @brief 获取控制循环统计信息。
 *
 * @return 最近一次写入的循环统计快照。
 */
loop_stats_t runtime_state_get_loop_stats(void);

/**
 * @brief 设置遥测流开关状态。
 *
 * @param[in] enabled `true` 表示允许发送遥测流。
 */
void runtime_state_set_stream_enabled(bool enabled);

/**
 * @brief 获取遥测流开关状态。
 *
 * @retval true 遥测流已开启。
 * @retval false 遥测流已关闭。
 */
bool runtime_state_get_stream_enabled(void);

/**
 * @brief 设置电机点动测试请求。
 *
 * @param[in] logical_motor 逻辑电机编号；小于 0 表示清除请求。
 * @param[in] duty 请求占空比，调用方通常使用 `0.0f` 到 `1.0f`。
 */
void runtime_state_set_motor_test(int logical_motor, float duty);

/**
 * @brief 获取当前电机点动测试请求。
 *
 * @param[out] out_logical_motor 输出逻辑电机编号，可为 `NULL`。
 * @param[out] out_duty 输出测试占空比，可为 `NULL`。
 */
void runtime_state_get_motor_test(int *out_logical_motor, float *out_duty);

void runtime_state_set_all_motor_test(float duty, uint32_t duration_ms, uint64_t start_us);
all_motor_test_state_t runtime_state_get_all_motor_test(void);
void runtime_state_clear_all_motor_test(void);

/**
 * @brief 设置当前控制模式。
 *
 * @param[in] mode 控制模式枚举值。
 */
void runtime_state_set_control_mode(control_mode_t mode);

/**
 * @brief 获取当前控制模式。
 *
 * @return 当前控制模式。
 */
control_mode_t runtime_state_get_control_mode(void);

/**
 * @brief 设置 axis-test 请求。
 *
 * @param[in] request 三轴开环请求，通常使用归一化控制量。
 */
void runtime_state_set_axis_test_request(axis3f_t request);

/**
 * @brief 获取 axis-test 请求。
 *
 * @return 当前 axis-test 请求值。
 */
axis3f_t runtime_state_get_axis_test_request(void);

/**
 * @brief 设置 rate-test 目标值。
 *
 * @param[in] request 三轴速率目标，单位为 deg/s。
 */
void runtime_state_set_rate_setpoint_request(axis3f_t request);

/**
 * @brief 获取 rate-test 目标值。
 *
 * @return 当前三轴速率目标，单位为 deg/s。
 */
axis3f_t runtime_state_get_rate_setpoint_request(void);

/**
 * @brief 璁剧疆 attitude hang 鍙傝€冨Э鎬併€? */
void runtime_state_set_attitude_reference(bool valid, quatf_t ref_q_body_to_world);

/**
 * @brief 娓呴櫎 attitude hang 鍙傝€冨Э鎬併€? */
void runtime_state_clear_attitude_reference(void);

/**
 * @brief 鑾峰彇 attitude hang 杩愯鎬佸揩鐓с€? */
attitude_hang_state_t runtime_state_get_attitude_hang_state(void);

/**
 * @brief 璁剧疆 attitude hang 杩愯鎬佸揩鐓с€? */
void runtime_state_set_attitude_hang_state(attitude_hang_state_t state);

void runtime_state_set_ground_reference(bool valid,
                                        quatf_t ref_q_body_to_world,
                                        float ref_kalman_roll_deg,
                                        float ref_kalman_pitch_deg);
void runtime_state_clear_ground_reference(void);
ground_tune_state_t runtime_state_get_ground_tune_state(void);
void runtime_state_set_ground_tune_state(ground_tune_state_t state);
