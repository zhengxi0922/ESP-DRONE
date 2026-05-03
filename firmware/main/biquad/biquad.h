/**
 * @file biquad.h
 * @brief 二阶 IIR biquad 滤波器框架。
 *
 * @note 默认所有滤波器处于 bypass 状态，不影响现有飞控动态。
 *       仅在明确启用对应参数时生效。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Biquad 滤波器模式。
 */
typedef enum {
    BIQUAD_MODE_BYPASS = 0,  /**< 直通，不滤波。 */
    BIQUAD_MODE_LOWPASS,     /**< 二阶低通 (Butterworth)。 */
    BIQUAD_MODE_NOTCH,       /**< 陷波滤波器。 */
} biquad_mode_t;

/**
 * @brief 单通道 biquad 滤波器状态。
 *
 * @note 所有历史值初始化为 0，首次调用自动视为滤波器预热。
 */
typedef struct {
    float b0, b1, b2;        /**< 分子系数。 */
    float a1, a2;            /**< 分母系数 (a0 隐含为 1.0)。 */
    float x1, x2;            /**< 输入历史 (x[n-1], x[n-2])。 */
    float y1, y2;            /**< 输出历史 (y[n-1], y[n-2])。 */
    bool initialized;         /**< 是否已至少处理过一个有效样本。 */
    biquad_mode_t mode;      /**< 当前模式。 */
} biquad_filter_t;

/**
 * @brief 初始化滤波器为 bypass 模式。
 *
 * @param[out] filt 滤波器状态指针。
 */
void biquad_init(biquad_filter_t *filt);

/**
 * @brief 设计二阶 Butterworth 低通滤波器。
 *
 * @param[out] filt 滤波器状态指针。
 * @param[in] cutoff_hz 截止频率，单位为 Hz。<= 0 时自动切换为 bypass。
 * @param[in] sample_hz 采样频率，单位为 Hz。
 */
void biquad_design_lpf(biquad_filter_t *filt, float cutoff_hz, float sample_hz);

/**
 * @brief 设计二阶陷波滤波器。
 *
 * @param[out] filt 滤波器状态指针。
 * @param[in] center_hz 陷波中心频率，单位为 Hz。<= 0 时自动切换为 bypass。
 * @param[in] bandwidth_hz 陷波带宽 (-3dB)，单位为 Hz。
 * @param[in] sample_hz 采样频率，单位为 Hz。
 */
void biquad_design_notch(biquad_filter_t *filt, float center_hz, float bandwidth_hz, float sample_hz);

/**
 * @brief 处理一个样本。
 *
 * @param[in,out] filt 滤波器状态指针。
 * @param[in] input 输入样本。
 * @return 滤波后的输出。若滤波器处于 bypass 模式或输入无效，返回原始输入。
 */
float biquad_update(biquad_filter_t *filt, float input);

/**
 * @brief 重置滤波器历史状态。
 *
 * @param[in,out] filt 滤波器状态指针。
 * @note 不清除系数，仅将历史值归零。
 */
void biquad_reset(biquad_filter_t *filt);

/**
 * @brief 设置滤波器为 bypass 模式。
 *
 * @param[in,out] filt 滤波器状态指针。
 */
void biquad_set_bypass(biquad_filter_t *filt);
