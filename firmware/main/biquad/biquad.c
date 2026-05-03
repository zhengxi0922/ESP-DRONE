/**
 * @file biquad.c
 * @brief 二阶 IIR biquad 滤波器实现。
 */

#include "biquad.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void biquad_init(biquad_filter_t *filt)
{
    if (filt == NULL) {
        return;
    }
    memset(filt, 0, sizeof(*filt));
    filt->b0 = 1.0f;
    filt->mode = BIQUAD_MODE_BYPASS;
}

void biquad_set_bypass(biquad_filter_t *filt)
{
    if (filt == NULL) {
        return;
    }
    filt->b0 = 1.0f;
    filt->b1 = 0.0f;
    filt->b2 = 0.0f;
    filt->a1 = 0.0f;
    filt->a2 = 0.0f;
    filt->mode = BIQUAD_MODE_BYPASS;
}

static bool biquad_params_valid(float *coeffs, int n)
{
    for (int i = 0; i < n; ++i) {
        if (!isfinite(coeffs[i])) {
            return false;
        }
    }
    return true;
}

void biquad_design_lpf(biquad_filter_t *filt, float cutoff_hz, float sample_hz)
{
    if (filt == NULL || cutoff_hz <= 0.0f || sample_hz <= 0.0f) {
        if (filt != NULL) {
            biquad_set_bypass(filt);
        }
        return;
    }

    /* 二阶 Butterworth LPF: 双线性变换 */
    const float w0 = 2.0f * (float)M_PI * cutoff_hz / sample_hz;
    if (w0 <= 0.0f || w0 >= (float)M_PI) {
        biquad_set_bypass(filt);
        return;
    }

    const float alpha = sinf(w0) / (2.0f * 0.7071067811865476f); /* Q = 1/sqrt(2) */
    const float cos_w0 = cosf(w0);

    const float b0 = (1.0f - cos_w0) / 2.0f;
    const float b1 = 1.0f - cos_w0;
    const float b2 = b0;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_w0;
    const float a2 = 1.0f - alpha;

    float coeffs[] = {b0, b1, b2, a0, a1, a2};
    if (!biquad_params_valid(coeffs, 6) || fabsf(a0) < 1e-9f) {
        biquad_set_bypass(filt);
        return;
    }

    filt->b0 = b0 / a0;
    filt->b1 = b1 / a0;
    filt->b2 = b2 / a0;
    filt->a1 = a1 / a0;
    filt->a2 = a2 / a0;
    filt->mode = BIQUAD_MODE_LOWPASS;
}

void biquad_design_notch(biquad_filter_t *filt, float center_hz, float bandwidth_hz, float sample_hz)
{
    if (filt == NULL || center_hz <= 0.0f || bandwidth_hz <= 0.0f || sample_hz <= 0.0f) {
        if (filt != NULL) {
            biquad_set_bypass(filt);
        }
        return;
    }

    const float w0 = 2.0f * (float)M_PI * center_hz / sample_hz;
    if (w0 <= 0.0f || w0 >= (float)M_PI) {
        biquad_set_bypass(filt);
        return;
    }

    const float bw = 2.0f * (float)M_PI * bandwidth_hz / sample_hz;
    if (bw <= 0.0f) {
        biquad_set_bypass(filt);
        return;
    }

    const float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw * w0 / sinf(w0));
    const float cos_w0 = cosf(w0);

    const float b0 = 1.0f;
    const float b1 = -2.0f * cos_w0;
    const float b2 = 1.0f;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_w0;
    const float a2 = 1.0f - alpha;

    float coeffs[] = {b0, b1, b2, a0, a1, a2};
    if (!biquad_params_valid(coeffs, 6) || fabsf(a0) < 1e-9f) {
        biquad_set_bypass(filt);
        return;
    }

    filt->b0 = b0 / a0;
    filt->b1 = b1 / a0;
    filt->b2 = b2 / a0;
    filt->a1 = a1 / a0;
    filt->a2 = a2 / a0;
    filt->mode = BIQUAD_MODE_NOTCH;
}

float biquad_update(biquad_filter_t *filt, float input)
{
    if (filt == NULL || filt->mode == BIQUAD_MODE_BYPASS) {
        return input;
    }

    if (!isfinite(input)) {
        biquad_reset(filt);
        return 0.0f;
    }

    if (!filt->initialized) {
        /* 首次输入：用当前值填充历史，避免瞬态 */
        filt->x1 = input;
        filt->x2 = input;
        filt->y1 = input;
        filt->y2 = input;
        filt->initialized = true;
        return input;
    }

    /* Direct Form I */
    const float y = filt->b0 * input + filt->b1 * filt->x1 + filt->b2 * filt->x2
                    - filt->a1 * filt->y1 - filt->a2 * filt->y2;

    if (!isfinite(y)) {
        biquad_reset(filt);
        return 0.0f;
    }

    filt->x2 = filt->x1;
    filt->x1 = input;
    filt->y2 = filt->y1;
    filt->y1 = y;

    return y;
}

void biquad_reset(biquad_filter_t *filt)
{
    if (filt == NULL) {
        return;
    }
    filt->x1 = 0.0f;
    filt->x2 = 0.0f;
    filt->y1 = 0.0f;
    filt->y2 = 0.0f;
    filt->initialized = false;
}

bool biquad_self_test(void)
{
    biquad_filter_t filt;

    /* 1. init 后状态为 bypass，系数安全 */
    biquad_init(&filt);
    if (filt.mode != BIQUAD_MODE_BYPASS || filt.b0 != 1.0f) {
        return false;
    }

    /* 2. 输入直接通过 bypass */
    const float bypass_out = biquad_update(&filt, 3.0f);
    if (fabsf(bypass_out - 3.0f) > 1e-6f) {
        return false;
    }

    /* 3. LPF 设计无 NaN/Inf */
    biquad_design_lpf(&filt, 40.0f, 1000.0f);
    if (!isfinite(filt.b0) || !isfinite(filt.b1) || !isfinite(filt.b2) ||
        !isfinite(filt.a1) || !isfinite(filt.a2)) {
        return false;
    }
    if (filt.mode != BIQUAD_MODE_LOWPASS) {
        return false;
    }

    /* 4. LPF step response 输出有限 */
    biquad_reset(&filt);
    biquad_design_lpf(&filt, 40.0f, 1000.0f);
    float lpf_out = 0.0f;
    for (int i = 0; i < 100; ++i) {
        lpf_out = biquad_update(&filt, 1.0f);
        if (!isfinite(lpf_out)) {
            return false;
        }
    }
    /* 100 步后应接近稳态值 1.0 */
    if (fabsf(lpf_out - 1.0f) > 0.3f) {
        return false;
    }

    /* 5. Notch 设计无 NaN/Inf */
    biquad_design_notch(&filt, 150.0f, 30.0f, 1000.0f);
    if (!isfinite(filt.b0) || !isfinite(filt.b1) || !isfinite(filt.b2) ||
        !isfinite(filt.a1) || !isfinite(filt.a2)) {
        return false;
    }
    if (filt.mode != BIQUAD_MODE_NOTCH) {
        return false;
    }

    /* 6. Notch step response 输出有限 */
    biquad_reset(&filt);
    biquad_design_notch(&filt, 150.0f, 30.0f, 1000.0f);
    for (int i = 0; i < 50; ++i) {
        const float notch_out = biquad_update(&filt, 0.5f);
        if (!isfinite(notch_out)) {
            return false;
        }
    }

    /* 7. reset 后状态归零 */
    biquad_reset(&filt);
    if (filt.x1 != 0.0f || filt.x2 != 0.0f || filt.y1 != 0.0f || filt.y2 != 0.0f) {
        return false;
    }
    if (filt.initialized) {
        return false;
    }

    /* 8. 极端参数自动 fallback 到 bypass */
    biquad_design_lpf(&filt, -1.0f, 1000.0f);
    if (filt.mode != BIQUAD_MODE_BYPASS) {
        return false;
    }
    biquad_design_notch(&filt, 0.0f, 0.0f, 1000.0f);
    if (filt.mode != BIQUAD_MODE_BYPASS) {
        return false;
    }

    /* 9. NaN 输入自动 bypass/reset */
    biquad_design_lpf(&filt, 40.0f, 1000.0f);
    biquad_update(&filt, 1.0f);
    const float nan_out = biquad_update(&filt, NAN);
    /* NaN 输入后应返回 0 并重置 */
    if (filt.initialized) {
        return false;
    }

    return true;
}
