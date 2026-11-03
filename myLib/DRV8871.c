#include "DRV8871.h"
#include <stddef.h>

/* Ban nay ho tro cac kenh PWM CH1 ... CH4. */
static uint8_t DRV8871_ValidChannel(uint32_t channel)
{
    return (channel == TIM_CHANNEL_1 ||
            channel == TIM_CHANNEL_2 ||
            channel == TIM_CHANNEL_3 ||
            channel == TIM_CHANNEL_4);
}

static DRV8871_Status DRV8871_Check(DRV8871_Handle *hDRV)
{
    if (hDRV == NULL) {
        return DRV8871_ERROR_ARGUMENT;
    }

    if (hDRV->Initialized == 0U) {
        return DRV8871_ERROR_NOT_INITIALIZED;
    }

    return DRV8871_OK;
}

static void DRV8871_WritePWM(
    DRV8871_Handle *hDRV,
    uint32_t in1,
    uint32_t in2
)
{
    __HAL_TIM_SET_COMPARE(
        hDRV->PWM.htim,
        hDRV->PWM.Channel_IN1,
        in1
    );

    __HAL_TIM_SET_COMPARE(
        hDRV->PWM.htim,
        hDRV->PWM.Channel_IN2,
        in2
    );
}

/*
 * Goi mot lan sau MX_TIMx_Init().
 * Timer phai duoc danh rieng cho driver.
 */
DRV8871_Status DRV8871_Init(
    DRV8871_Handle *hDRV,
    DRV8871_PWM pwm
)
{
    uint32_t arr;

    if (hDRV == NULL) {
        return DRV8871_ERROR_ARGUMENT;
    }

    *hDRV = (DRV8871_Handle){0};

    if (pwm.htim == NULL ||
        pwm.htim->Instance == NULL ||
        !DRV8871_ValidChannel(pwm.Channel_IN1) ||
        !DRV8871_ValidChannel(pwm.Channel_IN2) ||
        pwm.Channel_IN1 == pwm.Channel_IN2) {

        return DRV8871_ERROR_ARGUMENT;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(pwm.htim);

    /*
     * De CCR = ARR + 1 van nam trong 16 bit.
     * CCR = ARR + 1 tao muc cao lien tuc.
     */
    if (arr == 0U || arr > 65534U) {
        return DRV8871_ERROR_CONFIG;
    }

    hDRV->PWM = pwm;

    /* Dua hai dau vao ve 0 truoc khi bat PWM. */
    DRV8871_WritePWM(hDRV, 0U, 0U);

    pwm.htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(pwm.htim, TIM_FLAG_UPDATE);

    if (HAL_TIM_PWM_Start(
            pwm.htim, pwm.Channel_IN1) != HAL_OK) {

        return DRV8871_ERROR_HAL;
    }

    if (HAL_TIM_PWM_Start(
            pwm.htim, pwm.Channel_IN2) != HAL_OK) {

        (void)HAL_TIM_PWM_Stop(
            pwm.htim, pwm.Channel_IN1
        );

        return DRV8871_ERROR_HAL;
    }

    hDRV->Initialized = 1U;

    return DRV8871_OK;
}

/*
 * dutyPercent:
 *   +30: quay thuan, duty 30%
 *   -30: quay nghich, duty 30%
 *     0: tha troi
 *
 * Pham vi: -100 ... +100%.
 * Khong tu quan ly qua trinh dao chieu.
 */
DRV8871_Status DRV8871_SetDuty(
    DRV8871_Handle *hDRV,
    float dutyPercent
)
{
    DRV8871_Status status = DRV8871_Check(hDRV);
    uint32_t period, pulse;
    float magnitude;

    if (status != DRV8871_OK) {
        return status;
    }

    /* Loai bo gia tri ngoai pham vi, NaN va infinity. */
    if (!(dutyPercent >= -100.0f &&
          dutyPercent <= 100.0f)) {

        return DRV8871_ERROR_ARGUMENT;
    }

    if (dutyPercent == 0.0f) {
        return DRV8871_Coast(hDRV);
    }

    period =
        __HAL_TIM_GET_AUTORELOAD(hDRV->PWM.htim) + 1U;

    magnitude = (dutyPercent > 0.0f)
                ? dutyPercent
                : -dutyPercent;

    pulse = (uint32_t)(
        magnitude * (float)period / 100.0f + 0.5f
    );

    if (pulse > period) {
        pulse = period;
    }

    /*
     * PWM mode 1, active high.
     * Mot IN giu cao, IN con lai PWM dao duty.
     *
     * Thuan:  IN1 = 1, IN2 = PWM
     * Nghich: IN1 = PWM, IN2 = 1
     */
    if (dutyPercent > 0.0f) {
        DRV8871_WritePWM(
            hDRV, period, period - pulse
        );
    } else {
        DRV8871_WritePWM(
            hDRV, period - pulse, period
        );
    }

    hDRV->DutyPercent = dutyPercent;

    return DRV8871_OK;
}

/* IN1 = 0, IN2 = 0: dung tha troi. */
DRV8871_Status DRV8871_Coast(DRV8871_Handle *hDRV)
{
    DRV8871_Status status = DRV8871_Check(hDRV);

    if (status != DRV8871_OK) {
        return status;
    }

    DRV8871_WritePWM(hDRV, 0U, 0U);
    hDRV->DutyPercent = 0.0f;

    return DRV8871_OK;
}

/* IN1 = 1, IN2 = 1: phanh dien. */
DRV8871_Status DRV8871_Brake(DRV8871_Handle *hDRV)
{
    DRV8871_Status status = DRV8871_Check(hDRV);
    uint32_t period;

    if (status != DRV8871_OK) {
        return status;
    }

    period =
        __HAL_TIM_GET_AUTORELOAD(hDRV->PWM.htim) + 1U;

    DRV8871_WritePWM(hDRV, period, period);
    hDRV->DutyPercent = 0.0f;

    return DRV8871_OK;
}