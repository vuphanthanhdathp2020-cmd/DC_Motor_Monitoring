#include "GA25_370.h"
#include <stddef.h>
#include <float.h>

/*
 * Goi mot lan sau MX_TIMx_Init().
 * Timer encoder: PSC = 0, ARR = 65535.
 */
GA25_370_Status GA25_370_Init(
    GA25_370_Handle *hMotor,
    TIM_HandleTypeDef *htimEncoder,
    float countsPerRev,
    int8_t encoderSign
)
{
    if (hMotor == NULL) {
        return GA25_370_ERROR_ARGUMENT;
    }

    *hMotor = (GA25_370_Handle){0};

    if (htimEncoder == NULL ||
        htimEncoder->Instance == NULL ||
        !(countsPerRev > 0.0f && countsPerRev <= FLT_MAX) ||
        (encoderSign != 1 && encoderSign != -1)) {

        return GA25_370_ERROR_ARGUMENT;
    }

    if (__HAL_TIM_GET_AUTORELOAD(htimEncoder) != 65535U ||
        htimEncoder->Instance->PSC != 0U) {

        return GA25_370_ERROR_CONFIG;
    }

    hMotor->htimEncoder = htimEncoder;
    hMotor->CountsPerRev = countsPerRev;
    hMotor->EncoderSign = encoderSign;

    __HAL_TIM_SET_COUNTER(htimEncoder, 0U);

    if (HAL_TIM_Encoder_Start(
            htimEncoder, TIM_CHANNEL_ALL) != HAL_OK) {

        return GA25_370_ERROR_HAL;
    }

    hMotor->LastCounter =
        (uint16_t)__HAL_TIM_GET_COUNTER(htimEncoder);

    hMotor->LastTick = HAL_GetTick();
    hMotor->Initialized = 1U;

    return GA25_370_OK;
}

/*
 * Goi dinh ky, vi du moi 20 ms.
 * HAL_GetTick() phai tra ve thoi gian theo ms.
 *
 * Dich chuyen giua hai lan doc phai nho hon
 * 32768 so dem de xac dinh dung chieu khi tran.
 */
GA25_370_Status GA25_370_ReadEncoder(
    GA25_370_Handle *hMotor
)
{
    uint32_t now, dt;
    uint16_t counter, difference;
    int32_t delta;

    if (hMotor == NULL) {
        return GA25_370_ERROR_ARGUMENT;
    }

    if (hMotor->Initialized == 0U) {
        return GA25_370_ERROR_NOT_INITIALIZED;
    }

    now = HAL_GetTick();
    dt = now - hMotor->LastTick;

    if (dt == 0U) {
        return GA25_370_NO_NEW_SAMPLE;
    }

    counter = (uint16_t)__HAL_TIM_GET_COUNTER(
        hMotor->htimEncoder
    );

    /* Hieu modulo 65536, xu ly tran theo hai chieu. */
    difference = (uint16_t)(
        counter - hMotor->LastCounter
    );

    delta = (int32_t)difference;

    if (delta >= 32768) {
        delta -= 65536;
    }

    delta *= hMotor->EncoderSign;

    hMotor->Data.DeltaCount = delta;
    hMotor->Data.PositionCount += delta;

    hMotor->Data.RPM =
        (float)delta * 60000.0f /
        (hMotor->CountsPerRev * (float)dt);

    hMotor->LastCounter = counter;
    hMotor->LastTick = now;

    return GA25_370_OK;
}