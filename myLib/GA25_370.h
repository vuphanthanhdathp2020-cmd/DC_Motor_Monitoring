#ifndef GA25_370_H
#define GA25_370_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum {
    GA25_370_OK = 0,
    GA25_370_ERROR_ARGUMENT,
    GA25_370_ERROR_CONFIG,
    GA25_370_ERROR_HAL,
    GA25_370_ERROR_NOT_INITIALIZED,
    GA25_370_NO_NEW_SAMPLE
} GA25_370_Status;

typedef struct {
    int32_t DeltaCount;       // So dem thay doi giua hai lan doc
    int64_t PositionCount;    // Tong so dem co dau tu luc Init
    float RPM;               // Toc do truc ra, co dau
} GA25_370_Data;

typedef struct {
    TIM_HandleTypeDef *htimEncoder;

    float CountsPerRev;      // So dem / vong truc ra
    int8_t EncoderSign;      // +1 hoac -1

    uint16_t LastCounter;
    uint32_t LastTick;
    uint8_t Initialized;

    GA25_370_Data Data;
} GA25_370_Handle;

GA25_370_Status GA25_370_Init(
    GA25_370_Handle *hMotor,
    TIM_HandleTypeDef *htimEncoder,
    float countsPerRev,
    int8_t encoderSign
);

GA25_370_Status GA25_370_ReadEncoder(
    GA25_370_Handle *hMotor
);

#endif