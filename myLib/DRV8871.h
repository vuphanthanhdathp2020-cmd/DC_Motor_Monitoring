#ifndef DRV8871_H
#define DRV8871_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum {
    DRV8871_OK = 0,
    DRV8871_ERROR_ARGUMENT,
    DRV8871_ERROR_CONFIG,
    DRV8871_ERROR_HAL,
    DRV8871_ERROR_NOT_INITIALIZED
} DRV8871_Status;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t Channel_IN1;
    uint32_t Channel_IN2;
} DRV8871_PWM;

typedef struct {
    DRV8871_PWM PWM;
    float DutyPercent;
    uint8_t Initialized;
} DRV8871_Handle;

DRV8871_Status DRV8871_Init(
    DRV8871_Handle *hDRV,
    DRV8871_PWM pwm
);

DRV8871_Status DRV8871_SetDuty(
    DRV8871_Handle *hDRV,
    float dutyPercent
);

DRV8871_Status DRV8871_Coast(
    DRV8871_Handle *hDRV
);

DRV8871_Status DRV8871_Brake(
    DRV8871_Handle *hDRV
);

#endif