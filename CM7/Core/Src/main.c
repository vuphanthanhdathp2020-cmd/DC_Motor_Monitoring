/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DRV8871.h"
#include "GA25_370.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
#define MOTOR_OPEN_LOOP_TEST  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
DRV8871_Handle hDRV8871 = {0};
GA25_370_Handle hGA25 = {0};

volatile DRV8871_Status drv_status;
volatile GA25_370_Status encoder_status;

/* Cac bien theo doi trong Keil Watch */
volatile uint8_t test_ready   = 0;
volatile uint8_t test_request = 0;
volatile uint8_t test_running = 0;
volatile uint8_t test_done    = 0;

volatile float duty_percent = 0.0f;
volatile float rpm = 0.0f;
volatile int32_t delta_count = 0;
volatile uint32_t encoder_count = 0;

static uint32_t test_start_tick = 0;

/* Toc do dat: RPM */
volatile float speed_setpoint = 100.0f;

/* He so khoi dau de thu, chua phai thong so da tinh chinh */
volatile float pi_kp = 0.20f;
volatile float pi_ki = 0.50f;

/* Cac bien quan sat */
volatile float speed_error = 0.0f;
volatile float pi_integral = 0.0f;

/*
 * pi_fault:
 * 0: Khong loi
 * 1: RPM nguoc dau mong doi
 * 2: Loi doc encoder
 * 3: Loi goi ham dieu khien driver
 * 4: Toc do dat ngoai pham vi thu
 */
volatile uint8_t pi_fault = 0U;

static uint32_t pi_last_tick = 0U;

volatile HAL_StatusTypeDef uart_status = HAL_OK;
volatile uint32_t uart_tx_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_SendTelemetry(void)
{
    static uint32_t last_send_tick = 0U;
    uint32_t tick;
    char buffer[96];
    int length;

    tick = HAL_GetTick();

    /* Gui du lieu khoang moi 100 ms */
    if ((uint32_t)(tick - last_send_tick) < 100U)
    {
        return;
    }

    last_send_tick = tick;

    /*
     * Thu tu:
     * thoi gian ms, toc do dat x10, RPM x10,
     * PWM x10, trang thai chay, ma loi
     */
    length = snprintf(
        buffer,
        sizeof(buffer),
        "%lu,%ld,%ld,%ld,%u,%u\r\n",
        (unsigned long)tick,
        (long)(speed_setpoint * 10.0f),
        (long)(rpm * 10.0f),
        (long)(duty_percent * 10.0f),
        (unsigned int)test_running,
        (unsigned int)pi_fault
    );

    if ((length > 0) && (length < (int)sizeof(buffer)))
    {
        uart_status = HAL_UART_Transmit(
            &huart3,
            (uint8_t *)buffer,
            (uint16_t)length,
            10U
        );

        if (uart_status == HAL_OK)
        {
            uart_tx_count++;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
DRV8871_PWM motor_pwm = {
    &htim3,
    TIM_CHANNEL_1,   /* PC6 -> IN1 */
    TIM_CHANNEL_2    /* PC7 -> IN2 */
};

/* Khoi dong PWM o trang thai motor dung */
drv_status = DRV8871_Init(&hDRV8871, motor_pwm);

if (drv_status != DRV8871_OK)
{
    Error_Handler();
}

/*
 * Gia dinh encoder:
 * 11 chu ky/kenh/vong rotor, doc x4, hop so 21.3:1.
 * So dem/vong truc ra = 11 * 4 * 21.3 = 937.2.
 */
encoder_status = GA25_370_Init(
    &hGA25,
    &htim4,
    937.2f,
    -1
);

if (encoder_status != GA25_370_OK)
{
    (void)DRV8871_Coast(&hDRV8871);
    Error_Handler();
}

test_ready = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t now;
float dt;
float integral_candidate;
float output_candidate;
float output;

/* Doc encoder, ke ca khi motor dang dung */
encoder_status = GA25_370_ReadEncoder(&hGA25);
now = HAL_GetTick();

if (encoder_status == GA25_370_OK)
{
    rpm = hGA25.Data.RPM;
    delta_count = hGA25.Data.DeltaCount;
    encoder_count = __HAL_TIM_GET_COUNTER(&htim4);
}
else if (encoder_status != GA25_370_NO_NEW_SAMPLE)
{
    pi_fault = 2U;
}

/* Nhan yeu cau bat dau mot luot thu */
if (test_request == 1U)
{
    test_request = 0U;

    if ((test_ready == 1U) &&
        (test_running == 0U) &&
        (pi_fault == 0U))
    {
        /* Lan thu nay chi cho phep 0 < toc do dat <= 100 RPM */
        if (!(speed_setpoint > 0.0f &&
              speed_setpoint <= 100.0f))
        {
            pi_fault = 4U;
        }
        else
        {
            pi_integral = 0.0f;
            speed_error = 0.0f;
            duty_percent = 0.0f;

            test_done = 0U;
            test_start_tick = now;
            pi_last_tick = now;
            test_running = 1U;
        }
    }
}
#if MOTOR_OPEN_LOOP_TEST

if (test_running == 1 && pi_fault == 0)
{
    uint32_t elapsed_ms;
    float command_pwm;

    elapsed_ms = (uint32_t)(now - test_start_tick);

    /*
     * 0–2 giay: motor dung, ghi du lieu nen.
     * 2–12 giay: PWM co dinh 30%.
     * Sau 12 giay: dung motor.
     */
    if (elapsed_ms >= 12000U)
    {
        drv_status = DRV8871_Coast(&hDRV8871);

        duty_percent = 0.0f;
        pi_integral = 0.0f;
        test_running = 0;

        if (drv_status == DRV8871_OK)
        {
            test_done = 1;
        }
        else
        {
            pi_fault = 3;
        }
    }
    else
    {
        command_pwm = (elapsed_ms < 2000U) ? 0.0f : 30.0f;

        if (encoder_status == GA25_370_OK && rpm < -10.0f)
        {
            pi_fault = 1;
        }
        else
        {
            duty_percent = command_pwm;

            if (command_pwm == 0.0f)
            {
                drv_status = DRV8871_Coast(&hDRV8871);
            }
            else
            {
                drv_status = DRV8871_SetDuty(
                    &hDRV8871, command_pwm);
            }

            if (drv_status != DRV8871_OK)
            {
                pi_fault = 3;
            }
        }
    }
}

#else
if ((test_running == 1U) && (pi_fault == 0U))
{
    /* Ket thuc luot thu sau 5 giay */
    if ((uint32_t)(now - test_start_tick) >= 10000U)
    {
        drv_status = DRV8871_Coast(&hDRV8871);

        duty_percent = 0.0f;
        pi_integral = 0.0f;
        test_running = 0U;

        if (drv_status == DRV8871_OK)
        {
            test_done = 1U;
        }
        else
        {
            pi_fault = 3U;
        }
    }
    else if (encoder_status == GA25_370_OK)
    {
        /* Chi cap PWM duong: RPM am dang ke la sai quy uoc dau */
        if (rpm < -10.0f)
        {
            pi_fault = 1U;
        }
        else
        {
            dt = (float)((uint32_t)(now - pi_last_tick))
                 * 0.001f;

            if (dt > 0.0f)
            {
                pi_last_tick = now;

                speed_error = speed_setpoint - rpm;

                integral_candidate =
                    pi_integral + pi_ki * speed_error * dt;

                output_candidate =
                    pi_kp * speed_error + integral_candidate;

                /*
                 * Anti-windup:
                 * Cho tich phan cap nhat khi dau ra trong gioi han,
                 * hoac khi sai so giup dua dau ra ve khoi bao hoa.
                 */
                if (((output_candidate >= 0.0f) &&
                     (output_candidate <= 50.0f)) ||
                    ((output_candidate > 50.0f) &&
                     (speed_error < 0.0f)) ||
                    ((output_candidate < 0.0f) &&
                     (speed_error > 0.0f)))
                {
                    pi_integral = integral_candidate;
                }

                output = pi_kp * speed_error + pi_integral;

                /* Gioi han PWM cho lan thu dau */
                if (output > 50.0f)
                {
                    output = 50.0f;
                }
                else if (output < 0.0f)
                {
                    output = 0.0f;
                }

                duty_percent = output;

                drv_status = DRV8871_SetDuty(
                    &hDRV8871, output
                );

                if (drv_status != DRV8871_OK)
                {
                    pi_fault = 3U;
                }
            }
        }
    }
}
#endif

/* Dung va khoa luot thu neu phat hien loi */
if (pi_fault != 0U)
{
    (void)DRV8871_Coast(&hDRV8871);

    duty_percent = 0.0f;
    pi_integral = 0.0f;
    test_running = 0U;
    test_ready = 0U;
    test_done = 0U;
}
Motor_SendTelemetry();
HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
