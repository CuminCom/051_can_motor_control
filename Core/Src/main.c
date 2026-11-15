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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  CAN_HandleTypeDef *can_handle;
  uint32_t rx_id;
  CAN_TxHeaderTypeDef txconf;
  uint8_t tx_buff[8];
  uint8_t rx_buff[8];
  uint8_t rx_len;
  uint32_t tx_mailbox;
} motor_t;

typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t real_current;
  uint8_t temperature;
  float real_current_handle;
  float speed_aps;
  uint16_t last_ecd;
  float angle_single_round;
  int32_t total_round;
  float total_angle;
  uint8_t is_initialized;
} motor_measure_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI              (3.1415926f)
#define MOTOR_ID        (1U)
#define MOTOR_RX_ID     (0x200U + MOTOR_ID)
#define CURRENT_FILTER  (0.1f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
motor_t motor1;
motor_measure_t motor1_measure;
int16_t current_cmd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void motor_init(void);
static void motor_data_read(void);
static void motor_send(int16_t cmd);
static void can_fifo_callback(CAN_HandleTypeDef *hcan, uint32_t fifox);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  float phase = 0.0f;
  motor_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    current_cmd = (int16_t)(1000.0f * sinf(phase));
    phase += 0.02f * PI;
    if (phase >= 2.0f * PI)
    {
      phase -= 2.0f * PI;
    }
    motor_send(current_cmd);
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void motor_init(void)
{
  motor1.can_handle = &hcan1;
  motor1.rx_id = MOTOR_RX_ID;

  motor1.txconf.StdId = 0x200;
  motor1.txconf.IDE = CAN_ID_STD;
  motor1.txconf.RTR = CAN_RTR_DATA;
  motor1.txconf.DLC = 8;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.SlaveStartFilterBank = 14;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterIdHigh = motor1.rx_id << 5;
  filter.FilterIdLow = motor1.rx_id << 5;
  filter.FilterMaskIdHigh = motor1.rx_id << 5;
  filter.FilterMaskIdLow = motor1.rx_id << 5;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

static void motor_data_read(void)
{
  motor1_measure.ecd =
      (uint16_t)(((uint16_t)motor1.rx_buff[0] << 8) | motor1.rx_buff[1]);
  motor1_measure.speed_rpm =
      (int16_t)(((uint16_t)motor1.rx_buff[2] << 8) | motor1.rx_buff[3]);
  motor1_measure.real_current =
      (int16_t)(((uint16_t)motor1.rx_buff[4] << 8) | motor1.rx_buff[5]);
  motor1_measure.temperature = motor1.rx_buff[6];
}

static void motor_send(int16_t cmd)
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    return;
  }

  uint16_t raw = (uint16_t)cmd;
  motor1.tx_buff[0] = (uint8_t)(raw >> 8);
  motor1.tx_buff[1] = (uint8_t)(raw & 0xFFU);
  for (uint8_t i = 2; i < 8; ++i)
  {
    motor1.tx_buff[i] = 0;
  }

  (void)HAL_CAN_AddTxMessage(
      &hcan1, &motor1.txconf, motor1.tx_buff, &motor1.tx_mailbox);
}

static void can_fifo_callback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
  CAN_RxHeaderTypeDef rxconf;
  uint8_t can_rx_buff[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff) != HAL_OK)
    {
      break;
    }

    if (hcan == motor1.can_handle &&
        rxconf.IDE == CAN_ID_STD &&
        rxconf.RTR == CAN_RTR_DATA &&
        rxconf.DLC == 8U &&
        rxconf.StdId == motor1.rx_id)
    {
      uint16_t previous_ecd = motor1_measure.ecd;

      memcpy(motor1.rx_buff, can_rx_buff, sizeof(motor1.rx_buff));
      motor_data_read();

      if (motor1_measure.is_initialized == 0U)
      {
        motor1_measure.is_initialized = 1U;
        motor1_measure.last_ecd = motor1_measure.ecd;
        motor1_measure.speed_aps =
            (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
        motor1_measure.real_current_handle =
            (float)motor1_measure.real_current;
      }
      else
      {
        motor1_measure.last_ecd = previous_ecd;
        motor1_measure.speed_aps =
            (1.0f - 0.85f) * motor1_measure.speed_aps +
            0.85f * (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
        motor1_measure.real_current_handle =
            (1.0f - CURRENT_FILTER) * motor1_measure.real_current_handle +
            CURRENT_FILTER * (float)motor1_measure.real_current;

        if ((int32_t)motor1_measure.ecd -
                (int32_t)motor1_measure.last_ecd > 4096)
        {
          --motor1_measure.total_round;
        }
        else if ((int32_t)motor1_measure.ecd -
                     (int32_t)motor1_measure.last_ecd < -4096)
        {
          ++motor1_measure.total_round;
        }
      }

      motor1_measure.angle_single_round =
          360.0f / 8192.0f * (float)motor1_measure.ecd;
      motor1_measure.total_angle =
          (float)motor1_measure.total_round * 360.0f +
          motor1_measure.angle_single_round;
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_fifo_callback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_fifo_callback(hcan, CAN_RX_FIFO1);
}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
