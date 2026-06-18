/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// States of the state machine of the catapult
typedef enum {
    ETAT_REPOS      = 0,  // Initial position / Idle
    ETAT_RECHARGE   = 1,  // Recharge
    ETAT_SET_SERVO1 = 2,  // Interactive adjustment of servo 1
    ETAT_SET_SERVO2 = 3,  // Interactive adjustment of servo 2
	ETAT_ATTENTE    = 4,  // Wait
    ETAT_CATAPULTE  = 5,  // Fire
} Etat_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_CLOCK_HZ   32000000.0f  // TIM2 clock frequency (Hz)
#define FREQ_MIN         29000.0f     // Minimal frequency of the entry (Hz)
#define FREQ_MAX         62000.0f     // Maximal frequency of the entry (Hz)
#define FILTER_SIZE      10           // Filter size
#define DEBOUNCE_MS      200          // Anti-rebound button (ms)

// Mechanical limits servo 1
#define SERVO1_ANGLE_MIN 40
#define SERVO1_ANGLE_MAX 125

// Mechanical limits servo 2 and 3
#define SERVO2_ANGLE_MIN 0
#define SERVO2_ANGLE_MAX 180
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ISR variables (volatile)
static volatile uint32_t ic1_prev       = 0;
static volatile uint32_t diff1          = 0;
static volatile uint8_t  first_capture1 = 0;
static volatile uint8_t  ready1         = 0;

// Filter
static uint32_t capture_buffer1[FILTER_SIZE] = {0};
static uint8_t  filter_index1  = 0;
static uint32_t filter_sum1    = 0;
static uint8_t  filter_count1  = 0;  // Number of valid samples (≤ FILTER_SIZE)

// State machine
static Etat_t   etat           = ETAT_REPOS;
static uint8_t  angle1_bloque  = 90;
static uint8_t  angle2_bloque  = 90;
static uint32_t last_button_time = 0;

// Mesured frequency
static uint32_t freq1 = 0;

// UART for debug
static char     uart_buf[64];
static uint16_t print_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t servo_angle_to_ccr(uint8_t angle, uint8_t angle_min, uint8_t angle_max);
static uint32_t moving_average1(uint32_t new_val);
static uint8_t  freq_to_angle(uint32_t freq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// convert angle to ccr
static uint32_t servo_angle_to_ccr(uint8_t angle, uint8_t angle_min, uint8_t angle_max)
{
    if (angle < angle_min) angle = angle_min;
    if (angle > angle_max) angle = angle_max;
    return 150 + ((uint32_t)angle * 750) / 180;
}

// Moving average filter
// new_val is the new value to integrate.
static uint32_t moving_average1(uint32_t new_val)
{
    filter_sum1 -= capture_buffer1[filter_index1];
    capture_buffer1[filter_index1] = new_val;
    filter_sum1 += new_val;

    filter_index1 = (filter_index1 + 1) % FILTER_SIZE;

    if (filter_count1 < FILTER_SIZE)
    {
        filter_count1++;
    }

    // Avoid division by zero on the first call
    if (filter_count1 == 0)
    {
        return new_val;
    }

    return filter_sum1 / filter_count1;
}

// Converts a sensor frequency into a servo angle (0°–180°).
/* The frequency is clamped to [FREQ_MIN, FREQ_MAX] before conversion
 to prevent overflow or out-of-range values. */
static uint8_t freq_to_angle(uint32_t freq)
{
    float f = (float)freq;
    if (f < FREQ_MIN) f = FREQ_MIN;
    if (f > FREQ_MAX) f = FREQ_MAX;
    return (uint8_t)(180.0f - (FREQ_MAX - f) * 180.0f / (FREQ_MAX - FREQ_MIN));
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    // UART TEST at startup
    char test_buf[32];
    uint8_t test_len = sprintf(test_buf, "boot ok\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)test_buf, test_len, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET)
		{
			uint32_t now = HAL_GetTick();
			if ((now - last_button_time) > DEBOUNCE_MS)
			{
				last_button_time = now;
				switch (etat)
				{
					case ETAT_REPOS:
						etat = ETAT_RECHARGE;
						break;
					case ETAT_RECHARGE:
						etat = ETAT_SET_SERVO1;
						break;
					case ETAT_SET_SERVO1:
						angle1_bloque = freq_to_angle(freq1);
						etat = ETAT_SET_SERVO2;
						break;
					case ETAT_SET_SERVO2:
						angle2_bloque = freq_to_angle(freq1);
						etat = ETAT_ATTENTE;
						break;
					case ETAT_ATTENTE:
						etat = ETAT_CATAPULTE;
						break;
					case ETAT_CATAPULTE:
						etat = ETAT_REPOS;
						break;
					default:
						etat = ETAT_REPOS;
						break;
				}
			}
		}

		if (ready1)
		{
			uint32_t local_diff;
			__disable_irq();
			local_diff = diff1;
			ready1 = 0;
			__enable_irq();

			uint32_t diff_filtered = moving_average1(local_diff);
			freq1 = (diff_filtered > 0) ? (uint32_t)(TIMER_CLOCK_HZ / (float)diff_filtered) : 0;

			uint8_t angle_live = freq_to_angle(freq1);

			uint8_t angle1;
			if (etat == ETAT_REPOS || etat == ETAT_RECHARGE)
				angle1 = SERVO1_ANGLE_MAX;
			else if (etat == ETAT_SET_SERVO1)
				angle1 = angle_live;
			else
				angle1 = angle1_bloque;

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
				servo_angle_to_ccr(angle1, SERVO1_ANGLE_MIN, SERVO1_ANGLE_MAX));

			uint8_t angle2 = (etat == ETAT_SET_SERVO2) ? angle_live : angle2_bloque;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
				servo_angle_to_ccr(angle2, SERVO2_ANGLE_MIN, SERVO2_ANGLE_MAX));

			uint8_t angle3 = (etat == ETAT_REPOS || etat == ETAT_CATAPULTE) ? 180 : 90;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
				servo_angle_to_ccr(angle3, SERVO2_ANGLE_MIN, SERVO2_ANGLE_MAX));

			if (++print_cnt >= 500)
			{
				print_cnt = 0;
				if (huart2.gState == HAL_UART_STATE_READY)
				{
					uint8_t len = sprintf(uart_buf, "freq=%lu Hz  angle=%u\r\n",
										  freq1, (unsigned)freq_to_angle(freq1));
					HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, len);
				}
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

    /* Bouton utilisateur → PA12, pull-up interne */
    GPIO_InitStruct.Pin  = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// TIM input capture callback.
// Measures the period between two rising edges on TIM2 CH1.
// diff1 contains the number of ticks between two successive captures.
// ready1 is set to 1 to signal that a new measurement is available.

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (!first_capture1)
        {
            ic1_prev       = capture;
            first_capture1 = 1;
        }
        else
        {
            diff1  = capture - ic1_prev;
            ic1_prev = capture;
            ready1 = 1;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(200);
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
    (void)file;
    (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
