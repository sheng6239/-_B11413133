/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 美美小香腳－智慧除濕烘乾鞋架
  *                   STM32 NUCLEO-G474RE / STM32CubeIDE / HAL
  *
  * 功能：
  *   1. SAFE       : PA12 綠燈亮，PWM LED 關閉
  *   2. WET_WARN   : PB12 紅燈亮，等待 PC9 啟動
  *   3. DRYING     : PA11 黃燈亮，PB2 輸出 1 kHz / 20% PWM
  *   4. PC6        : EXTI，模擬濕度過高／恢復乾燥
  *   5. PC8        : EXTI，Demo 快進至剩餘 3 秒
  *   6. PC13       : EXTI，板載 B1 緊急重置（按下為 HIGH）
  *   7. PC9        : Polling，紅燈狀態啟動、烘乾狀態提前停止
  *   8. 烘乾 2400 秒後自動停止
  *
  * 注意：PB2 的 PWM 是 LED 展示，不是實際風扇馬達輸出。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#define DRYING_TIME_SEC          (2400U)     /* 40 分鐘 */
#define DEMO_REMAINING_SEC       (3U)        /* Demo 剩餘 3 秒 */
#define BUTTON_DEBOUNCE_MS       (50U)

/* TIM5 clock = 170 MHz（本程式 APB1 prescaler = 1）
 * PWM = 170 MHz / ((169 + 1) * (999 + 1)) = 1 kHz
 * Duty = 200 / (999 + 1) = 20%
 */
#define TIM5_PRESCALER           (169U)
#define TIM5_PERIOD              (999U)
#define PWM_LED_ON_COMPARE       (200U)
#define PWM_LED_OFF_COMPARE      (0U)

/* EXTI event flags */
#define EVENT_HUMIDITY           (0x01U)     /* PC6 */
#define EVENT_DEMO               (0x02U)     /* PC8 */
#define EVENT_RESET              (0x04U)     /* PC13 */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATE_SAFE = 0,
    STATE_DRYING,
    STATE_WET_WARN
} SystemState_t;

typedef struct
{
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t last_change_tick;
} ButtonDebounce_t;

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;

static SystemState_t g_system_state = STATE_SAFE;
static uint32_t g_remaining_seconds = 0U;
static uint32_t g_last_second_tick = 0U;

/* EXTI debounce state. ISR sets pending bits; main loop validates after 50 ms. */
static volatile uint8_t g_exti_pending = 0U;
static volatile uint32_t g_pc6_edge_tick = 0U;
static volatile uint32_t g_pc8_edge_tick = 0U;
static volatile uint32_t g_pc13_edge_tick = 0U;

static ButtonDebounce_t g_pc9_button;

/* Private function prototypes ----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void SetSystemState(SystemState_t new_state);
static void HandleHumidityEvent(void);
static void HandleDemoEvent(void);
static void HandleManualButton(void);
static uint8_t ProcessExtiDebounce(void);
static void ButtonDebounce_Init(ButtonDebounce_t *button,
                                GPIO_TypeDef *port,
                                uint16_t pin);
static uint8_t ButtonDebounce_GetPressEvent(ButtonDebounce_t *button,
                                            GPIO_TypeDef *port,
                                            uint16_t pin,
                                            uint32_t now);
static void UpdateDryingTimer(void);
void Error_Handler(void);

/* Main program --------------------------------------------------------------*/
int main(void)
{
    uint8_t exti_events;
    uint32_t now;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM5_Init();

    /* PWM compare 先設為 0，再啟動輸出，避免開機瞬間亮起。 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_LED_OFF_COMPARE);
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    ButtonDebounce_Init(&g_pc9_button, GPIOC, GPIO_PIN_9);

    /* 開機進入乾爽待機狀態。 */
    SetSystemState(STATE_SAFE);

    while (1)
    {
        now = HAL_GetTick();

        /* 先處理 EXTI 按鈕，並在主迴圈完成 50 ms 穩定狀態確認。 */
        exti_events = ProcessExtiDebounce();

        /* 緊急重置優先權最高；同一輪其他 EXTI 事件不再執行。 */
        if ((exti_events & EVENT_RESET) != 0U)
        {
            SetSystemState(STATE_SAFE);
        }
        else
        {
            if ((exti_events & EVENT_HUMIDITY) != 0U)
            {
                HandleHumidityEvent();
            }

            if ((exti_events & EVENT_DEMO) != 0U)
            {
                HandleDemoEvent();
            }
        }

        /* PC9 使用非阻塞式輪詢，不使用 HAL_Delay 等待放開。 */
        if (ButtonDebounce_GetPressEvent(&g_pc9_button,
                                         GPIOC,
                                         GPIO_PIN_9,
                                         now) != 0U)
        {
            HandleManualButton();
        }

        UpdateDryingTimer();

        /* Sleep：SysTick 約每 1 ms 喚醒一次，讓 PC9 輪詢仍可運作。
         * 這是 Cortex-M Sleep，不是 Stop/Standby 深度低功耗模式。
         */
        __WFI();
    }
}

/* State machine -------------------------------------------------------------*/
static void SetSystemState(SystemState_t new_state)
{
    g_system_state = new_state;

    /* 先全部熄滅，避免狀態燈重疊。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    /* 進入任何非烘乾狀態時，PWM LED 關閉、倒數清零。 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_LED_OFF_COMPARE);
    g_remaining_seconds = 0U;

    switch (new_state)
    {
        case STATE_SAFE:
            /* PA12：綠燈 */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
            break;

        case STATE_WET_WARN:
            /* PB12：紅燈 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            break;

        case STATE_DRYING:
            /* PA11：黃燈；PB2：PWM LED 20% */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim5,
                                  TIM_CHANNEL_1,
                                  PWM_LED_ON_COMPARE);
            g_remaining_seconds = DRYING_TIME_SEC;
            g_last_second_tick = HAL_GetTick();
            break;

        default:
            /* 不接受未定義狀態，回到安全狀態。 */
            g_system_state = STATE_SAFE;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
            break;
    }
}

static void HandleHumidityEvent(void)
{
    /* PC6 只在 SAFE / WET_WARN 間切換；烘乾期間忽略，避免誤停機。 */
    if (g_system_state == STATE_SAFE)
    {
        SetSystemState(STATE_WET_WARN);
    }
    else if (g_system_state == STATE_WET_WARN)
    {
        SetSystemState(STATE_SAFE);
    }
}

static void HandleDemoEvent(void)
{
    if (g_system_state == STATE_DRYING)
    {
        /* Demo 快進後重新設定計時基準，確保有完整約 3 秒。 */
        g_remaining_seconds = DEMO_REMAINING_SEC;
        g_last_second_tick = HAL_GetTick();
    }
}

static void HandleManualButton(void)
{
    if (g_system_state == STATE_WET_WARN)
    {
        /* 只有過濕警示狀態才能啟動烘乾。 */
        SetSystemState(STATE_DRYING);
    }
    else if (g_system_state == STATE_DRYING)
    {
        /* 烘乾期間再按一次 PC9，提前停止並回到 SAFE。 */
        SetSystemState(STATE_SAFE);
    }
    else
    {
        /* SAFE 狀態按 PC9 不動作，必須先有過濕警示。 */
    }
}

static void UpdateDryingTimer(void)
{
    uint32_t now;
    uint32_t elapsed_seconds;

    if (g_system_state != STATE_DRYING)
    {
        return;
    }

    now = HAL_GetTick();
    elapsed_seconds = (uint32_t)(now - g_last_second_tick) / 1000U;

    if (elapsed_seconds == 0U)
    {
        return;
    }

    /* 以經過的整秒數扣除，避免主迴圈執行延遲造成倒數逐漸漂移。 */
    g_last_second_tick += elapsed_seconds * 1000U;

    if (elapsed_seconds >= g_remaining_seconds)
    {
        SetSystemState(STATE_SAFE);
    }
    else
    {
        g_remaining_seconds -= elapsed_seconds;
    }
}

/* Non-blocking PC9 polling debounce ----------------------------------------*/
static void ButtonDebounce_Init(ButtonDebounce_t *button,
                                GPIO_TypeDef *port,
                                uint16_t pin)
{
    GPIO_PinState initial_state = HAL_GPIO_ReadPin(port, pin);

    button->raw_state = initial_state;
    button->stable_state = initial_state;
    button->last_change_tick = HAL_GetTick();
}

static uint8_t ButtonDebounce_GetPressEvent(ButtonDebounce_t *button,
                                            GPIO_TypeDef *port,
                                            uint16_t pin,
                                            uint32_t now)
{
    GPIO_PinState reading = HAL_GPIO_ReadPin(port, pin);

    if (reading != button->raw_state)
    {
        button->raw_state = reading;
        button->last_change_tick = now;
    }

    if (((uint32_t)(now - button->last_change_tick) >= BUTTON_DEBOUNCE_MS) &&
        (button->stable_state != button->raw_state))
    {
        button->stable_state = button->raw_state;

        /* PC9 使用上拉，按下為 LOW；只在穩定按下時回報一次。 */
        if (button->stable_state == GPIO_PIN_RESET)
        {
            return 1U;
        }
    }

    return 0U;
}

/* EXTI debounce: wait 50 ms, then sample the actual pin level ---------------*/
static uint8_t ProcessExtiDebounce(void)
{
    uint8_t ready_events = 0U;
    uint32_t now;
    uint32_t primask = __get_PRIMASK();

    /* 避免主迴圈清除 pending bit 時，剛好被 EXTI ISR 修改。 */
    __disable_irq();
    /* 必須在關閉中斷後讀 tick，避免 edge_tick 比 now 更新而產生無號下溢。 */
    now = HAL_GetTick();

    if (((g_exti_pending & EVENT_HUMIDITY) != 0U) &&
        ((uint32_t)(now - g_pc6_edge_tick) >= BUTTON_DEBOUNCE_MS))
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            ready_events |= EVENT_HUMIDITY;
        }
        g_exti_pending &= (uint8_t)(~EVENT_HUMIDITY);
    }

    if (((g_exti_pending & EVENT_DEMO) != 0U) &&
        ((uint32_t)(now - g_pc8_edge_tick) >= BUTTON_DEBOUNCE_MS))
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET)
        {
            ready_events |= EVENT_DEMO;
        }
        g_exti_pending &= (uint8_t)(~EVENT_DEMO);
    }

    if (((g_exti_pending & EVENT_RESET) != 0U) &&
        ((uint32_t)(now - g_pc13_edge_tick) >= BUTTON_DEBOUNCE_MS))
    {
        /* 板載 B1 按下時 PC13 為 HIGH。 */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
        {
            ready_events |= EVENT_RESET;
        }
        g_exti_pending &= (uint8_t)(~EVENT_RESET);
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return ready_events;
}

/* GPIO EXTI callback ---------------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_6)
    {
        if ((g_exti_pending & EVENT_HUMIDITY) == 0U)
        {
            g_pc6_edge_tick = now;
            g_exti_pending |= EVENT_HUMIDITY;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_8)
    {
        if ((g_exti_pending & EVENT_DEMO) == 0U)
        {
            g_pc8_edge_tick = now;
            g_exti_pending |= EVENT_DEMO;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_13)
    {
        if ((g_exti_pending & EVENT_RESET) == 0U)
        {
            g_pc13_edge_tick = now;
            g_exti_pending |= EVENT_RESET;
        }
    }
}

/* TIM5 PWM initialization ----------------------------------------------------*/
static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = TIM5_PRESCALER;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = TIM5_PERIOD;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_LED_OFF_COMPARE;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* PB2 = AF2 TIM5_CH1。 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* GPIO initialization --------------------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 先設定輸出初始電位為 LOW。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    /* PA11 黃燈、PA12 綠燈。 */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB12 紅燈。 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PC6、PC8：外接按鈕，按下接 GND，使用內部上拉與下降沿。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC13：NUCLEO-G474RE 板載 B1，按下為 HIGH。
     * 板上已配置外部下拉電阻與電容，因此使用 NOPULL + RISING。
     */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC9：外接輪詢按鈕，按下接 GND。 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI6 / EXTI8 共用 EXTI9_5；PC13 使用 EXTI15_10。 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* System clock configuration -----------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Error handler -------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* 發生初始化錯誤時停在這裡，方便用除錯器檢查。 */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
