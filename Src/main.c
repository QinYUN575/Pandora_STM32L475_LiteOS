/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "los_base.h"
#include "mcu_init.h"
#include "los_inspect_entry.h"
#include "lcd.h"
#include "liteos_logo.h"

#include "los_task.h"
#include "los_api_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static UINT32 g_Task1_Handle;
static UINT32 g_Task2_Handle;

#define TSK_PRIOR_HI 4
#define TSK_PRIOR_LO 5

static VOID *Example_Task1(UINT32 uwArg);
static VOID *Example_Task2(UINT32 uwArg);
UINT32 CreatAppTask(void);
/********************************************************/
//LCD刷屏时使用的颜色
int lcd_discolor[13] = {WHITE, BLUE, BRED, GRED, GBLUE, RED, MAGENTA,
                        GREEN, CYAN, YELLOW, BROWN, BRRED, GRAY};

/********************************************************/

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
    MX_USART1_UART_Init();
    MX_SPI3_Init();
    /* USER CODE BEGIN 2 */

    LCD_Init();
    LCD_DisplayOn();
    LCD_Show_Image(10, 14, 220, 50, gImage_liteos_logo);

    BACK_COLOR = WHITE;
    POINT_COLOR = BLUE;
    LCD_ShowString(10, 64, 240, 24, 24, "Pandora STM32L475");

    POINT_COLOR = RED;
    LCD_ShowString(10, 96, 240, 16, 16, "QinYUN575"
                                        "@" __DATE__);

    printf("MCU Hardware Initialize OK!\r\n");
    printf("\r\n===========================================================\r\n");
    printf("\r\n");
    printf("\tWelcome to Huawei LiteOS\r\n");
    printf("\tCompile Time:%s, %s\r\n", __TIME__, __DATE__);
    printf("\tCoding by QinYUN575\r\n");
    printf("\r\n");
    printf("===========================================================\r\n");

    if (LOS_OK != LOS_KernelInit())
    {
        printf("LiteOS Kernel Initialize Error!\r\n");
        return LOS_NOK;
    }

    printf("LiteOS Kernel Initialize Successful!\r\n");
    LOS_Inspect_Entry();
    CreatAppTask();
    LOS_Start();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure LSE Drive Capability 
  */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage 
  */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Enable MSI Auto calibration 
  */
    HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

UINT32 CreatAppTask(void)
{
    UINT32 uwRet = LOS_OK;
    TSK_INIT_PARAM_S task_init_param;

    // 创建第一个任务 Task_LED
    task_init_param.usTaskPrio = 2;
    task_init_param.pcName = "Task Led";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_Task1;
    task_init_param.uwStackSize = 0x400;
    uwRet = LOS_TaskCreate(&g_Task1_Handle, &task_init_param);
    if (LOS_OK != uwRet)
        return uwRet;

    // 创建第二个任务 Task_Red
    task_init_param.usTaskPrio = 3;
    task_init_param.pcName = "Task Red";
    task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)Example_Task2;
    task_init_param.uwStackSize = 0x400;
    uwRet = LOS_TaskCreate(&g_Task2_Handle, &task_init_param);
    if (LOS_OK != uwRet)
        return uwRet;

    return uwRet;
}
/**
* Taks1_task 任务函数
*
*/
// void entry_task1(void *arg)
static VOID *Example_Task1(UINT32 uwArg)
{
    uint8_t task1_count = 0;
    POINT_COLOR = BLUE;
    LCD_ShowString(10, 120, 215, 12, 12, "Task1 Run:");
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(10, 140, 90, 220);

    while (1)
    {
        LCD_Fill(11, 141, 89, 219, lcd_discolor[task1_count++ % 13]);
        LCD_ShowNum(75, 120, task1_count, 3, 12);
        HAL_GPIO_TogglePin(GPIOE, LED_R_Pin);
        LOS_TaskDelay(800);
    }
}

/**
* Taks2_task 任务函数
*
*/
// void entry_task2(void *arg)
static VOID *Example_Task2(UINT32 uwArg)
{
    uint8_t task2_count = 0;

    POINT_COLOR = BLUE;
    LCD_ShowString(135, 120, 215, 12, 12, "Task2 Run:");
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(135, 140, 215, 220);
    while (1)
    {
        LCD_Fill(136, 141, 214, 219, lcd_discolor[task2_count++ % 13]);
        LCD_ShowNum(195, 120, task2_count, 3, 12);
        HAL_GPIO_TogglePin(GPIOE, LED_B_Pin);
        LOS_TaskDelay(500);
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
    /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
