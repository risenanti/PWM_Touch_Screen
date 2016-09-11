/**
  ******************************************************************************
  * @file    main.c
  * @author  Keith Conley
  * @version V1.0
  * @date    1 MAY 2016
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f7xx.h"
#include "stm32746g_discovery.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include <stdio.h>

#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR
#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

//DEFINES FOR PWM
#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PERIODPWMSTEP      (uint32_t)(6.65)  /* Period Value  */


/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;
/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

static void SystemClock_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
void Setup(void);
void GPIOSetup(void);
void PWMSetup(void);
void LCD_TEXT_DEFAULT(void);
void Display_LCD_Button(void);
void Display_LCD_SetPWM(void);

//GPIO Variables
GPIO_InitTypeDef  gpio_init_structure;
GPIO_TypeDef*     gpio_toggle = GPIOB;

//Variables for Touch state
static TS_StateTypeDef  TS_State;
uint16_t x, y;

//PWMVAR
uint16_t PWMSETPOINT;

int main(void)
{
	//SysCoreClock etc
	Setup();
	//GPIO Stuff
	GPIOSetup();
	//TIMER 3 Channel 1 on pin PB3 AKA D3
	PWMSetup();

    //button exists between 220 to 260 x and 116 to 156 y
    Display_LCD_Button();

    PWMSETPOINT = 50;
    Display_LCD_SetPWM();

	while(1)
	{
		printf("HI!");
		//50% PWM Cycle if Middle is pushed
		//else 12.5% or whatever 50/8 is
		//Value only changes on Touch EG Touch Button 50%
		//Touch Blank Space 12.5%
		BSP_TS_GetState(&TS_State);
		    if(TS_State.touchDetected)
		    {
		      /* Get X and Y position of the touch post calibrated */
		      x = TS_State.touchX[0];
		      y = TS_State.touchY[0];

		      //RESET
		      if (x > 220 && x<260 && y>116 && y< 156)
		      {
				  sConfig.Pulse = PULSE1_VALUE;
				  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
				  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
				  PWMSETPOINT = 50;
				  char myBuff[20];
				  sprintf(myBuff, "%u", PWMSETPOINT);

				    BSP_LCD_SetFont(&Font16);
				    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				  BSP_LCD_DisplayStringAt(340, 130, (uint8_t *)myBuff, LEFT_MODE);
				  LCD_TEXT_DEFAULT();
		      }
		      //INCREMENT PWM
		      else if (x>320 && x<460 && y>0 && y<80)
		      {
				  PWMSETPOINT++;
				  HAL_Delay(10);

				  if (PWMSETPOINT>99)
				  {
					  PWMSETPOINT = 99;
				  }

				  char myBuff[2];
				  sprintf(myBuff, "%u", PWMSETPOINT);
				    BSP_LCD_SetFont(&Font16);
				    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				  BSP_LCD_DisplayStringAt(340, 130, (uint8_t *)myBuff, LEFT_MODE);
				  LCD_TEXT_DEFAULT();

				  //set PWM
				  sConfig.Pulse = (PWMSETPOINT * PERIODPWMSTEP);
				  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
				  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
				  HAL_Delay(50);
		      }
		      //DECREMENT PWM
		      else if (x>320 && x<460 && y>190 && y<270)
		      {
				  PWMSETPOINT--;
				  HAL_Delay(10);

				  if (PWMSETPOINT>99)
				  {
					  PWMSETPOINT = 0;
				  }

				  char myBuff[20];
				  sprintf(myBuff, "%u", PWMSETPOINT);
				    BSP_LCD_SetFont(&Font16);
				    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				  BSP_LCD_DisplayStringAt(340, 130, (uint8_t *)myBuff, LEFT_MODE);
				  LCD_TEXT_DEFAULT();

				  //set PWM
				  sConfig.Pulse = (PWMSETPOINT * PERIODPWMSTEP);
				  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
				  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
				  HAL_Delay(50);
		      }
		    }
	}
}

//480x272 resolution
void Display_LCD_SetPWM()
{
	//PWM++ Button
	BSP_LCD_DrawRect(320, 0, 140, 80);
	BSP_LCD_FillRect(320, 0, 140, 80);

	//DISP Value
	BSP_LCD_DrawRect(320, 90, 140, 80);
	BSP_LCD_FillRect(320, 90, 140, 80);

	//PWM-- Button
	BSP_LCD_DrawRect(320, 190, 140, 80);
	BSP_LCD_FillRect(320, 190, 140, 80);

    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(340, 40, (uint8_t *)"PWM++", LEFT_MODE);
    BSP_LCD_DisplayStringAt(340, 230, (uint8_t *)"PWM--", LEFT_MODE);

    /*DISP Val*/
	  char myBuff[20];
	  sprintf(myBuff, "%u", PWMSETPOINT);
	  BSP_LCD_DisplayStringAt(340, 130, (uint8_t *)myBuff, LEFT_MODE);
    /**/
    LCD_TEXT_DEFAULT();


}

//setup for PB4 pwm arduino connector D3
void GPIOSetup()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __TIM3_CLK_ENABLE();

    gpio_init_structure.Pin = GPIO_PIN_4;
    //SETUP for alternate Function
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(gpio_toggle, &gpio_init_structure);
}

void PWMSetup()
{
	  /* Compute the prescaler value to have TIM3 counter clock equal to 18000000 Hz */
      uhPrescalerValue = (uint32_t)((SystemCoreClock /12) / 18000000) - 1;

	  TimHandle.Instance = TIM3;
	  TimHandle.Init.Prescaler         = uhPrescalerValue;
	  TimHandle.Init.Period            = PERIOD_VALUE;
	  TimHandle.Init.ClockDivision     = 0;
	  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	  TimHandle.Init.RepetitionCounter = 0;
	  TimHandle.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	  HAL_TIM_PWM_Init(&TimHandle);

	  /*##-2- Configure the PWM channels #########################################*/
	  /* Common configuration for all channels */
	  sConfig.OCMode       = TIM_OCMODE_PWM1;
	  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

	  /* Set the pulse value for channel 1 */
	  sConfig.Pulse = PULSE1_VALUE;

	  HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
}

void Setup()
{
    MPU_Config();
    /* Enable the CPU Cache */
    CPU_CACHE_Enable();
    /* Configure the System clock to have a frequency of 216 MHz */
    SystemClock_Config();
	HAL_Init();

    // Set up the LCD
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER, ENABLE);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DisplayOn();

    //Set up TS
    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
}

void LCD_TEXT_DEFAULT()
{
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font24);
}

void Display_LCD_Button(void)
{
    BSP_LCD_DrawCircle(BSP_LCD_GetXSize()-(BSP_LCD_GetXSize()/2), BSP_LCD_GetYSize()-(BSP_LCD_GetYSize()/2), 40);
    BSP_LCD_FillCircle(BSP_LCD_GetXSize()-(BSP_LCD_GetXSize()/2), BSP_LCD_GetYSize()-(BSP_LCD_GetYSize()/2), 40);

    BSP_LCD_SetFont(&Font16);
    //480x272 resolution
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(220, BSP_LCD_GetYSize()-(BSP_LCD_GetYSize()/2), (uint8_t *)"RESET", LEFT_MODE);
    LCD_TEXT_DEFAULT();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
