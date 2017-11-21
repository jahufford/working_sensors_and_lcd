/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.4
  * @date    17-February-2017
  * @brief   This example shows how to retarget the C library printf function 
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ili9341.h"
#include "lcd_hardware.h"
#include "error.h"
#include <limits.h>
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
UART_HandleTypeDef UartHandleWireless;


/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
char received_char;
uint8_t has_new_char;

//#define ADCx ADC1
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
    HAL_Init();
  

  /* Configure the system clock to 180 MHz */
    SystemClock_Config();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin : LD2_Pin */
    GPIO_InitTypeDef GPIO_InitStruct;
//    GPIO_InitStruct.Pin = LD2_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

// set up the button
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // setup GPIO for PWM output
    GPIO_InitTypeDef GPIO_InitStruct2;
//    GPIO_InitStruct2.Pin = GPIO_PIN_15;
//    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
//   // GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct2.Alternate = GPIO_AF9_TIM12;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);
    GPIO_InitStruct2.Pin = GPIO_PIN_15;
    GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct2.Pull = GPIO_NOPULL;
    //GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct2.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);


// generate interrupt when button is pressed
    //SYSCFG_IEXTICR4;
    volatile uint32_t reg;
    reg = SYSCFG->EXTICR[3];
    //reg = 0xFFFFFFFF;
    reg &= ~(0xF << 4);
    //reg |= (0x2 << 4);
    reg |= (SYSCFG_EXTICR4_EXTI13_PC);
    //reg |= (SYSCFG_EXTICR4_EXTI13_PB);
    SYSCFG->EXTICR[3] = reg;
    EXTI->RTSR &= ~(1<<13);
    //EXTI->FTSR |= (1<<13);
    EXTI->FTSR |= EXTI_FTSR_TR13;
    //EXTI->IMR |= (1<<13);
    EXTI->IMR |= EXTI_IMR_MR13;
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);


    __HAL_RCC_TIM12_CLK_ENABLE();
    // set up a basic ticker
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM_HandleTypeDef h_basic_ticker;
    //h_basic_ticker.Instance = BASIC_TICKERx;
    h_basic_ticker.Instance = TIM5;
    h_basic_ticker.State = HAL_TIM_STATE_RESET;
    h_basic_ticker.Init.ClockDivision = 0;
    h_basic_ticker.Init.CounterMode = TIM_COUNTERMODE_UP;
    h_basic_ticker.Init.Period = UINT_MAX;
//    h_basic_ticker.Init.Period = 0xFFFFFFFF;
    h_basic_ticker.Init.Prescaler = 0;
    HAL_TIM_Base_Init(&h_basic_ticker);
    HAL_TIM_Base_Start(&h_basic_ticker);

    // configure timer
//    TIM_HandleTypeDef h_timer;
//    TIM_ClockConfigTypeDef h_timer_clk_config;
//    h_timer.Instance = TIMx;
//    h_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//    h_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
//    h_timer.Init.Period = 0xFFFF; // Max
//    h_timer.Init.Prescaler = 0;
//    h_timer.State = HAL_TIM_STATE_RESET;
//    h_timer.Channel = HAL_TIM_ACTIVE_CHANNEL_2;
//
//    h_timer_clk_config.ClockFilter = 5;
//    h_timer_clk_config.ClockPolarity = TIM_CLOCKPOLARITY_FALLING;
//    h_timer_clk_config.ClockPrescaler = 0;
//    h_timer_clk_config.ClockSource = TIM_CLOCKSOURCE_TI2;
//    HAL_TIM_ConfigClockSource(&h_timer, &h_timer_clk_config);
    //reg = h_timer.Instance->CCER;
    //asm("nop");
    // select TI2, external pin as clock
//    reg = h_timer.Instance->CCMR1;
//    reg |= (0x1<<8);
//    reg &= ~(0x1<<9);
//    h_timer.Instance->CCMR1 = reg;
//    reg = h_timer.Instance->CCMR1;
//    // no filter
//    // rising edge
//    h_timer.Instance->CCER &= ~(1<<5);
//    h_timer.Instance->CCER &= ~(1<<3);
//    // configure the timer in external clock mode 1
//    h_timer.Instance->SMCR |= 0x7;
//    // select TI2 as the trigger input source
//    h_timer.Instance->SMCR |= (0x1<< 6);
//    h_timer.Instance->SMCR &= ~(0x3 << 4);
//    // enable the timer
//    h_timer.Instance->CR1 |= 1;

//
//
//    who the fuck knows why the above didn't work, but the API calls work fine
//    HAL_TIM_ConfigClockSource(&h_timer, &h_timer_clk_config);
//    HAL_TIM_Base_Init(&h_timer);
//    HAL_TIM_Base_Start(&h_timer);

    TIM_HandleTypeDef h_timer;
    TIM_OC_InitTypeDef h_timer_oc_config;
    h_timer.Instance = TIMx;
    h_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    //h_timer.Init.Period = 0xFFFF; // Max
    //h_timer.Init.Period = 0xFFFF;// Max
    h_timer.Init.Period = 1000;// Max
    h_timer.Init.Prescaler = 2;
    h_timer.State = HAL_TIM_STATE_RESET;
    //h_timer.Channel = HAL_TIM_ACTIVE_CHANNEL_2;

    h_timer_oc_config.OCFastMode = TIM_OCFAST_ENABLE;
    h_timer_oc_config.OCMode = TIM_OCMODE_PWM1;
    h_timer_oc_config.OCNIdleState = TIM_OCNIDLESTATE_SET;
    //h_timer_oc_config.OCNPolarity = TIM_OCNPOLARITY_LOW;
    h_timer_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    //h_timer_oc_config.Pulse = 0xFFFF/2;
    h_timer_oc_config.Pulse = 500;
    HAL_TIM_Base_Init(&h_timer);
    HAL_TIM_PWM_Init(&h_timer);
    //HAL_TIM_Base_Start(&h_timer);
	HAL_TIM_PWM_ConfigChannel(&h_timer, &h_timer_oc_config,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&h_timer,TIM_CHANNEL_2);

    // set up a-to-d
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // main adc for reading the temperature
    ENABLE_TEMP_ADC_CLK();
    ADC_HandleTypeDef h_adc;
    //h_adc.Instance = ADCx;
    h_adc.Instance = TEMP_ADCx;
    h_adc.State = HAL_ADC_STATE_RESET;
    h_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // set to value, but it's not used in async mode
    h_adc.Init.Resolution = ADC_RESOLUTION_12B;
    h_adc.Init.ContinuousConvMode = DISABLE;
    h_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    h_adc.Init.ScanConvMode = DISABLE;
    h_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    h_adc.Init.NbrOfConversion = 1;
    h_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    h_adc.Init.DMAContinuousRequests = DISABLE;
    h_adc.Init.NbrOfDiscConversion = 1;
    //h_adc.Init.
    HAL_ADC_Init(&h_adc);
    ADC_ChannelConfTypeDef adc_config;
    adc_config.Channel = ADC_CHANNEL_1;
    adc_config.Rank = 1;
    adc_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&h_adc, &adc_config);


// adc to read the temp sensor's virtual ground
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    //GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    ENABLE_GND_ADC_CLK();
    ADC_HandleTypeDef h_gnd_adc;
    h_gnd_adc.Instance = GND_ADCx;
    h_gnd_adc.State = HAL_ADC_STATE_RESET;
    h_gnd_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // set to value, but it's not used in async mode
    h_gnd_adc.Init.Resolution = ADC_RESOLUTION_12B;
    h_gnd_adc.Init.ContinuousConvMode = DISABLE;
    h_gnd_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    h_gnd_adc.Init.ScanConvMode = DISABLE;
    h_gnd_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    h_gnd_adc.Init.NbrOfConversion = 1;
    h_gnd_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    h_gnd_adc.Init.DMAContinuousRequests = DISABLE;
    h_gnd_adc.Init.NbrOfDiscConversion = 1;

    if(HAL_ADC_Init(&h_gnd_adc) != HAL_OK){
    	Error_Handler();
    }

    ADC_ChannelConfTypeDef gnd_adc_config;
    //gnd_adc_config.Channel = ADC_CHANNEL_4;
    gnd_adc_config.Channel =ADC_CHANNEL_8;
    gnd_adc_config.Rank = 1;
    gnd_adc_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if(HAL_ADC_ConfigChannel(&h_gnd_adc, &gnd_adc_config) != HAL_OK){
    	Error_Handler();
    }

    // adc to read the light sensor
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    //GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    ENABLE_LIGHT_ADC_CLK();
    ADC_HandleTypeDef h_light_adc;
    h_light_adc.Instance = LIGHT_ADCx;
    h_light_adc.State = HAL_ADC_STATE_RESET;
    h_light_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // set to value, but it's not used in async mode
    h_light_adc.Init.Resolution = ADC_RESOLUTION_12B;
    h_light_adc.Init.ContinuousConvMode = DISABLE;
    h_light_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    h_light_adc.Init.ScanConvMode = DISABLE;
    h_light_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    h_light_adc.Init.NbrOfConversion = 1;
    h_light_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    h_light_adc.Init.DMAContinuousRequests = DISABLE;
    h_light_adc.Init.NbrOfDiscConversion = 1;

    if(HAL_ADC_Init(&h_light_adc) != HAL_OK){
      Error_Handler();
    }

    ADC_ChannelConfTypeDef light_adc_config;
    light_adc_config.Channel = ADC_CHANNEL_10;
    light_adc_config.Rank = 1;
    light_adc_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if(HAL_ADC_ConfigChannel(&h_light_adc, &light_adc_config) != HAL_OK){
      Error_Handler();
    }

    //LCD_IO_Init();
    //toggle reset
//   	HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN,GPIO_PIN_SET);
//   	HAL_Delay(5);
//   	HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN,GPIO_PIN_RESET);
//   	HAL_Delay(5);
//   	HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN,GPIO_PIN_SET);
//   	HAL_Delay(5);


    uint16_t read_val=0xDEADBEEF;


    uint8_t limit = 1;
//	fillscreen(10);
//	for(;;){
//      fillscreen(limit);
//      limit++;
//	}
//	asm("nop");
    //ili9341_WriteReg(LCD_SWRESET);
    //HAL_Delay(50);

//    ili9341_Init();
//    HAL_Delay(1000);
//    ili9341_DisplayOn();
//    ili9341_WriteReg(LCD_SLEEP_OUT);
//    read_val = ili9341_ReadData(LCD_RDDISBV,5);
//    ili9341_WriteReg(LCD_WCD);
//    ili9341_WriteData(0x00);
//    read_val = ili9341_ReadData(LCD_RDDISBV,5);
//    ili9341_DisplayOn();
//    //read_val = ili9341_ReadID();
//    //ili9341_WriteReg(LCD_DINVON);
//    HAL_Delay(3000);
//    ili9341_DisplayOff();
//    uint16_t ili9341_GetLcdPixelWidth(void);
//    uint16_t ili9341_GetLcdPixelHeight(void);

    uint8_t spival = (uint8_t)'A';
//    for(;;){
//    	//__HAL_SPI_ENABLE(&h_lcd_spi);
//    	HAL_GPIO_WritePin(LCD_SPI_NSS_PORT,LCD_SPI_NSS_PIN,GPIO_PIN_RESET);
//    	if(HAL_SPI_Transmit(&h_lcd_spi,&spival, 1,0xFFFF) != HAL_OK){
//    		Error_Handler();
//    	}
//    	HAL_GPIO_WritePin(LCD_SPI_NSS_PORT,LCD_SPI_NSS_PIN,GPIO_PIN_SET);
//    	//__HAL_SPI_DISABLE(&h_lcd_spi);
//    	asm("nop");
//    }

    volatile uint32_t result;
    volatile uint32_t val;

    volatile uint32_t ticks = TIMx->CNT;
    volatile uint32_t ticks2;
	volatile uint32_t elapsed_time1, elapsed_time2;
    volatile uint32_t button_read;


//    for(;;){
//    	//button_read = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
//    	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//    	ticks = TIMx->CNT;
//    	reg = TIMx->SR;
////    	while(ticks2 -ticks < 100){
////    		ticks2 = TIMx->CNT;
////    	}
////    	while(TIMx->CNT -ticks < 10){
////    	}
//    	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//    	HAL_ADC_Start(&h_adc);
//    	result = HAL_ADC_PollForConversion(&h_adc,100);
//    	if(result != HAL_OK)
//    	{
//    		asm("nop");
//    	}
//    	val = HAL_ADC_GetValue(&h_adc);
//    	//val = h_adc.Instance->DR;
//    	asm("nop");
//    }
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  //UartHandle.Instance          = USARTx;
  UartHandle.Instance 		   = USART2;
  UartHandle.Init.BaudRate     = 9600;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  //UartHandle.Init.WordLength   = UART_WORDLENGTH_9B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  // to get parity to work right, you must use 9 bit transmission
  //UartHandle.Init.Parity       = UART_PARITY_ODD;
  //UartHandle.Init.Parity       = UART_PARITY_EVEN;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  USART2->CR1 |= (0x1 << 5); // enable receive interrupt
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_ClearPendingIRQ(USART2_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);


  __HAL_RCC_USART1_CLK_ENABLE();
  // configure UART to talk to wireless module
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  UartHandleWireless.Instance 		   = USART1;
  UartHandleWireless.Init.BaudRate     = 9600;
  UartHandleWireless.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandleWireless.Init.StopBits     = UART_STOPBITS_1;
  UartHandleWireless.Init.Parity       = UART_PARITY_NONE;
  UartHandleWireless.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandleWireless.Init.Mode         = UART_MODE_TX_RX;
  UartHandleWireless.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&UartHandleWireless) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  //printf("Hello World\r\n");
  char var = 'A';

  uint8_t disp_brightness = 0;
  uint32_t frame_cnt = 0;
  uint16_t color = LCD_MakeColor(0xFF,0xFF,0);
  volatile uint32_t varr;
  color = 0xFF;
//  LCD_WriteCmd(0x35);
//  LCD_WriteData(0);
//  LCD_WriteCmd(0x44);
//  LCD_WriteData(0);
//  LCD_WriteData(0);
  varr = 0;

//	varr = LCD_ReadData(0x45,2);
//	varr = LCD_ReadData(0x45,2);
//	varr = LCD_ReadData(0x45,2);
//	varr = LCD_ReadData(0x45,2);
//	asm("nop");
    LCD_Hardware_Init();
    LCD_Module_Init();
    //uint32_t disp_start_time = h_basic_ticker.Instance->CNT;
    //while(h_basic_ticker.Instance->CNT - disp_start_time < 90000000){
    //while(1){
      //LCD_FillScreen2(LCD_MakeColor(0,0xAF,0x10));
      LCD_FillScreen2(color);
      //LCD_FillCircle(320/2,240/2,90,LCD_MakeColor(0xFF,0,0));
      //LCD_DrawLine(10,20,15,22,LCD_MakeColor(0,0x00,0xFF));
      LCD_DrawHLine(0,0,329,0xFFFF);
      LCD_DrawHLine(0,239,329,0xFFFF);
      LCD_DrawVLine(0,1,238,0xFFFF);
      LCD_DrawVLine(319,1,238,0xFFFF);
      color+=10;
      frame_cnt++;
    //}
  //LCD_FillRect(0,0,319,239,0);
    LCD_FillRect(0,180,22,9,color);
  //LCD_FillRect(50,10,80,45,color);
//    for(int uu=5;uu<15;uu++){
    LCD_DrawHLine(40,10,5,LCD_MakeColor(0xFF,0,0));
//    }
    LCD_DrawVLine(50,10,80,0xFF);
    //LCD_PutPixel(0,0,0xFFFF);
    LCD_PutPixel(319,0,0xFFFF);
    uint16_t cx=50,cy=50,radius = 20;
    LCD_FillCircle(320/2,240/2,100,LCD_MakeColor(0xFF,0xFF,0));
    LCD_FillCircle(320/2,240/2,80,LCD_MakeColor(0xFF,0,0));
    LCD_FillCircle(320/2,240/2,60,LCD_MakeColor(0xaF,0xcc,0));
    LCD_FillCircle(320/2,240/2,40,LCD_MakeColor(0x00,0xFF,0));
//  LCD_PutPixel(0,239,0xFFFF);
  //  LCD_PutPixel(319,239,0xFFFF);
  //  LCD_PutPixel(1,138, LCD_MakeColor(0,0xFF,0));
    volatile uint32_t max_avg=0;
    volatile uint32_t min_avg=0xFFFF;
    for(;;){
  //	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
  //		printf("Button Down\n");
  //	  }
  //	  HAL_UART_Transmit(&UartHandle, (uint8_t*)&var,1,0xFFFF);
  //      HAL_UART_Transmit(&UartHandleWireless, (uint8_t*)&var,1,0xFFFF);
  //	  //printf("%c", var);
  //	  //fflush(stdout);
  //	  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //	  HAL_Delay(1000);
  //	  if(var=='z'){
  //		  var = 'A';
  //		  //printf("\r\n");
  //		  printf("\n");
  //	  }else if(var=='Z'){
  //		  var = 'a';
  //	  }
  //	  var++;
  //	  if(has_new_char){
  //		  has_new_char = 0;
  //		  HAL_UART_Transmit(&UartHandle,(uint8_t*)&received_char,1,0xFFFF);
  //	  }

	  volatile uint32_t sum = 0;
	  volatile uint32_t min = 0xFFFF;
	  volatile uint32_t min2 = 0xFFFF;
	  volatile uint32_t max = 0;
	  volatile uint32_t max2 = 0;
	  volatile uint32_t gnd_read;
	  volatile uint32_t light_read, light_sum;

	  for(int i=0;i<30;i++){
         if(HAL_ADC_Start(&h_adc)!= HAL_OK){
           Error_Handler();
         }
         result = HAL_ADC_PollForConversion(&h_adc,100);
         if(result != HAL_OK)
         {
           printf("FUCKFUCKFUCKFUCKFUCK\n\r");
           asm("nop");
         }
         val = HAL_ADC_GetValue(&h_adc);
         if(val>=max){
           max2 = max;
           max = val;
         }
         if(val>=max2){
           max2 = val;
         }
         if(val<=min){
           min2 = min;
           min = val;
         }
         if(val<=min2){
           min2 = val;
         }
         if(val>2000){
           asm("nop");
         }
         sum+=val;
      // printf("%lu\r\n",val);
      }

      for(int i=0;i<30;i++){
        if(HAL_ADC_Start(&h_gnd_adc) != HAL_OK){
          Error_Handler();
        }
        if(HAL_ADC_PollForConversion(&h_gnd_adc,100) != HAL_OK){
          Error_Handler();
        }
        gnd_read = HAL_ADC_GetValue(&h_gnd_adc);
        //printf("Gnd = %lu\r\n",val);
	  }
      light_sum = 0;
      for(int i=0;i<30;i++){
        if(HAL_ADC_Start(&h_light_adc) != HAL_OK){
           Error_Handler();
        }
        if(HAL_ADC_PollForConversion(&h_light_adc,100) != HAL_OK){
           Error_Handler();
        }
        light_read = HAL_ADC_GetValue(&h_light_adc);
        light_sum += light_read;
      }
      light_read = light_sum/30;
      //printf("Gnd = %lu\r\n",gnd_read);

      //for(int a=0;a<5;a++){}
	  //uint32_t t1 = h_basic_ticker.Instance->CNT;

//	  uint32_t t2 = h_basic_ticker.Instance->CNT;
//	  uint32_t elapsed = ((t2-t1)*1111)/100000;
//
//      printf("Time to do 30 conversion %lu microseconds\n", elapsed);
	  // throw away the min and max's
	  sum -= max;
	  sum -= max2;
	  sum -= min;
	  sum -= min2;
	  volatile uint32_t avg = sum/26;
//	  min_avg = (avg<min_avg)? avg : min_avg;
//	  max_avg = (avg>max_avg)? avg : max_avg;
	  if(avg<min_avg){
		  min_avg = avg;
	  }
	  if(avg>max_avg){
		  max_avg = avg;
	  }
	  if(max_avg>2000){
		  asm("nop");
	  }
	  int temp_diff = avg - gnd_read;
	  int temp = (int)((temp_diff/12.412)*9)/5 + 32;
	  printf("temp is %d, light is %4d %d \r", temp,light_read,limit);
	  fprintf(stdout,"Hello World\r\n");
	  fflush(stdout);
	  // then average the rest
	  //printf("average=%lu, min=%lu, max=%lu, difference=%lu, max_avg=%lu, min_avg=%lu\r\n----------------------\r\n", avg,min,max,max-min,max_avg,min_avg);
	  HAL_Delay(250);
	  if(has_new_char){
		  if(received_char=='u'){
			  limit++;
		  }else if(received_char=='d'){
			  limit--;
		  }else if(received_char=='c'){
			  LCD_FillCircle(cx,cy,radius,0);
			  cy+=10;
			  LCD_FillCircle(cx,cy,radius,color);
		  }else if(received_char=='v'){
			  LCD_FillCircle(cx,cy,radius,0);
			  cx+=10;
			  LCD_FillCircle(cx,cy,radius,color);
		  }else if(received_char=='f'){
			  LCD_FillScreen2(LCD_MakeColor(0x0,0,0xFF));
		  }
		  has_new_char = 0;
	  }
      //val = h_adc.Instance->DR;
      asm("nop");
}
//  for(;;){
//    for(int i=0;i<17;i++){
//      HAL_UART_Transmit(&UartHandle, (uint8_t*)&v,1,0xFFFF);
//      ++v;
//      v %= 17;
//      HAL_Delay(2000);
//    }
//  }
//  /* Output a message on Hyperterminal using printf function */
  printf("UART Printf Example: retarget the C library printf function to the UART\n\r");
// // printf("ABCDEFG\n");
//  unsigned char x[5] = {0x5,0x00,0xCC,0xAA,0};
//  HAL_UART_Transmit(&UartHandle, (uint8_t *)&x[0],1 , 0xFFFF);

  /* Infinite loop */ 
  while (1)
  {
	//  printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
//	  HAL_Delay(500);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows: 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
   /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
