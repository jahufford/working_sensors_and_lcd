/*
 * lcd_hardware.h
 *
 *  Created on: Nov 5, 2017
 *      Author: joe
 */


#ifndef LCD_HARDWARE_H_
#define LCD_HARDWARE_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define LCD_SPIx						 SPI1
#define LCD_SPI_SCK_PIN					 GPIO_PIN_3
#define LCD_SPI_SCK_PORT				 GPIOB
#define LCD_SPI_MISO_PIN				 GPIO_PIN_4
#define LCD_SPI_MISO_PORT				 GPIOB
#define LCD_SPI_MOSI_PIN				 GPIO_PIN_5
#define LCD_SPI_MOSI_PORT				 GPIOB
#define LCD_SPI_NSS_PIN					 GPIO_PIN_10
#define LCD_SPI_NSS_PORT				 GPIOB
#define LCD_SPI_DC_PIN					 GPIO_PIN_8
#define LCD_SPI_DC_PORT                  GPIOA
#define LCD_RESET_PIN					 GPIO_PIN_7
#define LCD_RESET_PORT					 GPIOC

#define LCD_SPI_CLK_ENABLE()			 __HAL_RCC_SPI1_CLK_ENABLE()

extern SPI_HandleTypeDef h_lcd_spi;

/* LCD IO functions */
void     LCD_Hardware_Init(void); // init the hardware, SPI and IO
void	 LCD_Module_Init(void);
void     LCD_WriteDataDC(uint8_t data); // signals the Data/Command and Chip Select lines
void 	 LCD_WriteDataDCWord(uint16_t data); // signals the Data/Command and Chip Select lines
void 	 LCD_WriteWord(uint16_t data); // does NOT signal the Data/Command and Chip Select lines
void     LCD_WriteByte(uint8_t data); // does NOT signal the Data/Command and Chip Select lines
void     LCD_WriteCmd(uint8_t reg);
uint32_t LCD_ReadData(uint8_t RegValue, uint8_t ReadSize);
void     LCD_Delay (uint32_t delay);
void     LCD_FillScreen(uint16_t color);
void 	 LCD_FillScreen2(uint16_t color);
void     LCD_SetColumn(uint16_t col_left, uint16_t col_right);
void     LCD_SetRow(uint16_t row_top, uint16_t row_bottom);
uint16_t LCD_MakeColor(uint8_t red, uint8_t green, uint8_t blue);
void 	 LCD_FillRect(uint16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color);
void	 LCD_DrawHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
void	 LCD_DrawVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
void 	 LCD_FillCircle(int poX, int poY, int r,uint16_t color);
void     LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void 	 LCD_Set8Bit();
void 	 LCD_Set16Bit();
#endif /* LCD_HARDWARE_H_ */

