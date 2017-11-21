#include "lcd_hardware.h"

SPI_HandleTypeDef  h_lcd_spi;

void LCD_IO_Init(void)
{
    // set up LCD SPI ports and pins
    // SPI SCK
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LCD_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(LCD_SPI_SCK_PORT, &GPIO_InitStruct);

    // SPI MISO
    GPIO_InitStruct.Pin = LCD_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(LCD_SPI_MISO_PORT, &GPIO_InitStruct);

    // SPI MOSI
    GPIO_InitStruct.Pin = LCD_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(LCD_SPI_MOSI_PORT, &GPIO_InitStruct);

    // SPI NSS
    GPIO_InitStruct.Pin = LCD_SPI_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(LCD_SPI_NSS_PORT, &GPIO_InitStruct);
   	HAL_GPIO_WritePin(LCD_SPI_NSS_PORT,LCD_SPI_NSS_PIN,GPIO_PIN_SET);
    // set up LCD SPI module
    LCD_SPI_CLK_ENABLE();

    h_lcd_spi.Instance = LCD_SPIx;
    h_lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    h_lcd_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    h_lcd_spi.Init.CLKPolarity = SPI_POLARITY_LOW; // might need to be high with 2nd edge
    h_lcd_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    h_lcd_spi.Init.CRCPolynomial;
    h_lcd_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    h_lcd_spi.Init.Direction = SPI_DIRECTION_2LINES;
    h_lcd_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    h_lcd_spi.Init.Mode = SPI_MODE_MASTER;
    h_lcd_spi.Init.NSS = SPI_NSS_HARD_OUTPUT;
    //h_lcd_spi.Init.NSS = SPI_NSS_SOFT;
    h_lcd_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    h_lcd_spi.State = HAL_SPI_STATE_RESET;

    if(HAL_SPI_Init(&h_lcd_spi) != HAL_OK){
    	Error_Handler();
    }

}
void     LCD_IO_WriteData(uint16_t RegValue);
void     LCD_IO_WriteReg(uint8_t Reg);
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
void     LCD_Delay (uint32_t delay);
