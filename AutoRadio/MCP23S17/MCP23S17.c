/*
 * 
 *
 *  
 *      Author: Vincent Fernandes
 */

#include "MCP23S17.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stdio.h"

MCP23S17_GPIO MCP23S17_Gpio = {
    .GPIO_A = {
        .LED1 = 0,
        .LED2 = 0,
        .LED3 = 0,
        .LED4 = 0,
        .LED5 = 0,
        .LED6 = 0,
        .LED7 = 0,
        .LED8 = 0
    },
    .GPIO_B = {
        .LED1 = 0,
        .LED2 = 0,
        .LED3 = 0,
        .LED4 = 0,
        .LED5 = 0,
        .LED6 = 0,
        .LED7 = 0,
        .LED8 = 0
    }
};

void MCP23S17_Init(void) 
{
    // RESET pulse
    HAL_GPIO_WritePin(VU_nReset_GPIO_Port, VU_nReset_Pin, GPIO_PIN_RESET);  
    HAL_Delay(5);
    HAL_GPIO_WritePin(VU_nReset_GPIO_Port, VU_nReset_Pin, GPIO_PIN_SET);    // Release RESET
    HAL_Delay(10);

    // Configure ports as outputs
    MCP23S17_WriteRegister(MCP23S17_IODIRA_ADDR, 0x00);
    MCP23S17_WriteRegister(MCP23S17_IODIRB_ADDR, 0x00);
}


void MCP23S17_WriteRegister(uint8_t reg, uint8_t data) 
{
    uint8_t TX_Buffer[3];

    TX_Buffer[0] = 0x40; // Opcode: 0100 A2 A1 A0 0
    TX_Buffer[1] = reg;
    TX_Buffer[2] = data;

    // CS ACTIVE LOW : début de transmission
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi3, TX_Buffer, 3, HAL_MAX_DELAY);

    // Fin de transmission → CS HIGH
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
}

void MCP23S17_WriteGPIOA(uint8_t data) {
  MCP23S17_WriteRegister(MCP23S17_GPIOA_ADDR, data);
}

void MCP23S17_WriteGPIOB(uint8_t data) {
  MCP23S17_WriteRegister(MCP23S17_GPIOB_ADDR, data);
}


extern I2C_HandleTypeDef hi2c2; 

uint16_t chip_id = 0;

void Read_CODEC_ChipID(void)
{
    uint8_t buffer[2]; 

    if (HAL_I2C_Mem_Read(&hi2c2,0x14,0x0000,I2C_MEMADD_SIZE_16BIT,buffer,2,HAL_MAX_DELAY) == HAL_OK)
    {
        chip_id = (buffer[0] << 8) | buffer[1]; 
        printf("CODEC Chip ID: 0x%04X\r\n", chip_id);
    }
    else
    {
        printf("Failed to read CODEC Chip ID\r\n");
    }
}



// void MCP23S17_update_one_LED(char GPIO_name, int led_number,int value) {
//     uint8_t reg ;
//     uint8_t data;
//     if (led_number < 1 || led_number > 8) {
//         return; // Invalid LED number
//     }

//     if(GPIO_name != 'A' && GPIO_name != 'B') {
//         return; // Invalid GPIO name
//     }
//     if(GPIO_name == 'A'){
//       reg = MCP23S17_GPIOA_ADDR;
//     }
//     else{
//       reg = MCP23S17_GPIOB_ADDR;
//     }
//     if(GPIO_name == 'A'){
//     }
// }