/*
 * 
 *
 *  
 *      Author: Vincent Fernandes
 */

#include "MCP23S17.h"
#include "main.h"


void MCP23S17_Init(void) {
  //CS is active high by default
  HAL_GPIO_WritePin(VU_nReset_GPIO_Port,VU_nReset_Pin, GPIO_PIN_RESET); // Reset LOW
  HAL_Delay(10);
  HAL_GPIO_WritePin(VU_nCS_GPIO_Port,VU_nCS_Pin, GPIO_PIN_RESET); // CS LOW
  HAL_Delay(10);

  //Can begin sending commands
  // MCP23S17_WriteRegister(MCP23S17_IOCON_ADDR , 0x0); 
  // HAL_Delay(10);

  MCP23S17_WriteRegister(MCP23S17_IODIRA_ADDR , 0x00); //Set all A pins as output
  HAL_Delay(10);


  MCP23S17_WriteRegister(MCP23S17_IODIRB_ADDR , 0x00); //Set all B pins as output
  HAL_Delay(10);
}

void MCP23S17_WriteRegister(uint8_t reg, uint8_t data) {
  uint8_t TX_Buffer[3];
  
  TX_Buffer[0] = 0x40; //Opcode for write, R/W = 0
  TX_Buffer[1] = reg;  //Register address
  TX_Buffer[2] = data; //Data to write typ 0 or 1

  HAL_GPIO_WritePin(VU_nCS_GPIO_Port,VU_nCS_Pin, GPIO_PIN_SET); // CS HIGH
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi3, TX_Buffer, 3, HAL_MAX_DELAY);
  HAL_Delay(1);
  HAL_GPIO_WritePin(VU_nCS_GPIO_Port,VU_nCS_Pin, GPIO_PIN_RESET); // CS LOW
}

void MCP23S17_WriteGPIOA(uint8_t data) {
  MCP23S17_WriteRegister(MCP23S17_GPIOA_ADDR, data);
}

void MCP23S17_WriteGPIOB(uint8_t data) {
  MCP23S17_WriteRegister(MCP23S17_GPIOB_ADDR, data);
}