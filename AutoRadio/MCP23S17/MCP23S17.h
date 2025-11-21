/*
 * 
 *
 *
 *      Author: Vincent Fernandes
 */
#ifndef INC_MCP23S17_MCP23S17_H_
#define INC_MCP23S17_MCP23S17_H_

#include "spi.h"
#include "main.h"  

#define MCP23S17_IOCON_ADDR  0x05

//with bank 1
#define MCP23S17_IODIRA_ADDR 0x00
// #define MCP23S17_IODIRB_ADDR 0x10
#define MCP23S17_IODIRB_ADDR 0x01
#define MCP23S17_GPIOA_ADDR  0x12
#define MCP23S17_GPIOB_ADDR  0x13

void MCP23S17_Init(void);
void MCP23S17_WriteRegister(uint8_t reg, uint8_t data);
void MCP23S17_WriteGPIOA(uint8_t data);
void MCP23S17_WriteGPIOB(uint8_t data);
#endif /* INC_MCP23S17_MCP23S17_H_ */
