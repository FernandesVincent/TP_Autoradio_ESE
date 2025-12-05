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

typedef struct {
  uint8_t LED1;
  uint8_t LED2;
  uint8_t LED3;
  uint8_t LED4;
  uint8_t LED5;
  uint8_t LED6;
  uint8_t LED7;
  uint8_t LED8;

} MCP23S17_Port;

typedef struct {
  MCP23S17_Port GPIO_A;
  MCP23S17_Port GPIO_B;

} MCP23S17_GPIO;

void MCP23S17_Init(void);
void MCP23S17_WriteRegister(uint8_t reg, uint8_t data);
void MCP23S17_WriteGPIOA(uint8_t data);
void MCP23S17_WriteGPIOB(uint8_t data);
void MCP23S17_update_one_LED(char GPIO_name, int led_number,int value);
void Read_CODEC_ChipID(void);
#endif /* INC_MCP23S17_MCP23S17_H_ */
