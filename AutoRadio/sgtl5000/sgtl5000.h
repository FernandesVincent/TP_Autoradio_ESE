
/*
 * 
 *
 *
 *      Author: Vincent Fernandes
 */
#ifndef INC_STGL5000_SGTL5000_H_
#define INC_STGL5000_SGTL5000_H_

#include "stm32l4xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include <stdint.h>

#define AUDIO_BUFFER_SIZE 2048
extern uint8_t rx_buffer[AUDIO_BUFFER_SIZE];
extern uint8_t tx_buffer[AUDIO_BUFFER_SIZE];

#define SGTL5000_I2C_ADDRESS_READ 0x0A //R/w bit to 0
#define SGTL5000_I2C_ADDRESS_WRITE 0x0B //R/w bit to 1

typedef enum sgtl5000_registers_enum{
  SGTL5000_CHIP_ID = 0x0000,
  SGTL5000_CHIP_ANA_CTRL = 0x0024,
  STGTL5000_CHIP_LINREG_CTRL = 0x0026,
  SGTL5000_CHIP_REF_CTRL = 0x0028,
  SGTL5000_CHIP_LINE_OUT_CTRL = 0x002C,
  SGTL5000_CHIP_SHORT_CTRL = 0x003C,
  SGTL5000_CHIP_ANA_POWER = 0x0030,
  SGTL5000_CHIP_DIG_POWER = 0x0002,
  SGTL5000_CHIP_LINE_OUT_VOL = 0x002E,
  SGTL5000_CHIP_CLK_CTRL = 0x0004,
  SGTL5000_CHIP_I2S_CTRL = 0x0006,
  SGTL5000_CHIP_ADCDAC_CTRL = 0x000E,
  SGTL5000_CHIP_DAC_VOL = 0x0010
} sgtl5000_registers_t;

void sgtl5000_write_register_i2c(sgtl5000_registers_t reg_address, uint16_t data);
void sgtl5000_read_register_i2c(sgtl5000_registers_t reg_address, uint16_t *p_data);
void sgtl5000_clear_bit_i2c(sgtl5000_registers_t reg_address, uint16_t mask);
void sgtl5000_init(void);
void sgtl5000_generate_triangle(uint8_t *buffer, uint32_t sizeInBytes);
void sgtl5000_start_SAI(void);

#endif /* INC_STGL5000_SGTL5000_H_ */
