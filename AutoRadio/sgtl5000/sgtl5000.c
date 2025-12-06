#include "sgtl5000.h"
#include "i2c.h"
#include "main.h"
#include "stm32l4xx_hal_i2c.h"
#include "stdio.h"
#include <stdint.h>
#include "sai.h"


void sgtl5000_write_register_i2c(sgtl5000_registers_t reg_address, uint16_t data) 
{
  uint8_t buffer[2];

	buffer[0] = (data >> 8) & 0xFF;
	buffer[1] = data & 0xFF;

  HAL_I2C_Mem_Write(&hi2c2, SGTL5000_I2C_ADDRESS_WRITE, reg_address, I2C_MEMADD_SIZE_16BIT, buffer, 2, HAL_MAX_DELAY);

}

void sgtl5000_read_register_i2c(sgtl5000_registers_t reg_address, uint16_t *p_data) 
{
  uint8_t buffer[2];
  HAL_I2C_Mem_Read(&hi2c2,SGTL5000_I2C_ADDRESS_READ, reg_address, I2C_MEMADD_SIZE_16BIT, buffer, 2, HAL_MAX_DELAY);

  *p_data = (buffer[0] << 8) | buffer[1];
}

void sgtl5000_clear_bit_i2c(sgtl5000_registers_t reg_address, uint16_t mask) 
{
  uint16_t data;
  sgtl5000_read_register_i2c(reg_address, &data);
  data &= (~mask);
  sgtl5000_write_register_i2c(reg_address, data);
  
}
void sgtl5000_init(void) 
{
  uint16_t mask;
  //--------------- Power Supply Configuration----------------
  // NOTE: This next 2 Write calls is needed ONLY if VDDD is
  // internally driven by the chip
  // Configure VDDD level to 1.2V (bits 3:0)
  // Write CHIP_LINREG_CTRL 0x0008
  // Power up internal linear regulator (Set bit 9)
  // Write CHIP_ANA_POWER 0x7260
  // NOTE: This next Write call is needed ONLY if VDDD is
  // externally driven
  // Turn off startup power supplies to save power (Clear bit 12 and 13)
  // Write CHIP_ANA_POWER 0x4260
  mask = (1 << 12) | (1 << 13);
	sgtl5000_clear_bit_i2c(SGTL5000_CHIP_ANA_POWER, mask);
  // NOTE: The next Write calls is needed only if both VDDA and
  // VDDIO power supplies are less than 3.1V.
  // Enable the internal oscillator for the charge pump (Set bit 11)
  // Write CHIP_CLK_TOP_CTRL 0x0800
  // // Enable charge pump (Set bit 11)
  // Write CHIP_ANA_POWER 0x4A60

  // NOTE: The next modify call is only needed if both VDDA and
  // VDDIO are greater than 3.1 V
  // Configure the charge pump to use the VDDIO rail (set bit 5 andbit 6)
  // Write CHIP_LINREG_CTRL 0x006C
  mask = 0x006C;
  sgtl5000_write_register_i2c(STGTL5000_CHIP_LINREG_CTRL, mask);
  //---- Reference Voltage and Bias Current Configuration----
  // NOTE: The value written in the next 2 Write calls is dependent
  // on the VDDA voltage value.
  // Set ground, ADC, DAC reference voltage (bits 8:4). The valueshould
  // be set to VDDA/2. This example assumes VDDA = 1.8 V.VDDA/2 = 0.9 V.
  // The bias current should be set to 50% of the nominal value (bits3:1)
  //Write CHIP_REF_CTRL 0x004E
  mask = 0x01F2;	
	sgtl5000_write_register_i2c(SGTL5000_CHIP_REF_CTRL, mask);
  // Set LINEOUT reference voltage to VDDIO/2 (1.65 V) (bits 5:0)
  //and bias current (bits 11:8) to the recommended value of 0.36 mA
  //for 10 kOhm load with 1.0 nF capacitance
  //Write CHIP_LINE_OUT_CTRL 0x0322
	mask = 0x0F22;	// LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
	sgtl5000_write_register_i2c(SGTL5000_CHIP_LINE_OUT_CTRL, mask);

  //------------Other Analog Block Configurations--------------
  // Configure slow ramp up rate to minimize pop (bit 0)
  mask = 0x006C;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_REF_CTRL, mask);
  // Enable short detect mode for headphone left/right
  // and center channel and set short detect current trip level
  // to 75 mA
  mask = 0x4446;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_SHORT_CTRL, mask);
  // Enable Zero-cross detect if needed for HP_OUT (bit 5) and ADC (bit 1)

  mask = 0x0004;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_ANA_CTRL, mask);
  //------------Power up Inputs/Outputs/Digital Blocks---------
  // Power up LINEOUT, HP, ADC, DAC
  mask = 0x6AFF;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_ANA_POWER, mask);
  // Power up desired digital blocks
  // I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5),
  // ADC (bit 6) are powered on
  mask = 0x0073;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_DIG_POWER, mask);
  //----------------Set LINEOUT Volume Level-------------------
  
  // Set the LINEOUT volume level based on voltage reference
  
  // values using this formula
  // Value = (int)(40*log(VAG_VAL/LO_VAGCNTRL) + 15)
  // Assuming VAG_VAL and LO_VAGCNTRL is set to 0.9 V and
  mask = 0x1111;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_LINE_OUT_VOL, mask);
  /* System MCLK and Sample Clock */

  /* Input/Output Routing */
	// Laissons tout par défaut pour l'instant

	/* Le reste */
	mask = 0x0000;	// Unmute
	sgtl5000_write_register_i2c(SGTL5000_CHIP_ADCDAC_CTRL, mask);

	mask = 0x3C3C;
//	mask = 0x4747;
	sgtl5000_write_register_i2c(SGTL5000_CHIP_DAC_VOL, mask);

	mask = 0x0D0D;
  sgtl5000_write_register_i2c(SGTL5000_CHIP_LINE_OUT_VOL, mask);
	
}


void sgtl5000_generate_triangle(uint8_t *buffer, uint32_t sizeInBytes) {
    int16_t *pSample = (int16_t *)buffer;

    // 16 bits = 2 octets. Stereo = 2 canaux.
    // Donc 1 "Frame" audio (Gauche + Droite) = 4 octets.
    uint32_t totalFrames = sizeInBytes / 4;

    // On divise le buffer en deux phases : Montée et Descente
    uint32_t halfPeriod = totalFrames / 2;

    int16_t amplitude = 15000;
    float step = (float)(amplitude * 2) / (float)halfPeriod;
    float currentVal = -15000.0f;

    // Phase 1 : Montée
    for (uint32_t i = 0; i < halfPeriod; i++) {
        int16_t val = (int16_t)currentVal;

        pSample[i * 2]     = val; // Canal Gauche
        pSample[i * 2 + 1] = val; // Canal Droit (identique pour du mono centré)

        currentVal += step;
    }

    // Phase 2 : Descente
    currentVal = 15000.0f;
    for (uint32_t i = halfPeriod; i < totalFrames; i++) {
        int16_t val = (int16_t)currentVal;

        pSample[i * 2]     = val;
        pSample[i * 2 + 1] = val;

        currentVal -= step;
    }
}

void sgtl5000_start_SAI(void) {

  HAL_SAI_Receive_DMA(&hsai_BlockA2, rx_buffer, sizeof(rx_buffer));
  HAL_SAI_Transmit_DMA(&hsai_BlockA2, tx_buffer, sizeof(tx_buffer));
}