/**
 * @file    SHT21_I2C.c
 * @brief   I2C peripheral driver for STM32F411CEU6 microcontroller.
 *          Provides initialization functions for I2C1, I2C2, I2C3 peripherals
 *          and basic I2C communication functions for SHT21 sensor.
 *
 * @note    This driver uses CMSIS register-level access (no HAL).
 *          Configured for Standard Mode (Sm) at 100 kHz I2C clock speed.
 *
 */

#include "I2C.h"


//------------------------------------------------
// I2C Communication Direction Flags
//------------------------------------------------
#define I2C_REQUEST_WRITE 0x00 /**< Write request: LSB = 0 */
#define I2C_REQUEST_READ 0x01  /**< Read request: LSB = 1 */

//------------------------------------------------
// I2C Configuration Constants
//------------------------------------------------
#define I2C_OWN_ADDRESS 0x00 /**< Own address for master mode (not used, set to 0) */

//------------------------------------------------
// Dummy read variable for RCC clock enabling delay
// Required by STM32 Reference Manual after enabling peripheral clock
//------------------------------------------------
__IO uint32_t tmpreg1;

//------------------------------------------------
/**
 * @brief   Send a single byte to a specified I2C address.
 *
 * @details This function performs a complete I2C write transaction:
 *          1. Generates START condition
 *          2. Sends slave address with write flag
 *          3. Sends data byte
 *          4. Generates STOP condition
 *
 * @param   i2c   Pointer to I2C peripheral (I2C1, I2C2, or I2C3)
 * @param   data     Data byte to send
 * @param   addr  7-bit slave address (will be shifted left, write bit added automatically)
 *
 * @return  I2C_Status
 *          - I2C_OK:          Transaction completed successfully
 *          - I2C_ERROR_START: Failed to generate START condition (bus busy or error)
 *          - I2C_ERROR_ADDR:  Slave did not acknowledge address (no device or wrong address)
 *          - I2C_ERROR_TX:    Data transmission failed (NACK received or timeout)
 *
 * @note    The slave address should be provided as 7-bit address already shifted left by 1.
 *          Example: For SHT21 (address 0x40), pass 0x80 (0x40 << 1).
 */
I2C_Status I2C_SendByteByADDR(I2C_TypeDef *i2c, uint8_t data, uint8_t addr)
{
  // Step 1: Configure I2C for standard reception
  // Disable POS bit (used only for 2-byte reception with NACK)
  CLEAR_BIT(i2c->CR1, I2C_CR1_POS);

  // Enable ACK generation for received bytes
  MODIFY_REG(i2c->CR1, I2C_CR1_ACK, I2C_CR1_ACK);

  // Step 2: Generate START condition
  SET_BIT(i2c->CR1, I2C_CR1_START);

  // Wait for SB (Start Bit) flag - indicates START condition generated
  if (WaitFlagSet((volatile uint32_t *)&i2c->SR1, I2C_SR1_SB) != Timeout_OK)
  {
    return I2C_ERROR_START;
  }

  // Step 3: Send slave address with WRITE flag
  // Reading SR1 followed by writing DR clears SB flag
  (void)i2c->SR1;
  MODIFY_REG(i2c->DR, I2C_DR_DR, addr | I2C_REQUEST_WRITE);

  // Wait for ADDR flag - indicates address sent and ACK received
  if (WaitFlagSet((volatile uint32_t *)&i2c->SR1, I2C_SR1_ADDR) != Timeout_OK)
  {
    return I2C_ERROR_ADDR;
  }

  // Clear ADDR flag by reading SR1 and SR2
  (void)i2c->SR1;
  (void)i2c->SR2;

  // Step 4: Send data byte
  MODIFY_REG(i2c->DR, I2C_DR_DR, data);

  // Wait for TXE (Transmit buffer Empty) flag - indicates data sent
  if (WaitFlagSet((volatile uint32_t *)&i2c->SR1, I2C_SR1_TXE) != Timeout_OK)
  {
    return I2C_ERROR_TX;
  }

  // Step 5: Generate STOP condition
  SET_BIT(i2c->CR1, I2C_CR1_STOP);

  return I2C_OK;
}

//------------------------------------------------
/**
 * @brief   Initialize I2C1 peripheral for SHT21 communication.
 *
 * @details Pin Configuration (I2C1):
 *          - PB6: SCL (Serial Clock Line)
 *          - PB7: SDA (Serial Data Line)
 *
 *          GPIO Configuration:
 *          - Alternate Function mode (AF4 for I2C1)
 *          - Open-drain output type (required for I2C)
 *          - Pull-up resistors enabled
 *          - High speed output
 *
 *          I2C Configuration:
 *          - Standard Mode (Sm): 100 kHz clock speed
 *          - 7-bit addressing mode
 *          - Clock stretching enabled
 *          - ACK enabled
 *
 * @param   APB1_MHz  APB1 bus clock frequency in MHz (e.g., 50 for 50 MHz)
 *
 * @note    Make sure APB1 clock is properly configured in system clock setup.
 *          For STM32F411 at 100 MHz: APB1 max = 50 MHz (AHB/2).
 *
 * @warning External pull-up resistors (2.2k-10k) may be required if internal
 *          pull-ups are insufficient for the bus capacitance.
 */
void I2C1_Initialization(uint8_t APB1_MHz)
{
  //========================================
  // GPIO Clock and I2C Clock Enable
  //========================================

  // Enable GPIOB clock for I2C1 pins (PB6, PB7)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
  // Dummy read to ensure AHB bus synchronization after clock enable
  tmpreg1 = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

  // Enable I2C1 peripheral clock
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

  // Mandatory delay after enabling RCC peripheral clock
  // Prevents accessing peripheral registers before clock is stable
  tmpreg1 = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

  //========================================
  // GPIO Configuration for PB6 (SCL) and PB7 (SDA)
  //========================================

  // Configure PB6 as Alternate Function mode (MODER6 = 0b10)
  // Bits [13:12] control MODER for pin 6
  GPIOB->MODER = (GPIOB->MODER & ~(0b11 << 12)) | (0b10 << 12);

  // Configure PB7 as Alternate Function mode (MODER7 = 0b10)
  // Bits [15:14] control MODER for pin 7
  GPIOB->MODER = (GPIOB->MODER & ~(0b11 << 14)) | (0b10 << 14);

  // Configure PB6 as Open-Drain output
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_6);

  // Configure PB7 as Open-Drain output
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_7);

  // Enable internal Pull-Up resistor on PB6 (PUPDR6 = 0b01)
  // Bits [13:12] control PUPDR for pin 6
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(0b11 << 12)) | (0b01 << 12);

  // Enable internal Pull-Up resistor on PB7 (PUPDR7 = 0b01)
  // Bits [15:14] control PUPDR for pin 7
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(0b11 << 14)) | (0b01 << 14);

  // Set output speed to High for PB6 (OSPEEDR6 = 0b11)
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0b11 << 12)) | (0b11 << 12);

  // Set output speed to High for PB7 (OSPEEDR7 = 0b11)
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0b11 << 14)) | (0b11 << 14);

  // Set Alternate Function 4 (I2C1) for PB6
  // AFR[0] covers pins 0-7, bits [27:24] control AF for pin 6
  // AF4 = 0b0100
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0b1111 << 24)) | (0b0100 << 24);

  // Set Alternate Function 4 (I2C1) for PB7
  // Bits [31:28] control AF for pin 7
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0b1111 << 28)) | (0b0100 << 28);

  //========================================
  // I2C1 Peripheral Configuration
  //========================================

  // Disable dual addressing mode (single 7-bit address only)
  CLEAR_BIT(I2C1->OAR2, I2C_OAR2_ENDUAL);

  // Disable General Call address (0x00)
  CLEAR_BIT(I2C1->CR1, I2C_CR1_ENGC);

  // Enable clock stretching (slave can hold SCL low to slow master)
  // NOSTRETCH = 0 means stretching is ENABLED
  CLEAR_BIT(I2C1->CR1, I2C_CR1_NOSTRETCH);

  // Disable I2C peripheral before configuration
  // PE must be 0 when configuring CCR and TRISE
  CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);

  //========================================
  // I2C Timing Configuration (Standard Mode 100 kHz)
  //========================================

  // Set peripheral clock frequency (APB1 clock in MHz)
  MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ, APB1_MHz);

  // Configure rise time register for Standard Mode (Sm)
  // TRISE = (Trise_max / Tpclk) + 1
  // For Sm: Trise_max = 1000ns, so TRISE = APB1_MHz + 1
  MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE, APB1_MHz + 1);

  // Configure Clock Control Register for Standard Mode (100 kHz)
  // Clear FS bit: 0 = Standard Mode, 1 = Fast Mode
  CLEAR_BIT(I2C1->CCR, I2C_CCR_FS);

  // DUTY bit is not used in Standard Mode (only affects Fast Mode)
  CLEAR_BIT(I2C1->CCR, I2C_CCR_DUTY);

  // Calculate CCR value for 100 kHz in Standard Mode:
  // CCR = Fpclk / (2 * Fscl) = (APB1_MHz * 1,000,000) / (2 * 100,000)
  // CCR = APB1_MHz * 5
  // Example: APB1 = 42 MHz -> CCR = 210
  I2C1->CCR = (I2C1->CCR & ~(0x0FFF)) | ((APB1_MHz * 5) & 0x0FFF);

  //========================================
  // I2C Address Configuration (Master Mode)
  //========================================

  // Set 7-bit addressing mode (ADDMODE = 0)
  CLEAR_BIT(I2C1->OAR1, I2C_OAR1_ADDMODE);

  // Set own address (not really used in master mode, but must be set)
  // Bit 14 should always be kept at 1 by software (per Reference Manual)
  I2C1->OAR1 = (I2C1->OAR1 & ~(0x7F << 1)) | (I2C_OWN_ADDRESS << 1) | (1 << 14);

  //========================================
  // Configure I2C Mode (disable SMBus features)
  //========================================

  // Clear software reset
  CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST);

  // Disable SMBus alert
  CLEAR_BIT(I2C1->CR1, I2C_CR1_ALERT);

  // Disable packet error checking transfer
  CLEAR_BIT(I2C1->CR1, I2C_CR1_PEC);

  // Clear POS bit (NACK position)
  CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);

  // Disable packet error checking
  CLEAR_BIT(I2C1->CR1, I2C_CR1_ENPEC);

  // Disable SMBus mode (use I2C mode)
  CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBUS);

  //========================================
  // Enable I2C Peripheral
  //========================================

  // Enable I2C1 peripheral
  SET_BIT(I2C1->CR1, I2C_CR1_PE);

  // Enable ACK generation (acknowledge received data)
  MODIFY_REG(I2C1->CR1, I2C_CR1_ACK, I2C_CR1_ACK);

  // Clear secondary own address (not used)
  MODIFY_REG(I2C1->OAR2, I2C_OAR2_ADD2, 0);
}

//----------------------------------------------------------
/**
 * @brief   Initialize I2C2 peripheral.
 *
 * @details Pin Configuration (I2C2):
 *          - PB10: SCL (Serial Clock Line) - AF4
 *          - PB3:  SDA (Serial Data Line) - AF9
 *
 *          Configured for Standard Mode (Sm) at 100 kHz.
 *
 * @note    Make sure APB1 clock is properly configured in system clock setup.
 *          For STM32F411 at 100 MHz: APB1 max = 50 MHz (AHB/2).
 *
 * @warning PB3 uses AF9 for I2C2_SDA (not AF4 like other I2C pins).
 *          Verify your board's pinout matches this configuration.
 */
void I2C2_Initialization(uint8_t APB1_MHz)
{
  //========================================
  // GPIO Clock and I2C Clock Enable
  //========================================

  // Enable GPIOB clock for I2C2 pins (PB10, PB3)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
  // Dummy read to ensure AHB bus synchronization after clock enable
  tmpreg1 = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

  // Enable I2C2 peripheral clock
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

  // Mandatory delay after enabling RCC peripheral clock
  tmpreg1 = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

  //========================================
  // GPIO Configuration for PB10 (SCL) and PB3 (SDA)
  //========================================

  // Configure PB10 as Alternate Function mode (MODER10 = 0b10)
  // Bits [21:20] control MODER for pin 10
  GPIOB->MODER = (GPIOB->MODER & ~(0b11 << 20)) | (0b10 << 20);

  // Configure PB3 as Alternate Function mode (MODER3 = 0b10)
  // Bits [7:6] control MODER for pin 3
  GPIOB->MODER = (GPIOB->MODER & ~(0b11 << 6)) | (0b10 << 6);

  // Configure PB10 as Open-Drain output
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);

  // Configure PB3 as Open-Drain output
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_3);

  // Enable internal Pull-Up resistor on PB10 (PUPDR10 = 0b01)
  // Bits [21:20] control PUPDR for pin 10
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(0b11 << 20)) | (0b01 << 20);

  // Enable internal Pull-Up resistor on PB3 (PUPDR3 = 0b01)
  // Bits [7:6] control PUPDR for pin 3
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(0b11 << 6)) | (0b01 << 6);

  // Set output speed to High for PB10
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0b11 << 20)) | (0b11 << 20);

  // Set output speed to High for PB3
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0b11 << 6)) | (0b11 << 6);

  // Set Alternate Function 4 (I2C2) for PB10 (SCL)
  // AFR[1] covers pins 8-15, bits [11:8] control AF for pin 10
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0b1111 << 8)) | (0b0100 << 8);

  // Set Alternate Function 9 (I2C2) for PB3 (SDA)
  // AFR[0] covers pins 0-7, bits [15:12] control AF for pin 3
  // Note: PB3 uses AF9 for I2C2_SDA, not AF4!
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0b1111 << 12)) | (0b1001 << 12);

  //========================================
  // I2C2 Peripheral Configuration
  //========================================

  // Disable dual addressing mode
  CLEAR_BIT(I2C2->OAR2, I2C_OAR2_ENDUAL);

  // Disable General Call address
  CLEAR_BIT(I2C2->CR1, I2C_CR1_ENGC);

  // Enable clock stretching
  CLEAR_BIT(I2C2->CR1, I2C_CR1_NOSTRETCH);

  // Software reset to ensure clean state
  SET_BIT(I2C2->CR1, I2C_CR1_SWRST);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_SWRST);

  // Disable I2C peripheral before configuration
  CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);

  //========================================
  // I2C Timing Configuration
  // Assuming APB1 = 48 MHz, Standard Mode 100 kHz
  //========================================

  // Set peripheral clock frequency (APB1 clock in MHz)
  MODIFY_REG(I2C2->CR2, I2C_CR2_FREQ, APB1_MHz);

  // Configure rise time register for Standard Mode (Sm)
  // TRISE = (Trise_max / Tpclk) + 1
  // For Sm: Trise_max = 1000ns, so TRISE = APB1_MHz + 1
  MODIFY_REG(I2C2->TRISE, I2C_TRISE_TRISE, APB1_MHz + 1);

  // Configure for Standard Mode
  CLEAR_BIT(I2C2->CCR, I2C_CCR_FS);
  CLEAR_BIT(I2C2->CCR, I2C_CCR_DUTY);

  // Calculate CCR value for 100 kHz in Standard Mode:
  // CCR = Fpclk / (2 * Fscl) = (APB1_MHz * 1,000,000) / (2 * 100,000)
  // CCR = APB1_MHz * 5
  // Example: APB1 = 42 MHz -> CCR = 210
  I2C2->CCR = (I2C2->CCR & ~(0x0FFF)) | ((APB1_MHz * 5) & 0x0FFF);

  //========================================
  // I2C Address Configuration
  //========================================

  // Set 7-bit addressing mode
  CLEAR_BIT(I2C2->OAR1, I2C_OAR1_ADDMODE);

  // Set own address (master mode, can be any value)
  // Bit 14 should be kept at 1
  I2C2->OAR1 = (I2C2->OAR1 & ~(0x7F << 1)) | (I2C_OWN_ADDRESS << 1) | (1 << 14);

  //========================================
  // Configure I2C Mode
  //========================================

  CLEAR_BIT(I2C2->CR1, I2C_CR1_ALERT);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_PEC);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_POS);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_ENPEC);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_SMBUS);

  //========================================
  // Enable I2C Peripheral
  //========================================

  SET_BIT(I2C2->CR1, I2C_CR1_PE);
  MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
  MODIFY_REG(I2C2->OAR2, I2C_OAR2_ADD2, 0);
}

//----------------------------------------------------------
/**
 * @brief   Initialize I2C3 peripheral.
 *
 * @details Pin Configuration (I2C3):
 *          - PA8: SCL (Serial Clock Line) - AF4
 *          - PB4: SDA (Serial Data Line) - AF9
 *
 *          Configured for Standard Mode (Sm) at 100 kHz.
 *
 * @param   APB1_MHz  APB1 bus clock frequency in MHz (e.g., 50 for 50 MHz)
 *
 * @note    Make sure APB1 clock is properly configured in system clock setup.
 *          For STM32F411 at 100 MHz: APB1 max = 50 MHz (AHB/2).
 *
 * @warning PA8 uses AF4, PB4 uses AF9. These alternate functions are
 *          specific to STM32F411 - verify against your MCU's datasheet.
 */
void I2C3_Initialization(uint8_t APB1_MHz)
{
  //========================================
  // GPIO Clock and I2C Clock Enable
  //========================================

  // Enable GPIOB clock for PB4 (SDA)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
  // Dummy read to ensure AHB bus synchronization after clock enable
  tmpreg1 = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

  // Enable GPIOA clock for PA8 (SCL)
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
  // Dummy read to ensure AHB bus synchronization after clock enable
  tmpreg1 = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

  // Enable I2C3 peripheral clock
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);

  // Mandatory delay after enabling RCC peripheral clock
  tmpreg1 = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);

  //========================================
  // GPIO Configuration for PA8 (SCL) and PB4 (SDA)
  //========================================

  // Configure PB4 as Alternate Function mode (MODER4 = 0b10)
  // Bits [9:8] control MODER for pin 4
  GPIOB->MODER = (GPIOB->MODER & ~(0b11 << 8)) | (0b10 << 8);

  // Configure PA8 as Alternate Function mode (MODER8 = 0b10)
  // Bits [17:16] control MODER for pin 8
  GPIOA->MODER = (GPIOA->MODER & ~(0b11 << 16)) | (0b10 << 16);

  // Configure PB4 as Open-Drain output
  SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_4);

  // Configure PA8 as Open-Drain output
  SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_8);

  // Enable internal Pull-Up resistor on PB4 (PUPDR4 = 0b01)
  // Bits [9:8] control PUPDR for pin 4
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(0b11 << 8)) | (0b01 << 8);

  // Enable internal Pull-Up resistor on PA8 (PUPDR8 = 0b01)
  // Bits [17:16] control PUPDR for pin 8
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(0b11 << 16)) | (0b01 << 16);

  // Set output speed to High for PB4
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0b11 << 8)) | (0b11 << 8);

  // Set output speed to High for PA8
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0b11 << 16)) | (0b11 << 16);

  // Set Alternate Function 9 (I2C3) for PB4 (SDA)
  // AFR[0] covers pins 0-7, bits [19:16] control AF for pin 4
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0b1111 << 16)) | (0b1001 << 16);

  // Set Alternate Function 4 (I2C3) for PA8 (SCL)
  // AFR[1] covers pins 8-15, bits [3:0] control AF for pin 8
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0b1111 << 0)) | (0b0100 << 0);

  //========================================
  // I2C3 Peripheral Configuration
  //========================================

  // Disable dual addressing mode
  CLEAR_BIT(I2C3->OAR2, I2C_OAR2_ENDUAL);

  // Disable General Call address
  CLEAR_BIT(I2C3->CR1, I2C_CR1_ENGC);

  // Enable clock stretching
  CLEAR_BIT(I2C3->CR1, I2C_CR1_NOSTRETCH);

  // Software reset to ensure clean state
  SET_BIT(I2C3->CR1, I2C_CR1_SWRST);
  CLEAR_BIT(I2C3->CR1, I2C_CR1_SWRST);

  // Disable I2C peripheral before configuration
  CLEAR_BIT(I2C3->CR1, I2C_CR1_PE);

  //========================================
  // I2C Timing Configuration
  // Assuming APB1 = 48 MHz, Standard Mode 100 kHz
  //========================================

  // Set peripheral clock frequency (APB1 clock in MHz)
  MODIFY_REG(I2C3->CR2, I2C_CR2_FREQ, APB1_MHz);

  // Configure rise time register for Standard Mode (Sm)
  // TRISE = (Trise_max / Tpclk) + 1
  // For Sm: Trise_max = 1000ns, so TRISE = APB1_MHz + 1
  MODIFY_REG(I2C3->TRISE, I2C_TRISE_TRISE, APB1_MHz + 1);

  // Configure for Standard Mode
  CLEAR_BIT(I2C3->CCR, I2C_CCR_FS);
  CLEAR_BIT(I2C3->CCR, I2C_CCR_DUTY);

  // Calculate CCR value for 100 kHz in Standard Mode:
  // CCR = Fpclk / (2 * Fscl) = (APB1_MHz * 1,000,000) / (2 * 100,000)
  // CCR = APB1_MHz * 5
  // Example: APB1 = 42 MHz -> CCR = 210
  I2C3->CCR = (I2C3->CCR & ~(0x0FFF)) | ((APB1_MHz * 5) & 0x0FFF);

  //========================================
  // I2C Address Configuration
  //========================================

  // Set 7-bit addressing mode
  CLEAR_BIT(I2C3->OAR1, I2C_OAR1_ADDMODE);

  // Set own address (master mode)
  // Bit 14 should be kept at 1
  I2C3->OAR1 = (I2C3->OAR1 & ~(0x7F << 1)) | (I2C_OWN_ADDRESS << 1) | (1 << 14);

  //========================================
  // Configure I2C Mode
  //========================================

  CLEAR_BIT(I2C3->CR1, I2C_CR1_ALERT);
  CLEAR_BIT(I2C3->CR1, I2C_CR1_PEC);
  CLEAR_BIT(I2C3->CR1, I2C_CR1_POS);
  CLEAR_BIT(I2C3->CR1, I2C_CR1_ENPEC);
  CLEAR_BIT(I2C3->CR1, I2C_CR1_SMBUS);

  //========================================
  // Enable I2C Peripheral
  //========================================

  SET_BIT(I2C3->CR1, I2C_CR1_PE);
  MODIFY_REG(I2C3->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
  MODIFY_REG(I2C3->OAR2, I2C_OAR2_ADD2, 0);
}
//----------------------------------------------------------
