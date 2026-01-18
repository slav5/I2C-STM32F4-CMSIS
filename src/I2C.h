/**
 * @file    SHT21_I2C.h
 * @brief   I2C peripheral driver header.
 *
 * @details This module provides low-level I2C initialization and communication
 *          functions for use with SHT21 temperature/humidity sensor.
 *
 *          Supported I2C peripherals:
 *          - I2C1: PB6 (SCL), PB7 (SDA)
 *          - I2C2: PB10 (SCL), PB3 (SDA)
 *          - I2C3: PA8 (SCL), PB4 (SDA)
 *
 * @note    All I2C peripherals configured for Standard Mode (100 kHz).
 *
 */

#ifndef I2C_USER_H_
#define I2C_USER_H_

//------------------------------------------------
// Includes
//------------------------------------------------
#include "stm32f4xx.h"
#include "Timeout.h"
//------------------------------------------------
/**
 * @brief   I2C operation status codes.
 *
 * @details Enumeration of possible return values from I2C functions.
 *          Used to indicate success or specific error conditions:
 *
 *          - I2C_OK:          Operation completed successfully
 *          - I2C_ERROR_START: Failed to generate START condition
 *                             (bus busy, arbitration lost, or hardware error)
 *          - I2C_ERROR_ADDR:  Slave address was not acknowledged
 *                             (no device at address or device not responding)
 *          - I2C_ERROR_TX:    Data transmission error
 *                             (NACK received during data phase or timeout)
 */
typedef enum
{
    I2C_OK,          /**< Operation successful */
    I2C_ERROR_START, /**< START condition generation failed */
    I2C_ERROR_ADDR,  /**< Address phase failed (no ACK from slave) */
    I2C_ERROR_TX,    /**< Data transmission failed */
} I2C_Status;

//------------------------------------------------
// Function Prototypes
//------------------------------------------------

/**
 * @brief   Send a single byte to a specified I2C slave address.
 *
 * @param   i2c   Pointer to I2C peripheral (I2C1, I2C2, or I2C3)
 * @param   data     Data byte to transmit
 * @param   addr  7-bit slave address (already shifted left by 1)
 *
 * @return  I2C_Status indicating operation result
 *
 * @example
 *          // Send byte 0x55 to SHT21 (address 0x40)
 *          I2C_Status status = I2C_SendByteByADDR(I2C1, 0x55, 0x80);
 *          if (status != I2C_OK) {
 *              // Handle error
 *          }
 */
I2C_Status I2C_SendByteByADDR(I2C_TypeDef *i2c, uint8_t data, uint8_t addr);

/**
 * @brief   Initialize I2C1 peripheral with specified APB1 clock.
 *
 * @param   APB1_MHz  APB1 bus clock frequency in MHz
 *
 * @note    Pin configuration:
 *          - PB6: SCL (AF4, open-drain, pull-up)
 *          - PB7: SDA (AF4, open-drain, pull-up)
 *
 * @warning Call this function after system clock configuration is complete.
 */
void I2C1_Initialization(uint8_t APB1_MHz);

/**
 * @brief   Initialize I2C2 peripheral.
 *
 * @note    Pin configuration:
 *          - PB10: SCL (AF4, open-drain, pull-up)
 *          - PB3:  SDA (AF9, open-drain, pull-up)
 *
 * @warning Call this function after system clock configuration is complete.
 */
void I2C2_Initialization(uint8_t APB1_MHz);

/**
 * @brief   Initialize I2C3 peripheral.
 *
 * @note    Pin configuration:
 *          - PA8: SCL (AF4, open-drain, pull-up)
 *          - PB4: SDA (AF9, open-drain, pull-up)
 *
 * @warning Call this function after system clock configuration is complete.
 */
void I2C3_Initialization(uint8_t APB1_MHz);

//------------------------------------------------
#endif /* I2C_USER_H_ */
