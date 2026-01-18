# I2C-STM32F4-CMSIS
CMSIS library for I2C peripheral for the STM32F4 microcontrollers



[![standard-readme compliant](https://img.shields.io/badge/created_at-January_2026-green.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)
[![standard-readme compliant](https://img.shields.io/badge/license-MIT-green.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)
[![standard-readme compliant](https://img.shields.io/badge/STM32_series-F4-green.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)


This is the library for I2C communication protocol. This library provides low-level I2C initialization and basic communication functions. Library written on CMSIS, that makes this library more efficient than simular solutions on HAL. I tested I2C library on STM32F411CEU6. If it doesn't work on other MCU's of stm32f4 series make me know.


## Table of Contents

- [Features](#features)
- [Install](#install)
- [Commands](#commands)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Features
- Based on direct register access (no HAL dependency);
- Timeout handling is implemented to avoid infinite loops if the bus does not respond;
- Supports I2C1, I2C2, and I2C3 peripherals;
- Configured for Standard Mode (100 kHz) I2C communication;
- Provides initialization functions with specific pin configurations for each I2C peripheral.

## Install

To install this lib you need to copy I2C directory to the directory of your project and add it to the project. In C file, where you gonna use this lib, include this library by writting #include "I2C.h".


## Commands

`Note about return values:` The I2C_SendByteByADDR function returns an I2C_Status enum indicating the operation result.

This library provides the following commands:

**INITIALIZATION FUNCTIONS:**

- `I2C1_Initialization(uint8_t APB1_MHz)`

  Initializes the I2C1 peripheral with the specified APB1 clock frequency in MHz. Configures GPIO pins PB6 (SCL) and PB7 (SDA) for I2C1 operation in Standard Mode (100 kHz).

  **Parameters:**
  - `APB1_MHz:` APB1 bus clock frequency in MHz (e.g., 50 for 50 MHz)

  **Note:** Call this function after system clock configuration is complete.

- `I2C2_Initialization(uint8_t APB1_MHz)`

  Initializes the I2C2 peripheral with the specified APB1 clock frequency in MHz. Configures GPIO pins PB10 (SCL) and PB3 (SDA) for I2C2 operation in Standard Mode (100 kHz).

  **Parameters:**
  - `APB1_MHz:` APB1 bus clock frequency in MHz

  **Note:** PB3 uses AF9 for I2C2_SDA. Call this function after system clock configuration is complete.

- `I2C3_Initialization(uint8_t APB1_MHz)`

  Initializes the I2C3 peripheral with the specified APB1 clock frequency in MHz. Configures GPIO pins PA8 (SCL) and PB4 (SDA) for I2C3 operation in Standard Mode (100 kHz).

  **Parameters:**
  - `APB1_MHz:` APB1 bus clock frequency in MHz

  **Note:** PA8 uses AF4, PB4 uses AF9. Call this function after system clock configuration is complete.

**COMMUNICATION FUNCTIONS:**

- `I2C_SendByteByADDR(I2C_TypeDef *i2c, uint8_t data, uint8_t addr)`

  Sends a single byte to a specified I2C slave address. This function performs a complete I2C write transaction: generates START condition, sends slave address with write flag, sends data byte, and generates STOP condition.

  **Parameters:**
  - `i2c:` Pointer to I2C peripheral (I2C1, I2C2, or I2C3)
  - `data:` Data byte to send
  - `addr:` 7-bit slave address (already shifted left by 1, e.g., 0x80 for address 0x40)

  **Returns:** `I2C_Status`
  - `I2C_OK:` Operation completed successfully
  - `I2C_ERROR_START:` Failed to generate START condition
  - `I2C_ERROR_ADDR:` Slave address was not acknowledged
  - `I2C_ERROR_TX:` Data transmission failed

  **Example:**
  ```c
  I2C_Status status = I2C_SendByteByADDR(I2C1, 0x55, 0x80);
  if (status != I2C_OK) {
      // Handle error
  }
  ```


## Maintainers

[@SvyatoslavPetyukevich](https://github.com/slav5)

## Contributing

Feel free to dive in! Open an issue or submit PRs.


## License

MIT © 2026 SvyatoslavPetyukevich
