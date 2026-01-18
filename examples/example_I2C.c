/**
 * @file example_I2C.c
 * @brief Example demonstrating I2C peripheral initialization and basic communication.
 *
 * This example shows how to initialize I2C1 peripheral and perform basic
 * I2C operation like sending a byte to a slave device.
 *
 * Hardware Requirements:
 * - STM32F4 microcontroller
 * - I2C1 pins: PB6 (SCL), PB7 (SDA)
 * - I2C slave device connected (e.g., EEPROM, sensor)
 *
 * @note This example uses 50 MHz APB1 clock. Adjust according to your setup.
 *       For SHT21 sensor, use address 0x80 (0x40 shifted left).
 */

#include "I2C.h"
#include "USART.h"
#include <stdio.h>
#include <string.h>

// Buffer for formatted strings
uint8_t tx_buffer[64];
// Temporary register for clock enable synchronization
__IO uint32_t tmpreg;
// System timer millisecond counter (incremented in SysTick ISR)
volatile uint32_t SysTimer_ms = 0;
// Counter for delay function
volatile uint32_t Delay_counter_ms = 0;

// Delay_ms: Blocking delay function using SysTick timer
// Parameters: Milliseconds - delay duration in milliseconds
void Delay_ms(uint32_t Milliseconds)
{
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0)
    {
    }
}

// SysTick_Handler: Interrupt service routine for SysTick timer
// Increments system timer counter and decrements delay counter
void SysTick_Handler(void)
{
    SysTimer_ms++;

    if (Delay_counter_ms)
    {
        Delay_counter_ms--;
    }
}

int main(void)
{
    // Configure SysTick timer for 1ms interrupts
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);                                    // Disable counter just in case it's already running
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);                                     // Enable timer interrupts
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);                                   // choose without a divider. It will be 100 MHz.
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 99999 << SysTick_LOAD_RELOAD_Pos); // Setting to 1ms
    MODIFY_REG(SysTick->LOAD, SysTick_VAL_CURRENT_Msk, 99999 << SysTick_VAL_CURRENT_Pos); // start counting from 99999 to 0
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);                                      // enable counter

    // Configure RCC for 100MHz system clock using HSE and PLL
    SET_BIT(RCC->CR, RCC_CR_HSION); // start HSI 8 MHz RC oscillator
    while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET)
    {
    } // wait till HSI oscillator getting stable

    SET_BIT(RCC->CR, RCC_CR_HSEON); // launch an external 25 MHz resonator
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET)
    {
    } // wait till HSE oscillator getting stable

    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); // 0: HSE oscillator not bypassed

    SET_BIT(RCC->CR, RCC_CR_CSSON); // enable Clock detector

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); // PLL selected as system clock

    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); // AHB Prescaler /1

    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS); // 011 Three wait states
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);                            // 1: Prefetch is enabled

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2); // APB1 Prescaler /2 (PCLR 1 max 50 MHz)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1); // APB2 Prescaler /1

    // config PLL = 100 MHz, 40 MHz for USB OTG FS SDIO clocks (USB won't work, it requires 48 MHZ)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);                  // Select HSE as the input signal for the PLL
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(0x3F << 0)) | (25 << 0);   // PLLM = 5
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(0x1FF << 6)) | (200 << 6); // PLLN = 40
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(0x3 << 16)) | (0 << 16);   // PLLP = 2
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(0xF << 24)) | (4 << 24);   // PLLQ = 4

    SET_BIT(RCC->CR, RCC_CR_PLLON); // enable PLL
    // Waiting for the PLL to turn on
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET)
    {
    }

    // Configure GPIO PC13 as output pin
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); // enable clocking for GPIOC
    // Dummy read to ensure AHB bus synchronization after clock enable
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    GPIOC->MODER = (GPIOC->MODER & ~(0b11 << 26)) | (0b01 << 26);     // 01: General purpose output mode
    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT_13);                      // Output push-pull (reset state)
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~(0b11 << 26)) | (0b00 << 26); // 00: Low speed]
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(0b11 << 26)) | (0b00 << 26);     // PC13 No pull-up, pull-down
    TIM3_Init(50);

    // Ensure APB1 clock is configured (example: 50 MHz)

    // Initialize I2C1 peripheral with 50 MHz APB1 clock
    I2C1_Initialization(50);
    USART1_Init();
    // Transmit the message that USART initialized
    USART_TX((uint8_t *)"USART OK\n\r", 10);
    // Example slave device address (modify according to your device)
    // For SHT21 temperature sensor: 0x80 (0x40 << 1)
    // For EEPROM 24LC256: 0xA0 (0x50 << 1)
    uint8_t slave_address = 0x80; // Example: EEPROM address

    // Example 1: Send a command byte to a slave device
    uint8_t command = 0xE3; // Example command (memory address for EEPROM)
    I2C_Status status = I2C_SendByteByADDR(I2C1, command, slave_address);

    if (status == I2C_OK)
    {
        // Byte sent successfully
        sprintf((char *)tx_buffer, "Byte 0x%X sent successfully\n\r", command);
        USART_TX(tx_buffer, strlen((char *)tx_buffer));
    }
    else
    {
        // Handle error based on status code
        switch (status)
        {
        case I2C_ERROR_START:
            // START condition failed
            USART_TX((uint8_t *)"START condition failed (I2C_ERROR_START)\n\r", 42);
            break;
        case I2C_ERROR_ADDR:
            // Slave not responding
            USART_TX((uint8_t *)"Slave not responding (I2C_ERROR_ADDR)\n\r", 39);
            break;
        case I2C_ERROR_TX:
            // Data transmission failed
            USART_TX((uint8_t *)"Data transmission failed (I2C_ERROR_TX)\n\r", 41);
            break;
        default:
            break;
        }
    }

    while (1)
    {
    }

    return 0;
}
