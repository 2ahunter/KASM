/**
 * @file dac80508.h
 * @author Aaron Hunter
 * @brief Header file for TI DAC80508 (8-Channel, 16-Bit, SPI DAC)
 * * Note: This DAC uses a 24-bit SPI frame (8-bit Address + 16-bit Data).
 */

#ifndef DAC80508_H
#define DAC80508_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip.h"

/* --- DAC80508 Register Map --- */
#define DAC80508_REG_NOP           0x00
#define DAC80508_REG_DEVICE_ID     0x01
#define DAC80508_REG_SYNC          0x02
#define DAC80508_REG_CONFIG        0x03
#define DAC80508_REG_GAIN          0x04
#define DAC80508_REG_TRIGGER       0x05
#define DAC80508_REG_BROADCAST     0x06
#define DAC80508_REG_STATUS        0x07
#define DAC80508_REG_DAC0          0x08
#define DAC80508_REG_DAC1          0x09
#define DAC80508_REG_DAC2          0x0A
#define DAC80508_REG_DAC3          0x0B
#define DAC80508_REG_DAC4          0x0C
#define DAC80508_REG_DAC5          0x0D
#define DAC80508_REG_DAC6          0x0E
#define DAC80508_REG_DAC7          0x0F

/* --- Frame Configuration --- */
#define DAC80508_FRAME_SIZE		3     // 8 bit header, 16 bits value
#define DAC80508_READ_BIT       0x80  // MSB of the address byte
#define DAC80508_NUM_CHANNELS	8
#define DAC80508_MID_VAL 		32737

/* --- Typedefs --- */
typedef struct frame {
    uint8_t reg;      // DAC Register address
    uint16_t data;    // Command data 16 bits (0-65535)
} frame_t;

/**
 * @brief Configuration settings for hardware initialization.
 */
struct DAC80508_Config {
    /** * @brief Internal Reference Enable.
     * true  = Internal reference is powered up.
     * false = Internal reference is powered down (External Ref mode).
     */
    bool use_internal_ref;

    /** * @brief Internal Reference Divider.
     * true  = Internal reference is divided by 2.
     * false = Internal reference is not divided.
     */
    bool div_internal_ref;

    /**
     * @brief Bitmask for channel gains (0-7).
     * 0 = Gain of 1 | 1 = Gain of 2
     */
    uint16_t channel_gain_mask;

    /**
     * @brief Bitmask for LDAC synchronization (0-7).
     * 0 = Asynchronous (Output updates immediately on SPI write)
     * 1 = Synchronous (Output updates only on LDAC pin/bit trigger)
     */
    uint16_t sync_mask;
};




/**
 * @brief Formats a read SPI transaction into a frame.
 * @param reg The register address.
 * @param tx_buffer Buffer to hold frame.
 * @note this needs to be converted to big-endian before sending over SPI
 */
int DAC80508_read_reg(uint8_t reg, uint32_t *tx_buffer);

/**
 * @brief Formats a 24bit SPI frame for a generic register write.
 * @param reg The register address.
 * @param value 16-bit value.
 * @param tx_buffer Buffer to hold one frame.
 * @note this needs to be converted to big-endian before sending over SPI
 */
int DAC80508_write_reg(uint8_t reg, uint16_t value, uint32_t *tx_buffer);



#endif /* DAC80508_H */
