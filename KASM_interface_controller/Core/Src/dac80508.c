#include "dac80508.h"

/**
 * @brief Private helper to format a single 24-bit SPI frame into a buffer.
 * @param reg The register address.
 * @param value The 16-bit data value.
 * @param buf The buffer (3 bytes) to fill.
 */
static void _format_frame(uint8_t reg, uint16_t value, uint8_t *buf) {
    buf[0] = reg;           // Register address, MSB set to 1 for a read
    buf[1] = (uint8_t)(value >> 8);     // MSB of data
    buf[2] = (uint8_t)(value & 0xFF);   // LSB of data
}


/**
 * @brief Formats a 3-byte SPI frame for a generic register write.
 * @param reg The register address.
 * @param value 16-bit value.
 * @param tx_buffer Buffer to hold 3 bytes.
 */
int DAC80508_write_reg(uint8_t reg, uint16_t value, uint8_t *tx_buffer) {
    if (!tx_buffer) return -1;
	int frames={0};

    _format_frame(reg, value, tx_buffer);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats a 3-byte SPI sequence to read a register.
 * @param reg The register address to read.
 * @param tx_buffer Buffer to hold 3 bytes.
 */
int DAC80508_read_reg(uint8_t reg, uint8_t *tx_buffer) {
    if (!tx_buffer) return -1;
    int frames={0};

    uint8_t read_addr = reg | DAC80508_READ_BIT;
    _format_frame(read_addr, 0x0000, &tx_buffer[0]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

