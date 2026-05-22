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
 * @brief Formats the initialization sequence.
 * Buffer size required: 9 bytes (3 frames * 3 bytes).
 */
int DAC80508_init(struct DAC80508_Config *config, uint8_t *tx_buffer) {
    if (!config || !tx_buffer) return -1;
	int frames = 0;

    /* Format SYNC Register (0x02)
     * Determines which channels update synchronously with LDAC.
     */
    uint16_t sync_val = config->sync_mask;
    _format_frame(DAC80508_REG_SYNC, sync_val, &tx_buffer[frames*DAC80508_FRAME_SIZE]);
    frames++;

    /* Format CONFIG Register (0x03)
     * Bit 8: REF_PWDWN (0 = Internal Ref ON, 1 = Internal Ref OFF)
     */
    uint16_t config_val = 0x0000;
    if (!config->use_internal_ref) {
        config_val |= (1 << 8); // Power down internal reference
    }
    _format_frame(DAC80508_REG_CONFIG, config_val, &tx_buffer[frames*DAC80508_FRAME_SIZE]);
    frames++;

    /* Format GAIN Register (0x04)
     * Bit 8: REF_DIV (0 = Ref not divided, 1 = Ref divided by 2)
     * Bits 0-7: GAIN for each channel (0 = Gain of 1, 1 = Gain of 2)
     */
    uint16_t gain_val = 0x0000;
    if (config->div_internal_ref) {
        gain_val |= (1 << 8);
    }
    gain_val |= (uint16_t)config->channel_gain_mask;
    
    _format_frame(DAC80508_REG_GAIN, gain_val, &tx_buffer[frames*DAC80508_FRAME_SIZE]);
    frames++;


    /* NOP Register (0x00) */
    uint16_t val=0x0;
    _format_frame(DAC80508_REG_NOP, val, &tx_buffer[frames*DAC80508_FRAME_SIZE]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
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

    // Standard write: Address MSB is 0
    _format_frame(reg, value, tx_buffer);
    frames++;

    /* NOP
     * Send 24 bits of zeros (NOP) to clock the data out of the MISO pin.
     */
    _format_frame(DAC80508_REG_NOP, 0x0000, &tx_buffer[3]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats a 6-byte SPI sequence to read a register.
 * @param reg The register address to read.
 * @param tx_buffer Buffer to hold 6 bytes (Read Command + NOP).
 */
int DAC80508_read_reg(uint8_t reg, uint8_t *tx_buffer) {
    if (!tx_buffer) return -1;
    int frames={0};

    /* Read: Address MSB must be 1. Data bits are "don't care" */
    uint8_t read_addr = reg | DAC80508_READ_BIT;
    _format_frame(read_addr, 0x0000, &tx_buffer[0]);
    frames++;

    /* Write NOP to clock the data out of the MISO pin.*/
    _format_frame(DAC80508_REG_NOP, 0x0000, &tx_buffer[3]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats a 24-bit SPI frame for a single channel update.
 * @param channel DAC channel (0-7).
 * @param value 16-bit digital value.
 * @param tx_buffer Buffer to hold 3 bytes.
 */
int DAC80508_set_output(int channel, uint16_t value, uint8_t *tx_buffer) {
    if (!tx_buffer || channel < 0 || channel > 7) return -1;
    int frames={0};

    /* DAC registers start at offset 0x08 (DAC0) through 0x0F (DAC7) */
    uint8_t reg = DAC80508_REG_DAC0 + (uint8_t)channel;
    _format_frame(reg, value, tx_buffer);
    frames++;

    /* Write NOP to clock the data out of the MISO pin.*/
    _format_frame(DAC80508_REG_NOP, 0x0000, &tx_buffer[3]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats 8 consecutive SPI frames to update all DAC channels.
 * @param value_array Pointer to 8 uint16_t values.
 * @param tx_buffer Buffer to hold 24 bytes (8 channels * 3 bytes).
 */
int DAC80508_set_outputs(uint8_t * addr_array, uint16_t *value_array, uint8_t *tx_buffer) {
    if (!tx_buffer || !value_array) return -1;
    int frames={0};

    for (int i = 0; i < DAC80508_NUM_CHANNELS; i++) {
        /* Calculate register address for each channel (0x08 to 0x0F) */
        uint8_t reg = DAC80508_REG_DAC0 + (uint8_t)i;
        
        /* Pack each 3-byte frame into the continuous buffer */
        _format_frame(reg, value_array[i], &tx_buffer[frames*DAC80508_FRAME_SIZE]);
        frames++;
    }
    /* send trigger */
    _format_frame(DAC80508_REG_TRIGGER, 0x0000, &tx_buffer[frames*DAC80508_FRAME_SIZE]);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats a 3-byte SPI frame for a broadcast update.
 */
int DAC80508_broadcast(uint16_t value, uint8_t *tx_buffer) {
    if (!tx_buffer) return -1;
    int frames={0};

    /* The Broadcast register (0x06) updates all DACx registers 
     * with the same 16-bit value simultaneously.
     */
    _format_frame(DAC80508_REG_BROADCAST, value, tx_buffer);
    frames++;
    /* no reason to read prior register value */
    return(frames*DAC80508_FRAME_SIZE);
}

/**
 * @brief Formats a 3-byte SPI frame to trigger a software LDAC.
 */
int DAC80508_send_trigger(uint8_t *tx_buffer) {
    if (!tx_buffer) return -1;
    int frames={0};

    /* TRIGGER Register (0x05)
     * Bit 4: SOFT_LDAC. When set to 1, it updates all DAC registers
     * that have been configured for synchronous mode in the SYNC register.
     */
    uint16_t trigger_val = (1 << 4);
    
    _format_frame(DAC80508_REG_TRIGGER, trigger_val, tx_buffer);
    frames++;

    return(frames*DAC80508_FRAME_SIZE);
}
