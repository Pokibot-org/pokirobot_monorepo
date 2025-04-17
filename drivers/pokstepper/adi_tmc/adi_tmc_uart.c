#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/shared_uart.h>

LOG_MODULE_REGISTER(tmc_uart, CONFIG_POKSTEPPER_LOG_LEVEL);

#define SYNC_NIBBLE        0x05
#define REG_WRITE_BIT      0x80
#define TMC_ANSWER_TIMEOUT K_MSEC(10)

uint8_t tmc_uart_crc(uint8_t *data, uint8_t data_len)
{
    uint8_t crc = 0;

    for (int i = 0; i < data_len; i++) { // Execute for all bytes of a message
        uint8_t currentByte = data[i];   // Retrieve a byte to be sent from Array
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) { // update CRC based result of XOR operation
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
    return crc;
}

// TMC r/w registers
int tmc_reg_read(const struct device *shared_uart_dev, uint8_t slave_address, uint8_t reg,
                 uint32_t *data)
{
    const uint8_t tx_len = 4;
    const uint8_t rx_len = 8;
    uint8_t rx_buf[tx_len + rx_len];
    uint8_t *rx_data = rx_buf;
    uint8_t tx_buf[] = {SYNC_NIBBLE, slave_address, reg, 0};

    tx_buf[tx_len - 1] = tmc_uart_crc(tx_buf, tx_len - 1);

    if (shared_uart_send_receive(shared_uart_dev, tx_buf, tx_len, rx_len, &rx_data,
                                 TMC_ANSWER_TIMEOUT)) {
        LOG_WRN("tmc reg read timeout: 0x%02x/0x%02x", slave_address, reg);
        return -ETIMEDOUT;
    }

    uint8_t calc_crc = tmc_uart_crc(rx_data, rx_len - 1);
    if (calc_crc != rx_data[rx_len - 1]) {
        LOG_WRN("tmc reg read wrong crc (wanted %d got %d)", calc_crc, rx_data[rx_len - 1]);
        LOG_HEXDUMP_DBG(rx_data, rx_len, "rx frame");
        return -EIO;
    }

    *data = sys_get_be32(&rx_data[3]);
    return 0;
}

int tmc_reg_write(const struct device *shared_uart_dev, uint8_t slave_address, uint8_t reg,
                  uint32_t value)
{

    const uint8_t tx_len = 8;
    const uint8_t rx_len = 0;
    uint8_t rx_buf[tx_len + rx_len];
    uint8_t *rx_data = rx_buf;

    uint8_t tx_buf[] = {SYNC_NIBBLE, slave_address, REG_WRITE_BIT | reg, 0, 0, 0, 0, 0};
    sys_put_be32(value, &tx_buf[3]);
    tx_buf[tx_len - 1] = tmc_uart_crc(tx_buf, tx_len - 1);

    if (shared_uart_send_receive(shared_uart_dev, tx_buf, tx_len, rx_len, &rx_data,
                                 TMC_ANSWER_TIMEOUT)) {
        LOG_WRN("tmc reg write timeout");
        return -ETIMEDOUT;
    }

    return 0;
}
