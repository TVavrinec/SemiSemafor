#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

#include <cassert>
#include <cstdint>
#include <tuple>
#include <vector>

class IMU {
private:
    spi_device_handle_t m_handle;

    gpio_num_t m_sdo;
    gpio_num_t m_sdi;
    gpio_num_t m_sck;
    gpio_num_t m_cs;

    constexpr std::uint8_t makeCmd(bool read, bool multi)
    {
        enum {
            IsMulti = 1 << 0,
            IsRead = 1 << 1,
        };
        return (read ? IsRead : 0) | (multi ? IsMulti : 0);
    }

public:
    enum Register {
        STATUS_REG_AUX = 0x07, // r 07 000 0111 Output
        OUT_TEMP_L = 0x0C, // r 0C 000 1100 Output
        OUT_TEMP_H = 0x0D, // r 0D 000 1101 Output
        WHO_AM_I = 0x0F, // r 0F 000 1111 00110011 Dummy register
        CTRL_REG0 = 0x1E, // rw 1E 001 1110 00010000
        TEMP_CFG_REG = 0x1F, // rw 1F 001 1111 00000000
        CTRL_REG1 = 0x20, // rw 20 010 0000 00000111
        CTRL_REG2 = 0x21, // rw 21 010 0001 00000000
        CTRL_REG3 = 0x22, // rw 22 010 0010 00000000
        CTRL_REG4 = 0x23, // rw 23 010 0011 00000000
        CTRL_REG5 = 0x24, // rw 24 010 0100 00000000
        CTRL_REG6 = 0x25, // rw 25 010 0101 00000000
        REFERENCE = 0x26, // rw 26 010 0110 00000000
        STATUS_REG = 0x27, // r 27 010 0111 Output
        OUT_X_L = 0x28, // r 28 010 1000 Output
        OUT_X_H = 0x29, // r 29 010 1001 Output
        OUT_Y_L = 0x2A, // r 2A 010 1010 Output
        OUT_Y_H = 0x2B, // r 2B 010 1011 Output
        OUT_Z_L = 0x2C, // r 2C 010 1100 Output
        OUT_Z_H = 0x2D, // r 2D 010 1101 Output
        FIFO_CTRL_REG = 0x2E, // rw 2E 010 1110 00000000
        FIFO_SRC_REG = 0x2F, // r 2F 010 1111 Output
        INT1_CFG = 0x30, // rw 30 011 0000 00000000
        INT1_SRC = 0x31, // r 31 011 0001 Output
        INT1_THS = 0x32, // rw 32 011 0010 00000000
        INT1_DURATION = 0x33, // rw 33 011 0011 00000000
        INT2_CFG = 0x34, // rw 34 011 0100 00000000
        INT2_SRC = 0x35, // r 35 011 0101 Output
        INT2_THS = 0x36, // rw 36 011 0110 00000000
        INT2_DURATION = 0x37, // rw 37 011 0111 00000000
        CLICK_CFG = 0x38, // rw 38 011 1000 00000000
        CLICK_SRC = 0x39, // r 39 011 1001 Output
        CLICK_THS = 0x3A, // rw 3A 011 1010 00000000
        TIME_LIMIT = 0x3B, // rw 3B 011 1011 00000000
        TIME_LATENCY = 0x3C, // rw 3C 011 1100 00000000
        TIME_WINDOW = 0x3D, // rw 3D 011 1101 00000000
        ACT_THS = 0x3E, // rw 3E 011 1110 00000000
        ACT_DUR = 0x3F, // rw 3F 011 1111 00000000
    };

    IMU(gpio_num_t sdo, gpio_num_t sdi, gpio_num_t sck, gpio_num_t cs)
        : m_sdo(sdo)
        , m_sdi(sdi)
        , m_sck(sck)
        , m_cs(cs)
    {
        spi_bus_config_t bus_config = {
            .mosi_io_num = m_sdi,
            .miso_io_num = m_sdo,
            .sclk_io_num = m_sck,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
        };

        spi_device_interface_config_t dev_config = {
            .command_bits = 2,
            .address_bits = 6,
            .dummy_bits = 0,
            .mode = 0,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = 1000000,
            .input_delay_ns = 0,
            .spics_io_num = m_cs,
            .flags = 0,
            .queue_size = 1,
            .pre_cb = nullptr,
            .post_cb = nullptr,
        };

        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &m_handle));
    }

    ~IMU()
    {
        spi_bus_remove_device(m_handle);
        spi_bus_free(SPI2_HOST);
    }

    std::vector<std::uint8_t> read(Register addr, std::size_t size)
    {
        std::vector<std::uint8_t> data(size);
        spi_transaction_t t = {
            .flags = 0,
            .cmd = makeCmd(true, size > 1),
            .addr = addr,
            .length = size * 8,
            .rxlength = size * 8,
            .user = nullptr,
            .tx_buffer = nullptr,
            .rx_buffer = data.data(),
        };

        ESP_ERROR_CHECK(spi_device_transmit(m_handle, &t));

        return data;
    }

    void write(Register addr, std::vector<std::uint8_t> data)
    {
        spi_transaction_t t = {
            .flags = 0,
            .cmd = makeCmd(false, data.size() > 1),
            .addr = addr,
            .length = data.size() * 8,
            .rxlength = 0,
            .user = nullptr,
            .tx_buffer = data.data(),
            .rx_buffer = nullptr,
        };

        ESP_ERROR_CHECK(spi_device_transmit(m_handle, &t));
    }

    void update(Register addr, std::vector<std::uint8_t> data, std::vector<std::uint8_t> mask)
    {
        assert(data.size() == mask.size());
        auto current = read(addr, data.size());
        for (std::size_t i = 0; i < data.size(); ++i)
            current[i] = (current[i] & ~mask[i]) | (data[i] & mask[i]);
        write(addr, current);
    }

    std::uint8_t readByte(Register addr)
    {
        return read(addr, 1)[0];
    }

    void writeByte(Register addr, std::uint8_t data)
    {
        write(addr, { data });
    }

    void updateByte(Register addr, std::uint8_t data, std::uint8_t mask)
    {
        update(addr, { data }, { mask });
    }

    void init()
    {
    }

    std::tuple<std::int16_t, std::int16_t, std::int16_t> accel(int bitWidth = 10)
    {
        auto data = read(OUT_X_L, 6);
        return {
            static_cast<std::int16_t>((data[1] << 8) | data[0]) >> (16 - bitWidth),
            static_cast<std::int16_t>((data[3] << 8) | data[2]) >> (16 - bitWidth),
            static_cast<std::int16_t>((data[5] << 8) | data[4]) >> (16 - bitWidth),
        };
    }
};
