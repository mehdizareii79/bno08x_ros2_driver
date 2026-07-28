#ifndef SPI_INTERFACE_HPP
#define SPI_INTERFACE_HPP

#include <stdexcept>
#include "comm_interface.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <bno08x_driver/gpio-sysfs.h>

/**
 * @brief SPI communication interface (not implemented)
 */
class SPIInterface : public CommInterface {
public:

    SPIInterface(const std::string& spi_device, int int_pin, int wak_pin, int rst_pin)
        : spi_device_(spi_device), int_pin_(int_pin), wak_pin_(wak_pin), rst_pin_(rst_pin),
          spi_fd_(-1), int_fd_(-1), wak_fd_(-1), rst_fd_(-1) {
        std::cout << "SPI Device: " << spi_device_ << std::endl;
        std::cout << "Pins -> INT: " << int_pin_ << ", WAK: " << wak_pin_ << ", RST: " << rst_pin_ << std::endl;
        DEBUG_LOG("BNO08x - SPI Interface Created");
    }

    int open() override {
        // 1. Export and configure sysfs GPIO pins
        if (!setup_gpio(int_pin_, "in", "falling")) {
            std::cout<<"setup int_pin_ failed"<<std::endl;
            return -1;
        }
        if (!setup_gpio(wak_pin_, "out", "")) {
            std::cout<<"setup wak_pin_ failed"<<std::endl;
            return -1;
        }
        if (!setup_gpio(rst_pin_, "out", "")) {
            std::cout<<"setup rst_pin_ failed"<<std::endl;
            return -1;
        }

        // Open value handles for fast low-overhead pin access
        int_fd_ = ::open(("/sys/class/gpio/gpio" + std::to_string(int_pin_) + "/value").c_str(), O_RDONLY);
        wak_fd_ = ::open(("/sys/class/gpio/gpio" + std::to_string(wak_pin_) + "/value").c_str(), O_WRONLY);
        rst_fd_ = ::open(("/sys/class/gpio/gpio" + std::to_string(rst_pin_) + "/value").c_str(), O_WRONLY);

        if (int_fd_ < 0 || wak_fd_ < 0 || rst_fd_ < 0) {
            std::cerr << "BNO08x - Failed to open GPIO value files" << std::endl;
            return -1;
        }

        // 2. Open SPI bus device node
        spi_fd_ = ::open(spi_device_.c_str(), O_RDWR);
        if (spi_fd_ < 0) {
            std::cerr << "BNO08x - Failed to open SPI bus" << std::endl;
            return -1;
        }

        // Set SPI Mode 3 (CPOL=1, CPHA=1 as required by BNO08x)
        uint8_t mode = SPI_MODE_3;
        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
            std::cerr << "BNO08x - Failed to set SPI Mode 3" << std::endl;
            ::close(spi_fd_);
            return -1;
        }

        uint8_t bits = 8;
        if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return -1;

        uint32_t speed = 3000000; // 3 MHz maximum speed for BNO08x
        if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return -1;

        // 3. Hardware Reset Sequence
        set_gpio(wak_fd_, 1); // De-assert Wake (Active Low)
        set_gpio(rst_fd_, 0); // Assert Hardware Reset (Active Low)
        usleep(10000);        // Hold reset low for 10ms
        set_gpio(rst_fd_, 1); // Release Reset
        usleep(300000);       // Wait 300ms for IMU Cortex-M0+ firmware bootup

        DEBUG_LOG("BNO08x - SPI Comm Opened and Hardware Reset Sent");
        return 0;
    }

    void close() override {
        if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }
        if (int_fd_ >= 0) { ::close(int_fd_); int_fd_ = -1; }
        if (wak_fd_ >= 0) { ::close(wak_fd_); wak_fd_ = -1; }
        if (rst_fd_ >= 0) { ::close(rst_fd_); rst_fd_ = -1; }
        DEBUG_LOG("BNO08x - SPI Comm Closed");
    }

    int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) override {
        // if (!wait_for_int(100)) {
        //     return 0;
        // }
        //
        // // The SH-2 library passes in 'len' (384 bytes).
        // // Read the whole block in one continuous transfer to keep CS LOW.
        // if (!spi_transfer(nullptr, pBuffer, len)) {
        //     return 0;
        // }
        //
        // if (t_us) *t_us = getTimeUs();
        // return len; // Return full length; the SH-2 library parses the header itself
        // 1. Non-blocking check for INT pin going LOW (Data Available)
        if (!wait_for_int(1000)) { // 100ms timeout
            std::cout<<"[read] wait for int time out! "<<std::endl;
            return 0;
        }

        // 2. Read 4-byte SHTP header to obtain packet size
        uint8_t header[4] = {0};
        if (!spi_transfer(nullptr, header, 4)) {
            return 0;
        }

        // Reconstruct 16-bit packet size from Little-Endian bytes
        uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
        packet_size &= ~0x8000; // Mask out SHTP continuation bit

        DEBUG_LOG("BNO08x - SPI Packet size: " << packet_size);
        DEBUG_LOG_BUFFER(header, 4);

        if (packet_size > len || packet_size == 0) {
            return 0; // Buffer overflow safety check
        }

        // 3. Copy already-received header into caller's buffer
        memcpy(pBuffer, header, 4);

        // 4. Read remaining cargo in a single SPI transfer
        if (packet_size > 4) {
            if (!spi_transfer(nullptr, pBuffer + 4, packet_size - 4)) {
                return 0;
            }
        }

        if (t_us) *t_us = getTimeUs();
        return packet_size;
    }

    int write(uint8_t *pBuffer, unsigned len) override {
        // 1. Assert WAK line low to signal host wants to transmit
        set_gpio(wak_fd_, 0);

        // 2. Wait for BNO08x to pull INT low, acknowledging it is awake
        if (!wait_for_int(100)) {
            set_gpio(wak_fd_, 1); // De-assert WAK on timeout
            return 0;
        }

        // 3. Perform full packet SPI transfer
        bool success = spi_transfer(pBuffer, nullptr, len);
        usleep(10);
        // 4. De-assert WAK high
        set_gpio(wak_fd_, 1);

        return success ? len : 0;
    }

private:
    std::string spi_device_;
    int int_pin_, wak_pin_, rst_pin_;
    int spi_fd_, int_fd_, wak_fd_, rst_fd_;

    // Configures sysfs GPIO directions and interrupt edges
    bool setup_gpio(int pin, const std::string& dir, const std::string& edge) {
        // int fd = ::open("/sys/class/gpio/export", O_WRONLY);
        // if (fd >= 0) {
        //     std::string p = std::to_string(pin);
        //     ::write(fd, p.c_str(), p.length());
        //     ::close(fd);
        // }
        // usleep(10000); // Short pause for sysfs node creation
        //
        // std::string dir_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
        // fd = ::open(dir_path.c_str(), O_WRONLY);
        // if (fd < 0) return false;
        // ::write(fd, dir.c_str(), dir.length());
        // ::close(fd);
        //
        // if (!edge.empty()) {
        //     std::string edge_path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/edge";
        //     fd = ::open(edge_path.c_str(), O_WRONLY);
        //     if (fd >= 0) {
        //         ::write(fd, edge.c_str(), edge.length());
        //         ::close(fd);
        //     }
        // }
        // return true;
        SysGPIO gpio(pin);

        // 1. Export the GPIO
        if (gpio.gpio_export() < 0) {
            return false;
        }

        usleep(10000); // Short pause for sysfs node creation

        // 2. Set direction
        bool is_out = (dir == "out");
        if (gpio.gpio_set_dir(is_out) < 0) {
            return false;
        }

        // 3. Set edge if provided
        if (!edge.empty()) {
            if (gpio.gpio_set_edge(edge.c_str()) < 0) {
                return false;
            }
        }

        return true;
    }

    // Fast inline wrapper to toggle sysfs GPIOs
    inline void set_gpio(int fd, int val) {
        if (fd >= 0) {
            ::pwrite(fd, val ? "1" : "0", 1, 0);
        }
    }

    // Polls the INT pin file descriptor without wasting CPU cycles in busy-wait
    bool wait_for_int(int timeout_ms) {
        struct pollfd pfd;
        pfd.fd = int_fd_;
        pfd.events = POLLPRI; // Sysfs GPIO interrupts register as POLLPRI

        char c;
        ::pread(int_fd_, &c, 1, 0); // Read to clear any stale interrupt flag
        if (c == '0') return true;

        if (::poll(&pfd, 1, timeout_ms) > 0) {
            if (pfd.revents & POLLPRI) {
                ::pread(int_fd_, &c, 1, 0); // Consume event
                return true;
            }
        }
        return false;
        // auto start = std::chrono::steady_clock::now();
        // char c;
        //
        // while (true) {
        //     // Read the exact physical state of the pin right now
        //     ::pread(int_fd_, &c, 1, 0);
        //
        //     if (c == '0') {
        //         return true; // Sensor pulled INT low, it's ready!
        //     }
        //
        //     // Check for timeout
        //     auto now = std::chrono::steady_clock::now();
        //     if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= timeout_ms) {
        //         return false; // Timed out
        //     }
        //
        //     // Yield CPU for 100 microseconds to prevent 100% core lockup
        //     usleep(100);
        // }
    }

    // Core spidev full-duplex driver wrapper
    bool spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
        std::vector<uint8_t> dummy_tx(len, 0x00);
        std::vector<uint8_t> dummy_rx(len, 0x00);

        struct spi_ioc_transfer tr = {0};
        tr.tx_buf = tx ? (uintptr_t)tx : (uintptr_t)dummy_tx.data();
        tr.rx_buf = rx ? (uintptr_t)rx : (uintptr_t)dummy_rx.data();
        tr.len = len;
        tr.speed_hz = 3000000;
        tr.bits_per_word = 8;

        return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
    }
};

#endif // SPI_INTERFACE_HPP