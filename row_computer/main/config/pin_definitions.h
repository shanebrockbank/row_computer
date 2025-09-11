#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// I2C Configuration
#define I2C_MASTER_SCL_IO           22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           21    // GPIO number for I2C master data
#define I2C_MASTER_NUM              0     // I2C master port number
#define I2C_MASTER_FREQ_HZ          400000 // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// UART Configuration (for GPS)
#define GPS_UART_NUM                UART_NUM_1
#define GPS_UART_BAUD_RATE          9600
#define GPS_UART_TXD_PIN            4
#define GPS_UART_RXD_PIN            5
#define GPS_UART_RTS_PIN            UART_PIN_NO_CHANGE
#define GPS_UART_CTS_PIN            UART_PIN_NO_CHANGE
#define GPS_UART_BUF_SIZE           1024

// SPI Configuration (if needed for other sensors)
#define SPI_MOSI_PIN                23
#define SPI_MISO_PIN                19
#define SPI_SCLK_PIN                18
#define SPI_CS_PIN                  5

// MPU6050
#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3
#define MPU6050_ACCEL_CONFIG_REG    0x1C
#define LSB_SENSITIVITY_2g          16384.0f

#endif // PIN_DEFINITIONS_H