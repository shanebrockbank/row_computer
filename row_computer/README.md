# ESP32 Rowing Computer Project

A comprehensive rowing performance tracker built on ESP32 using ESP-IDF, designed for real-time stroke analysis and boat motion tracking.

## Project Overview

This project creates a rowing computer that captures high-frequency motion data from multiple sensors and provides real-time analysis of rowing performance. The system is built with a focus on precise timing, robust error handling, and scalable architecture for future enhancements.

## Current Status

### ✅ Implemented Features

#### Core Timing System
- **1ms timestamp resolution** for all sensor data
- **Queue-based inter-task communication** with overflow protection
- **Rate-limited logging** to prevent serial port flooding
- **Comprehensive health monitoring** for all sensors
- **Precise task scheduling** with configurable frequencies

#### Hardware Support
- **MPU6050 IMU** (accelerometer + gyroscope) at 100 Hz
- **HMC5883L magnetometer** at 100 Hz (shared with IMU task)
- **NEO-6M GPS** at 1 Hz with NMEA parsing
- **I2C and UART protocols** with error handling

#### Task Architecture
- **IMU Task**: High-priority (100 Hz) motion sensor reading
- **GPS Task**: Medium-priority (1 Hz) position tracking  
- **Logging Task**: Low-priority (20 Hz) data processing and stroke detection
- **Health Monitor Task**: System health reporting every 5 seconds

#### Data Management
- **Power-of-2 queue sizing** for memory efficiency
- **Automatic overflow handling** (drop oldest data)
- **Memory usage tracking** (~18.4 KB for all queues)
- **Queue health monitoring** with buffer depth reporting

#### Error Handling & Recovery
- **Progressive sensor failure handling** with automatic reset attempts
- **Runtime health monitoring** with detailed status reporting
- **Graceful degradation** when non-critical sensors fail
- **Memory leak detection** and system resource monitoring

### 🚧 Architecture Design

```
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
│ IMU Sensors │───▶│ IMU Queue    │───▶│                 │
│ (100 Hz)    │    │ (256 samples)│    │ Logging Task    │
└─────────────┘    └──────────────┘    │ (20 Hz)         │
                                       │ - Stroke detect │
┌─────────────┐    ┌──────────────┐    │ - Data process  │──┐
│ GPS Module  │───▶│ GPS Queue    │───▶│ - Queue health  │  │
│ (1 Hz)      │    │ (16 samples) │    └─────────────────┘  │
└─────────────┘    └──────────────┘                         │
                                                             │
┌─────────────────────────────────────────────────────────────▼───┐
│ Health Monitor Task (0.2 Hz)                                    │
│ - System status reporting                                       │
│ - Memory usage tracking                                         │
│ - Sensor failure recovery                                       │
└─────────────────────────────────────────────────────────────────┘
```

## Hardware Configuration

### Pin Assignments
```c
// I2C (for IMU and Magnetometer)
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21

// UART (for GPS)  
#define GPS_UART_TXD_PIN     17
#define GPS_UART_RXD_PIN     16
```

### Sensor Addresses
- **MPU6050**: 0x68 (accelerometer + gyroscope)
- **HMC5883L**: 0x1E (magnetometer)
- **NEO-6M GPS**: UART2 at 9600 baud

## System Performance

### Memory Usage
| Component | Size | Usage |
|-----------|------|-------|
| IMU Queue | 256 × 36B | ~9.2 KB |
| GPS Queue | 16 × 64B | ~1.0 KB |
| Output Queue | 128 × 64B | ~8.2 KB |
| Task Stacks | 4 tasks | ~20 KB |
| **Total** | | **~38.4 KB