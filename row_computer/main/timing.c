// Simple unified timing system - 1ms timestamps for everything
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "TIMING";

// Universal timestamp - 1ms resolution for all sensors
typedef struct {
    uint32_t timestamp_ms;    // Milliseconds since boot (simple and consistent)
    bool data_valid;          // Flag if sensor read was successful
} sensor_timestamp_t;

// IMU data with 1ms timestamp
typedef struct {
    sensor_timestamp_t time;
    float accel_x, accel_y, accel_z;    // g
    float gyro_x, gyro_y, gyro_z;       // deg/s  
    float mag_x, mag_y, mag_z;          // gauss
} imu_data_t;

// GPS data with 1ms timestamp  
typedef struct {
    sensor_timestamp_t time;
    char time_string[16];       // "HH:MM:SS"
    double latitude;            // degrees
    double longitude;           // degrees
    float speed_knots;          // knots
    float heading;              // degrees
    int satellites;             // count
    bool valid_fix;             // GPS fix status
} gps_data_t;

// Simple system timer
uint32_t get_system_time_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Sensor reading frequencies (sensors can run at different rates)
#define IMU_TASK_RATE_MS        10      // 100 Hz (every 10ms)
#define GPS_TASK_RATE_MS        1000    // 1 Hz (every 1000ms)
#define LOG_TASK_RATE_MS        50      // 20 Hz (every 50ms)

// Task priorities
#define IMU_TASK_PRIORITY       6       // High (time-critical)
#define GPS_TASK_PRIORITY       4       // Medium
#define LOG_TASK_PRIORITY       2       // Low (processing)

// Queue sizes (based on task rate differences)
#define IMU_QUEUE_SIZE          20      // ~200ms buffer at 100Hz
#define GPS_QUEUE_SIZE          5       // ~5 seconds buffer at 1Hz

// Updated task functions with simple timing
void imu_task(void *parameters) {
    ESP_LOGI("IMU_TASK", "Starting IMU task at 100Hz (10ms intervals)");
    
    imu_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get timestamp first (consistent across all sensors)
        imu_data.time.timestamp_ms = get_system_time_ms();
        
        // Read all IMU sensors
        esp_err_t accel_err = accel_read(&imu_data.accel_x, &imu_data.accel_y, &imu_data.accel_z);
        esp_err_t gyro_err = gyro_read(&imu_data.gyro_x, &imu_data.gyro_y, &imu_data.gyro_z);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);
        
        // Mark data as valid if at least accelerometer worked
        imu_data.time.data_valid = (accel_err == ESP_OK);
        
        // Send to queue (non-blocking to maintain timing)
        if (xQueueSend(imu_data_queue, &imu_data, 0) != pdTRUE) {
            ESP_LOGW("IMU_TASK", "Queue full - dropping sample at %lu ms", imu_data.time.timestamp_ms);
        }
        
        // Log sensor health occasionally
        static int health_counter = 0;
        if (++health_counter >= 1000) { // Every 10 seconds at 100Hz
            ESP_LOGI("IMU_TASK", "Health check at %lu ms - Accel:%s Gyro:%s Mag:%s", 
                    imu_data.time.timestamp_ms,
                    (accel_err == ESP_OK) ? "OK" : "FAIL",
                    (gyro_err == ESP_OK) ? "OK" : "FAIL", 
                    (mag_err == ESP_OK) ? "OK" : "FAIL");
            health_counter = 0;
        }
        
        // Maintain exact 10ms timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_TASK_RATE_MS));
    }
}

void gps_task(void *parameters) {
    ESP_LOGI("GPS_TASK", "Starting GPS task at 1Hz (1000ms intervals)");
    
    gps_data_t gps_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get timestamp first
        gps_data.time.timestamp_ms = get_system_time_ms();
        
        // Read GPS (this function handles UART reading and parsing)
        esp_err_t gps_err = gps_read_simple(&gps_data);
        
        gps_data.time.data_valid = (gps_err == ESP_OK);
        
        // Send to queue
        if (xQueueSend(gps_data_queue, &gps_data, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW("GPS_TASK", "Failed to send GPS data to queue");
        }
        
        // Log GPS status every 10 readings
        static int gps_log_counter = 0;
        if (++gps_log_counter >= 10) {
            if (gps_data.valid_fix) {
                ESP_LOGI("GPS_TASK", "Fix at %lu ms: %.6f°, %.6f° | Speed: %.1f kts | Sats: %d",
                        gps_data.time.timestamp_ms, gps_data.latitude, gps_data.longitude, 
                        gps_data.speed_knots, gps_data.satellites);
            } else {
                ESP_LOGI("GPS_TASK", "No fix at %lu ms | Sats: %d", 
                        gps_data.time.timestamp_ms, gps_data.satellites);
            }
            gps_log_counter = 0;
        }
        
        // Maintain exact 1000ms timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(GPS_TASK_RATE_MS));
    }
}

void logging_task(void *parameters) {
    ESP_LOGI("LOG_TASK", "Starting logging task at 20Hz (50ms intervals)");
    
    imu_data_t imu_data;
    gps_data_t gps_data;
    
    while (1) {
        uint32_t current_time = get_system_time_ms();
        
        // Process all available IMU data
        while (xQueueReceive(imu_data_queue, &imu_data, 0) == pdTRUE) {
            if (imu_data.time.data_valid) {
                // Simple rowing analysis
                float accel_magnitude = sqrt(imu_data.accel_x * imu_data.accel_x + 
                                           imu_data.accel_y * imu_data.accel_y + 
                                           imu_data.accel_z * imu_data.accel_z);
                
                // Detect potential stroke (simple threshold)
                if (accel_magnitude > 2.0f) {
                    ESP_LOGI("LOG_TASK", "Stroke at %lu ms: %.2fg", 
                            imu_data.time.timestamp_ms, accel_magnitude);
                }
                
                // Here you would save to SD card, calculate metrics, etc.
                log_imu_data_to_storage(&imu_data);
            }
        }
        
        // Process GPS data (less frequent)
        if (xQueueReceive(gps_data_queue, &gps_data, 0) == pdTRUE) {
            if (gps_data.time.data_valid && gps_data.valid_fix) {
                // Calculate distance, speed, track, etc.
                ESP_LOGI("LOG_TASK", "GPS logged at %lu ms: %.6f,%.6f",
                        gps_data.time.timestamp_ms, gps_data.latitude, gps_data.longitude);
                
                log_gps_data_to_storage(&gps_data);
            }
        }
        
        // Process at 20Hz
        vTaskDelay(pdMS_TO_TICKS(LOG_TASK_RATE_MS));
    }
}

// Simple storage functions (placeholder)
void log_imu_data_to_storage(imu_data_t *data) {
    // Save to SD card, CSV format:
    // timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
    
    // For now just track data rate
    static uint32_t imu_samples_logged = 0;
    static uint32_t last_log_time = 0;
    
    imu_samples_logged++;
    
    if (data->time.timestamp_ms - last_log_time >= 10000) { // Every 10 seconds
        float data_rate = (float)imu_samples_logged / 10.0f;
        ESP_LOGI("STORAGE", "IMU logging rate: %.1f Hz", data_rate);
        imu_samples_logged = 0;
        last_log_time = data->time.timestamp_ms;
    }
}

void log_gps_data_to_storage(gps_data_t *data) {
    // Save to SD card, CSV format:
    // timestamp_ms,gps_time,lat,lon,speed_knots,heading,satellites,valid_fix
    
    static uint32_t gps_samples_logged = 0;
    gps_samples_logged++;
    
    if (gps_samples_logged % 60 == 0) { // Every minute
        ESP_LOGI("STORAGE", "GPS samples logged: %lu", gps_samples_logged);
    }
}

// Simple GPS read function (wrapper for existing GPS code)
esp_err_t gps_read_simple(gps_data_t *data) {
    // Use existing GPS parsing but return simple structure
    gps_data_t temp_gps;
    esp_err_t err = gps_read(&temp_gps); // Your existing function
    
    if (err == ESP_OK) {
        // Copy to simple structure
        data->latitude = temp_gps.latitude;
        data->longitude = temp_gps.longitude;
        data->speed_knots = temp_gps.speed_knots;
        data->heading = temp_gps.heading;
        data->satellites = temp_gps.satellites;
        data->valid_fix = temp_gps.valid_fix;
        strncpy(data->time_string, temp_gps.time, sizeof(data->time_string));
    }
    
    return err;
}

// System status
void print_timing_status(void) {
    uint32_t current_time = get_system_time_ms();
    ESP_LOGI(TAG, "=== System Timing Status ===");
    ESP_LOGI(TAG, "Current time: %lu ms (%.1f seconds)", current_time, current_time / 1000.0f);
    ESP_LOGI(TAG, "IMU task: %d ms intervals (%.1f Hz)", IMU_TASK_RATE_MS, 1000.0f / IMU_TASK_RATE_MS);
    ESP_LOGI(TAG, "GPS task: %d ms intervals (%.1f Hz)", GPS_TASK_RATE_MS, 1000.0f / GPS_TASK_RATE_MS);
    ESP_LOGI(TAG, "Log task: %d ms intervals (%.1f Hz)", LOG_TASK_RATE_MS, 1000.0f / LOG_TASK_RATE_MS);
    ESP_LOGI(TAG, "All timestamps in consistent 1ms resolution");
}