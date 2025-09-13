// main/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "config/pin_definitions.h"
#include "sensors/sensors_common.h"
#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag.h"
#include "sensors/gps.h"
#include "utils/protocol_init.h"

static const char *TAG = "MAIN";

// Task handles for monitoring/control
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t logging_task_handle = NULL;

// Communication queues
QueueHandle_t imu_data_queue = NULL;
QueueHandle_t gps_data_queue = NULL;

// High-frequency IMU task - combines all motion sensors
void imu_task(void *parameters) {
    ESP_LOGI("IMU_TASK", "Starting IMU task at 100Hz");
    
    imu_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Read all IMU sensors in sequence (they share I2C bus)
        esp_err_t accel_err = accel_read(&imu_data.accel_x, &imu_data.accel_y, &imu_data.accel_z);
        esp_err_t gyro_err = gyro_read(&imu_data.gyro_x, &imu_data.gyro_y, &imu_data.gyro_z);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);
        
        // Only send data if at least accelerometer worked (critical for stroke detection)
        if (accel_err == ESP_OK) {
            imu_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue (non-blocking to maintain timing)
            if (xQueueSend(imu_data_queue, &imu_data, 0) != pdTRUE) {
                ESP_LOGW("IMU_TASK", "Queue full - dropping IMU sample");
            }
        } else {
            ESP_LOGW("IMU_TASK", "Failed to read accelerometer: %s", esp_err_to_name(accel_err));
        }
        
        // Log sensor health periodically
        static int health_counter = 0;
        if (++health_counter >= 1000) { // Every 10 seconds at 100Hz
            ESP_LOGI("IMU_TASK", "Sensor health - Accel:%s Gyro:%s Mag:%s", 
                    (accel_err == ESP_OK) ? "OK" : "FAIL",
                    (gyro_err == ESP_OK) ? "OK" : "FAIL", 
                    (mag_err == ESP_OK) ? "OK" : "FAIL");
            health_counter = 0;
        }
        
        // Maintain exact 100Hz timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

// Low-frequency GPS task
void gps_task(void *parameters) {
    ESP_LOGI("GPS_TASK", "Starting GPS task at 1Hz");
    
    gps_data_t gps_data;
    
    while (1) {
        esp_err_t gps_err = gps_read(&gps_data);
        
        if (gps_err == ESP_OK) {
            // Add timestamp
            gps_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue
            if (xQueueSend(gps_data_queue, &gps_data, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGW("GPS_TASK", "Failed to send GPS data to queue");
            }
            
            // Log GPS status every 10 seconds
            static int gps_log_counter = 0;
            if (++gps_log_counter >= 10) {
                if (gps_data.valid_fix) {
                    ESP_LOGI("GPS_TASK", "Position: %.6f°, %.6f° | Speed: %.1f kts | Sats: %d",
                            gps_data.latitude, gps_data.longitude, 
                            gps_data.speed_knots, gps_data.satellites);
                } else {
                    ESP_LOGI("GPS_TASK", "Searching for fix... Satellites: %d", gps_data.satellites);
                }
                gps_log_counter = 0;
            }
        }
        
        // 1Hz update rate
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Data processing and logging task
void logging_task(void *parameters) {
    ESP_LOGI("LOG_TASK", "Starting logging task");
    
    imu_data_t imu_data;
    gps_data_t gps_data;
    
    while (1) {
        // Process IMU data (high frequency)
        while (xQueueReceive(imu_data_queue, &imu_data, 0) == pdTRUE) {
            // Here you would:
            // 1. Apply filtering/calibration
            // 2. Detect rowing strokes
            // 3. Calculate boat motion metrics
            // 4. Log to SD card
            
            // For now, just log interesting events
            float accel_magnitude = sqrt(imu_data.accel_x * imu_data.accel_x + 
                                       imu_data.accel_y * imu_data.accel_y + 
                                       imu_data.accel_z * imu_data.accel_z);
            
            // Detect potential stroke (high acceleration event)
            if (accel_magnitude > 2.0f) { // 2g threshold
                ESP_LOGI("LOG_TASK", "Stroke detected: %.2fg at %lu ms", 
                        accel_magnitude, imu_data.timestamp_ms);
            }
        }
        
        // Process GPS data (low frequency)
        if (xQueueReceive(gps_data_queue, &gps_data, 0) == pdTRUE) {
            if (gps_data.valid_fix) {
                // Calculate distance, track boat path, etc.
                ESP_LOGI("LOG_TASK", "GPS logged: %.6f,%.6f @ %.1f kts (%lu ms)",
                        gps_data.latitude, gps_data.longitude, 
                        gps_data.speed_knots, gps_data.timestamp_ms);
            }
        }
        
        // Run at moderate frequency to process queued data
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz processing
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");
    
    // Initialize hardware protocols
    ESP_LOGI(TAG, "Initializing communication protocols...");
    if (protocols_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocols - system halt");
        return;
    }
    
    // Initialize sensors
    ESP_LOGI(TAG, "Initializing sensors...");
    if (accel_init() != ESP_OK) ESP_LOGW(TAG, "Accelerometer init failed");
    if (gyro_init() != ESP_OK) ESP_LOGW(TAG, "Gyroscope init failed"); 
    if (mag_init() != ESP_OK) ESP_LOGW(TAG, "Magnetometer init failed");
    if (gps_init() != ESP_OK) ESP_LOGW(TAG, "GPS init failed");
    
    // Create inter-task communication queues
    imu_data_queue = xQueueCreate(50, sizeof(imu_data_t));   // Buffer 0.5 seconds at 100Hz
    gps_data_queue = xQueueCreate(10, sizeof(gps_data_t)); // Buffer 10 GPS fixes
    
    if (imu_data_queue == NULL || gps_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create communication queues");
        return;
    }
    ESP_LOGI(TAG, "Communication queues created");
    
    // Create sensor tasks with appropriate priorities
    xTaskCreatePinnedToCore(
        imu_task,           // Task function
        "IMU_TASK",         // Task name
        4096,               // Stack size
        NULL,               // Parameters
        6,                  // Priority (high - time critical)
        &imu_task_handle,   // Task handle
        1                   // Pin to core 1 (app core)
    );
    
    xTaskCreatePinnedToCore(
        gps_task,
        "GPS_TASK", 
        4096,
        NULL,
        4,                  // Priority (medium)
        &gps_task_handle,
        1                   // Pin to core 1
    );
    
    xTaskCreatePinnedToCore(
        logging_task,
        "LOG_TASK",
        8192,               // Larger stack for data processing
        NULL, 
        2,                  // Priority (low - non-time critical)
        &logging_task_handle,
        0                   // Pin to core 0 (protocol core)
    );
    
    ESP_LOGI(TAG, "All tasks created and running");
    ESP_LOGI(TAG, "=== System Ready for Rowing ===");
    
    // Main task becomes system monitor
    while (1) {
        // Monitor system health
        ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());
        
        // Check task health (optional)
        if (eTaskGetState(imu_task_handle) == eSuspended) {
            ESP_LOGE(TAG, "IMU task has stopped!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Report every 10 seconds
    }
}

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Rowing computer starting...");
//     // Allow system to stabilize before sensor initialization
//     vTaskDelay(pdMS_TO_TICKS(1000));
    
//     // Initialize communication protocols first
//     protocols_init();

//     // Initialize all sensors - order matters for shared buses
//     sensors_init();    

//     // Initialize data
//     float ax, ay, az;
//     float gx, gy, gz;
//     float mx, my, mz;
//     gps_data_t gps_data;

//     while (1) {
//         // Read IMU at 10Hz for now
//         accel_read(&ax, &ay, &az);
//         gyro_read(&gx, &gy, &gz);
//         mag_read(&mx, &my, &mz);
//         ESP_LOGI(TAG, "A[%.2f,%.2f,%.2f] G[%.3f,%.3f,%.3f] M[%.3f,%.3f,%.3f]",
//         ax, ay, az, gx, gy, gz, mx, my, mz);

//         // Read GPS every 1 second
//         static int gps_counter = 0;
//         if (++gps_counter >= 10) {
//         gps_counter = 0;
//             if (gps_read(&gps_data) == ESP_OK) {
//             ESP_LOGI(TAG, "GPS: X=%.2f Y=%.2f Z=%.2f", 
//                     gps_data.latitude, gps_data.longitude, gps_data.speed_knots);
//             }
//         }   

//         vTaskDelay(pdMS_TO_TICKS(100)); 
//     }
// }