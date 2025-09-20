#ifndef BOOT_PROGRESS_H
#define BOOT_PROGRESS_H

#include "esp_err.h"
#include <stdbool.h>

// Boot progress categories
typedef enum {
    BOOT_PROTOCOLS,
    BOOT_SENSORS,
    BOOT_QUEUES,
    BOOT_TASKS,
    BOOT_CATEGORY_MAX
} boot_category_t;

// Initialize boot progress tracking
void boot_progress_init(void);

// Track successful completion of an item in a category
void boot_progress_success(boot_category_t category, const char* item_name);

// Track failure of an item in a category
void boot_progress_failure(boot_category_t category, const char* item_name, const char* error_msg);

// Report summary for a category (shows condensed format)
void boot_progress_report_category(boot_category_t category, const char* category_name);

// Report final boot summary
void boot_progress_report_final(void);

// Enable/disable verbose logging (for debugging)
void boot_progress_set_verbose(bool verbose);

#endif // BOOT_PROGRESS_H