#include "boot_progress.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BOOT";

typedef struct {
    int success_count;
    int failure_count;
    char last_error[128];
} boot_category_progress_t;

static boot_category_progress_t progress[BOOT_CATEGORY_MAX];
static bool verbose_mode = false;

void boot_progress_init(void) {
    memset(progress, 0, sizeof(progress));
    verbose_mode = false;
}

void boot_progress_success(boot_category_t category, const char* item_name) {
    if (category >= BOOT_CATEGORY_MAX) return;

    progress[category].success_count++;

    if (verbose_mode) {
        ESP_LOGD(TAG, "%s: Success", item_name);
    }
}

void boot_progress_failure(boot_category_t category, const char* item_name, const char* error_msg) {
    if (category >= BOOT_CATEGORY_MAX) return;

    progress[category].failure_count++;
    snprintf(progress[category].last_error, sizeof(progress[category].last_error),
             "%s: %s", item_name, error_msg);

    // Always show failures
    ESP_LOGW(TAG, "%s", progress[category].last_error);
}

void boot_progress_report_category(boot_category_t category, const char* category_name) {
    if (category >= BOOT_CATEGORY_MAX) return;

    boot_category_progress_t *cat = &progress[category];
    int total = cat->success_count + cat->failure_count;

    if (total == 0) return;

    if (cat->failure_count == 0) {
        ESP_LOGI(TAG, "%s: %s [%d/%d]", category_name, "Complete", cat->success_count, total);
    } else {
        ESP_LOGW(TAG, "%s: %s [%d/%d] - %d failed", category_name, "Partial",
                 cat->success_count, total, cat->failure_count);
    }
}

void boot_progress_report_final(void) {
    int total_success = 0;
    int total_failures = 0;

    for (int i = 0; i < BOOT_CATEGORY_MAX; i++) {
        total_success += progress[i].success_count;
        total_failures += progress[i].failure_count;
    }

    if (total_failures == 0) {
        ESP_LOGI(TAG, "=== System Ready for Rowing ===");
    } else {
        ESP_LOGW(TAG, "=== System Ready (with %d warnings) ===", total_failures);
    }
}

void boot_progress_set_verbose(bool verbose) {
    verbose_mode = verbose;
}