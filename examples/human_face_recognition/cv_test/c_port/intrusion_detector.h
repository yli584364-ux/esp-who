#ifndef INTRUSION_DETECTOR_H
#define INTRUSION_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ID_BG_KNN_STANDARD = 0,
    ID_BG_MOG2_STABLE = 1,
} id_bg_method_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    float intrusion_threshold;
    float temporal_threshold;
    float border_width_ratio;
    uint16_t border_width_min_px;
    uint16_t min_blob_area;
    uint16_t temporal_window;
    uint16_t consecutive_trigger_frames;
    uint16_t alarm_hold_frames;
    id_bg_method_t bg_method;
    uint8_t knn_match_count;
    uint8_t knn_sample_count;
    uint8_t knn_distance_threshold;
    uint8_t knn_update_period;
    uint8_t mog2_alpha_num;
    uint8_t mog2_alpha_den;
    uint8_t mog2_std_init;
    uint8_t mog2_std_min;
    uint8_t mog2_threshold_scale;
    bool custom_border_enabled;
    uint16_t custom_x1;
    uint16_t custom_y1;
    uint16_t custom_x2;
    uint16_t custom_y2;
} intrusion_detector_config_t;

typedef struct {
    bool intrusion;
    float current_ratio;
    float smoothed_ratio;
    uint16_t trigger_count;
    uint16_t hold_count;
    uint32_t fg_pixels;
    uint32_t border_pixels;
} intrusion_detector_result_t;

typedef struct {
    intrusion_detector_config_t cfg;
    uint8_t *border_mask;
    uint8_t *fg_mask;
    uint8_t *work_mask;
    uint16_t *labels;
    float *ratio_history;
    uint16_t ratio_index;
    uint16_t ratio_count;
    uint16_t trigger_count;
    uint16_t hold_count;
    uint32_t frame_index;

    uint8_t *bg_mean;
    uint8_t *bg_dev;
    uint8_t *knn_samples;
    uint16_t *neighbor_queue_x;
    uint16_t *neighbor_queue_y;
    uint32_t neighbor_queue_capacity;
} intrusion_detector_t;

void intrusion_detector_default_config(intrusion_detector_config_t *cfg, uint16_t width, uint16_t height);
bool intrusion_detector_init(intrusion_detector_t *detector, const intrusion_detector_config_t *cfg);
void intrusion_detector_deinit(intrusion_detector_t *detector);
void intrusion_detector_reset(intrusion_detector_t *detector);
void intrusion_detector_set_custom_border_rect(
    intrusion_detector_t *detector,
    bool enabled,
    uint16_t x1,
    uint16_t y1,
    uint16_t x2,
    uint16_t y2
);
bool intrusion_detector_bootstrap(intrusion_detector_t *detector, const uint8_t *const *frames, size_t frame_count);
bool intrusion_detector_process(
    intrusion_detector_t *detector,
    const uint8_t *gray_frame,
    intrusion_detector_result_t *result
);

#ifdef __cplusplus
}
#endif

#endif
