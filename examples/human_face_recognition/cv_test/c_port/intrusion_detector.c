#include "intrusion_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ID_QUEUE_EMPTY 0xFFu

static size_t id_pixel_count(const intrusion_detector_t *detector) {
    return (size_t)detector->cfg.width * (size_t)detector->cfg.height;
}

static uint16_t id_border_width(const intrusion_detector_t *detector) {
    uint16_t min_dim = detector->cfg.width < detector->cfg.height ? detector->cfg.width : detector->cfg.height;
    uint16_t bw = (uint16_t)(min_dim * detector->cfg.border_width_ratio);
    if (bw < detector->cfg.border_width_min_px) {
        bw = detector->cfg.border_width_min_px;
    }
    if (bw > min_dim / 2u) {
        bw = min_dim / 2u;
    }
    if (bw == 0u) {
        bw = 1u;
    }
    return bw;
}

static void id_build_border_mask(intrusion_detector_t *detector) {
    const uint16_t width = detector->cfg.width;
    const uint16_t height = detector->cfg.height;
    const uint16_t bw = id_border_width(detector);

    memset(detector->border_mask, 0, id_pixel_count(detector));
    if (detector->cfg.custom_border_enabled) {
        uint16_t x1 = detector->cfg.custom_x1;
        uint16_t y1 = detector->cfg.custom_y1;
        uint16_t x2 = detector->cfg.custom_x2;
        uint16_t y2 = detector->cfg.custom_y2;

        if (x1 > x2) {
            uint16_t t = x1;
            x1 = x2;
            x2 = t;
        }
        if (y1 > y2) {
            uint16_t t = y1;
            y1 = y2;
            y2 = t;
        }

        if (x2 >= width) {
            x2 = width > 0u ? (uint16_t)(width - 1u) : 0u;
        }
        if (y2 >= height) {
            y2 = height > 0u ? (uint16_t)(height - 1u) : 0u;
        }
        if (x1 >= width) {
            x1 = width > 0u ? (uint16_t)(width - 1u) : 0u;
        }
        if (y1 >= height) {
            y1 = height > 0u ? (uint16_t)(height - 1u) : 0u;
        }

        if (x2 > x1 && y2 > y1) {
            for (uint16_t y = y1; y <= y2; ++y) {
                for (uint16_t x = x1; x <= x2; ++x) {
                    bool on_border = ((uint16_t)(x - x1) < bw) || ((uint16_t)(x2 - x) < bw) ||
                                     ((uint16_t)(y - y1) < bw) || ((uint16_t)(y2 - y) < bw);
                    detector->border_mask[(size_t)y * width + x] = on_border ? 1u : 0u;
                }
            }
            return;
        }
    }

    for (uint16_t y = 0; y < height; ++y) {
        for (uint16_t x = 0; x < width; ++x) {
            bool on_border = (x < bw) || (x >= width - bw) || (y < bw) || (y >= height - bw);
            detector->border_mask[(size_t)y * width + x] = on_border ? 1u : 0u;
        }
    }
}

static void id_reset_history(intrusion_detector_t *detector) {
    memset(detector->ratio_history, 0, sizeof(float) * detector->cfg.temporal_window);
    detector->ratio_index = 0;
    detector->ratio_count = 0;
    detector->trigger_count = 0;
    detector->hold_count = 0;
    detector->frame_index = 0;
}

static void id_init_manual_bg(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    for (size_t i = 0; i < pixels; ++i) {
        detector->bg_mean[i] = frame[i];
        detector->bg_dev[i] = detector->cfg.mog2_std_init;
    }
}

static void id_init_knn_bg(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    const uint8_t sample_count = detector->cfg.knn_sample_count;
    for (uint8_t s = 0; s < sample_count; ++s) {
        memcpy(detector->knn_samples + (size_t)s * pixels, frame, pixels);
    }
}

static void id_bootstrap_single(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    if (detector->cfg.bg_method == ID_BG_KNN_STANDARD) {
        if (detector->frame_index == 0u) {
            id_init_knn_bg(detector, frame);
        } else {
            uint8_t slot = (uint8_t)(detector->frame_index % detector->cfg.knn_sample_count);
            memcpy(detector->knn_samples + (size_t)slot * pixels, frame, pixels);
        }
    } else {
        if (detector->frame_index == 0u) {
            id_init_manual_bg(detector, frame);
        } else {
            for (size_t i = 0; i < pixels; ++i) {
                uint8_t mean = detector->bg_mean[i];
                uint8_t value = frame[i];
                int diff = (int)value - (int)mean;
                mean = (uint8_t)(mean + (diff * detector->cfg.mog2_alpha_num) / detector->cfg.mog2_alpha_den);
                detector->bg_mean[i] = mean;
            }
        }
    }
    detector->frame_index++;
}

static void id_update_manual_bg(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    const uint8_t alpha_num = detector->cfg.mog2_alpha_num;
    const uint8_t alpha_den = detector->cfg.mog2_alpha_den;

    for (size_t i = 0; i < pixels; ++i) {
        if (detector->fg_mask[i] != 0u || detector->border_mask[i] == 0u) {
            continue;
        }

        int value = frame[i];
        int mean = detector->bg_mean[i];
        int dev = detector->bg_dev[i];
        int diff = value - mean;
        if (diff < 0) {
            diff = -diff;
        }

        mean += ((value - mean) * alpha_num) / alpha_den;
        dev += ((diff - dev) * alpha_num) / alpha_den;
        if (dev < detector->cfg.mog2_std_min) {
            dev = detector->cfg.mog2_std_min;
        }
        detector->bg_mean[i] = (uint8_t)mean;
        detector->bg_dev[i] = (uint8_t)dev;
    }
}

static void id_update_knn_bg(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    const uint8_t sample_count = detector->cfg.knn_sample_count;

    for (size_t i = 0; i < pixels; ++i) {
        if (detector->fg_mask[i] != 0u || detector->border_mask[i] == 0u) {
            continue;
        }
        if ((detector->frame_index + i) % detector->cfg.knn_update_period != 0u) {
            continue;
        }
        uint8_t slot = (uint8_t)((detector->frame_index + i) % sample_count);
        detector->knn_samples[(size_t)slot * pixels + i] = frame[i];
    }
}

static void id_extract_fg_knn(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);
    const uint8_t sample_count = detector->cfg.knn_sample_count;

    memset(detector->fg_mask, 0, pixels);
    for (size_t i = 0; i < pixels; ++i) {
        if (detector->border_mask[i] == 0u) {
            continue;
        }
        uint8_t matches = 0u;
        for (uint8_t s = 0; s < sample_count; ++s) {
            uint8_t sample = detector->knn_samples[(size_t)s * pixels + i];
            int diff = (int)frame[i] - (int)sample;
            if (diff < 0) {
                diff = -diff;
            }
            if ((uint8_t)diff <= detector->cfg.knn_distance_threshold) {
                matches++;
                if (matches >= detector->cfg.knn_match_count) {
                    break;
                }
            }
        }
        detector->fg_mask[i] = (matches >= detector->cfg.knn_match_count) ? 0u : 1u;
    }
}

static void id_extract_fg_mog2(intrusion_detector_t *detector, const uint8_t *frame) {
    const size_t pixels = id_pixel_count(detector);

    memset(detector->fg_mask, 0, pixels);
    for (size_t i = 0; i < pixels; ++i) {
        if (detector->border_mask[i] == 0u) {
            continue;
        }
        int diff = (int)frame[i] - (int)detector->bg_mean[i];
        if (diff < 0) {
            diff = -diff;
        }
        int threshold = detector->bg_dev[i] * detector->cfg.mog2_threshold_scale;
        if (threshold < detector->cfg.mog2_std_min) {
            threshold = detector->cfg.mog2_std_min;
        }
        detector->fg_mask[i] = (diff > threshold) ? 1u : 0u;
    }
}

static void id_morph_open_close(intrusion_detector_t *detector) {
    const uint16_t width = detector->cfg.width;
    const uint16_t height = detector->cfg.height;

    memset(detector->work_mask, 0, id_pixel_count(detector));
    for (uint16_t y = 1; y + 1 < height; ++y) {
        for (uint16_t x = 1; x + 1 < width; ++x) {
            size_t idx = (size_t)y * width + x;
            if (detector->border_mask[idx] == 0u) {
                continue;
            }
            uint8_t keep = 1u;
            for (int dy = -1; dy <= 1 && keep; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    size_t nidx = (size_t)(y + dy) * width + (x + dx);
                    if (detector->fg_mask[nidx] == 0u) {
                        keep = 0u;
                        break;
                    }
                }
            }
            detector->work_mask[idx] = keep;
        }
    }

    memset(detector->fg_mask, 0, id_pixel_count(detector));
    for (uint16_t y = 1; y + 1 < height; ++y) {
        for (uint16_t x = 1; x + 1 < width; ++x) {
            size_t idx = (size_t)y * width + x;
            if (detector->border_mask[idx] == 0u) {
                continue;
            }
            uint8_t fill = 0u;
            for (int dy = -1; dy <= 1 && !fill; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    size_t nidx = (size_t)(y + dy) * width + (x + dx);
                    if (detector->work_mask[nidx] != 0u) {
                        fill = 1u;
                        break;
                    }
                }
            }
            detector->fg_mask[idx] = fill;
        }
    }
}

static uint32_t id_filter_connected_components(intrusion_detector_t *detector) {
    const uint16_t width = detector->cfg.width;
    const uint16_t height = detector->cfg.height;
    const size_t pixels = id_pixel_count(detector);
    const uint32_t queue_capacity = detector->neighbor_queue_capacity;
    uint32_t kept_pixels = 0u;
    uint16_t current_label = 1u;

    memset(detector->labels, 0, pixels * sizeof(uint16_t));
    memset(detector->work_mask, 0, pixels);

    for (uint16_t y = 0; y < height; ++y) {
        for (uint16_t x = 0; x < width; ++x) {
            size_t idx = (size_t)y * width + x;
            if (detector->fg_mask[idx] == 0u || detector->labels[idx] != 0u) {
                continue;
            }

            uint32_t head = 0u;
            uint32_t tail = 0u;
            uint32_t area = 0u;
            detector->neighbor_queue_x[tail] = x;
            detector->neighbor_queue_y[tail] = y;
            tail++;
            detector->labels[idx] = current_label;

            while (head < tail) {
                uint16_t cx = detector->neighbor_queue_x[head];
                uint16_t cy = detector->neighbor_queue_y[head];
                head++;
                area++;

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        int nx = (int)cx + dx;
                        int ny = (int)cy + dy;
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }
                        size_t nidx = (size_t)ny * width + (size_t)nx;
                        if (detector->fg_mask[nidx] == 0u || detector->labels[nidx] != 0u) {
                            continue;
                        }
                        if (tail >= queue_capacity) {
                            continue;
                        }
                        detector->labels[nidx] = current_label;
                        detector->neighbor_queue_x[tail] = (uint16_t)nx;
                        detector->neighbor_queue_y[tail] = (uint16_t)ny;
                        tail++;
                    }
                }
            }

            if (area >= detector->cfg.min_blob_area) {
                for (uint32_t i = 0; i < tail; ++i) {
                    size_t keep_idx = (size_t)detector->neighbor_queue_y[i] * width + detector->neighbor_queue_x[i];
                    detector->work_mask[keep_idx] = 1u;
                    kept_pixels++;
                }
            }
            current_label++;
        }
    }

    memcpy(detector->fg_mask, detector->work_mask, pixels);
    return kept_pixels;
}

void intrusion_detector_default_config(intrusion_detector_config_t *cfg, uint16_t width, uint16_t height) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->width = width;
    cfg->height = height;
    cfg->intrusion_threshold = 0.03f;
    cfg->temporal_threshold = 0.02f;
    cfg->border_width_ratio = 0.08f;
    cfg->border_width_min_px = 3u;
    cfg->min_blob_area = 15u;
    cfg->temporal_window = 2u;
    cfg->consecutive_trigger_frames = 3u;
    cfg->alarm_hold_frames = 10u;
    cfg->bg_method = ID_BG_KNN_STANDARD;
    cfg->knn_match_count = 2u;
    cfg->knn_sample_count = 8u;
    cfg->knn_distance_threshold = 20u;
    cfg->knn_update_period = 16u;
    cfg->mog2_alpha_num = 1u;
    cfg->mog2_alpha_den = 32u;
    cfg->mog2_std_init = 12u;
    cfg->mog2_std_min = 4u;
    cfg->mog2_threshold_scale = 3u;
    cfg->custom_border_enabled = false;
    cfg->custom_x1 = 0u;
    cfg->custom_y1 = 0u;
    cfg->custom_x2 = 0u;
    cfg->custom_y2 = 0u;
}

bool intrusion_detector_init(intrusion_detector_t *detector, const intrusion_detector_config_t *cfg) {
    memset(detector, 0, sizeof(*detector));
    detector->cfg = *cfg;

    const size_t pixels = (size_t)cfg->width * cfg->height;
    detector->border_mask = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    detector->fg_mask = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    detector->work_mask = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    detector->labels = (uint16_t *)calloc(pixels, sizeof(uint16_t));
    detector->ratio_history = (float *)calloc(cfg->temporal_window, sizeof(float));
    detector->neighbor_queue_capacity = pixels;
    detector->neighbor_queue_x = (uint16_t *)malloc(pixels * sizeof(uint16_t));
    detector->neighbor_queue_y = (uint16_t *)malloc(pixels * sizeof(uint16_t));

    if (cfg->bg_method == ID_BG_MOG2_STABLE) {
        detector->bg_mean = (uint8_t *)calloc(pixels, sizeof(uint8_t));
        detector->bg_dev = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    } else {
        detector->knn_samples = (uint8_t *)calloc((size_t)cfg->knn_sample_count * pixels, sizeof(uint8_t));
    }

    if (detector->border_mask == NULL || detector->fg_mask == NULL || detector->work_mask == NULL ||
        detector->labels == NULL || detector->ratio_history == NULL || detector->neighbor_queue_x == NULL ||
        detector->neighbor_queue_y == NULL ||
        (cfg->bg_method == ID_BG_MOG2_STABLE && (detector->bg_mean == NULL || detector->bg_dev == NULL)) ||
        (cfg->bg_method == ID_BG_KNN_STANDARD && detector->knn_samples == NULL)) {
        intrusion_detector_deinit(detector);
        return false;
    }

    id_build_border_mask(detector);
    id_reset_history(detector);
    return true;
}

void intrusion_detector_deinit(intrusion_detector_t *detector) {
    free(detector->border_mask);
    free(detector->fg_mask);
    free(detector->work_mask);
    free(detector->labels);
    free(detector->ratio_history);
    free(detector->bg_mean);
    free(detector->bg_dev);
    free(detector->knn_samples);
    free(detector->neighbor_queue_x);
    free(detector->neighbor_queue_y);
    memset(detector, 0, sizeof(*detector));
}

void intrusion_detector_reset(intrusion_detector_t *detector) {
    id_build_border_mask(detector);
    if (detector->cfg.bg_method == ID_BG_MOG2_STABLE) {
        memset(detector->bg_mean, 0, id_pixel_count(detector));
        memset(detector->bg_dev, 0, id_pixel_count(detector));
    } else {
        memset(detector->knn_samples, 0, (size_t)detector->cfg.knn_sample_count * id_pixel_count(detector));
    }
    id_reset_history(detector);
}

void intrusion_detector_set_custom_border_rect(
    intrusion_detector_t *detector,
    bool enabled,
    uint16_t x1,
    uint16_t y1,
    uint16_t x2,
    uint16_t y2
) {
    detector->cfg.custom_border_enabled = enabled;
    detector->cfg.custom_x1 = x1;
    detector->cfg.custom_y1 = y1;
    detector->cfg.custom_x2 = x2;
    detector->cfg.custom_y2 = y2;
    id_build_border_mask(detector);
    id_reset_history(detector);
}

bool intrusion_detector_bootstrap(intrusion_detector_t *detector, const uint8_t *const *frames, size_t frame_count) {
    if (frames == NULL || frame_count == 0u) {
        return false;
    }
    intrusion_detector_reset(detector);
    for (size_t i = 0; i < frame_count; ++i) {
        id_bootstrap_single(detector, frames[i]);
    }
    id_reset_history(detector);
    return true;
}

bool intrusion_detector_process(
    intrusion_detector_t *detector,
    const uint8_t *gray_frame,
    intrusion_detector_result_t *result
) {
    if (gray_frame == NULL || result == NULL) {
        return false;
    }

    if (detector->frame_index == 0u) {
        id_bootstrap_single(detector, gray_frame);
    }

    if (detector->cfg.bg_method == ID_BG_KNN_STANDARD) {
        id_extract_fg_knn(detector, gray_frame);
    } else {
        id_extract_fg_mog2(detector, gray_frame);
    }

    id_morph_open_close(detector);
    uint32_t fg_pixels = id_filter_connected_components(detector);
    uint32_t border_pixels = 0u;
    const size_t pixels = id_pixel_count(detector);
    for (size_t i = 0; i < pixels; ++i) {
        border_pixels += detector->border_mask[i] != 0u;
    }

    float current_ratio = border_pixels > 0u ? (float)fg_pixels / (float)border_pixels : 0.0f;
    detector->ratio_history[detector->ratio_index] = current_ratio;
    detector->ratio_index = (uint16_t)((detector->ratio_index + 1u) % detector->cfg.temporal_window);
    if (detector->ratio_count < detector->cfg.temporal_window) {
        detector->ratio_count++;
    }

    float sum = 0.0f;
    for (uint16_t i = 0; i < detector->ratio_count; ++i) {
        sum += detector->ratio_history[i];
    }
    float smoothed_ratio = detector->ratio_count > 0u ? sum / detector->ratio_count : 0.0f;

    bool triggered = (smoothed_ratio > detector->cfg.temporal_threshold) ||
                     (current_ratio > detector->cfg.intrusion_threshold);
    if (triggered) {
        detector->trigger_count++;
    } else {
        detector->trigger_count = 0u;
    }

    if (detector->trigger_count >= detector->cfg.consecutive_trigger_frames) {
        detector->hold_count = detector->cfg.alarm_hold_frames;
    } else if (detector->hold_count > 0u) {
        detector->hold_count--;
    }

    if (detector->cfg.bg_method == ID_BG_KNN_STANDARD) {
        id_update_knn_bg(detector, gray_frame);
    } else {
        id_update_manual_bg(detector, gray_frame);
    }

    detector->frame_index++;

    result->intrusion = detector->hold_count > 0u;
    result->current_ratio = current_ratio;
    result->smoothed_ratio = smoothed_ratio;
    result->trigger_count = detector->trigger_count;
    result->hold_count = detector->hold_count;
    result->fg_pixels = fg_pixels;
    result->border_pixels = border_pixels;
    return true;
}
