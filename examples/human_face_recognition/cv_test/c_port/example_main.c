#include "intrusion_detector.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 这是一个最小示例：
 * - 假设你已经拿到了灰度图像 roi_gray
 * - 每一帧直接调用 intrusion_detector_process()
 * - 在 ESP-IDF 中可把 main() 换成 app_main()
 */

int main(void) {
    const uint16_t width = 160;
    const uint16_t height = 120;
    const size_t pixels = (size_t)width * height;

    intrusion_detector_config_t cfg;
    intrusion_detector_default_config(&cfg, width, height);
    cfg.bg_method = ID_BG_KNN_STANDARD;
    cfg.consecutive_trigger_frames = 3;
    cfg.alarm_hold_frames = 10;

    intrusion_detector_t detector;
    if (!intrusion_detector_init(&detector, &cfg)) {
        fprintf(stderr, "intrusion_detector_init failed\n");
        return 1;
    }

    uint8_t *bootstrap_frame = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    uint8_t *frame = (uint8_t *)calloc(pixels, sizeof(uint8_t));
    const uint8_t *bootstrap_frames[8];
    if (bootstrap_frame == NULL || frame == NULL) {
        fprintf(stderr, "frame allocation failed\n");
        intrusion_detector_deinit(&detector);
        free(bootstrap_frame);
        free(frame);
        return 1;
    }

    for (int i = 0; i < 8; ++i) {
        memset(bootstrap_frame, 0, pixels);
        bootstrap_frames[i] = bootstrap_frame;
    }

    if (!intrusion_detector_bootstrap(&detector, bootstrap_frames, 8)) {
        fprintf(stderr, "bootstrap failed\n");
        intrusion_detector_deinit(&detector);
        free(bootstrap_frame);
        free(frame);
        return 1;
    }

    /* 模拟一帧：在左边框制造一小块亮区域，触发边界入侵。 */
    memset(frame, 0, pixels);
    for (uint16_t y = 20; y < 40; ++y) {
        for (uint16_t x = 0; x < 6; ++x) {
            frame[(size_t)y * width + x] = 255;
        }
    }

    intrusion_detector_result_t result;
    if (!intrusion_detector_process(&detector, frame, &result)) {
        fprintf(stderr, "process failed\n");
        intrusion_detector_deinit(&detector);
        free(bootstrap_frame);
        free(frame);
        return 1;
    }

    printf("intrusion=%d current_ratio=%.4f smoothed_ratio=%.4f trigger=%u hold=%u\n",
           result.intrusion,
           result.current_ratio,
           result.smoothed_ratio,
           result.trigger_count,
           result.hold_count);

    intrusion_detector_deinit(&detector);
    free(bootstrap_frame);
    free(frame);
    return 0;
}
