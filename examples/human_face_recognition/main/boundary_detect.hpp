#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "dl_image.hpp"
#include "intrusion_detector.h"
#include "who_frame_cap.hpp"
#include "who_task.hpp"

namespace who {
namespace app {

class IntrusionMonitorTask : public task::WhoTask {
public:
    static inline constexpr EventBits_t NEW_FRAME = frame_cap::WhoFrameCapNode::NEW_FRAME;
    static inline constexpr uint16_t DETECT_WIDTH = 128;
    static inline constexpr uint16_t DETECT_HEIGHT = 75;
    static inline constexpr size_t BOOTSTRAP_FRAMES = 8;

    IntrusionMonitorTask(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node);
    ~IntrusionMonitorTask();

    bool get_result(intrusion_detector_result_t *result, bool *ready, size_t *bootstrap_count, uint16_t *border_width);
    void request_reset();
    void request_custom_boundary(bool enabled, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void set_processing_enabled(bool enabled);

private:
    void task() override;
    void cleanup() override;

    bool init_detector_if_needed(who::cam::cam_fb_t *fb);
    bool convert_to_gray(who::cam::cam_fb_t *fb);
    static uint16_t calc_border_width(const intrusion_detector_config_t &cfg);

    frame_cap::WhoFrameCapNode *m_frame_cap_node;
    dl::image::ImageTransformer m_image_transformer;
    intrusion_detector_t m_detector;
    intrusion_detector_config_t m_cfg;
    SemaphoreHandle_t m_result_mutex;
    intrusion_detector_result_t m_result;
    uint8_t *m_gray_frame;
    uint8_t **m_bootstrap_frames;
    size_t m_pixels;
    size_t m_bootstrap_count;
    bool m_detector_ready;
    uint16_t m_border_width;
    uint32_t m_caps;
    std::atomic<bool> m_reset_requested;
    std::atomic<bool> m_processing_enabled;
    std::atomic<bool> m_custom_boundary_update_pending;
    std::atomic<bool> m_custom_boundary_enable_pending;
    std::atomic<uint32_t> m_custom_boundary_rect_pending;
    bool m_custom_boundary_enabled;
    uint16_t m_custom_x1;
    uint16_t m_custom_y1;
    uint16_t m_custom_x2;
    uint16_t m_custom_y2;
    uint16_t m_consecutive_failures;
};

} // namespace app
} // namespace who
