#pragma once

#include <atomic>
#include <vector>

#include "dl_detect_define.hpp"
#include "intrusion_detector.h"
#include "who_frame_cap.hpp"
#include "who_frame_lcd_disp.hpp"
#include "who_task.hpp"

#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include "lvgl.h"
#endif

class HandDetect;

namespace who {
namespace app {

struct hand_detection_result_t {
    bool ready;
    uint16_t width;
    uint16_t height;
    std::vector<dl::detect::result_t> boxes;
};

class IntrusionMonitorTask : public task::WhoTask {
public:
    static inline constexpr EventBits_t NEW_FRAME = frame_cap::WhoFrameCapNode::NEW_FRAME;
    static inline constexpr uint16_t DETECT_WIDTH = 128;
    static inline constexpr uint16_t DETECT_HEIGHT = 75;

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

class HandDetectTask : public task::WhoTask {
public:
    static inline constexpr EventBits_t NEW_FRAME = frame_cap::WhoFrameCapNode::NEW_FRAME;

    HandDetectTask(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node);
    ~HandDetectTask();

    bool get_result(hand_detection_result_t *result);
    void clear_result();

private:
    void task() override;
    void cleanup() override;

    frame_cap::WhoFrameCapNode *m_frame_cap_node;
    HandDetect *m_detector;
    SemaphoreHandle_t m_result_mutex;
    hand_detection_result_t m_result;
};

class BoundaryMonitorAppLCD {
public:
    explicit BoundaryMonitorAppLCD(frame_cap::WhoFrameCap *frame_cap);
    ~BoundaryMonitorAppLCD();

    bool run();

private:
    static void reset_button_event_cb(lv_event_t *e);
    static void combined_button_event_cb(lv_event_t *e);
    static void start_button_event_cb(lv_event_t *e);
    static void feature2_button_event_cb(lv_event_t *e);
    static void feature3_button_event_cb(lv_event_t *e);
    static void preview_click_event_cb(lv_event_t *e);
    void lcd_disp_cb(who::cam::cam_fb_t *fb);
    void draw_overlay(who::cam::cam_fb_t *fb, bool intrusion, uint16_t border_width);
    void draw_hand_boxes(who::cam::cam_fb_t *fb, const hand_detection_result_t &result);
    bool hand_touches_boundary(
        who::cam::cam_fb_t *fb,
        const hand_detection_result_t &result,
        uint16_t border_width
    ) const;
    uint16_t scale_border_width(uint16_t display_w, uint16_t display_h, uint16_t detect_border_width) const;
    void request_range_reset();
    void ensure_preview_layout(who::cam::cam_fb_t *fb);
    void set_combined_detection_enabled(bool enabled);
    void set_detection_enabled(bool enabled);
    void set_hand_detection_enabled(bool enabled);
    void update_combined_button_text();
    void update_start_button_text();
    void update_feature3_button_text();
    void handle_feature2_action();
    void handle_preview_click();
    void apply_custom_boundary();

    frame_cap::WhoFrameCap *m_frame_cap;
    lcd_disp::WhoFrameLCDDisp *m_lcd_disp;
    IntrusionMonitorTask *m_monitor_task;
    HandDetectTask *m_hand_task;

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    lv_obj_t *m_label;
    lv_obj_t *m_control_panel;
    lv_obj_t *m_combined_btn;
    lv_obj_t *m_start_btn;
    lv_obj_t *m_reset_btn;
    lv_obj_t *m_placeholder_btn_1;
    lv_obj_t *m_placeholder_btn_2;
    uint8_t *m_preview_buf;
    uint16_t m_preview_w;
    uint16_t m_preview_h;
    bool m_preview_ready;
    bool m_combined_detection_enabled;
    bool m_detection_enabled;
    bool m_hand_detection_enabled;
    bool m_boundary_edit_mode;
    bool m_boundary_has_first_point;
    bool m_boundary_enabled;
    lv_point_t m_boundary_p1;
    lv_point_t m_boundary_p2;
#endif
};

} // namespace app
} // namespace who
