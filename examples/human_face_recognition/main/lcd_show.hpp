#pragma once

#include <cstdint>

#include "boundary_detect.hpp"
#include "hand_detect_app.hpp"
#include "who_frame_lcd_disp.hpp"

#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include "lvgl.h"
#endif

namespace who {
namespace app {

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
