#include "lcd_show.hpp"

#include <functional>
#include <vector>

#include "boundary_monitor_lvgl.hpp"
#include "bsp/esp-bsp.h"
#include "dl_image.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "who_yield2idle.hpp"

namespace {

static constexpr const char *TAG = "LCDShow";
static constexpr uint16_t kPreviewScalePercent = 78;
static constexpr uint16_t kMinCustomBoundarySpan = 8;
static constexpr int32_t kHandBoundaryTouchPixels = 5;
static const std::vector<uint8_t> kNormalColor = {0, 255, 0};
static const std::vector<uint8_t> kAlertColor = {255, 0, 0};
static const std::vector<uint8_t> kRgb565LeRed = {0x00, 0xF8};

struct rect_i32_t {
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
};

int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int32_t overlap_len(int32_t a1, int32_t a2, int32_t b1, int32_t b2)
{
    int32_t lo = a1 > b1 ? a1 : b1;
    int32_t hi = a2 < b2 ? a2 : b2;
    return hi > lo ? hi - lo : 0;
}

bool box_touches_rect_edge(const rect_i32_t &box, const rect_i32_t &rect, int32_t threshold)
{
    bool touches_left = box.x1 <= rect.x1 + threshold && box.x2 >= rect.x1 - threshold &&
                        overlap_len(box.y1, box.y2, rect.y1, rect.y2) > threshold;
    bool touches_right = box.x1 <= rect.x2 + threshold && box.x2 >= rect.x2 - threshold &&
                         overlap_len(box.y1, box.y2, rect.y1, rect.y2) > threshold;
    bool touches_top = box.y1 <= rect.y1 + threshold && box.y2 >= rect.y1 - threshold &&
                       overlap_len(box.x1, box.x2, rect.x1, rect.x2) > threshold;
    bool touches_bottom = box.y1 <= rect.y2 + threshold && box.y2 >= rect.y2 - threshold &&
                          overlap_len(box.x1, box.x2, rect.x1, rect.x2) > threshold;
    return touches_left || touches_right || touches_top || touches_bottom;
}

void resize_rgb565_nearest(
    const uint16_t *src,
    uint16_t src_w,
    uint16_t src_h,
    uint16_t *dst,
    uint16_t dst_w,
    uint16_t dst_h
)
{
    for (uint16_t y = 0; y < dst_h; ++y) {
        uint16_t sy = static_cast<uint16_t>((static_cast<uint32_t>(y) * src_h) / dst_h);
        for (uint16_t x = 0; x < dst_w; ++x) {
            uint16_t sx = static_cast<uint16_t>((static_cast<uint32_t>(x) * src_w) / dst_w);
            dst[static_cast<size_t>(y) * dst_w + x] = src[static_cast<size_t>(sy) * src_w + sx];
        }
    }
}

} // namespace

namespace who {
namespace app {

BoundaryMonitorAppLCD::BoundaryMonitorAppLCD(frame_cap::WhoFrameCap *frame_cap) :
    m_frame_cap(frame_cap),
    m_lcd_disp(new lcd_disp::WhoFrameLCDDisp("LCDDisp", frame_cap->get_last_node(), 0)),
    m_monitor_task(new IntrusionMonitorTask("BoundaryDetect", frame_cap->get_last_node())),
    m_hand_task(new HandDetectTask("HandDetect", frame_cap->get_last_node()))
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    ,
    m_label(nullptr),
    m_control_panel(nullptr),
    m_combined_btn(nullptr),
    m_start_btn(nullptr),
    m_reset_btn(nullptr),
    m_placeholder_btn_1(nullptr),
    m_placeholder_btn_2(nullptr),
    m_preview_buf(nullptr),
    m_preview_w(0),
    m_preview_h(0),
    m_preview_ready(false),
    m_combined_detection_enabled(false),
    m_detection_enabled(false),
    m_hand_detection_enabled(false),
    m_boundary_edit_mode(false),
    m_boundary_has_first_point(false),
    m_boundary_enabled(false),
    m_boundary_p1({0, 0}),
    m_boundary_p2({0, 0})
#endif
{
    m_lcd_disp->set_lcd_disp_cb(std::bind(&BoundaryMonitorAppLCD::lcd_disp_cb, this, std::placeholders::_1));

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    bsp_display_lock(0);
    m_label = create_status_label("Waiting: press Start Detect");
    m_control_panel = create_control_panel();
    lv_obj_t *button_col = create_button_column(m_control_panel);

    m_combined_btn =
        create_action_button(button_col, "Dual Detect", BoundaryMonitorAppLCD::combined_button_event_cb, this);
    m_start_btn = create_action_button(button_col, "Start Detect", BoundaryMonitorAppLCD::start_button_event_cb, this);
    m_reset_btn = create_action_button(button_col, "Reset Range", BoundaryMonitorAppLCD::reset_button_event_cb, this);
    m_placeholder_btn_1 =
        create_action_button(button_col, "Feature 2: Boundary", BoundaryMonitorAppLCD::feature2_button_event_cb, this);
    m_placeholder_btn_2 =
        create_action_button(button_col, "Feature 3: Hand", BoundaryMonitorAppLCD::feature3_button_event_cb, this);

    update_combined_button_text();
    update_start_button_text();
    update_feature3_button_text();
    bsp_display_unlock();
#endif
}

BoundaryMonitorAppLCD::~BoundaryMonitorAppLCD()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_buf) {
        heap_caps_free(m_preview_buf);
        m_preview_buf = nullptr;
    }

    if (m_control_panel) {
        bsp_display_lock(0);
        lv_obj_del(m_control_panel);
        m_control_panel = nullptr;
        m_combined_btn = nullptr;
        m_start_btn = nullptr;
        m_reset_btn = nullptr;
        m_placeholder_btn_1 = nullptr;
        m_placeholder_btn_2 = nullptr;
        bsp_display_unlock();
    }

    if (m_label) {
        bsp_display_lock(0);
        lv_obj_del(m_label);
        m_label = nullptr;
        bsp_display_unlock();
    }
#endif
    delete m_hand_task;
    delete m_monitor_task;
    delete m_lcd_disp;
}

bool BoundaryMonitorAppLCD::run()
{
    bool ret = WhoYield2Idle::get_instance()->run();
    for (const auto &frame_cap_node : m_frame_cap->get_all_nodes()) {
        ret &= frame_cap_node->run(4096, 2, 0);
    }
    ret &= m_lcd_disp->run(2560, 2, 0);
    ret &= m_monitor_task->run(4096, 2, 1);
    ret &= m_hand_task->run(6144, 2, 1);

    if (ret) {
        m_monitor_task->set_processing_enabled(false);
        m_monitor_task->pause();
        m_hand_task->pause();
#if !BSP_CONFIG_NO_GRAPHIC_LIB
        m_combined_detection_enabled = false;
        m_detection_enabled = false;
        m_hand_detection_enabled = false;
        update_combined_button_text();
        update_start_button_text();
        update_feature3_button_text();
#endif
    }

    return ret;
}

void BoundaryMonitorAppLCD::lcd_disp_cb(who::cam::cam_fb_t *fb)
{
    ensure_preview_layout(fb);

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready) {
        if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565) {
            resize_rgb565_nearest(
                static_cast<const uint16_t *>(fb->buf),
                fb->width,
                fb->height,
                reinterpret_cast<uint16_t *>(m_preview_buf),
                m_preview_w,
                m_preview_h);
            lv_canvas_set_buffer(
                m_lcd_disp->get_canvas(), m_preview_buf, m_preview_w, m_preview_h, LV_COLOR_FORMAT_NATIVE);
        }
    }
#endif

    intrusion_detector_result_t result = {};
    bool ready = false;
    size_t bootstrap_count = 0;
    uint16_t border_width = 0;
    m_monitor_task->get_result(&result, &ready, &bootstrap_count, &border_width);

    hand_detection_result_t hand_result = {};
    m_hand_task->get_result(&hand_result);
    bool intrusion = m_detection_enabled && ready && result.intrusion;
    bool hand_boundary_alarm = m_hand_detection_enabled && hand_result.ready &&
                               hand_touches_boundary(fb, hand_result, border_width);
    bool alarm = intrusion || hand_boundary_alarm;

    draw_overlay(fb, alarm, border_width);

    if (m_hand_detection_enabled && hand_result.ready) {
        draw_hand_boxes(fb, hand_result);
    }

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_detection_enabled && !m_hand_detection_enabled) {
        lv_label_set_text(m_label, "Camera paused. Press Start Detect.");
    } else if (alarm) {
        lv_label_set_text_fmt(m_label,
                              "ALERT %s%s hands=%u",
                              intrusion ? "boundary " : "",
                              hand_boundary_alarm ? "hand " : "",
                              static_cast<unsigned>(hand_result.boxes.size()));
    } else if (m_detection_enabled && !ready) {
        lv_label_set_text_fmt(m_label,
                              "Calibrating %u/%u",
                              static_cast<unsigned>(bootstrap_count),
                              static_cast<unsigned>(IntrusionMonitorTask::BOOTSTRAP_FRAMES));
    } else {
        lv_label_set_text_fmt(
            m_label,
            "NORMAL ratio=%.3f fg=%lu/%lu hands=%u",
            result.smoothed_ratio,
            static_cast<unsigned long>(result.fg_pixels),
            static_cast<unsigned long>(result.border_pixels),
            static_cast<unsigned>(m_hand_detection_enabled && hand_result.ready ? hand_result.boxes.size() : 0));
    }
#else
    if (m_detection_enabled && ready) {
        ESP_LOGI(TAG,
                 "%s ratio=%.3f fg=%lu/%lu",
                 alarm ? "ALERT" : "NORMAL",
                 result.smoothed_ratio,
                 static_cast<unsigned long>(result.fg_pixels),
                 static_cast<unsigned long>(result.border_pixels));
    }
#endif
}

void BoundaryMonitorAppLCD::reset_button_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->request_range_reset();
    }
}

void BoundaryMonitorAppLCD::combined_button_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->set_combined_detection_enabled(!self->m_combined_detection_enabled);
    }
}

void BoundaryMonitorAppLCD::start_button_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->set_detection_enabled(!self->m_detection_enabled);
    }
}

void BoundaryMonitorAppLCD::feature2_button_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->handle_feature2_action();
    }
}

void BoundaryMonitorAppLCD::feature3_button_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->set_hand_detection_enabled(!self->m_hand_detection_enabled);
    }
}

void BoundaryMonitorAppLCD::preview_click_event_cb(lv_event_t *e)
{
    auto *self = static_cast<BoundaryMonitorAppLCD *>(lv_event_get_user_data(e));
    if (self) {
        self->handle_preview_click();
    }
}

void BoundaryMonitorAppLCD::request_range_reset()
{
    m_monitor_task->request_reset();
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_label) {
        lv_label_set_text(m_label, "Reset requested, recalibrating...");
    }
#endif
}

void BoundaryMonitorAppLCD::ensure_preview_layout(who::cam::cam_fb_t *fb)
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready || fb == nullptr) {
        return;
    }

    uint32_t preview_w = static_cast<uint32_t>(fb->width) * kPreviewScalePercent / 100u;
    uint32_t preview_h = static_cast<uint32_t>(fb->height) * kPreviewScalePercent / 100u;
    if (preview_w == 0u || preview_h == 0u) {
        return;
    }

    m_preview_w = static_cast<uint16_t>(preview_w);
    m_preview_h = static_cast<uint16_t>(preview_h);
    size_t preview_bytes = static_cast<size_t>(m_preview_w) * static_cast<size_t>(m_preview_h) * sizeof(lv_color_t);
    m_preview_buf = static_cast<uint8_t *>(heap_caps_malloc(preview_bytes, MALLOC_CAP_DEFAULT));
    if (!m_preview_buf) {
        ESP_LOGE(TAG, "preview buffer allocation failed");
        return;
    }

    configure_preview_canvas(
        m_lcd_disp->get_canvas(), m_preview_w, m_preview_h, BoundaryMonitorAppLCD::preview_click_event_cb, this);

    m_preview_ready = true;
#endif
}

void BoundaryMonitorAppLCD::handle_feature2_action()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_boundary_edit_mode) {
        m_boundary_edit_mode = false;
        m_boundary_has_first_point = false;
        if (m_label) {
            lv_label_set_text(m_label, "Boundary edit canceled");
        }
        return;
    }

    if (m_boundary_enabled) {
        m_boundary_enabled = false;
        m_monitor_task->request_custom_boundary(false, 0, 0, 0, 0);
        if (m_label) {
            lv_label_set_text(m_label, "Custom boundary disabled");
        }
        return;
    }

    m_boundary_edit_mode = true;
    m_boundary_has_first_point = false;
    if (m_label) {
        lv_label_set_text(m_label, "Boundary edit: tap top-left then bottom-right");
    }
#endif
}

void BoundaryMonitorAppLCD::handle_preview_click()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_boundary_edit_mode || !m_preview_ready) {
        return;
    }

    lv_indev_t *indev = lv_indev_active();
    if (!indev) {
        return;
    }

    lv_point_t p = {0, 0};
    lv_indev_get_point(indev, &p);
    lv_area_t area;
    lv_obj_get_coords(m_lcd_disp->get_canvas(), &area);

    int32_t lx = p.x - area.x1;
    int32_t ly = p.y - area.y1;
    if (lx < 0) {
        lx = 0;
    }
    if (ly < 0) {
        ly = 0;
    }
    if (lx >= m_preview_w) {
        lx = m_preview_w - 1;
    }
    if (ly >= m_preview_h) {
        ly = m_preview_h - 1;
    }

    if (!m_boundary_has_first_point) {
        m_boundary_p1 = {static_cast<int16_t>(lx), static_cast<int16_t>(ly)};
        m_boundary_p2 = m_boundary_p1;
        m_boundary_has_first_point = true;
        if (m_label) {
            lv_label_set_text(m_label, "First point set, tap second point");
        }
        return;
    }

    m_boundary_p2 = {static_cast<int16_t>(lx), static_cast<int16_t>(ly)};
    m_boundary_has_first_point = false;
    m_boundary_edit_mode = false;
    apply_custom_boundary();
#endif
}

void BoundaryMonitorAppLCD::apply_custom_boundary()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    int32_t x1 = m_boundary_p1.x < m_boundary_p2.x ? m_boundary_p1.x : m_boundary_p2.x;
    int32_t x2 = m_boundary_p1.x < m_boundary_p2.x ? m_boundary_p2.x : m_boundary_p1.x;
    int32_t y1 = m_boundary_p1.y < m_boundary_p2.y ? m_boundary_p1.y : m_boundary_p2.y;
    int32_t y2 = m_boundary_p1.y < m_boundary_p2.y ? m_boundary_p2.y : m_boundary_p1.y;

    if ((x2 - x1) < kMinCustomBoundarySpan || (y2 - y1) < kMinCustomBoundarySpan) {
        if (m_label) {
            lv_label_set_text(m_label, "Boundary too small, please retry");
        }
        m_boundary_enabled = false;
        return;
    }

    m_boundary_enabled = true;
    uint16_t dx1 = static_cast<uint16_t>((static_cast<uint32_t>(x1) * IntrusionMonitorTask::DETECT_WIDTH) / m_preview_w);
    uint16_t dx2 = static_cast<uint16_t>((static_cast<uint32_t>(x2) * IntrusionMonitorTask::DETECT_WIDTH) / m_preview_w);
    uint16_t dy1 = static_cast<uint16_t>((static_cast<uint32_t>(y1) * IntrusionMonitorTask::DETECT_HEIGHT) / m_preview_h);
    uint16_t dy2 = static_cast<uint16_t>((static_cast<uint32_t>(y2) * IntrusionMonitorTask::DETECT_HEIGHT) / m_preview_h);

    if (dx2 >= IntrusionMonitorTask::DETECT_WIDTH) {
        dx2 = IntrusionMonitorTask::DETECT_WIDTH - 1;
    }
    if (dy2 >= IntrusionMonitorTask::DETECT_HEIGHT) {
        dy2 = IntrusionMonitorTask::DETECT_HEIGHT - 1;
    }

    m_monitor_task->request_custom_boundary(true, dx1, dy1, dx2, dy2);
    if (m_label) {
        lv_label_set_text(m_label, "Custom boundary applied, recalibrating...");
    }
#endif
}

void BoundaryMonitorAppLCD::set_detection_enabled(bool enabled)
{
    if (enabled == m_detection_enabled) {
        return;
    }

    if (m_combined_detection_enabled) {
        set_combined_detection_enabled(false);
        if (!enabled) {
            return;
        }
    }

    if (enabled) {
        if (m_hand_detection_enabled) {
            set_hand_detection_enabled(false);
        }
        m_monitor_task->set_processing_enabled(true);
        m_monitor_task->request_reset();
        m_monitor_task->resume();
        for (const auto &frame_cap_node : m_frame_cap->get_all_nodes()) {
            frame_cap_node->resume();
        }
    } else {
        m_monitor_task->set_processing_enabled(false);
        m_monitor_task->pause();
    }

    m_detection_enabled = enabled;
    update_start_button_text();
}

void BoundaryMonitorAppLCD::set_hand_detection_enabled(bool enabled)
{
    if (enabled == m_hand_detection_enabled) {
        return;
    }

    if (m_combined_detection_enabled) {
        set_combined_detection_enabled(false);
        if (!enabled) {
            return;
        }
    }

    if (enabled) {
        if (m_detection_enabled) {
            set_detection_enabled(false);
        }
        m_hand_task->clear_result();
        m_hand_task->resume();
        for (const auto &frame_cap_node : m_frame_cap->get_all_nodes()) {
            frame_cap_node->resume();
        }
    } else {
        m_hand_task->pause();
        m_hand_task->clear_result();
    }

    m_hand_detection_enabled = enabled;
    update_feature3_button_text();
}

void BoundaryMonitorAppLCD::set_combined_detection_enabled(bool enabled)
{
    if (enabled == m_combined_detection_enabled) {
        return;
    }

    if (enabled) {
        m_monitor_task->set_processing_enabled(true);
        m_monitor_task->request_reset();
        m_monitor_task->resume();
        m_hand_task->clear_result();
        m_hand_task->resume();
        for (const auto &frame_cap_node : m_frame_cap->get_all_nodes()) {
            frame_cap_node->resume();
        }
        m_detection_enabled = true;
        m_hand_detection_enabled = true;
    } else {
        m_monitor_task->set_processing_enabled(false);
        m_monitor_task->pause();
        m_hand_task->pause();
        m_hand_task->clear_result();
        m_detection_enabled = false;
        m_hand_detection_enabled = false;
    }

    m_combined_detection_enabled = enabled;
    update_combined_button_text();
    update_start_button_text();
    update_feature3_button_text();
}

void BoundaryMonitorAppLCD::update_combined_button_text()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_combined_btn) {
        return;
    }
    set_button_text(m_combined_btn, m_combined_detection_enabled ? "Stop Dual Detect" : "Dual Detect");
#endif
}

void BoundaryMonitorAppLCD::update_start_button_text()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_start_btn) {
        return;
    }
    set_button_text(m_start_btn, m_detection_enabled ? "Pause Detect" : "Start Detect");
#endif
}

void BoundaryMonitorAppLCD::update_feature3_button_text()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_placeholder_btn_2) {
        return;
    }
    set_button_text(m_placeholder_btn_2, m_hand_detection_enabled ? "Stop Hand Detect" : "Feature 3: Hand");
#endif
}

void BoundaryMonitorAppLCD::draw_overlay(who::cam::cam_fb_t *fb, bool intrusion, uint16_t border_width)
{
    uint16_t draw_w = fb->width;
    uint16_t draw_h = fb->height;
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready) {
        draw_w = m_preview_w;
        draw_h = m_preview_h;
    }
#endif

    border_width = scale_border_width(draw_w, draw_h, border_width);
    if (border_width == 0 || border_width * 2 >= draw_w || border_width * 2 >= draw_h) {
        return;
    }

#if BSP_CONFIG_NO_GRAPHIC_LIB
    const auto &color = intrusion ? kAlertColor : kNormalColor;
    dl::image::draw_hollow_rectangle(*fb,
                                     border_width,
                                     border_width,
                                     draw_w - border_width - 1,
                                     draw_h - border_width - 1,
                                     color,
                                     intrusion ? 4 : 2);
#else
    if (m_boundary_enabled) {
        draw_custom_boundary_on_canvas(m_lcd_disp->get_canvas(), m_boundary_p1, m_boundary_p2, intrusion);
    } else {
        draw_boundary_on_canvas(m_lcd_disp->get_canvas(), draw_w, draw_h, border_width, intrusion);
    }
#endif
}

void BoundaryMonitorAppLCD::draw_hand_boxes(who::cam::cam_fb_t *fb, const hand_detection_result_t &result)
{
    if (!fb || result.width == 0 || result.height == 0 || result.boxes.empty()) {
        return;
    }

    uint16_t draw_w = fb->width;
    uint16_t draw_h = fb->height;
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready) {
        draw_w = m_preview_w;
        draw_h = m_preview_h;
    }
#endif

    for (const auto &hand : result.boxes) {
        if (hand.box.size() < 4) {
            continue;
        }

        int32_t x1 = static_cast<int32_t>((static_cast<int64_t>(hand.box[0]) * draw_w) / result.width);
        int32_t y1 = static_cast<int32_t>((static_cast<int64_t>(hand.box[1]) * draw_h) / result.height);
        int32_t x2 = static_cast<int32_t>((static_cast<int64_t>(hand.box[2]) * draw_w) / result.width);
        int32_t y2 = static_cast<int32_t>((static_cast<int64_t>(hand.box[3]) * draw_h) / result.height);

        if (x1 < 0) {
            x1 = 0;
        }
        if (y1 < 0) {
            y1 = 0;
        }
        if (x2 >= draw_w) {
            x2 = draw_w - 1;
        }
        if (y2 >= draw_h) {
            y2 = draw_h - 1;
        }
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

#if BSP_CONFIG_NO_GRAPHIC_LIB
        dl::image::img_t img = *fb;
        const std::vector<uint8_t> &color =
            img.pix_type == dl::image::DL_IMAGE_PIX_TYPE_RGB565LE ? kRgb565LeRed : kAlertColor;
        dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2, color, 3);
#else
        draw_rect_on_canvas(m_lcd_disp->get_canvas(), x1, y1, x2, y2, boundary_monitor_lv_color(true), 3);
#endif
    }
}

bool BoundaryMonitorAppLCD::hand_touches_boundary(
    who::cam::cam_fb_t *fb,
    const hand_detection_result_t &result,
    uint16_t border_width
) const
{
    if (!fb || result.width == 0 || result.height == 0 || result.boxes.empty()) {
        return false;
    }

    uint16_t draw_w = fb->width;
    uint16_t draw_h = fb->height;
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready) {
        draw_w = m_preview_w;
        draw_h = m_preview_h;
    }
#endif

    rect_i32_t boundary = {};
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_boundary_enabled) {
        boundary.x1 = m_boundary_p1.x < m_boundary_p2.x ? m_boundary_p1.x : m_boundary_p2.x;
        boundary.x2 = m_boundary_p1.x < m_boundary_p2.x ? m_boundary_p2.x : m_boundary_p1.x;
        boundary.y1 = m_boundary_p1.y < m_boundary_p2.y ? m_boundary_p1.y : m_boundary_p2.y;
        boundary.y2 = m_boundary_p1.y < m_boundary_p2.y ? m_boundary_p2.y : m_boundary_p1.y;
    } else
#endif
    {
        uint16_t scaled_border_width = scale_border_width(draw_w, draw_h, border_width);
        if (scaled_border_width == 0 || scaled_border_width * 2 >= draw_w || scaled_border_width * 2 >= draw_h) {
            return false;
        }
        boundary = {
            static_cast<int32_t>(scaled_border_width),
            static_cast<int32_t>(scaled_border_width),
            static_cast<int32_t>(draw_w - scaled_border_width - 1),
            static_cast<int32_t>(draw_h - scaled_border_width - 1),
        };
    }

    for (const auto &hand : result.boxes) {
        if (hand.box.size() < 4) {
            continue;
        }

        rect_i32_t box = {
            static_cast<int32_t>((static_cast<int64_t>(hand.box[0]) * draw_w) / result.width),
            static_cast<int32_t>((static_cast<int64_t>(hand.box[1]) * draw_h) / result.height),
            static_cast<int32_t>((static_cast<int64_t>(hand.box[2]) * draw_w) / result.width),
            static_cast<int32_t>((static_cast<int64_t>(hand.box[3]) * draw_h) / result.height),
        };

        box.x1 = clamp_i32(box.x1, 0, draw_w - 1);
        box.y1 = clamp_i32(box.y1, 0, draw_h - 1);
        box.x2 = clamp_i32(box.x2, 0, draw_w - 1);
        box.y2 = clamp_i32(box.y2, 0, draw_h - 1);
        if (box.x2 <= box.x1 || box.y2 <= box.y1) {
            continue;
        }

        if (box_touches_rect_edge(box, boundary, kHandBoundaryTouchPixels)) {
            return true;
        }
    }

    return false;
}

uint16_t BoundaryMonitorAppLCD::scale_border_width(
    uint16_t display_w,
    uint16_t display_h,
    uint16_t detect_border_width
) const
{
    uint32_t scaled_x =
        static_cast<uint32_t>(detect_border_width) * display_w / IntrusionMonitorTask::DETECT_WIDTH;
    uint32_t scaled_y =
        static_cast<uint32_t>(detect_border_width) * display_h / IntrusionMonitorTask::DETECT_HEIGHT;
    uint32_t scaled = scaled_x < scaled_y ? scaled_x : scaled_y;
    if (scaled == 0u) {
        scaled = 1u;
    }
    return static_cast<uint16_t>(scaled);
}

} // namespace app
} // namespace who
