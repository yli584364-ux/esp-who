#include "boundary_monitor_app.hpp"

#include <cstdlib>
#include <cstring>
#include <functional>

#include "bsp/esp-bsp.h"
#include "dl_image.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "who_lvgl_utils.hpp"
#include "who_yield2idle.hpp"

namespace {

static constexpr size_t kBootstrapFrames = 8;
static constexpr uint16_t kMaxConsecutiveFailures = 8;
static constexpr const char *TAG = "BoundaryMonitor";
static constexpr uint16_t kPreviewScalePercent = 78;
static constexpr uint16_t kMinCustomBoundarySpan = 8;
static const std::vector<uint8_t> kNormalColor = {0, 255, 0};
static const std::vector<uint8_t> kAlertColor = {255, 0, 0};

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

#if !BSP_CONFIG_NO_GRAPHIC_LIB
lv_color_t get_lv_color(bool intrusion)
{
    return who::cvt_to_lv_color(intrusion ? kAlertColor : kNormalColor);
}

void draw_custom_boundary_on_canvas(lv_obj_t *canvas, lv_point_t p1, lv_point_t p2, bool intrusion)
{
    int32_t x1 = p1.x < p2.x ? p1.x : p2.x;
    int32_t x2 = p1.x < p2.x ? p2.x : p1.x;
    int32_t y1 = p1.y < p2.y ? p1.y : p2.y;
    int32_t y2 = p1.y < p2.y ? p2.y : p1.y;

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_opa = LV_OPA_TRANSP;
    rect_dsc.border_width = intrusion ? 4 : 2;
    rect_dsc.border_color = intrusion ? get_lv_color(true) : lv_color_hex(0x4FC3F7);

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    lv_area_t area = {x1, y1, x2, y2};
    lv_draw_rect(&layer, &rect_dsc, &area);
    lv_canvas_finish_layer(canvas, &layer);
}

void draw_boundary_on_canvas(lv_obj_t *canvas, uint16_t width, uint16_t height, uint16_t border_width, bool intrusion)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_opa = LV_OPA_TRANSP;
    rect_dsc.border_width = intrusion ? 4 : 2;
    rect_dsc.border_color = get_lv_color(intrusion);

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    lv_area_t area = {
        static_cast<int32_t>(border_width),
        static_cast<int32_t>(border_width),
        static_cast<int32_t>(width - border_width - 1),
        static_cast<int32_t>(height - border_width - 1),
    };
    lv_draw_rect(&layer, &rect_dsc, &area);
    lv_canvas_finish_layer(canvas, &layer);
}

lv_obj_t *create_action_button(lv_obj_t *parent, const char *text, lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_width(btn, LV_PCT(100));
    lv_obj_set_height(btn, 42);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    return btn;
}
#endif

} // namespace

namespace who {
namespace app {

IntrusionMonitorTask::IntrusionMonitorTask(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node) :
    task::WhoTask(name),
    m_frame_cap_node(frame_cap_node),
    m_result_mutex(xSemaphoreCreateMutex()),
    m_result(),
    m_gray_frame(nullptr),
    m_bootstrap_frames(nullptr),
    m_pixels(0),
    m_bootstrap_count(0),
    m_detector_ready(false),
    m_border_width(0),
    m_caps(0),
    m_reset_requested(false),
    m_custom_boundary_update_pending(false),
    m_custom_boundary_enable_pending(false),
    m_custom_boundary_rect_pending(0),
    m_custom_boundary_enabled(false),
    m_custom_x1(0),
    m_custom_y1(0),
    m_custom_x2(0),
    m_custom_y2(0),
    m_consecutive_failures(0)
{
    memset(&m_detector, 0, sizeof(m_detector));
    memset(&m_cfg, 0, sizeof(m_cfg));
    frame_cap_node->add_new_frame_signal_subscriber(this);
#if CONFIG_IDF_TARGET_ESP32S3
    m_caps = dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN;
#endif
    m_image_transformer.set_caps(m_caps);
}

IntrusionMonitorTask::~IntrusionMonitorTask()
{
    cleanup();
    vSemaphoreDelete(m_result_mutex);
}

bool IntrusionMonitorTask::get_result(
    intrusion_detector_result_t *result,
    bool *ready,
    size_t *bootstrap_count,
    uint16_t *border_width
)
{
    xSemaphoreTake(m_result_mutex, portMAX_DELAY);
    if (result) {
        *result = m_result;
    }
    if (ready) {
        *ready = m_detector_ready;
    }
    if (bootstrap_count) {
        *bootstrap_count = m_bootstrap_count;
    }
    if (border_width) {
        *border_width = m_border_width;
    }
    xSemaphoreGive(m_result_mutex);
    return true;
}

void IntrusionMonitorTask::request_reset()
{
    m_reset_requested.store(true);
}

void IntrusionMonitorTask::request_custom_boundary(bool enabled, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint32_t packed = (static_cast<uint32_t>(x1) << 24) | (static_cast<uint32_t>(y1) << 16) |
                      (static_cast<uint32_t>(x2) << 8) | static_cast<uint32_t>(y2);
    m_custom_boundary_rect_pending.store(packed);
    m_custom_boundary_enable_pending.store(enabled);
    m_custom_boundary_update_pending.store(true);
}

void IntrusionMonitorTask::task()
{
    while (true) {
        EventBits_t event_bits =
            xEventGroupWaitBits(m_event_group, NEW_FRAME | TASK_PAUSE | TASK_STOP, pdTRUE, pdFALSE, portMAX_DELAY);
        if (event_bits & TASK_STOP) {
            break;
        } else if (event_bits & TASK_PAUSE) {
            xEventGroupSetBits(m_event_group, TASK_PAUSED);
            EventBits_t pause_event_bits =
                xEventGroupWaitBits(m_event_group, TASK_RESUME | TASK_STOP, pdTRUE, pdFALSE, portMAX_DELAY);
            if (pause_event_bits & TASK_STOP) {
                break;
            }
            continue;
        }

        if (m_reset_requested.exchange(false)) {
            ESP_LOGI(TAG, "manual reset requested, restarting calibration");
            cleanup();
            continue;
        }

        if (m_custom_boundary_update_pending.exchange(false)) {
            uint32_t packed = m_custom_boundary_rect_pending.load();
            m_custom_x1 = static_cast<uint16_t>((packed >> 24) & 0xFFu);
            m_custom_y1 = static_cast<uint16_t>((packed >> 16) & 0xFFu);
            m_custom_x2 = static_cast<uint16_t>((packed >> 8) & 0xFFu);
            m_custom_y2 = static_cast<uint16_t>(packed & 0xFFu);
            m_custom_boundary_enabled = m_custom_boundary_enable_pending.load();
            ESP_LOGI(TAG,
                     "custom boundary %s (%u,%u)-(%u,%u)",
                     m_custom_boundary_enabled ? "enabled" : "disabled",
                     static_cast<unsigned>(m_custom_x1),
                     static_cast<unsigned>(m_custom_y1),
                     static_cast<unsigned>(m_custom_x2),
                     static_cast<unsigned>(m_custom_y2));
            cleanup();
            continue;
        }

        auto fb = m_frame_cap_node->cam_fb_peek();
        if (!fb || !init_detector_if_needed(fb) || !convert_to_gray(fb)) {
            if (m_consecutive_failures < UINT16_MAX) {
                m_consecutive_failures++;
            }
            if (m_consecutive_failures >= kMaxConsecutiveFailures) {
                ESP_LOGW(TAG, "too many detector failures, resetting detector state");
                cleanup();
                m_consecutive_failures = 0;
            }
            continue;
        }

        if (m_bootstrap_count < kBootstrapFrames) {
            memcpy(m_bootstrap_frames[m_bootstrap_count], m_gray_frame, m_pixels);
            m_bootstrap_count++;
            if (m_bootstrap_count == kBootstrapFrames) {
                const uint8_t *bootstrap_frames[kBootstrapFrames];
                for (size_t i = 0; i < kBootstrapFrames; ++i) {
                    bootstrap_frames[i] = m_bootstrap_frames[i];
                }
                if (!intrusion_detector_bootstrap(&m_detector, bootstrap_frames, kBootstrapFrames)) {
                    ESP_LOGE(TAG, "intrusion_detector_bootstrap failed");
                }
            }
            m_consecutive_failures = 0;
            continue;
        }

        intrusion_detector_result_t result = {};
        if (!intrusion_detector_process(&m_detector, m_gray_frame, &result)) {
            ESP_LOGW(TAG, "intrusion_detector_process failed");
            if (m_consecutive_failures < UINT16_MAX) {
                m_consecutive_failures++;
            }
            if (m_consecutive_failures >= kMaxConsecutiveFailures) {
                ESP_LOGW(TAG, "processing keeps failing, resetting detector state");
                cleanup();
                m_consecutive_failures = 0;
            }
            continue;
        }
        m_consecutive_failures = 0;

        xSemaphoreTake(m_result_mutex, portMAX_DELAY);
        m_result = result;
        m_detector_ready = true;
        xSemaphoreGive(m_result_mutex);
    }

    xEventGroupSetBits(m_event_group, TASK_STOPPED);
    vTaskDelete(NULL);
}

void IntrusionMonitorTask::cleanup()
{
    if (m_gray_frame) {
        heap_caps_free(m_gray_frame);
        m_gray_frame = nullptr;
    }

    if (m_bootstrap_frames) {
        for (size_t i = 0; i < kBootstrapFrames; ++i) {
            if (m_bootstrap_frames[i]) {
                heap_caps_free(m_bootstrap_frames[i]);
            }
        }
        free(m_bootstrap_frames);
        m_bootstrap_frames = nullptr;
    }

    intrusion_detector_deinit(&m_detector);
    memset(&m_detector, 0, sizeof(m_detector));
    m_pixels = 0;
    m_bootstrap_count = 0;
    m_detector_ready = false;
    m_border_width = 0;

    xSemaphoreTake(m_result_mutex, portMAX_DELAY);
    memset(&m_result, 0, sizeof(m_result));
    xSemaphoreGive(m_result_mutex);
}

bool IntrusionMonitorTask::init_detector_if_needed(who::cam::cam_fb_t *fb)
{
    if (m_gray_frame) {
        return true;
    }

    (void)fb;
    m_pixels = static_cast<size_t>(DETECT_WIDTH) * static_cast<size_t>(DETECT_HEIGHT);
    m_gray_frame = static_cast<uint8_t *>(heap_caps_malloc(m_pixels, MALLOC_CAP_DEFAULT));
    m_bootstrap_frames = static_cast<uint8_t **>(calloc(kBootstrapFrames, sizeof(uint8_t *)));
    if (!m_gray_frame || !m_bootstrap_frames) {
        ESP_LOGE(TAG, "gray frame allocation failed");
        cleanup();
        return false;
    }

    for (size_t i = 0; i < kBootstrapFrames; ++i) {
        m_bootstrap_frames[i] = static_cast<uint8_t *>(heap_caps_malloc(m_pixels, MALLOC_CAP_DEFAULT));
        if (!m_bootstrap_frames[i]) {
            ESP_LOGE(TAG, "bootstrap frame allocation failed");
            cleanup();
            return false;
        }
    }

    intrusion_detector_default_config(&m_cfg, DETECT_WIDTH, DETECT_HEIGHT);
    m_cfg.bg_method = ID_BG_KNN_STANDARD;
    m_cfg.consecutive_trigger_frames = 3;
    m_cfg.alarm_hold_frames = 10;
    m_cfg.temporal_window = 2;
    m_cfg.custom_border_enabled = m_custom_boundary_enabled;
    m_cfg.custom_x1 = m_custom_x1;
    m_cfg.custom_y1 = m_custom_y1;
    m_cfg.custom_x2 = m_custom_x2;
    m_cfg.custom_y2 = m_custom_y2;
    m_border_width = calc_border_width(m_cfg);

    if (!intrusion_detector_init(&m_detector, &m_cfg)) {
        ESP_LOGE(TAG, "intrusion_detector_init failed");
        cleanup();
        return false;
    }

    return true;
}

bool IntrusionMonitorTask::convert_to_gray(who::cam::cam_fb_t *fb)
{
    dl::image::img_t gray = {
        .data = m_gray_frame,
        .width = DETECT_WIDTH,
        .height = DETECT_HEIGHT,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_GRAY,
    };
    return m_image_transformer.set_src_img(*fb).set_dst_img(gray).transform() == ESP_OK;
}

uint16_t IntrusionMonitorTask::calc_border_width(const intrusion_detector_config_t &cfg)
{
    uint16_t min_dim = cfg.width < cfg.height ? cfg.width : cfg.height;
    uint16_t border_width = static_cast<uint16_t>(min_dim * cfg.border_width_ratio);
    if (border_width < cfg.border_width_min_px) {
        border_width = cfg.border_width_min_px;
    }
    if (border_width > min_dim / 2u) {
        border_width = min_dim / 2u;
    }
    return border_width == 0u ? 1u : border_width;
}

BoundaryMonitorAppLCD::BoundaryMonitorAppLCD(frame_cap::WhoFrameCap *frame_cap) :
    m_frame_cap(frame_cap),
    m_lcd_disp(new lcd_disp::WhoFrameLCDDisp("LCDDisp", frame_cap->get_last_node(), 0)),
    m_monitor_task(new IntrusionMonitorTask("BoundaryDetect", frame_cap->get_last_node())),
    m_hand_detect(nullptr)
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    ,
    m_label(nullptr),
    m_control_panel(nullptr),
    m_start_btn(nullptr),
    m_reset_btn(nullptr),
    m_placeholder_btn_1(nullptr),
    m_placeholder_btn_2(nullptr),
    m_preview_buf(nullptr),
    m_preview_w(0),
    m_preview_h(0),
    m_preview_ready(false),
    m_detection_enabled(false),
    m_boundary_edit_mode(false),
    m_boundary_has_first_point(false),
    m_boundary_enabled(false),
    m_boundary_p1({0, 0}),
    m_boundary_p2({0, 0})
#endif
{
    m_lcd_disp->set_lcd_disp_cb(std::bind(&BoundaryMonitorAppLCD::lcd_disp_cb, this, std::placeholders::_1));
#if 1
    // instantiate hand detector (load model now)
    m_hand_detect = new HandDetect(static_cast<HandDetect::model_type_t>(0), false);
#endif
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    bsp_display_lock(0);
    m_label = create_lvgl_label("Waiting: press Start Detect", LV_FONT_DEFAULT, {255, 255, 255});
    lv_obj_align(m_label, LV_ALIGN_TOP_LEFT, 8, 8);

    m_control_panel = lv_obj_create(lv_screen_active());
    lv_obj_set_size(m_control_panel, 180, 220);
    lv_obj_align(m_control_panel, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_set_style_pad_all(m_control_panel, 8, 0);
    lv_obj_set_style_bg_opa(m_control_panel, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(m_control_panel, lv_color_hex(0x1E242B), 0);

    lv_obj_t *title = lv_label_create(m_control_panel);
    lv_label_set_text(title, "Control Panel");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *button_col = lv_obj_create(m_control_panel);
    lv_obj_set_size(button_col, LV_PCT(100), 168);
    lv_obj_align(button_col, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(button_col, 0, 0);
    lv_obj_set_style_border_width(button_col, 0, 0);
    lv_obj_set_style_bg_opa(button_col, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(button_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(button_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(button_col, 6, 0);

    m_start_btn = create_action_button(button_col, "Start Detect", BoundaryMonitorAppLCD::start_button_event_cb, this);
    m_reset_btn = create_action_button(button_col, "Reset Range", BoundaryMonitorAppLCD::reset_button_event_cb, this);
    m_placeholder_btn_1 =
        create_action_button(button_col, "Feature 2: Boundary", BoundaryMonitorAppLCD::feature2_button_event_cb, this);
    m_placeholder_btn_2 =
        create_action_button(button_col, "Feature 3 (Todo)", BoundaryMonitorAppLCD::feature2_button_event_cb, this);

    update_start_button_text();
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
    delete m_monitor_task;
    if (m_hand_detect) {
        delete m_hand_detect;
        m_hand_detect = nullptr;
    }
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

    if (ret) {
        set_detection_enabled(false);
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

    bool intrusion = ready && result.intrusion;
    draw_overlay(fb, intrusion, border_width);

    // Run hand detection and draw boxes when enabled
    uint16_t _draw_w = fb->width;
    uint16_t _draw_h = fb->height;
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_preview_ready) {
        _draw_w = m_preview_w;
        _draw_h = m_preview_h;
    }
#endif

    if (m_detection_enabled && m_hand_detect && fb) {
        dl::image::img_t img = {.data = fb->buf, .width = fb->width, .height = fb->height, .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565};
        std::list<dl::detect::result_t> &res = m_hand_detect->run(img);
        unsigned cnt = 0;
        for (auto &r : res) {
            if (r.box.size() < 4) continue;
            r.limit_box(static_cast<int>(fb->width), static_cast<int>(fb->height));
            int x1 = r.box[0];
            int y1 = r.box[1];
            int x2 = r.box[2];
            int y2 = r.box[3];
            int sx1 = static_cast<int>((static_cast<int64_t>(x1) * _draw_w) / fb->width);
            int sy1 = static_cast<int>((static_cast<int64_t>(y1) * _draw_h) / fb->height);
            int sx2 = static_cast<int>((static_cast<int64_t>(x2) * _draw_w) / fb->width);
            int sy2 = static_cast<int>((static_cast<int64_t>(y2) * _draw_h) / fb->height);
#if BSP_CONFIG_NO_GRAPHIC_LIB
            dl::image::draw_hollow_rectangle(*fb, sx1, sy1, sx2, sy2, kAlertColor, 2);
#else
            lv_draw_rect_dsc_t rect_dsc;
            lv_draw_rect_dsc_init(&rect_dsc);
            rect_dsc.bg_opa = LV_OPA_TRANSP;
            rect_dsc.border_width = 2;
            rect_dsc.border_color = get_lv_color(true);
            lv_layer_t layer;
            lv_canvas_init_layer(m_lcd_disp->get_canvas(), &layer);
            lv_area_t area = {sx1, sy1, sx2, sy2};
            lv_draw_rect(&layer, &rect_dsc, &area);
            lv_canvas_finish_layer(m_lcd_disp->get_canvas(), &layer);
#endif
            cnt++;
        }
#if !BSP_CONFIG_NO_GRAPHIC_LIB
        if (m_label) {
            if (!ready) {
                lv_label_set_text_fmt(m_label, "Calibrating %u/%u", static_cast<unsigned>(bootstrap_count), kBootstrapFrames);
            } else {
                lv_label_set_text_fmt(m_label, "Hands: %u", cnt);
            }
        }
#else
        ESP_LOGI(TAG, "Hands: %u", cnt);
#endif
    }

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_detection_enabled) {
        lv_label_set_text(m_label, "Camera paused. Press Start Detect.");
    } else if (!ready) {
        lv_label_set_text_fmt(m_label, "Calibrating %u/%u", static_cast<unsigned>(bootstrap_count), kBootstrapFrames);
    } else {
        lv_label_set_text_fmt(
            m_label,
            "%s ratio=%.3f fg=%lu/%lu",
            intrusion ? "ALERT" : "NORMAL",
            result.smoothed_ratio,
            static_cast<unsigned long>(result.fg_pixels),
            static_cast<unsigned long>(result.border_pixels));
    }
#else
    if (ready) {
        ESP_LOGI(TAG,
                 "%s ratio=%.3f fg=%lu/%lu",
                 intrusion ? "ALERT" : "NORMAL",
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

    lv_obj_t *canvas = m_lcd_disp->get_canvas();
    lv_obj_set_size(canvas, m_preview_w, m_preview_h);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(canvas, BoundaryMonitorAppLCD::preview_click_event_cb, LV_EVENT_CLICKED, this);

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

    if (enabled) {
        m_monitor_task->request_reset();
        m_monitor_task->resume();
        for (const auto &frame_cap_node : m_frame_cap->get_all_nodes()) {
            frame_cap_node->resume();
        }
    } else {
        m_monitor_task->pause();
    }

    m_detection_enabled = enabled;
    update_start_button_text();
}

void BoundaryMonitorAppLCD::update_start_button_text()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!m_start_btn) {
        return;
    }
    lv_obj_t *label = lv_obj_get_child(m_start_btn, 0);
    if (label) {
        lv_label_set_text(label, m_detection_enabled ? "Pause Detect" : "Start Detect");
    }
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
