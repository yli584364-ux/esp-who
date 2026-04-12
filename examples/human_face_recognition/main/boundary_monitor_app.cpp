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
static constexpr const char *TAG = "BoundaryMonitor";
static const std::vector<uint8_t> kNormalColor = {0, 255, 0};
static const std::vector<uint8_t> kAlertColor = {255, 0, 0};

#if !BSP_CONFIG_NO_GRAPHIC_LIB
lv_color_t get_lv_color(bool intrusion)
{
    return who::cvt_to_lv_color(intrusion ? kAlertColor : kNormalColor);
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
    m_caps(0)
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

        auto fb = m_frame_cap_node->cam_fb_peek();
        if (!fb || !init_detector_if_needed(fb) || !convert_to_gray(fb)) {
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
            continue;
        }

        intrusion_detector_result_t result = {};
        if (!intrusion_detector_process(&m_detector, m_gray_frame, &result)) {
            ESP_LOGW(TAG, "intrusion_detector_process failed");
            continue;
        }

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

    m_pixels = static_cast<size_t>(fb->width) * static_cast<size_t>(fb->height);
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

    intrusion_detector_default_config(&m_cfg, fb->width, fb->height);
    m_cfg.bg_method = ID_BG_KNN_STANDARD;
    m_cfg.consecutive_trigger_frames = 3;
    m_cfg.alarm_hold_frames = 10;
    m_cfg.temporal_window = 2;
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
        .width = fb->width,
        .height = fb->height,
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
    m_lcd_disp(new lcd_disp::WhoFrameLCDDisp("LCDDisp", frame_cap->get_last_node(), 1)),
    m_monitor_task(new IntrusionMonitorTask("BoundaryDetect", frame_cap->get_last_node()))
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    ,
    m_label(nullptr)
#endif
{
    m_lcd_disp->set_lcd_disp_cb(std::bind(&BoundaryMonitorAppLCD::lcd_disp_cb, this, std::placeholders::_1));

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    bsp_display_lock(0);
    m_label = create_lvgl_label("", LV_FONT_DEFAULT, {255, 255, 255});
    lv_obj_align(m_label, LV_ALIGN_TOP_LEFT, 12, 12);
    bsp_display_unlock();
#endif
}

BoundaryMonitorAppLCD::~BoundaryMonitorAppLCD()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (m_label) {
        bsp_display_lock(0);
        lv_obj_del(m_label);
        bsp_display_unlock();
    }
#endif
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
    return ret;
}

void BoundaryMonitorAppLCD::lcd_disp_cb(who::cam::cam_fb_t *fb)
{
    intrusion_detector_result_t result = {};
    bool ready = false;
    size_t bootstrap_count = 0;
    uint16_t border_width = 0;
    m_monitor_task->get_result(&result, &ready, &bootstrap_count, &border_width);

    bool intrusion = ready && result.intrusion;
    draw_overlay(fb, intrusion, border_width);

#if !BSP_CONFIG_NO_GRAPHIC_LIB
    if (!ready) {
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

void BoundaryMonitorAppLCD::draw_overlay(who::cam::cam_fb_t *fb, bool intrusion, uint16_t border_width)
{
    if (border_width == 0 || border_width * 2 >= fb->width || border_width * 2 >= fb->height) {
        return;
    }

#if BSP_CONFIG_NO_GRAPHIC_LIB
    const auto &color = intrusion ? kAlertColor : kNormalColor;
    dl::image::draw_hollow_rectangle(*fb,
                                     border_width,
                                     border_width,
                                     fb->width - border_width - 1,
                                     fb->height - border_width - 1,
                                     color,
                                     intrusion ? 4 : 2);
#else
    draw_boundary_on_canvas(m_lcd_disp->get_canvas(), fb->width, fb->height, border_width, intrusion);
#endif
}

} // namespace app
} // namespace who
