#include "boundary_detect.hpp"

#include <cstdlib>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"

namespace {

static constexpr uint16_t kMaxConsecutiveFailures = 8;
static constexpr const char *TAG = "BoundaryDetect";
static constexpr size_t kImageBufferAlignment = 16;

uint8_t rgb565_to_gray(uint16_t pixel)
{
    uint8_t r = static_cast<uint8_t>(((pixel >> 11) & 0x1F) << 3);
    uint8_t g = static_cast<uint8_t>(((pixel >> 5) & 0x3F) << 2);
    uint8_t b = static_cast<uint8_t>((pixel & 0x1F) << 3);
    return static_cast<uint8_t>((static_cast<uint16_t>(r) * 77u + static_cast<uint16_t>(g) * 150u +
                                 static_cast<uint16_t>(b) * 29u) >>
                                8);
}

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
    m_processing_enabled(false),
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

void IntrusionMonitorTask::set_processing_enabled(bool enabled)
{
    m_processing_enabled.store(enabled);
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

        if (!m_processing_enabled.load()) {
            cleanup();
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

        if (m_bootstrap_count < BOOTSTRAP_FRAMES) {
            memcpy(m_bootstrap_frames[m_bootstrap_count], m_gray_frame, m_pixels);
            m_bootstrap_count++;
            if (m_bootstrap_count == BOOTSTRAP_FRAMES) {
                const uint8_t *bootstrap_frames[BOOTSTRAP_FRAMES];
                for (size_t i = 0; i < BOOTSTRAP_FRAMES; ++i) {
                    bootstrap_frames[i] = m_bootstrap_frames[i];
                }
                if (!intrusion_detector_bootstrap(&m_detector, bootstrap_frames, BOOTSTRAP_FRAMES)) {
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
        for (size_t i = 0; i < BOOTSTRAP_FRAMES; ++i) {
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
    m_gray_frame = static_cast<uint8_t *>(heap_caps_aligned_calloc(kImageBufferAlignment, 1, m_pixels, MALLOC_CAP_DEFAULT));
    m_bootstrap_frames = static_cast<uint8_t **>(calloc(BOOTSTRAP_FRAMES, sizeof(uint8_t *)));
    if (!m_gray_frame || !m_bootstrap_frames) {
        ESP_LOGE(TAG, "gray frame allocation failed");
        cleanup();
        return false;
    }

    for (size_t i = 0; i < BOOTSTRAP_FRAMES; ++i) {
        m_bootstrap_frames[i] =
            static_cast<uint8_t *>(heap_caps_aligned_calloc(kImageBufferAlignment, 1, m_pixels, MALLOC_CAP_DEFAULT));
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
    if (!fb || !fb->buf || !m_gray_frame || fb->width == 0 || fb->height == 0) {
        return false;
    }

    if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565) {
        const uint16_t *src = static_cast<const uint16_t *>(fb->buf);
        for (uint16_t y = 0; y < DETECT_HEIGHT; ++y) {
            uint16_t sy = static_cast<uint16_t>((static_cast<uint32_t>(y) * fb->height) / DETECT_HEIGHT);
            for (uint16_t x = 0; x < DETECT_WIDTH; ++x) {
                uint16_t sx = static_cast<uint16_t>((static_cast<uint32_t>(x) * fb->width) / DETECT_WIDTH);
                m_gray_frame[static_cast<size_t>(y) * DETECT_WIDTH + x] =
                    rgb565_to_gray(src[static_cast<size_t>(sy) * fb->width + sx]);
            }
        }
        return true;
    }

    if (fb->format == who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888) {
        const uint8_t *src = static_cast<const uint8_t *>(fb->buf);
        for (uint16_t y = 0; y < DETECT_HEIGHT; ++y) {
            uint16_t sy = static_cast<uint16_t>((static_cast<uint32_t>(y) * fb->height) / DETECT_HEIGHT);
            for (uint16_t x = 0; x < DETECT_WIDTH; ++x) {
                uint16_t sx = static_cast<uint16_t>((static_cast<uint32_t>(x) * fb->width) / DETECT_WIDTH);
                const uint8_t *pixel = src + (static_cast<size_t>(sy) * fb->width + sx) * 3u;
                m_gray_frame[static_cast<size_t>(y) * DETECT_WIDTH + x] =
                    static_cast<uint8_t>((static_cast<uint16_t>(pixel[0]) * 77u +
                                          static_cast<uint16_t>(pixel[1]) * 150u +
                                          static_cast<uint16_t>(pixel[2]) * 29u) >>
                                         8);
            }
        }
        return true;
    }

    return false;
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

} // namespace app
} // namespace who
