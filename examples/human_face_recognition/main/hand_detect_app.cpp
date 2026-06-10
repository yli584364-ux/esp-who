#include "hand_detect_app.hpp"

#include <list>

#include "dl_image.hpp"
#include "hand_detect.hpp"

namespace who {
namespace app {

HandDetectTask::HandDetectTask(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node) :
    task::WhoTask(name),
    m_frame_cap_node(frame_cap_node),
    m_detector(new HandDetect(HandDetect::ESPDET_PICO_224_224_HAND, true)),
    m_result_mutex(xSemaphoreCreateMutex()),
    m_result()
{
    m_result.ready = false;
    frame_cap_node->add_new_frame_signal_subscriber(this);
}

HandDetectTask::~HandDetectTask()
{
    cleanup();
    vSemaphoreDelete(m_result_mutex);
    delete m_detector;
}

bool HandDetectTask::get_result(hand_detection_result_t *result)
{
    if (!result) {
        return false;
    }
    xSemaphoreTake(m_result_mutex, portMAX_DELAY);
    *result = m_result;
    xSemaphoreGive(m_result_mutex);
    return true;
}

void HandDetectTask::clear_result()
{
    xSemaphoreTake(m_result_mutex, portMAX_DELAY);
    m_result.ready = false;
    m_result.width = 0;
    m_result.height = 0;
    m_result.boxes.clear();
    xSemaphoreGive(m_result_mutex);
}

void HandDetectTask::task()
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
        if (!fb || !fb->buf ||
            (fb->format != who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565 &&
             fb->format != who::cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888)) {
            continue;
        }

        dl::image::img_t img = *fb;
        std::list<dl::detect::result_t> &detect_results = m_detector->run(img);

        hand_detection_result_t result = {};
        result.ready = true;
        result.width = fb->width;
        result.height = fb->height;
        for (const auto &box : detect_results) {
            if (box.box.size() >= 4) {
                result.boxes.push_back(box);
            }
        }

        xSemaphoreTake(m_result_mutex, portMAX_DELAY);
        m_result = result;
        xSemaphoreGive(m_result_mutex);
    }

    xEventGroupSetBits(m_event_group, TASK_STOPPED);
    vTaskDelete(NULL);
}

void HandDetectTask::cleanup()
{
    clear_result();
}

} // namespace app
} // namespace who
