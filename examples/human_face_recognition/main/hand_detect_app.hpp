#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dl_detect_define.hpp"
#include "who_frame_cap.hpp"
#include "who_task.hpp"

class HandDetect;

namespace who {
namespace app {

struct hand_detection_result_t {
    bool ready;
    uint16_t width;
    uint16_t height;
    std::vector<dl::detect::result_t> boxes;
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

} // namespace app
} // namespace who
