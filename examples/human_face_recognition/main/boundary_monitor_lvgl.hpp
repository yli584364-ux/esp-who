#pragma once

#include <cstdint>

#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include "lvgl.h"

namespace who {
namespace app {

lv_color_t boundary_monitor_lv_color(bool intrusion);
lv_obj_t *create_status_label(const char *text);
lv_obj_t *create_control_panel();
lv_obj_t *create_button_column(lv_obj_t *panel);
void draw_rect_on_canvas(lv_obj_t *canvas, int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color, uint8_t width);
void draw_custom_boundary_on_canvas(lv_obj_t *canvas, lv_point_t p1, lv_point_t p2, bool intrusion);
void draw_boundary_on_canvas(lv_obj_t *canvas, uint16_t width, uint16_t height, uint16_t border_width, bool intrusion);
lv_obj_t *create_action_button(lv_obj_t *parent, const char *text, lv_event_cb_t cb, void *user_data);
void configure_preview_canvas(lv_obj_t *canvas, uint16_t width, uint16_t height, lv_event_cb_t cb, void *user_data);
void set_button_text(lv_obj_t *button, const char *text);

} // namespace app
} // namespace who
#endif
