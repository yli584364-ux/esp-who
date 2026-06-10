#include "boundary_monitor_lvgl.hpp"

#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include <vector>

#include "who_lvgl_utils.hpp"

namespace {

static const std::vector<uint8_t> kNormalColor = {0, 255, 0};
static const std::vector<uint8_t> kAlertColor = {255, 0, 0};

} // namespace

namespace who {
namespace app {

lv_color_t boundary_monitor_lv_color(bool intrusion)
{
    return who::cvt_to_lv_color(intrusion ? kAlertColor : kNormalColor);
}

lv_obj_t *create_status_label(const char *text)
{
    lv_obj_t *label = create_lvgl_label(text, LV_FONT_DEFAULT, {255, 255, 255});
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 8, 8);
    return label;
}

lv_obj_t *create_control_panel()
{
    lv_obj_t *panel = lv_obj_create(lv_screen_active());
    lv_obj_set_size(panel, 180, 276);
    lv_obj_align(panel, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_set_style_pad_all(panel, 8, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1E242B), 0);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Control Panel");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    return panel;
}

lv_obj_t *create_button_column(lv_obj_t *panel)
{
    lv_obj_t *button_col = lv_obj_create(panel);
    lv_obj_set_size(button_col, LV_PCT(100), 224);
    lv_obj_align(button_col, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(button_col, 0, 0);
    lv_obj_set_style_border_width(button_col, 0, 0);
    lv_obj_set_style_bg_opa(button_col, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(button_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(button_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(button_col, 6, 0);
    return button_col;
}

void draw_rect_on_canvas(lv_obj_t *canvas, int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color, uint8_t width)
{
    if (x2 <= x1 || y2 <= y1) {
        return;
    }

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_opa = LV_OPA_TRANSP;
    rect_dsc.border_width = width;
    rect_dsc.border_color = color;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);
    lv_area_t area = {x1, y1, x2, y2};
    lv_draw_rect(&layer, &rect_dsc, &area);
    lv_canvas_finish_layer(canvas, &layer);
}

void draw_custom_boundary_on_canvas(lv_obj_t *canvas, lv_point_t p1, lv_point_t p2, bool intrusion)
{
    int32_t x1 = p1.x < p2.x ? p1.x : p2.x;
    int32_t x2 = p1.x < p2.x ? p2.x : p1.x;
    int32_t y1 = p1.y < p2.y ? p1.y : p2.y;
    int32_t y2 = p1.y < p2.y ? p2.y : p1.y;

    draw_rect_on_canvas(canvas,
                        x1,
                        y1,
                        x2,
                        y2,
                        intrusion ? boundary_monitor_lv_color(true) : lv_color_hex(0x4FC3F7),
                        intrusion ? 4 : 2);
}

void draw_boundary_on_canvas(lv_obj_t *canvas, uint16_t width, uint16_t height, uint16_t border_width, bool intrusion)
{
    draw_rect_on_canvas(canvas,
                        border_width,
                        border_width,
                        width - border_width - 1,
                        height - border_width - 1,
                        boundary_monitor_lv_color(intrusion),
                        intrusion ? 4 : 2);
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

void configure_preview_canvas(lv_obj_t *canvas, uint16_t width, uint16_t height, lv_event_cb_t cb, void *user_data)
{
    lv_obj_set_size(canvas, width, height);
    lv_obj_align(canvas, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(canvas, cb, LV_EVENT_CLICKED, user_data);
}

void set_button_text(lv_obj_t *button, const char *text)
{
    if (!button) {
        return;
    }
    lv_obj_t *label = lv_obj_get_child(button, 0);
    if (label) {
        lv_label_set_text(label, text);
    }
}

} // namespace app
} // namespace who
#endif
