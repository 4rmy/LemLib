#include "lemlib/ui/sidebar/sidebar.hpp"
#include "lemlib/ui/page.hpp"
#include "lemlib/ui/ui.hpp"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/font/lv_font.h"

using namespace lemlib::ui::sidebar;

Sidebar::Sidebar(lv_obj_t* screen, lv_obj_t* toggleButton, bool active) {
    this->screen = screen;
    this->toggleButton = toggleButton;
    this->active = active;
    this->elements = std::vector<lv_obj_t*>();
    
    buttonLabels = std::map<lv_obj_t*, lv_obj_t*>();

    this->render();
    // this->toggle(false);
}

void Sidebar::toggle(bool change) {
    for (lv_obj_t* element : this->elements) {
        if (this->active) {
            lv_obj_add_flag(element, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(element, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (change) this->active = !this->active;

    std::cout << "Sidebar toggled, active: " << this->active << std::endl;
}

lv_obj_t* Sidebar::createPageButton(std::string name, lv_obj_t* screen, int x, int y, lv_event_cb_t callback, bool active) {
    double PIXEL_WIDTH_PER_CHAR = 6.75;
    
    double width = name.length() * PIXEL_WIDTH_PER_CHAR + 10;

    lv_obj_t* button = lv_btn_create(screen);
    lv_obj_set_width(button, width);
    lv_obj_set_height(button, 20);
    lv_obj_set_x(button, x);
    lv_obj_set_y(button, y);
    lv_obj_set_align(button, LV_ALIGN_CENTER);
    lv_obj_add_flag(button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(button, active ? lv_color_hex(0x52CD27) : lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(button, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(button, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(button, LV_TEXT_ALIGN_AUTO, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_decor(button, LV_TEXT_DECOR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(button, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* label = lemlib::ui::util::createLabel(name, screen, x, y);
    if (active) lv_obj_set_style_text_color(label, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);    

    buttonLabels[button] = label;

    return label;
}

std::vector<std::string> sortByLength(std::vector<std::string> input) {
    std::vector<std::string> output = std::vector<std::string>();

    while (input.size() > 0) {
        int longestIndex = 0;
        int longestLength = 0;

        for (int i = 0; i < input.size(); i++) {
            std::string name = input[i];
            int length = name.length();

            if (length > longestLength) {
                longestIndex = i;
                longestLength = length;
            }
        }

        output.push_back(input[longestIndex]);
        input.erase(input.begin() + longestIndex);
    }

    return output;
}

void Sidebar::render() {
    this->sidebar = lv_obj_create(this->screen);
    lv_obj_set_width(this->sidebar, 120);
    lv_obj_set_height(this->sidebar, 240);
    lv_obj_set_x(this->sidebar, 180);
    lv_obj_set_y(this->sidebar, 0);
    lv_obj_set_align(this->sidebar, LV_ALIGN_CENTER);
    lv_obj_clear_flag(this->sidebar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(this->sidebar, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(this->sidebar, lv_color_hex(0x2E2E2E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(this->sidebar, 245, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(this->sidebar, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(this->sidebar, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // lv_obj_t* hamburger = lemlib::ui::util::createImage(&hamburgerImage, this->screen, LV_SIZE_CONTENT, LV_SIZE_CONTENT, 212, -93);
    // lv_obj_add_flag(hamburger, LV_OBJ_FLAG_ADV_HITTEST);
    // lv_obj_clear_flag(hamburger, LV_OBJ_FLAG_SCROLLABLE);

    /* Titles */
    lv_obj_t* titleOne = lemlib::ui::util::createLabel("L", this->screen, 135, -93);
    lv_obj_t* titleTwo = lemlib::ui::util::createLabel("em", this->screen, 152, -93);
    lv_obj_t* titleThree = lemlib::ui::util::createLabel("L", this->screen, 168, -93);
    lv_obj_t* titleFour = lemlib::ui::util::createLabel("ib", this->screen, 180, -93);
    
    lv_obj_set_style_text_color(titleOne, lv_color_hex(0x52CD27), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(titleOne, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(titleThree, lv_color_hex(0x52CD27), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(titleThree, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // lv_obj_t* lowerLabel = lemlib::ui::util::createLabel("Powered by LemLib", this->screen, 163, 112);
    // lv_obj_set_style_text_font(lowerLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    /* Backend */

    this->elements.push_back(this->sidebar);
    this->elements.push_back(titleOne);
    this->elements.push_back(titleTwo);
    this->elements.push_back(titleThree);
    this->elements.push_back(titleFour);
    // this->elements.push_back(hamburger);
    // this->elements.push_back(lowerLabel);

    std::vector<std::string> sorted = sortByLength(lemlib::ui::getRegisteredPageNames());

    lv_event_cb_t callback = [](lv_event_t* event) {
        lv_obj_t* button = lv_event_get_target(event);

        lv_obj_t* label = buttonLabels[button];
        std::string name = lv_label_get_text(label);

        lemlib::ui::setCurrentPage(name);
    };

    // loop over all registered pages
    for (int i = 0; i < sorted.size(); i++) {
        std::string name = sorted[i];

        lemlib::ui::Page* page = lemlib::ui::getPage(name);

        int x = page->getSidebarX();
        int y = page->getSidebarY();

        lv_obj_t* button = createPageButton(name, this->screen, x, y, callback, name == lemlib::ui::getCurrentPage());
       
        this->elements.push_back(button);
    }

    std::cout << "Sidebar rendered, " << this->elements.size() << " elements and " << sorted.size() << " pages" << std::endl;
}