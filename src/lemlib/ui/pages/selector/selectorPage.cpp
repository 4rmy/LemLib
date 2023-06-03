#include "lemlib/ui/pages/selector/selectorPage.hpp"

using namespace lemlib::ui::pages;

AutonSelectorPage::AutonSelectorPage(lemlib::Chassis* chassis): Page(chassis, "Auton Selector", 192, -38) {

}

void AutonSelectorPage::destroy() {

}

void AutonSelectorPage::initialize(lv_obj_t* screen) {
    this->header = lemlib::ui::util::createLabel("Auton Selector", this->screen, 119, -84);
    lv_obj_set_style_text_font(this->header, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void AutonSelectorPage::render() {

}