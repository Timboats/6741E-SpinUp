#include "braingui_callbacks.h"

lv_res_t sideBtnCallback(lv_obj_t* btn){
    settings storedSettings = getSettings();
    storedSettings.isOnBlueSide = !storedSettings.isOnBlueSide;
    writeSettings(storedSettings);
    lv_btn_set_state(btn, storedSettings.isOnBlueSide);

    return LV_RES_OK;
}
lv_res_t inertialBtnCallback(lv_obj_t* btn){
    settings storedSettings = getSettings();
    storedSettings.isInertial = !storedSettings.isInertial;
    writeSettings(storedSettings);
    lv_btn_set_state(btn, storedSettings.isInertial);

    return LV_RES_OK;
}
lv_res_t gpsBtnCallback(lv_obj_t* btn){
    settings storedSettings = getSettings();
    storedSettings.isGpsAvaiable = !storedSettings.isGpsAvaiable;
    writeSettings(storedSettings);
    lv_btn_set_state(btn, storedSettings.isGpsAvaiable);

    return LV_RES_OK;
}
