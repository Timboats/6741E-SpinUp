#include "braingui.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_core/lv_style.h"
#include "display/lv_misc/lv_color.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_cont.h"
#include "display/lv_objx/lv_page.h"
#include "display/lv_objx/lv_win.h"
#include "vexfs.h"
#include "braingui_callbacks.h"
#include <cstdio>



void tuneWindow(const char* title){
    lv_obj_t* window = lv_win_create(lv_scr_act(), NULL);
    lv_win_set_title(window, title);
    lv_win_add_btn(window, SYMBOL_CLOSE, lv_win_close_action);

    // lv_obj_t* tuneMenuBtns = lv_btnm_create(window, NULL);
    // static const char * btnMap[] = {"Tune", "Ports", "Func", ""};
    // lv_btnm_set_map(tuneMenuBtns, btnMap);
}
void funcWindow(const char* title){
    settings storedSettings = getSettings();
    static const char * btnMap[] = {"Side", "Inertial", "Gps", ""};
    lv_obj_t* window = lv_win_create(lv_scr_act(), NULL);
    lv_win_set_title(window, title);
    lv_win_add_btn(window, SYMBOL_CLOSE, lv_win_close_action);
    lv_win_set_layout(window, LV_LAYOUT_GRID);
    lv_win_set_sb_mode(window, LV_SB_MODE_DRAG);
    // static lv_style_t btnStyle;
    
    lv_obj_t* funcBtnsTemplate = lv_btn_create(window, NULL);
    // lv_style_copy(&btnStyle, lv_btn_get_style(funcBtnsTemplate, LV_BTN_STYLE_TGL_REL));

    // btnStyle.body.border.color = LV_COLOR_AQUA;

    lv_obj_set_size(funcBtnsTemplate, 85, 40);
    
    lv_btn_set_toggle(funcBtnsTemplate, true);

    lv_obj_t* sideBtn = lv_btn_create(window, funcBtnsTemplate);
    lv_obj_t* inertialBtn = lv_btn_create(window, funcBtnsTemplate);
    lv_obj_t* gpsBtn = lv_btn_create(window, funcBtnsTemplate);

    
    lv_obj_set_hidden(funcBtnsTemplate, true); //Put this at the end of the initialization of each button

    ///////////////////////////////////////////

    lv_btn_set_action(sideBtn, LV_BTN_ACTION_CLICK, sideBtnCallback);
    lv_btn_set_action(inertialBtn, LV_BTN_ACTION_CLICK, inertialBtnCallback);
    lv_btn_set_action(gpsBtn, LV_BTN_ACTION_CLICK, gpsBtnCallback);

    // lv_btn_set_style(sideBtn, LV_BTN_STYLE_TGL_PR, &btnStyle);
    
    lv_btn_set_state(sideBtn, storedSettings.isOnBlueSide);
    lv_btn_set_state(inertialBtn, storedSettings.isInertial);
    lv_btn_set_state(gpsBtn, storedSettings.isGpsAvaiable);
}

static lv_res_t menuBtnsEventHandler(lv_obj_t* btnMatrix, const char* txt){
    if(txt == "Tune"){
        tuneWindow(txt);
    }
    if(txt == "Func"){
        funcWindow(txt);
    }
    return LV_RES_OK;
}
void lvglInitEx(){
    static const char * btnMap[] = {"Tune", "Ports", "Func", ""};

    lv_obj_t* mainMenuBtns = lv_btnm_create(lv_scr_act(), NULL);
    lv_btnm_set_map(mainMenuBtns, btnMap);
    lv_btnm_set_action(mainMenuBtns, menuBtnsEventHandler);




}



