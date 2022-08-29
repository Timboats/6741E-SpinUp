#include "braingui.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_win.h"

//mv stands for multi-variable
void mvEditWindow(){

}

void mvSelectWindow(const char* title){
    lv_obj_t* window = lv_win_create(lv_scr_act(), NULL);
    lv_win_set_title(window, title);
    lv_win_add_btn(window, SYMBOL_CLOSE, lv_win_close_action);

}

static lv_res_t menuBtnsEventHandler(lv_obj_t* btnMatrix, const char* txt){
    if(txt == "Tune"){
        mvSelectWindow(txt);
    }
    return LV_RES_OK;
}
void lvglInitEx(){
    static const char * btnMap[] = {"Tune", "Ports", ""};

    lv_obj_t* mainMenuBtns = lv_btnm_create(lv_scr_act(), NULL);
    lv_btnm_set_map(mainMenuBtns, btnMap);
    lv_btnm_set_action(mainMenuBtns, menuBtnsEventHandler);




}



