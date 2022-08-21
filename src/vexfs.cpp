#include "vexfs.h"
#include "errno.h"
#include "unistd.h"


#define SETTINGSPATH "/usd/code_settings.txt"



void fileSysInit(){
    FILE* file = fopen(SETTINGSPATH, "r");
    if(file != NULL){
        printf("code_settings.txt file is present :)\n");
        fclose(file);
        return;
    }
    // perror("access error");
    printf("code_settings.txt does not exist :(\n");
    printf("building code_settings.txt file ...\n");

    settings defaultSettings;
    
    defaultSettings.faceHeading_kp = 50;
    defaultSettings.faceHeading_ki = 0;
    defaultSettings.faceHeading_kd = 0;

    defaultSettings.goToPos_kp = 100;
    defaultSettings.goToPos_ki = 0;
    defaultSettings.goToPos_kd = 0;

    file = fopen(SETTINGSPATH, "w");
    fprintf(file, "ayee it created me\n");
    fclose(file);
    printf("code_settings.txt was built succesfully :)\n");
}
settings getSettings(){
    settings settingsContents;
    FILE* file = fopen(SETTINGSPATH, "r");

    fclose(file);
    return settingsContents;
}