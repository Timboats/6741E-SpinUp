#include "vexfs.h"
#include "errno.h"
#include "unistd.h"
#include <cstdio>
#include <string>
#include "jsonhpp/json.h"




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
    Json::Value convertedSettings;
    Json::FastWriter writer;
    //Sets the default settings in the settings struct
    defaultSettings.faceHeading_kp = 50;
    defaultSettings.faceHeading_ki = 0;
    defaultSettings.faceHeading_kd = 0;

    defaultSettings.goToPos_kp = 100;
    defaultSettings.goToPos_ki = 0;
    defaultSettings.goToPos_kd = 0;

    convertedSettings["faceHeading_kp"] = defaultSettings.faceHeading_kp;
    convertedSettings["faceHeading_ki"] = defaultSettings.faceHeading_ki;
    convertedSettings["faceHeading_kd"] = defaultSettings.faceHeading_kd;
    convertedSettings["goToPos_kp"] = defaultSettings.goToPos_kp;
    convertedSettings["goToPos_ki"] = defaultSettings.goToPos_ki;
    convertedSettings["goToPos_kd"] = defaultSettings.goToPos_kd;

    const std::string jsonFile = writer.write(convertedSettings);


    

    //Saves writes the json settings and closes the file
    file = fopen(SETTINGSPATH, "w");
    fprintf(file, "%s", jsonFile.c_str());
    fclose(file);
    printf("code_settings.txt was built succesfully :)\n");
}
settings getSettings(){
    settings settingsContents;
    char* formattedData;
    Json::Value parsedData;
    FILE* file = fopen(SETTINGSPATH, "r");

    fscanf(file, "%s", formattedData);
    parsedData = formattedData;
    printf("%s", parsedData.asCString());

    // fscanf(file, "%s", num);

    // printf("%s", num);

    fclose(file);
    
    return settingsContents;
}



void writeSettings(settings tempSettings){
    FILE* file = fopen(SETTINGSPATH, "w");
    fprintf(file, "%f\n%f\n%f\n%f\n%f\n%f", tempSettings.goToPos_kp, tempSettings.goToPos_ki, tempSettings.goToPos_kd, tempSettings.faceHeading_kp, tempSettings.faceHeading_ki, tempSettings.faceHeading_kd);

    printf("Settings have been saved succesfully! :)\n");
    fclose(file);
}