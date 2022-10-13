#include "vexfs.h"
#include "errno.h"
#include "unistd.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include "jsonhpp/json.h"




#define SETTINGSPATH "/usd/code_settings.json"



void fileSysInit(){
    FILE* file = fopen(SETTINGSPATH, "r");
    if(file != NULL){
        printf("code_settings.json file is present :)\n");
        fclose(file);
        return;
    }
    // perror("access error");
    printf("code_settings.json does not exist :(\n");
    printf("building code_settings.json file ...\n");

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

    defaultSettings.isGpsAvaiable = true;
    defaultSettings.isOnBlueSide = true;

    defaultSettings.isInertial = false;

    defaultSettings.rpmOptionOne = 200;
    defaultSettings.rpmOptionTwo = 300;
    defaultSettings.rpmOptionThree = 400;

    defaultSettings.RHO = 0;

    convertedSettings["faceHeading_kp"] = defaultSettings.faceHeading_kp;
    convertedSettings["faceHeading_ki"] = defaultSettings.faceHeading_ki;
    convertedSettings["faceHeading_kd"] = defaultSettings.faceHeading_kd;

    convertedSettings["goToPos_kp"] = defaultSettings.goToPos_kp;
    convertedSettings["goToPos_ki"] = defaultSettings.goToPos_ki;
    convertedSettings["goToPos_kd"] = defaultSettings.goToPos_kd;

    convertedSettings["isGpsAvaiable"] = defaultSettings.isGpsAvaiable;
    convertedSettings["isOnBlueSide"] = defaultSettings.isOnBlueSide;
    
    convertedSettings["isInertial"] = defaultSettings.isInertial;

    convertedSettings["rpmOptionOne"] = defaultSettings.rpmOptionOne;
    convertedSettings["rpmOptionTwo"] = defaultSettings.rpmOptionTwo;
    convertedSettings["rpmOptionThree"] = defaultSettings.rpmOptionThree;
    convertedSettings["RHO"] = defaultSettings.RHO;




    const std::string jsonFile = writer.write(convertedSettings);


    

    //Saves writes the json settings and closes the file
    file = fopen(SETTINGSPATH, "w");
    fprintf(file, "%s", jsonFile.c_str());
    fclose(file);
    printf("code_settings.json was built succesfully :)\n");
}
settings getSettings(){
    settings settingsContents;
    char* fileData = (char*)std::malloc(5000); //Allocates 5kb of memory to char* which would also the maximum json file size the method can read
    Json::Value parsedData;
    FILE* file = fopen(SETTINGSPATH, "r");

    fscanf(file, "%s", fileData);
    fclose(file);

    Json::Reader reader;
    
    reader.parse(fileData, parsedData);
    free(fileData);
    
    settingsContents.faceHeading_kp = parsedData["faceHeading_kp"].asFloat();
    settingsContents.faceHeading_ki = parsedData["faceHeading_ki"].asFloat();
    settingsContents.faceHeading_kd = parsedData["faceHeading_kd"].asFloat();
    settingsContents.goToPos_kp = parsedData["goToPos_kp"].asFloat();
    settingsContents.goToPos_ki = parsedData["goToPos_ki"].asFloat();
    settingsContents.goToPos_kd = parsedData["goToPos_kd"].asFloat();
    settingsContents.isGpsAvaiable = parsedData["isGpsAvaiable"].asBool();
    settingsContents.isOnBlueSide = parsedData["isOnBlueSide"].asBool();
    settingsContents.isInertial = parsedData["isInertial"].asBool();
    settingsContents.rpmOptionOne = parsedData["rpmOptionOne"].asInt();
    settingsContents.rpmOptionTwo = parsedData["rpmOptionTwo"].asInt();
    settingsContents.rpmOptionThree = parsedData["rpmOptionThree"].asInt();
    settingsContents.RHO = parsedData["RHO"].asDouble();
    
    
    
    return settingsContents;
}



void writeSettings(settings tempSettings){
    Json::Value convertedSettings;
    Json::FastWriter writer;

    convertedSettings["faceHeading_kp"] = tempSettings.faceHeading_kp;
    convertedSettings["faceHeading_ki"] = tempSettings.faceHeading_ki;
    convertedSettings["faceHeading_kd"] = tempSettings.faceHeading_kd;
    convertedSettings["goToPos_kp"] = tempSettings.goToPos_kp;
    convertedSettings["goToPos_ki"] = tempSettings.goToPos_ki;
    convertedSettings["goToPos_kd"] = tempSettings.goToPos_kd;
    convertedSettings["isGpsAvaiable"] = tempSettings.isGpsAvaiable;
    convertedSettings["isOnBlueSide"] = tempSettings.isOnBlueSide;

    convertedSettings["isInertial"] = tempSettings.isInertial;

    convertedSettings["rpmOptionOne"] = tempSettings.rpmOptionOne;
    convertedSettings["rpmOptionTwo"] = tempSettings.rpmOptionTwo;
    convertedSettings["rpmOptionThree"] = tempSettings.rpmOptionThree;
    convertedSettings["RHO"] = tempSettings.RHO;

    const std::string jsonFile = writer.write(convertedSettings);

    FILE* file = fopen(SETTINGSPATH, "w");

    fprintf(file, "%s", jsonFile.c_str());

    fclose(file);

    printf("Settings have been saved succesfully! :)\n");
}