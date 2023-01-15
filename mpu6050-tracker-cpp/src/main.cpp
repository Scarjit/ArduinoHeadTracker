#include "Wire.h"
#include <MPU6050_light/src/MPU6050_light.h>
#include "ArduinoJson.h"

MPU6050 mpu(Wire);
unsigned long timer = 0;

void printSensorDataJson(StaticJsonDocument<128> data){
    String outputString;
    serializeJson(data, outputString);
    Serial.println(outputString);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);

    byte status = mpu.begin();

    StaticJsonDocument<48> initData;
    initData.createNestedObject("data");
    initData["status"] = "initialising";
    initData["data"]["mpu_begin_status"] = status;
    printSensorDataJson(initData);

    while (status!=0){

        StaticJsonDocument<48> errorData;
        errorData.createNestedObject("data");
        errorData["status"] = "error";
        errorData["data"]["mpu_begin_status"] = status;
        printSensorDataJson(errorData);
        delay(100);

    } // stop everything if we could not connect to MPU6050

    StaticJsonDocument<48> calibrateData;
    calibrateData.createNestedObject("data");
    calibrateData["status"] = "calibrating";
    printSensorDataJson(calibrateData);

    delay(1000);
    mpu.calcOffsets(true,true); // gyro and accelero

    StaticJsonDocument<48> readyData;
    readyData.createNestedObject("data");
    readyData["status"] = "ready";
    printSensorDataJson(readyData);
}
void loop() {
    mpu.update();

    if(millis() - timer > 1){ // print data every ms
        StaticJsonDocument<128> sensorDataJson;
        sensorDataJson.createNestedObject("data");
        // toggle LED to show that we are alive
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        sensorDataJson["status"] = "ready";

        sensorDataJson["data"]["temp"] = mpu.getTemp();
        sensorDataJson["data"]["accX"] = mpu.getAccX();
        sensorDataJson["data"]["accY"] = mpu.getAccY();
        sensorDataJson["data"]["accZ"] = mpu.getAccZ();
        sensorDataJson["data"]["gyroX"] = mpu.getGyroX();
        sensorDataJson["data"]["gyroY"] = mpu.getGyroY();
        sensorDataJson["data"]["gyroZ"] = mpu.getGyroZ();
        sensorDataJson["data"]["accAngleX"] = mpu.getAccAngleX();
        sensorDataJson["data"]["accAngleY"] = mpu.getAccAngleY();
        sensorDataJson["data"]["angleX"] = mpu.getAngleX();
        sensorDataJson["data"]["angleY"] = mpu.getAngleY();
        sensorDataJson["data"]["angleZ"] = mpu.getAngleZ();

        printSensorDataJson(sensorDataJson);
        timer = millis();
    }
}
