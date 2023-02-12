// 2023-02-06
// 範例：analogRead()

#include <Arduino.h>

#define PIN_VR A1 // 可變電阻分壓輸入
#define CONFIG_ANALOG_READ_RES 10 // 解析度是 10，代表值的範圍是 0 ~ 1023 (2^10)，0 對應到 0 V，1023 對應 3.3 V。

void setup()
{
    // 初始化序列埠 
    Serial.begin(115200);

    // 腳位初始化與設定解析度
    pinMode(PIN_VR, INPUT);
    analogReadResolution(CONFIG_ANALOG_READ_RES);
}

void loop()
{
    // 利用 analogRead() 讀取可變電阻讀值
    int voltage = analogRead(PIN_VR);

    // Print 資料
    // Serial.print("可變電阻分壓： ");
    Serial.print(voltage);
    Serial.print(" ");
    Serial.print(1.123);
    Serial.println();
    Serial.flush();

    delay(1);
}