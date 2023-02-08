// 2023-02-06
// 範例：analogWrite() and angloagRead()

#include <Arduino.h>

#define PIN_VR A1 // 輸入
#define PIN_LED A9 // 輸出

#define CONFIG_ANALOG_READ_RES 10 // 解析度是 10，代表值的範圍是 0 ~ 1023 (2^10)，0 對應到 0 V，1023 對應 3.3 V。
#define CONFIG_ANALOG_WRITE_RES 10 // 同上邏輯
#define CONFIG_ANALOG_WRITE_FREQ_HZ 10000 // 寫入 PWM 的頻率是 10 Hz

void setup()
{
    // 初始化序列埠 
    Serial.begin(115200);

    // 腳位初始化與設定解析度
    pinMode(PIN_VR, INPUT);
    analogReadResolution(CONFIG_ANALOG_READ_RES);

    pinMode(PIN_LED, OUTPUT);
    analogWriteResolution(CONFIG_ANALOG_WRITE_RES);
    analogWriteFrequency(PIN_LED, CONFIG_ANALOG_WRITE_FREQ_HZ);

    // (習慣上) 輸出腳位會預設是最低值，避免記憶體裡面的殘值，讓硬體暴衝
    analogWrite(PIN_LED, 0);
}

void loop()
{
    // 利用 analogRead() 讀取可變電阻讀值
    int vr_val = analogRead(PIN_VR);

    // 利用 analogWrite() 控制 LED 亮度
    analogWrite(PIN_LED, vr_val);

    // Print 資料
    Serial.print("可變電阻讀值: ");
    Serial.print(vr_val);
    Serial.println();
}