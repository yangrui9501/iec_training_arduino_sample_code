// 2023-02-06
// 範例：digitalRead() and digitalWrite()

#include <Arduino.h>

#define PIN_BUTTON_1 12 // 上拉電阻型按鈕，預設是 1，按下變 0
#define PIN_BUTTON_2 11 // 下拉電阻型按鈕，預設是 0，按下變 1

void setup()
{
    // 初始化序列埠 
    Serial.begin(115200);

    // 腳位初始化
    pinMode(PIN_BUTTON_1, INPUT);
    pinMode(PIN_BUTTON_2, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // (習慣上) 輸出腳位會預設是最低值，避免記憶體裡面的殘值，讓硬體暴衝
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    // 利用 digitalRead() 讀取腳位
    uint8_t button_1 = digitalRead(PIN_BUTTON_1);
    uint8_t button_2 = digitalRead(PIN_BUTTON_2);

    // 利用 digitalWrite() 寫入腳位
    digitalWrite(LED_BUILTIN, button_2);

    // Print 資料
    Serial.print(button_1);
    Serial.print(" ");
    Serial.print(button_2);
    Serial.print(" ");
    Serial.println();
    Serial.flush();
}