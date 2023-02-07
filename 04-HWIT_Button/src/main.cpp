// 2023-02-06
// 範例：attachInterrupt()

#include <Arduino.h>

#define PIN_BUTTON_1 2 // 上拉電阻型按鈕，預設是 1，按下變 0

int idx = 0;

// 宣告硬體中斷服務函式的原型
void isr_hwit_button_1();

void setup()
{
    // 初始化序列埠 
    Serial.begin(115200);

    // 腳位初始化
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_BUTTON_1, INPUT);
   
    // 硬體中斷初始化
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), isr_hwit_button_1, FALLING);
}

void loop()
{
    Serial.print("觸發 HWIT_ISR 的次數：");
    Serial.print(idx);
    Serial.println();
    Serial.flush();

    delay(1);
}

// 硬體中斷服務函式的實作
void isr_hwit_button_1()
{
    idx++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}