// 2023-02-06
// 範例：attachInterrupt()

#include <Arduino.h>

#define PIN_BUTTON_1 12 // 上拉電阻型按鈕，預設是 1，按下變 0

int idx = 0; // 用來計數進去  isr_hwit_button_1() 的次數
bool flag = false; // 可以利用單次觸發的 flag 來進來某些事情

// 硬體中斷服務函式的宣告與定義
void isr_hwit_button_1()
{
    idx++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    flag = true;
}

void setup()
{
    // 初始化序列埠 
    Serial.begin(115200);

    // 腳位初始化
    pinMode(PIN_BUTTON_1, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // 硬體中斷初始化，PIN_BUTTON_1 這的腳位在電壓從 HIGH 變成 LOW 的時候會執行 isr_hwit_button_1()
    attachInterrupt(PIN_BUTTON_1, isr_hwit_button_1, FALLING);
}

void loop()
{
    Serial.print("觸發 HWIT_ISR 的次數：");
    Serial.print(idx);
    Serial.println();
    Serial.flush();

    if (!flag) // or if (flag)
    {
        /* do something */
    }
    delay(10);
}