// 2023-02-06
// 範例：IntervalTimer

#include <Arduino.h>

#define ISR_PERIOD_MICROS 1000000

IntervalTimer isr_timer;
double t = 0.0;
double Ts = (double)(ISR_PERIOD_MICROS) / 1000000;
int idx = 0;

// 宣告時間中斷服務函式的原型
void isr();

void setup()
{
    // 初始化序列埠
    Serial.begin(115200);

    // 腳位初始化
    pinMode(LED_BUILTIN, OUTPUT);

    // 時間中斷初始化
    isr_timer.begin(isr, ISR_PERIOD_MICROS); // 設定經過 ISR_PERIOD_MICROS 時間執行 isr()
    isr_timer.priority(255); // 0 ~ 255 預設 128，255 是最低
}

void loop()
{
}

// 時間中斷函式的實作
void isr()
{
    idx++;
    t = (double)(idx)*Ts;

    Serial.print(t);
    Serial.println();
    Serial.flush();
}