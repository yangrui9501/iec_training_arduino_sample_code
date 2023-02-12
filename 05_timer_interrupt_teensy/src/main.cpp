// 2023-02-06
// 範例：IntervalTimer (Teensyduino)

#include <Arduino.h>

#define CONFIG_TIMER_PERIOD_MICROS 1000000

IntervalTimer timer;
double t = 0.0; // 絕對時間 (以第一次進入 ISR 的瞬間作為時間原點)
double Ts = (double)(CONFIG_TIMER_PERIOD_MICROS) / 1000000; // 內部中斷時間間隔 (秒)
int idx = 0; // 內部中斷函數執行幾次的計數器

unsigned long t_duration = 0; // 用來夾兩次執行 ISR 的時間間隔 (確認用)
unsigned long t_enter = 0;

// 宣告時間中斷服務函式的原型
void isr();

void setup()
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    timer.begin(isr, CONFIG_TIMER_PERIOD_MICROS);
    timer.priority(255);
}

void loop()
{
}

// 時間中斷函式的實作
void isr()
{
    idx++;
    t = (double)(idx)*Ts;

    t_duration = micros() - t_enter;
    t_enter = micros();

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    Serial.print(t);
    Serial.print(" ");
    Serial.print(t_duration);
    Serial.print(" ");
    Serial.println();
    Serial.flush();
}