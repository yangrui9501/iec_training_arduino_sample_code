// 2023-02-06
// 範例：analogWrite()

#include <Arduino.h>

#define PIN_LED_PWM_WRITE A9          // 輸出
#define CONFIG_ANALOG_WRITE_RES 10    // 同上邏輯
#define CONFIG_ANALOG_WRITE_FREQ_HZ 1000 // 寫入 PWM 的頻率是 1000 Hz

const int PWM_MAX = (int)(pow(2.0, CONFIG_ANALOG_WRITE_RES)) - 1.0;
int led_brightness = 0;
bool status = true;

void setup()
{
    // 初始化序列埠
    Serial.begin(115200);

    // 腳位初始化與設定解析度
    pinMode(PIN_LED_PWM_WRITE, OUTPUT);
    analogWriteResolution(CONFIG_ANALOG_WRITE_RES);
    analogWriteFrequency(PIN_LED_PWM_WRITE, CONFIG_ANALOG_WRITE_FREQ_HZ);

    // (習慣上) 輸出腳位會預設是最低值，避免記憶體裡面的殘值讓硬體暴衝
    analogWrite(PIN_LED_PWM_WRITE, 0);
}

void loop()
{
    if (led_brightness == PWM_MAX)
    {
        status = false;
    }
    else if (led_brightness == 0)
    {
        status = true;
    }

    if (status)
    {
        led_brightness++;
    }
    else
    {
        led_brightness--;
    }

    // 利用 analogWrite() 控制 LED 亮度
    analogWrite(PIN_LED_PWM_WRITE, led_brightness);

    // Print data
    Serial.print(status);
    Serial.print(" ");
    Serial.print(led_brightness);
    Serial.print(" ");
    Serial.println();
    Serial.flush();

    delay(1);
}