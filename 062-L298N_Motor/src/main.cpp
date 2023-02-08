#include <Arduino.h>
#include <LS7366R.h>

#define PIN_QEI_CS 10 
#define PIN_MOTOR_PWM_CW 22   // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23  // 馬達負轉的 PWM 腳位

#define CONFIG_MOTOR_PWM_WRITE_FREQ 17000 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define CONFIG_MOTOR_PWM_WRITE_RES 11     // 馬達 PWM 命令的解析度 (Resolution)

LS7366R qei;

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_MOTOR_PWM_CW, OUTPUT);                                    // 設定 PIN_MOTOR_PWM_CW 腳位為輸出
    pinMode(PIN_MOTOR_PWM_CCW, OUTPUT);                                   // 設定 PIN_MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(PIN_MOTOR_PWM_CW, CONFIG_MOTOR_PWM_WRITE_FREQ);  // 設定 PIN_MOTOR_PWM_CW 腳位 PWM 的輸出頻率
    analogWriteFrequency(PIN_MOTOR_PWM_CCW, CONFIG_MOTOR_PWM_WRITE_FREQ); // 設定 PIN_MOTOR_PWM_CCW 腳位 PWM 的輸出頻率
    analogWriteResolution(CONFIG_MOTOR_PWM_WRITE_RES);                    // 設定所有 PWM 的解析度

    qei.begin(PIN_QEI_CS);
}

void loop()
{
    long pulse = qei.read();

    int pwm_cmd = 1023;

    // 馬達 PWM 命令更新
    if (pwm_cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, pwm_cmd);
        analogWrite(PIN_MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, 0);
        analogWrite(PIN_MOTOR_PWM_CCW, -pwm_cmd);
    }
    
    Serial.print(pulse);
    Serial.print(" ");
    Serial.print(pwm_cmd);
    Serial.println();
    Serial.flush();

    delay(1);
}