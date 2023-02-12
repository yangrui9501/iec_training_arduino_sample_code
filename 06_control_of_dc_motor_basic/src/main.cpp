#include <Arduino.h>
#include <LS7366R.h>
#include <moving_average_filter.h>

#define PIN_MOTOR_PWM_CW 22 // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23  // 馬達負轉的 PWM 腳位
#define PIN_QEI_CS 10 // LS7366R (QEI) 的 CS 腳位

#define CONFIG_PWM_WRITE_RES 11 // 馬達 PWM 命令的解析度 (Resolution)
#define CONFIG_PWM_WRITE_FREQ 17000 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define CONFIG_TIMER_PERIOD_MICROS 1000  // 內部中斷的時間間隔

const int PWM_MAX = (int)(pow(2.0, CONFIG_PWM_WRITE_RES)) - 1;

IntervalTimer timer;
MovingAverageFilter filter;
LS7366R qei;

long pos = 0;
long pos_pre = 0;
double vel = 0.0;
double vel_f = 0.0;
double vel_d = 0.0;
double e_vel = 0.0;
double kp = 0.0, ki = 0.0, kd = 0.0;
double u = 0.0;
double i_term = 0.0;
double T = (double)(CONFIG_TIMER_PERIOD_MICROS) / 1000000.0;
int idx = 0;
double t = 0.0;

int pwm = 0;

unsigned long time_spend_isr = 0, time_spend_isr_begin = 0; // 用來夾執行完 ISR 所有程式所需要的時候，請務必確認這個時間會小於你設定的時間，否則將會發生中斷堆疊

// 函數宣告
void isr_control();
int sign(int u_in);
double sign(double u_in);
void motor_reset();

void setup()
{
    // 開啟序列埠傳輸，設定 baud rate 為 115200
    Serial.begin(115200);

    // 定義腳位的功能與設定解析度
    pinMode(PIN_MOTOR_PWM_CW, OUTPUT);
    pinMode(PIN_MOTOR_PWM_CCW, OUTPUT);
    analogWriteFrequency(PIN_MOTOR_PWM_CW, CONFIG_PWM_WRITE_FREQ);
    analogWriteFrequency(PIN_MOTOR_PWM_CCW, CONFIG_PWM_WRITE_FREQ);
    analogWriteResolution(CONFIG_PWM_WRITE_RES);

    // 將馬達正轉跟負轉的 PWM 腳位設定成 0，避免暴衝。
    motor_reset();

    // 初始化 QEI
    qei.begin(PIN_QEI_CS);

    // 初始化移動平均濾波器
    filter.init(10);

    // 初始化時間中斷函式
    timer.begin(isr_control, CONFIG_TIMER_PERIOD_MICROS);
}

void loop()
{
}

void isr_control()
{
    time_spend_isr_begin = micros();

    idx++;
    t = (double)(idx)*T;

    // Read Motor position
    pos = -qei.read();

    // Velocity Decode
    vel = (double)(pos - pos_pre) / T;
    pos_pre = pos;

    // Filter
    vel_f = filter.update(vel);

    // Velocity Controller
    // kp = 0.05;
    // ki = 0.01;
    // vel_d = 300000.0;
    // e_vel = vel_d - vel_f;

    // u = kp * e_vel + i_term;
    long pos_d = 30000*sign(sin(2.0*PI/6.0*t));
    // long pos_d = 60000;
    double e_pos = (double)(pos_d - pos);

    kp = 0.5;
    kd = 0.1;
    ki = 0.00;
    u = 1.0*(kp*e_pos - kd*vel_f + i_term);
    pwm = (int)(u);
    
    /* Saturation Protection */
    if (abs(pwm) > PWM_MAX)
    {
        pwm = PWM_MAX * sign(pwm);
    }
    else
    {   
        // i_term += ki*e_vel*T; // Velocity controller
        i_term += ki*e_pos*T; // Position controller
    }


    /* Update PWM command */
    if (pwm > 0)
    {
        analogWrite(PIN_MOTOR_PWM_CW, pwm);
        analogWrite(PIN_MOTOR_PWM_CCW, 0);
    }
    else
    {
        analogWrite(PIN_MOTOR_PWM_CW, 0);
        analogWrite(PIN_MOTOR_PWM_CCW, -pwm);
    }

    // Print data
    Serial.print(pos);
    Serial.print(" ");
    Serial.print(pos_d);
    Serial.print(" ");
    Serial.print(u);
    // Serial.print(vel);
    // Serial.print(" ");
    // Serial.print(vel_f);
    // Serial.print(" ");
    // Serial.print(vel_d);
    Serial.print(" ");
    Serial.print(time_spend_isr);
    Serial.print(" ");
    Serial.println();
    Serial.flush();

    time_spend_isr = micros() - time_spend_isr_begin;
}

int sign(int u_in)
{
    if (u_in > 0)
        return 1;
    else if (u_in < 0)
        return -1;
    return 0;
}

double sign(double u_in)
{
    if (u_in > 0.0)
        return 1.0;
    else if (u_in < 0.0)
        return -1.0;
    return 0.0;
}

void motor_reset()
{
    analogWrite(PIN_MOTOR_PWM_CW, 0);
    analogWrite(PIN_MOTOR_PWM_CCW, 0);
}