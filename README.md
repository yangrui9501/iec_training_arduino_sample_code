# Arduino 基礎教育訓練

###### tags: `Arduino` `IEC Training`

## 環境建置

要安裝的東西列表如下：

- VS Code 
- PlatformIO 
    - Platform: Teensy
    - Platform: ESP32
    - Platform: ESP8266
- SerialPlot
- Arduino IDE (optional)
- 可以參考的資源
    - [PlatformIO_Installation_Guide](https://github.com/Intelligent-Embedded-Control-Laboratory/PlatformIO_Installation_Guide.git)
    - [SerialPlot（序列埠繪圖家）工具軟體（二）：安裝與執行SerialPlot](https://swf.com.tw/?p=1591&fbclid=IwAR1NVpCXGTSNsDdeE7JW4qH8onToeQqVxaSSz722NUzTz_RuzmcZxCWVymY)

安裝完成後，可以在 `src/main.cpp` 放入以下 hello world 測試 code：

```cpp=
// 2023-02-03
#include <Arduino.h>

String s = "Hello World! Hell PlatformIO!";

void print_string(const String &_string) { Serial.println(_string); }

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
}

void loop()
{
    print_string(s);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
}
```

如果能成功燒錄，那最後應該會看到 LED 燈閃爍，打開序列埠會一直列印文字。

## 簡介

- 嵌入式在做什麼? 簡單的說，就是把一支程式，燒錄到單晶片或者其他運算單元裡面。而這支程式包含你要做的所有事情，可以是控制馬達，控制飛機，或者任何。
- 所以接下來就是要教你怎麼樣在 Arduino 架構之下，寫這支程式，達成你想做的事情。
- MCU 的最基礎構造：核心、腳位、外設
    - 核心運算單元與時脈
    - 通用目的輸入腳位 (General Purpose Input/Ouput, GPIO)
        - 數位輸入/輸出: `digitalRead()`, `digitalWrite()`
        - 類比輸入/輸出: `analogRead()`, `analogWrite()`
    - 通訊模組
        - 通用非同步收發傳輸器 (Universal Asynchronous Receiver/Transmitter, UART)
        - 序列外設介面 (Serial Peripheral Interface Bus, SPI)
        - 積體匯流排電路 (Inter-Integrated Circuit, I²C)
- Arduino 程式的基本構造：

```cpp=
#include <Arduino.h>

void setup()
{
    // 這裡放只會執行一次的程式。例如：腳位初始化、變數初始化、或者一些硬體安全檢查。
}

void loop()
{
    // 這裡會放不斷被重複執行的程式。這個程式是被放在一個 while(1) 迴圈裏面被呼叫。
}
```

&emsp; 事實上，以 Teensyduino 的 Arduino 核心來說，main 檔如下： 

```cpp=
#include <Arduino.h>

extern "C" int main(void)
{
#ifdef USING_MAKEFILE

    // To use Teensy 4.0 without Arduino, simply put your code here.
    // For example:

    pinMode(13, OUTPUT);
    while (1) {
        digitalWriteFast(13, HIGH);
        delay(500);
        digitalWriteFast(13, LOW);
        delay(500);
    }


    #else
    // Arduino's main() function just calls setup() and loop()....
    setup();
    while (1) {
        loop();
        yield();
    }
#endif
}
```

## 序列埠溝通 (UART)

### 相關 API 說明

:::warning
- [Serial.begin(speed), Serial.begin(speed, config)](https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/)
    - 設定 Serial 的 baud rate (`speed`)，`config` 是序列埠傳輸的一些協議，預設是 `SERIAL_8N1`，沒有特別需求使用時可以可以忽略這個引數。
- [Serial.print(val), Serial.print(val, format)](https://www.arduino.cc/reference/en/language/functions/communication/serial/print/)
    - 將資料轉換成 ASCII 的文字，列印至序列埠。
    - 舉例來說，`Serial.print("Hello");` 就是寫入 `"Hello"`
    - `Serial.print(1.234, 2);` 則是寫入 `"1.23"`
- [Serial.write(val), Serial.write(str), Serial.write(buf, len)](https://www.arduino.cc/reference/en/language/functions/communication/serial/write/)
    - 將資料的 binary 寫入至序列埠。
    - 舉例來說，如果寫入 Serial.write()
- `Serial.read()`
    * 從 Serial 讀取一個 byte，回傳型態是整數。
    * 所以如果要接收字元資料，如果直接用 int 去接會獲得的是，這個字元根據 ASCII 編碼所對應的整數。如果想獲得字元資料，請用 char 型態去接。
- Serial.available()
- Serial.println()
- Serial.flush()
:::

其它資料：
- [ASCII Table](https://zh.wikipedia.org/wiki/ASCII)


### 原理說明

- 在進行序列埠傳輸的時候，都是傳 binary。

### 實作：列印資料

```cpp=
// 2023-02-06
// 範例：Serial.print(), Serial.begin(), Serial.println(), Serial.flush()
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    double val = 1.2345;
    Serial.print(val, 2);
    Serial.print(" ");
    Serial.print(val, 4);
    Serial.println();
    Serial.flush();

    delay(1000);
}
```

### 實作 (稍微進階一點)

```cpp=
// 2022-06-07
#include <Arduino.h>

#define BUF_SIZE 1024

const char *msg = "哈囉";
const char *response = "好";

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    static int i;                       // Counter of serial data
    static unsigned char buf[BUF_SIZE]; // Buffer of serial data

    for (i = 0; Serial.available() > 0; i++)
    {
        buf[i] = Serial.read(); // Read one byte from serial
    }

    // Print data
    if (i > 0)
    {
        if (i >= 6)
        {
            if (strcmp((char *)buf, msg) == 0)
            {
                Serial.print("OK");
            }
            else
            {
                Serial.print("??");
            }
            Serial.println();
        }

        Serial.print("Received " + String(i) + " Bytes: ");
        Serial.write((char *)buf, i); // Write i bytes to serial (buf[0], buf[1], ..., buf[i] <-- i+1 binary array data)
        Serial.println();
        memset(buf, 0, i); // Reset buffer
    }

    delay(1);
}
```

### 補充：`Serial.print()` 跟 `Serial.write()` 的差別?

:::success
- `Serial.print()` 會把資料傳換成字元，再根據 ASCII 表格找到對應的整數，然後把這個整數所對應的 binary 傳送出去。
* 例如：浮點數 `1.234` 則是會轉換成 `'1'`, `'.'`, `'2'`, `'3'`, `'4'` 這五個字元，根據 ASCII 表格，找到對應的整數會是 `49`, `46`, `50`, `51`, `52`。
- 接下來再把這些整數對應的 binary 傳送出去。要注意的是，一個整數雖然是 2 bytes / 4 bytes，但只會傳這個整數的第一個 byte，因為 ASCII 表格的值只有 0~127 共 128 個，也就是說 $2^7$ 就足以儲存這些資料。
- `Serial.write()` 則是會直接把資料對應的位元組傳送出去。以同樣的範例，若剛剛的浮點數 `1.234` 宣告成雙精度浮點數 `double`，則會佔有 8 bytes 的空間，而 `Serial.write()` 把這些 binary 傳送出去。
:::


## 數位腳位輸入輸出 (Digital IO)

### 相關 API 說明

:::warning
- [pinMode(pin, mode)](https://www.arduino.cc/reference/en/language/functions/digital-io/pinmode/)
    - 設定一個數位腳位 `pin` 的模式：輸入 (`INPUT`) 或輸出 (`OUTPUT`)
    - `pin`: the Arduino pin number to set the mode of.
    - `mode`: `INPUT`, `OUTPUT`, or `INPUT_PULLUP`.
- [digitalWrite(pin, value)](https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/) 
    - 對一個數位腳位 `pin` 寫入 `HIGH` 或 `LOW` 的 `value`。
    - `pin`: the Arduino pin number.
    - `value`: `HIGH` or `LOW`.
- [digitalRead(pin)](https://www.arduino.cc/reference/en/language/functions/digital-io/digitalread/) 
    - 讀一個數位腳位 `pin` 的值。
    - `pin`: the Arduino pin number you want to read.
:::

### 硬體介紹：上拉電阻與下拉電阻

- [電路圖](https://www.tinkercad.com/things/is0dTtxzg72-copy-of-switch-pull-up-amp-pull-down/editel?sharecode=zKJHoPkaN7cas7E8m7LCUM5VlTfSC2SA65QWkj5ELaA)

### 實作：按鈕、LED與數位腳位輸入輸出

硬體電路



範例程式碼
```cpp=
// 2023-02-06
// 範例：digitalRead() and digitalWrite()

#include <Arduino.h>

#define PIN_BUTTON_1 2 // 上拉電阻型按鈕，預設是 1，按下變 0
#define PIN_BUTTON_2 3 // 下拉電阻型按鈕，預設是 0，按下變 1

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
    Serial.print("Button 1: ");
    Serial.print(button_1);
    Serial.print(" | Button 2: ");
    Serial.print(button_2);
    Serial.println();
}
```

## 類比輸入輸出 (Analog IO)

### 相關 API 介紹

:::warning
- [analogRead(pin)](https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/)
    - 從 `pin` 讀取類比訊號
    - pin: the name of the analog input pin to read
- [analogReadResolution(bits)](https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/)
    - 設定類比輸入的解析度。
    - `bits`: determines the resolution (in bits) of the value returned by the analogRead() function. You can set this between 1 and 32. You can set resolutions higher than the supported 12 or 16 bits, but values returned by analogRead() will suffer approximation. See the note below for details.
- [analogWrite(pin, value)](https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/)
    - 寫 `value` (PWM duty cycle) 到 `pin`
    - `pin`: the Arduino pin to write to. Allowed data types: int.
    - `value`: the duty cycle: between 0 (always off) and 255 (always on). Allowed data types: int.
- [analogWriteResolution(bits)](https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogwriteresolution/)
    - 設定類比輸出的解析度
    - bits: determines the resolution (in bits) of the values used in the analogWrite() function. The value can range from 1 to 32. If you choose a resolution higher or lower than your board’s hardware capabilities, the value used in analogWrite() will be either truncated if it’s too high or padded with zeros if it’s too low. See the note below for details.
- analogWriteFrequency(pin, freq)
    - 設定寫入 PWM 的頻率
:::

### 脈衝寬度調變 (Pulse-Width Modulation, PWM)

- [介紹](https://zh.wikipedia.org/zh-tw/%E8%84%88%E8%A1%9D%E5%AF%AC%E5%BA%A6%E8%AA%BF%E8%AE%8A)
- [Basics of PWM (Pulse Width Modulation)](https://docs.arduino.cc/learn/microcontrollers/analog-output)

### 固定 duty cycle 看看 PWM 頻率對連續訊號的影響


### 實作：可變電阻與 RC 電路

```cpp=
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
```

## 中斷 (Interrupts)

所謂的中斷，指的是某些事件發生時，CPU 在執行期間被中斷，而進行「某些規定好的行為」。可以按照中斷的來源分為：

- 外部中斷 (external interrupt)：又稱為「硬體中斷」(hardware interrupt)，如其名，由硬體等外部設備所觸發。
- 內部中斷 (internal interrupt)：又稱為「時間中斷」、「計時器中斷」(timer interrput)，是根據 CPU 時脈，固定周期來觸發。

而所謂「某些規定好的行為」則是稱為「中斷服務函式」(interrupt service routine, ISR)，我們會把「中斷觸發時要進行的動作」放到這裡面。

對於外部中斷，各種晶片用法都差不多；但是內部中斷的話差異會比較大，以下介紹 Teensyduino 的時間中斷。

### 相關 API 介紹：外部中斷

:::warning
[attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
- 設定腳位 `pin` 為中斷腳位，觸發的模式是 `mode`，觸發時會執行 `ISR`。
- `digitalPinToInterrupt(pin)` 是把 `pin` (腳位)轉換成中斷編號，中斷編號是給晶片看的。
- `mode`: 觸發模式，有
    - `LOW` to trigger the interrupt whenever the pin is low,
    - `CHANGE` to trigger the interrupt whenever the pin changes value
    - `RISING` to trigger when the pin goes from low to high,
    - `FALLING` for when the pin goes from high to low.
- `ISR`: 即中斷服務常式，必須是以下的原型 (函數名稱 `hwit_isr` 可以改)

```cpp=
// The prototype of ISR executed by hardware interrupt (HWIT)
void isr_hwit()
{
    // Do something when the ISR is trigger. OuO
}
```
:::

### 實作：按鈕點亮 LED

```cpp=
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
```

### 相關 API 介紹：內部中斷

:::warning
請直接參考以下範例。
:::

### 實作：時間中斷 (Teensyduino)

```cpp=
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
```

## Arduino 專題實作：直流碳刷馬達的位置與速度控制

### 硬體簡介


## 基本程式庫與腳位定義

需要引入的程式庫

```cpp=
#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解編碼器位置
```

腳位定義
```cpp=
#define PIN_QEI 10       // 馬達的 Encoder 腳位
#define PIN_MOTOR_PWM_CW 22   // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23  // 馬達負轉的 PWM 腳位
#define PIN_VR_INPUT A7 // 輸入

#define CONFIG_MOTOR_PWM_WRITE_FREQ 17000 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define CONFIG_MOTOR_PWM_WRITE_RES 11     // 馬達 PWM 命令的解析度 (Resolution)
#define CONFIG_ISR_TIME_MICROS 1000       // 內部中斷的時間間隔
```

### 使用 LS7366R QEI 程式庫


首先，先到[這裡](https://github.com/Intelligent-Embedded-Control-Laboratory/LS7366R.git)，複製網址。

![](https://hackmd.io/_uploads/SkMnN01as.png)

接下來，打開 PlatformIO 專案的 `platformio.ini`，加入以下命令：

```bash
lib_deps = 
    https://github.com/Intelligent-Embedded-Control-Laboratory/LS7366R.git
```

加完之後可能會是這樣：

![](https://hackmd.io/_uploads/H1pt8CJpi.png)


PlatformIO 就會自動幫你各位安裝程式庫囉！非常方便。安裝好的程式庫會在 `.pio/libdeps/teensy31` 路徑之下。

### 積分器實作

Consider

$$
y(t) = \int_{0}^{t}\!f(t)\,dt
$$  

or

$$
\dot{y}(t) = f(t)
$$

Backward difference discretization

$$
\dot{y}(kT) \approx \frac{y(kT)-y(kT-T)}{T}
$$

which implies

$$ 
\begin{align}
y[k] &= y[k-1] + f[k] \times T \\[1em]
現在位置 &= 前一筆位置 + 速度 \times 時間
\end{align}
$$

Pseudo code: intergator with a sample interval of $T$

```cpp
// error = f(t)
i_term += error * T;
```

Pseudo code: Reseting the intergrator value

```cpp
i_term = 0.0;
```

### 速度解算

速度的定義：單位時間的速度變化量

$$
V(t) = \frac{dx(t)}{dt} \approx \frac{\Delta x}{\Delta t} \quad \left(速度 = \frac{位置的變化量}{所經過時間}\right)
$$

所以實作上在進行速度解算時，會有以下兩個方法：

- I. 固定時間檢查位置，計算位置增量，然後相除。常見於馬達的高精度定位控制。舉例來說，如果經過 1秒，發現馬達轉了 3 pulse，速度就是：

$$
v = \frac{3\,\,\mbox{pulse}}{1\,\,\mbox{sec}} = 3 \,\,\mbox{pulse/sec}
$$

- II. 經過固定距離，計算所經過的時間，然後相除。常見於風扇的速度控制。舉例來說，如果經過 1 pulse，量測需要 0.2 秒，速度就是：

$$
v = \frac{1\,\,\mbox{pulse}}{0.2\,\,\mbox{sec}} = 5 \,\,\mbox{pulse/sec}
$$

- Pseudo code (I)

```cpp 
long pos, pos_pre;
double vel, dt;

pos = get_position_pulse();
vel = (double)(pos - pos_pre) / dt; // pulse/time
pos_pre = pos;
```

- Pseudo code (II)

```cpp 
long t_duration, t_enter;
double speed;

// Executing the following routine (ISR) when passed 1 pulse distance
void isr_hwit_speed_decode()
{
    t_duration = get_current_time_sec() - t_enter;
    t_enter = get_current_time_sec();
    
    speed = 1.0/(double)(t_duration); // pulse/sec
}
```


### 參考程式庫

- [移動平均濾波器](https://github.com/yangrui9501/dsp_moving_average_filter.git)


## 參考資料

- [Arduino Language Reference](https://www.arduino.cc/reference/en/)

