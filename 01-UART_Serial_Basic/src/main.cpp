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