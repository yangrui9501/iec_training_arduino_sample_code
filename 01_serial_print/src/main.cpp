// 2023-02-06
// 範例：Serial.print(), Serial.begin(), Serial.println(), Serial.flush()

#include <Arduino.h>

unsigned long t_enter;

void setup()
{
    Serial.begin(115200);

    t_enter = micros();
}

void loop()
{
    int a = 1;
    double b = 1.2345;

    if (micros() - t_enter >= 1000000)
    {
        t_enter = micros();
        
        Serial.print("Hello World! Hello Arduino!");
        Serial.print(" ");
        Serial.print(a);
        Serial.print(" ");
        Serial.print(b); // char '1', '.', '2', '3', '4', '5'
        Serial.print(" ");
        Serial.print(b, 3);
        Serial.println();
    }

    // delay(1000);
}