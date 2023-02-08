#include <Arduino.h>
#include <LS7366R.h>

#define PIN_QEI_CS 10 

LS7366R qei;

void setup()
{
    Serial.begin(115200);

    qei.begin(PIN_QEI_CS);
}

void loop()
{
    long pulse = qei.read();

    Serial.print(pulse);
    Serial.println();
    Serial.flush();

    delay(1);
}