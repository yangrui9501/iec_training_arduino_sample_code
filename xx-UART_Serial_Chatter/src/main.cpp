// 2022-06-07 Author: Yang-Rui Li

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