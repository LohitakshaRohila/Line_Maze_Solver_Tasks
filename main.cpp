#include <Arduino.h>

#define S1 32
#define S2 33
#define S3 25
#define S4 26
#define S5 27

void setup()
{
    Serial.begin(115200);

    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);
}

float Line_Centroid(int P1, int P2, int P3, int P4, int P5)
{
    int R1 = digitalRead(P1);
    int R2 = digitalRead(P2);
    int R3 = digitalRead(P3);
    int R4 = digitalRead(P4);
    int R5 = digitalRead(P5);

    int den = R1 + R2 + R3 + R4 + R5;

    if (den == 0)
    {
        return 999.9;
    }

    return float((-2 * R1) - R2 + R4 + (2 * R5)) / den;
}

void loop()
{
    float centroid = Line_Centroid(S1, S2, S3, S4, S5);

    if (centroid == 999.9)
    {
        Serial.println("Line lost");
    }
    else if (centroid == 0)
    {
        Serial.println("Center");
    }
    else if (centroid < 0)
    {
        Serial.println("Left");
    }
    else
    {
        Serial.println("Right");
    }

    delay(100);
}