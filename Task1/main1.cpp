#include <Arduino.h>

void Clamp_Motor_Power(int &left, int &right, int max_power)
{
    if (left > max_power)
    {
        left = max_power;
    }
    else if (left < -max_power)
    {
        left = -max_power;
    }

    if (right > max_power)
    {
        right = max_power;
    }
    else if (right < -max_power)
    {
        right = -max_power;
    }
}