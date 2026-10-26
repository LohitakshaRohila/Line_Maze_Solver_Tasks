#include <Arduino.h>

// We use if/else statements to clamp the values of left and right motor power, we could have also used the constrain() function.

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
