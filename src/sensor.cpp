#include "sensor.h"
#include <Wire.h>
#include "globalValue.h"

void initSensor(void)
{
    Wire.begin(5, 4); // 初始化IIC，用于HWT101
    paw3395Init();
}
bool checkSensor(void)
{
    Wire.beginTransmission(80);
    delay(5);
    if (Wire.endTransmission() == 0)
    {
        hwt101Ready = true;
        return true;
    }
    else
    {
        hwt101Ready = false;
        return false;
    }
}