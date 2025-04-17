#include "sensorEx.h"
#include <Arduino.h>
#include "motor.h"
#include "globalValue.h"
#include "myTaskManager.h"
float adjustSpeed;
float adjustAngle;
float yawError;

void adjustYaw()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    yawError = currentYaw - targetYaw;
    xSemaphoreGive(mutex);

    if (fabs(yawError) > 10.0f)
    {
        adjustSpeed = 3.0f;
        adjustAngle = 3.0f;
    }
    else if (fabs(yawError) > 5.0f)
    {
        adjustSpeed = 1.5f;
        adjustAngle = 1.5f;
    }
    else
    {
        adjustSpeed = 1.0f;
        adjustAngle = 1.0f;
    }

    if (yawError > deadband)
    {
        for (int i = 0; i < 4; i++)
        {
            if (YawPIDmotor[i] != 0)
                motorWrite(i, MOTOR_MODE_POSITION, MOTOR_DIRECTION_CW, adjustAngle, adjustSpeed);
        }
    }
    else if (yawError < -deadband)
    {
        for (int i = 0; i < 4; i++)
        {
            if (YawPIDmotor[i] != 0)
                motorWrite(i, MOTOR_MODE_POSITION, MOTOR_DIRECTION_CCW, adjustAngle, adjustSpeed);
        }
    }
}
