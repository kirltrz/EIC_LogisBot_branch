#include "motorEx.h"
#include <Arduino.h>
#include "motor.h"
#include "myTaskManager.h"
#include "globalValue.h"
#include "LED.h"
#define MAX_CALI_TIMES 5000

struct moveData
{
    int direction;
    float angle;
    float speed;
    int acc;
} moveData;

bool onMove = false;
bool motorEnable[4] = {0, 0, 0, 0};
bool motorDirection[4] = {0, 0, 0, 0};

void moveTask(void *ptr)
{

    switch (moveData.direction)
    {
    case 0:
        motorEnable[0] = 1;
        motorEnable[1] = 0;
        motorEnable[2] = 1;
        motorEnable[3] = 0;
        motorDirection[0] = 1;
        motorDirection[1] = 0;
        motorDirection[2] = 0;
        motorDirection[3] = 0;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 1;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 1;
        break;
    case 1:
        motorEnable[0] = 1;
        motorEnable[1] = 1;
        motorEnable[2] = 1;
        motorEnable[3] = 1;
        motorDirection[0] = 1;
        motorDirection[1] = 1;
        motorDirection[2] = 0;
        motorDirection[3] = 0;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 0;
        break;
    case 2:
        motorEnable[0] = 0;
        motorEnable[1] = 1;
        motorEnable[2] = 0;
        motorEnable[3] = 1;
        motorDirection[0] = 0;
        motorDirection[1] = 1;
        motorDirection[2] = 0;
        motorDirection[3] = 0;

        YawPIDmotor[0] = 1;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 1;
        YawPIDmotor[3] = 0;
        break;
    case 3:
        motorEnable[0] = 1;
        motorEnable[1] = 1;
        motorEnable[2] = 1;
        motorEnable[3] = 1;
        motorDirection[0] = 0;
        motorDirection[1] = 1;
        motorDirection[2] = 1;
        motorDirection[3] = 0;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 0;
        break;
    case 4:
        motorEnable[0] = 1;
        motorEnable[1] = 0;
        motorEnable[2] = 1;
        motorEnable[3] = 0;
        motorDirection[0] = 0;
        motorDirection[1] = 0;
        motorDirection[2] = 1;
        motorDirection[3] = 0;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 1;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 1;
        break;
    case 5:
        motorEnable[0] = 1;
        motorEnable[1] = 1;
        motorEnable[2] = 1;
        motorEnable[3] = 1;
        motorDirection[0] = 0;
        motorDirection[1] = 0;
        motorDirection[2] = 1;
        motorDirection[3] = 1;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 0;
        break;
    case 6:
        motorEnable[0] = 0;
        motorEnable[1] = 1;
        motorEnable[2] = 0;
        motorEnable[3] = 1;
        motorDirection[0] = 0;
        motorDirection[1] = 0;
        motorDirection[2] = 0;
        motorDirection[3] = 1;

        YawPIDmotor[0] = 1;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 1;
        YawPIDmotor[3] = 0;
        break;
    case 7:
        motorEnable[0] = 1;
        motorEnable[1] = 1;
        motorEnable[2] = 1;
        motorEnable[3] = 1;
        motorDirection[0] = 1;
        motorDirection[1] = 0;
        motorDirection[2] = 0;
        motorDirection[3] = 1;

        YawPIDmotor[0] = 0;
        YawPIDmotor[1] = 0;
        YawPIDmotor[2] = 0;
        YawPIDmotor[3] = 0;
        break;
    default:
        break;
    }
    unsigned long startTime = millis();
    float currentSpeed = 0.0f;
    // 匀加速
    while (currentSpeed < moveData.speed)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        currentSpeed = moveData.acc * (millis() - startTime) * 0.001f;
        for (int i = 0; i < 4; i++)
        {
            motorWrite(i, 2, motorDirection[i], 100, motorEnable[i] ? currentSpeed * DEG_TO_RAD : 0);
        }
    }
    // 匀速
    float maxSpeedAngle = moveData.angle - (((millis() - startTime) * 0.001f * moveData.speed) / 2) * 2;
    for (int i = 0; i < 4; i++)
    {
        motorWrite(i, 2, motorDirection[i], maxSpeedAngle, motorEnable[i] ? moveData.speed * DEG_TO_RAD : 0);
    }
    vTaskDelay(((maxSpeedAngle / moveData.speed) * 1000.0f) / portTICK_PERIOD_MS);
    // 匀减速
    startTime = millis();
    while (currentSpeed > 0)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        currentSpeed = moveData.speed - moveData.acc * (millis() - startTime) * 0.001f;
        if (currentSpeed < 0)
            currentSpeed = 0;
        for (int i = 0; i < 4; i++)
        {
            motorWrite(i, 2, motorDirection[i], 100, motorEnable[i] ? currentSpeed * DEG_TO_RAD : 0);
        }
    }
    onMove = false;

    YawPIDmotor[0] = 1;
    YawPIDmotor[1] = 1;
    YawPIDmotor[2] = 1;
    YawPIDmotor[3] = 1;

    vTaskDelete(NULL);
}

void move(int direction, float distance, float speed, int acc)
{
    while (onMove)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    onMove = true;
    moveData.direction = direction;
    moveData.angle /*度*/ = (distance / (PI * 0.085f)) * 360.0f;
    moveData.speed /*度/s*/ = speed;
    moveData.acc /*度/s^2*/ = acc;
    xTaskCreate(
        moveTask,
        "move task",
        4096,
        NULL,
        3,
        NULL);
}

void rotateTask(void *ptr)
{
    YawPIDmotor[0] = 0;
    YawPIDmotor[1] = 0;
    YawPIDmotor[2] = 0;
    YawPIDmotor[3] = 0;

    unsigned long startTime = millis();
    float currentSpeed = 0.0f;
    while (currentSpeed < moveData.speed)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        currentSpeed = moveData.acc * (millis() - startTime) * 0.001f;
        for (int i = 0; i < 4; i++)
        {
            motorWrite(i, 2, moveData.direction, 100, currentSpeed * DEG_TO_RAD);
        }
    }
    float maxSpeedAngle = moveData.angle - (((millis() - startTime) * 0.001f * moveData.speed) / 2) * 2;
    for (int i = 0; i < 4; i++)
    {
        motorWrite(i, 2, moveData.direction, maxSpeedAngle, moveData.speed * DEG_TO_RAD);
    }
    vTaskDelay(((maxSpeedAngle / moveData.speed) * 1000.0f) / portTICK_PERIOD_MS);
    startTime = millis();
    while (currentSpeed > 0)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        currentSpeed = moveData.speed - moveData.acc * (millis() - startTime) * 0.001f;
        if (currentSpeed < 0)
            currentSpeed = 0;
        for (int i = 0; i < 4; i++)
        {
            motorWrite(i, 2, moveData.direction, 100, currentSpeed * DEG_TO_RAD);
        }
    }

    YawPIDmotor[0] = 1;
    YawPIDmotor[1] = 1;
    YawPIDmotor[2] = 1;
    YawPIDmotor[3] = 1;
    while (abs(currentYaw - targetYaw) > deadband)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    onMove = false;
    vTaskDelete(NULL);
}
void rotate(bool direction, float angle, float speed, int acc)
{
    while (onMove)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    onMove = true;
    moveData.direction = direction;
    moveData.angle /*度*/ = angle * 2.6f;
    moveData.speed /*度/s*/ = speed;
    moveData.acc /*度/s^2*/ = acc;
    xTaskCreate(
        rotateTask,
        "rotate task",
        4096,
        NULL,
        3,
        NULL);
}
void caliPos(int targetPosX, int targetPosY, bool flag)
{
    int caliTimes = 0;
    int caliSpeed = 3;
    int caliDeadZone = 7;
    int offsetX = targetPosX - rpi.data1;
    int offsetY = targetPosY - rpi.data2;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uniStop = true;
    //LED(1);
    motorWrite(0, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(1, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(2, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(3, MOTOR_MODE_POSITION, 0, 0, 0);
    while (abs(offsetX) > caliDeadZone || abs(offsetY) > caliDeadZone)
    {
        caliTimes++;
        if (caliTimes > MAX_CALI_TIMES)
            break;
        // 校准车身位置
        if (abs(offsetX) > caliDeadZone && rpi.data1 != 0 && rpi.data2 != 0)
        {
            motorWrite(flag ? 1 : 0, MOTOR_MODE_POSITION, offsetX > 0 ? 0 : 1, abs(offsetX) * 0.075, caliSpeed);
            motorWrite(flag ? 3 : 2, MOTOR_MODE_POSITION, offsetX > 0 ? 1 : 0, abs(offsetX) * 0.075, caliSpeed);
        }
        if (abs(offsetY) > caliDeadZone && rpi.data1 != 0 && rpi.data2 != 0)
        {
            motorWrite(flag ? 0 : 3, MOTOR_MODE_POSITION, offsetY > 0 ? 1 : 0, abs(offsetY) * 0.075, caliSpeed);
            motorWrite(flag ? 2 : 1, MOTOR_MODE_POSITION, offsetY > 0 ? 0 : 1, abs(offsetY) * 0.075, caliSpeed);
        }
        vTaskDelay(((max(abs(offsetX), abs(offsetY)) / caliSpeed) * 11) / portTICK_PERIOD_MS);
        offsetX = targetPosX - rpi.data1;
        offsetY = targetPosY - rpi.data2;
    }
    if (abs(offsetX) > caliDeadZone && rpi.data1 != 0 && rpi.data2 != 0)
    {
        motorWrite(flag ? 1 : 0, MOTOR_MODE_POSITION, offsetX > 0 ? 0 : 1, abs(offsetX) * 0.075, caliSpeed);
        motorWrite(flag ? 3 : 2, MOTOR_MODE_POSITION, offsetX > 0 ? 1 : 0, abs(offsetX) * 0.075, caliSpeed);
    }
    if (abs(offsetY) > caliDeadZone && rpi.data1 != 0 && rpi.data2 != 0)
    {
        motorWrite(flag ? 0 : 3, MOTOR_MODE_POSITION, offsetY > 0 ? 1 : 0, abs(offsetY) * 0.075, caliSpeed);
        motorWrite(flag ? 2 : 1, MOTOR_MODE_POSITION, offsetY > 0 ? 0 : 1, abs(offsetY) * 0.075, caliSpeed);
    }
    vTaskDelay(((max(abs(offsetX), abs(offsetY)) / caliSpeed) * 11) / portTICK_PERIOD_MS);

    motorWrite(0, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(1, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(2, MOTOR_MODE_POSITION, 0, 0, 0);
    motorWrite(3, MOTOR_MODE_POSITION, 0, 0, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uniStop = false;
    //LED(false);
}