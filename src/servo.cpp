#include "servo.h"
#include "globalValue.h"

#define S_RXD 14
#define S_TXD 13

SMS_STS servo;

void initServo(void)
{
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    servo.pSerial = &Serial1;
}
bool checkServo(void)
{
    if (readServo(0) != -1 && readServo(1) != -1 && readServo(2) != -1 && readServo(3) != -1 && readServo(4) != -1)
    {
        servoReady = true;
        return true;
    }
    else
    {
        servoReady = false;
        return false;
    }
}
void stopAllServo(void)
{
    servo.EnableTorque(0, 0);
    servo.EnableTorque(1, 0);
    servo.EnableTorque(2, 0);
    servo.EnableTorque(3, 0);
    servo.EnableTorque(4, 0);
}

int writeServo(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
    while (servo.WritePosEx(ID, Position+temperOffset[ID], Speed, ACC) != 1) // 如果校验不通过则重新发送，防止跳过步骤
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return 0;
}

int regWriteServo(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
    while (servo.RegWritePosEx(ID, Position+temperOffset[ID], Speed, ACC) != 1)//如果校验不通过则重新发送，防止跳过步骤
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return 0;
}

int regWriteServoAction(u8 ID)
{
    return servo.RegWriteAction(ID);
}
void syncWriteServo(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
    return servo.SyncWritePosEx(ID, IDN, Position, Speed, ACC);
}
int readServo(int ID)
{
    return servo.ReadPos(ID);
}

void setClaw(int state)
{
    int temp;
    switch (state)
    {
    case 0:
        temp = 2300;
        break;
    case 1:
        temp = 2035;
        break;
    case 2:
        temp = 2400;
        break;
    default:
        break;
    }
    writeServo(4, temp, GLOBAL_SERVO_SPEED, GLOBAL_SERVO_ACC);
}