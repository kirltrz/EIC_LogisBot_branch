#include <Arduino.h>
#include <motor.h>

#define DEBUG 0

byte frame[LENGTH_OF_FRAME];
byte ID, MODE, DIRECTION, POS_H, POS_L, SPEED_H, SPEED_L, BCC;
int16_t currentSpd;
int32_t currentPos;
HardwareSerial *motorSerial;

void calcPOS(byte *mode, float data, byte *pos_h, byte *pos_l)
{ // 计算POS_H位和POS_L位
    int thisData;
    switch (*mode)
    {
    case MOTOR_MODE_SPEED:
        *pos_h = 0x00;
        *pos_l = 0x00;
        break;

    case MOTOR_MODE_FORCE:
        thisData = (int)data;
        // 确保值是正数并且不超过16位的范围
        if (thisData < 0)
            thisData = 0;
        if (thisData > 0xFFFF)
            thisData = 0xFFFF;

        // 将值右移8位得到高八位
        *pos_h = (thisData >> 8) & 0xFF;

        // 使用掩码获取低八位
        *pos_l = thisData & 0xFF;
        break;
    case MOTOR_MODE_POSITION:
    case MOTOR_MODE_ANGLE:
        data *= 10.0f;
        thisData = (int)data;
        // 确保值是正数并且不超过16位的范围
        if (thisData < 0)
            thisData = 0;
        if (thisData > 0xFFFF)
            thisData = 0xFFFF;

        // 将值右移8位得到高八位
        *pos_h = (thisData >> 8) & 0xFF;

        // 使用掩码获取低八位
        *pos_l = thisData & 0xFF;
        break;
    default:
        break;
    }
}

void calcSPD(float rawSpd, byte *spd_h, byte *spd_l)
{ // 计算SPEED_H位和SPEED_L位
    rawSpd *= 10.0f;
    int spd = (int)rawSpd;
    // 确保值是正数并且不超过16位的范围
    if (spd < 0)
        spd = 0;
    if (spd > 0xFFFF)
        spd = 0xFFFF;

    // 将值右移8位得到高八位
    *spd_h = (spd >> 8) & 0xFF;

    // 使用掩码获取低八位
    *spd_l = spd & 0xFF;
    // Serial.println(rawSpd);
    // Serial.println(spd);
}

void initMotor(void)
{
    Serial2.begin(115200, SERIAL_8N1, 10 /*RX*/, 9 /*TX*/); // 初始化串口2
    motorSerial = &Serial2;                                 // 绑定电机串口
}
void motorWrite(int id, int mode, int direc, float angle, float spd)
{
    ID = id & 0xFF;
    MODE = mode & 0xFF;
    DIRECTION = direc & 0xFF;

    calcPOS(&MODE, angle, &POS_H, &POS_L);
    calcSPD(spd, &SPEED_H, &SPEED_L);

    // 组合帧
    frame[0] = MOTOR_HEAD;
    frame[1] = ID;
    frame[2] = MODE;
    frame[3] = DIRECTION;
    frame[4] = MOTOR_SUBDIVISION;
    frame[5] = POS_H;
    frame[6] = POS_L;
    frame[7] = SPEED_H;
    frame[8] = SPEED_L;

    // 计算BCC
    BCC = 0x00;
    for (int i = 0; i < 9; i++)
    {
        BCC ^= frame[i];
    }

    frame[9] = BCC;
    frame[10] = MOTOR_END;

    // 发送帧
    motorSerial->flush();
    motorSerial->write(frame, LENGTH_OF_FRAME);
#if DEBUG
    Serial.println(" ");
    Serial.write(frame, 11); // 调试用
#endif
}

// 电机硬件问题，UART读取不可用
uint32_t motorRead(int id)
{

    uint32_t pos = 0;
    bool unavailable = false;
    byte motorReadFrame[11] = {0x7B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};
    motorReadFrame[1] = id;
    for (int i = 0; i < 9; i++)
    { // id不同所以需要动态计算BCC
        motorReadFrame[9] ^= motorReadFrame[i];
    }
    motorSerial->write(motorReadFrame, LENGTH_OF_FRAME);
    int count = 0;
    while (!motorSerial->available())
    {
        delay(5);
        count++;
        if (count > 10)
        {
            unavailable = true;
            break;
        }
    }
    Serial.println("read once");
    while (motorSerial->available() && !unavailable)
    {
        motorSerial->read();
        motorSerial->read();
        motorSerial->read();
        motorSerial->read(); // 暂时忽略掉用不到的字节
        uint8_t HH = motorSerial->read();
        uint8_t HL = motorSerial->read();
        uint8_t LH = motorSerial->read();
        uint8_t LL = motorSerial->read();
        pos = ((uint32_t)HH << 24) |
              ((uint32_t)HL << 16) |
              ((uint32_t)LH << 8) |
              (uint32_t)LL;
        motorSerial->read(); // 同上
    }
    return pos;
}