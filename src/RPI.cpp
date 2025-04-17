#include "RPI.h"
#include <Arduino.h>
#include "myTaskManager.h"
#include "globalValue.h"

RPI_t rpi = {-1, -1, -1, -1};

void initRPI(void)
{
    Serial.begin(115200);
}

void writeRPI(int mode)
{
    byte frame[9] = {0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};

    frame[1] = mode & 0xFF;
    for (int i = 0; i < 7; i++)
    {
        frame[7] ^= frame[i];
    }
    Serial.flush();
    Serial.write(frame, 9);
}

void readRPI()
{
    // 定义一个长度为9的字节数组，用于存储从串口读取的数据
    byte buffer[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 等待串口有数据可读，如果没有数据则延迟10ms
    while (Serial.available() == 0)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // 读取第一个字节，检查是否为帧头0x7B
    int count = 0;
    do
    {
        count++;
        if (count > 10)
        {
            //Serial.println("don't detect head");
            return;
        }
        Serial.read(buffer, 1);
    } while (buffer[0] != 0x7B);

    // 读取接下来的8个字节，存储到buffer中
    Serial.readBytes((byte *)buffer + 1, 8);

    // 检查最后一个字节是否为帧尾0x7D
    if (buffer[8] != 0x7D)
    {
        //Serial.println("don't detect end");
        return; // 如果不是帧尾，直接返回
    }
    // 检查校验位是否正确，校验位为buffer[7]，应为buffer[0]到buffer[6]的异或结果
    if (buffer[7] != (buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5] ^ buffer[6]))
    {
        //Serial.println("bcc wrong");
        return; // 如果校验失败，直接返回
    }
    // 将读取到的数据解析并存储到rpi结构体中
    rpi.mode = buffer[1];
    rpi.data1 = (((int16_t)buffer[2]) << 8) | ((int16_t)buffer[3]);
    rpi.data2 = (((int16_t)buffer[4]) << 8) | ((int16_t)buffer[5]);
    rpi.color = buffer[6] == 4 ? buffer[6] - 1 : buffer[6]; // 如果是蓝色0x04则减一，方便后续使用

    /*
    Serial.print("mode:");
    Serial.print(rpi.mode);
    Serial.print(" data1:");
    Serial.print(rpi.data1);
    Serial.print(" data2:");
    Serial.print(rpi.data2);
    Serial.print(" color:");
    Serial.println(rpi.color);
    */
}
