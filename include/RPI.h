/*
 *用于与树莓派通信
 */
#ifndef RPI_H
#define RPI_H

#include <Arduino.h>

struct RPI_t
{
    int mode;
    int16_t data1;
    int16_t data2;
    int8_t color;
};
extern RPI_t rpi;

/// @brief 初始化串口
/// @param  无
void initRPI(void);

void writeRPI(int mode);

void readRPI();
#endif