#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "RPI.h"
#include "LED.h"
#include "servo.h"
#include "motor.h"
#include "sensor.h"
#include "sensorEx.h"
#include "myInterface.h"
#include "myTaskManager.h"

void setup()
{
  initRPI();         // 初始化与树莓派的通信
  initLED();         // 初始化LED
  initSensor();      // 初始化传感器
  initServo();       // 初始化舵机
  initMotor();       // 初始化电机
  initScreen();      // 初始化屏幕
  initLVGL();        // 初始化LVGL
  initTaskManager(); // 初始化任务管理器

  //Serial.println("Setup done");
}

void loop()
{
}
