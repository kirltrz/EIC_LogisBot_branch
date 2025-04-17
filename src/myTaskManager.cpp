#include "myTaskManager.h"
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "globalValue.h"
#include "lvgl.h"
#include "RPI.h"
#include "HWT101.h"
#include "servo.h"
#include "servoEx.h"
#include "motor.h"
#include "motorEx.h"
#include "uniMotionCtrl.h"
#include "sensor.h"
#include "sensorEx.h"
#include "ui.h"
#include "LED.h"
#include "math.h"

// 全局常量定义
#define GLOBAL_MAX_SPEED 0.5    // 全局最高速度
#define GLOBAL_MIN_ACC_TIME 30  // 全局最小加速步长
#define GLOBAL_SWERVE_SPEED 0.3 // 全局转弯速度
#define GLOBAL_SWERVE_OMEGA 1.0 // 全局转弯角速度

int targetPosX = 320, targetPosY = 267;
int taskCode1Temp, taskCode2Temp;
bool ready = 0;          // 检查是否所有部件准备就绪
bool lvglNeedUpdate = 0; // 标志lvgl数据是否需要更新
bool readyNeedUpdate = 0;
bool taskCodeNeedUpdate = 0;
bool posNeedUpdate = 0;
char buffer[25][32];     // 用于将其他线程数据传递至lvgl线程
int temper[4];

// 对于dx dy globalx globaly的全局互斥锁
SemaphoreHandle_t mutex = xSemaphoreCreateMutex(); 

void check(void)
{
    if (!hwt101Ready)
    {
        checkSensor();
    }
    if (!servoReady)
    {
        checkServo();
    }
    if (!rpiReady)
    {
        writeRPI(0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (rpi.mode == 0 && rpi.data1 == 0 && rpi.data2 == 0 && rpi.color == 0)
            rpiReady = true;
    }
    if (!motorReady)
    {
        // 硬件问题，如果坚持使用UART则无法正常读取
    }
    if (!ready)
    {
        ready = rpiReady && hwt101Ready && motorReady && servoReady;
        readyNeedUpdate = 1;
    }

    // 更新显示的角度数据
    if (hwt101Ready)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        snprintf(buffer[0], sizeof(buffer[0]), "%.3f", currentYaw);
        xSemaphoreGive(mutex);
        strcat(buffer[0], "°");
        lvglNeedUpdate = true;
    }

    // 更新显示的舵机数据
    if (servoReady)
    {
        snprintf(buffer[1], sizeof(buffer[1]), "%d", readServo(0));
        snprintf(buffer[2], sizeof(buffer[2]), "%d", readServo(1));
        snprintf(buffer[3], sizeof(buffer[3]), "%d", readServo(2));
        snprintf(buffer[4], sizeof(buffer[4]), "%d", readServo(3));
        snprintf(buffer[5], sizeof(buffer[5]), "%d", readServo(4));

        temper[0] = servo.ReadTemper(0);
        snprintf(buffer[13], sizeof(buffer[13]), "%d", temper[0]);
        temperOffset[0] = (temper[0] - temperRefer) * Kt;

        temper[1] = servo.ReadTemper(1);
        snprintf(buffer[14], sizeof(buffer[14]), "%d", temper[1]);
        temperOffset[1] = (temper[1] - temperRefer) * Kt;

        temper[2] = servo.ReadTemper(2);
        snprintf(buffer[15], sizeof(buffer[15]), "%d", temper[2]);
        temperOffset[2] = (temper[2] - temperRefer) * Kt;

        temper[3] = servo.ReadTemper(3);
        snprintf(buffer[16], sizeof(buffer[16]), "%d", temper[3]);
        temperOffset[3] = (temper[3] - temperRefer) * Kt;

        lvglNeedUpdate = true;
    }
    
    // 更新paw3395数据
    if (1)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        snprintf(buffer[9], sizeof(buffer[9]), "%lf", globalX);
        snprintf(buffer[10], sizeof(buffer[10]), "%lf", globalY);
        snprintf(buffer[11], sizeof(buffer[11]), "%d", dx);
        snprintf(buffer[12], sizeof(buffer[12]), "%d", dy);
        xSemaphoreGive(mutex);

        posNeedUpdate = true;
    }
}

void lvglTask(void *pvParameters)
{
    while (true)
    {
        lv_timer_handler();
        if (lvglNeedUpdate)
        { 
            // lvgl非线程安全，将对lvgl的操作放在同一线程
            lv_label_set_text(ui_hwt101data, buffer[0]);
            lv_label_set_text(ui_servo0Text, buffer[1]);
            lv_label_set_text(ui_servo1Text, buffer[2]);
            lv_label_set_text(ui_servo2Text, buffer[3]);
            lv_label_set_text(ui_servo3Text, buffer[4]);
            lv_label_set_text(ui_servoClawText, buffer[5]);
            lv_label_set_text(ui_temp0, buffer[13]);
            lv_label_set_text(ui_temp1, buffer[14]);
            lv_label_set_text(ui_temp2, buffer[15]);
            lv_label_set_text(ui_temp3, buffer[16]);

            lv_label_set_text(ui_modeText, buffer[6]);
            lv_label_set_text(ui_data1Text, buffer[7]);
            lv_label_set_text(ui_data2Text, buffer[8]);
            switch (rpi.color)
            {
            case 1:
                lv_label_set_text(ui_colorText, "红");
                break;
            case 2:
                lv_label_set_text(ui_colorText, "绿");
                break;
            case 3:
                lv_label_set_text(ui_colorText, "蓝");
                break;
            default:
                lv_label_set_text(ui_colorText, "N/A");
                break;
            }

            lvglNeedUpdate = false;
        }
        if (posNeedUpdate)
        {
            lv_label_set_text(ui_xtext, buffer[9]);
            lv_label_set_text(ui_ytext, buffer[10]);
            lv_label_set_text(ui_dxtext, buffer[11]);
            lv_label_set_text(ui_dytext, buffer[12]);
            posNeedUpdate = 0;
        }
        if (taskCodeNeedUpdate)
        { 
            // 更新任务码
            lv_label_set_text(ui_taskcode1, taskCode1Full);
            lv_label_set_text(ui_taskcode2, taskCode2Full);
            taskCodeNeedUpdate = 0;
        }
        if (readyNeedUpdate)
        { 
            // 更新准备状态信息
            lv_obj_set_state(ui_Checkbox4, LV_STATE_CHECKED, rpiReady);
            lv_obj_set_state(ui_Checkbox1, LV_STATE_CHECKED, hwt101Ready);
            lv_obj_set_state(ui_Checkbox2, LV_STATE_CHECKED, motorReady);
            lv_obj_set_state(ui_Checkbox3, LV_STATE_CHECKED, servoReady);

            lv_obj_set_state(ui_Checkbox5, LV_STATE_CHECKED, ready);
            readyNeedUpdate = 0;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void checkTask(void *pvParameters)
{
    while (true)
    {
        check();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void pidTask(void *ptr)
{
    while (true)
    {
        if (yawAdjustOn)
            adjustYaw();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void rpiTask(void *ptr)
{
    while (true)
    {
        readRPI();

        snprintf(buffer[6], sizeof(buffer[6]), "%d", rpi.mode);
        snprintf(buffer[7], sizeof(buffer[7]), "%d", rpi.data1);
        snprintf(buffer[8], sizeof(buffer[8]), "%d", rpi.data2);

        lvglNeedUpdate = true;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void paw3395Task(void *ptr)
{
    volatile int16_t local_dx, local_dy;
    float yawInRad; // 用于存储弧度制的yaw角
    static float lastYaw = 0.0f; // 记录上次的yaw角以检测跳变
    static bool firstRun = true; // 第一次运行的标志
    
    while (true)
    {
        Motion_Burst(&local_dx, &local_dy);
        xSemaphoreTake(mutex, portMAX_DELAY); // 互斥锁，避免数据竞争
        dx = local_dx;
        dy = local_dy;
        
        if (hwt101Ready)
        {
            float newYaw = HWT101.getZ();
            currentYaw = newYaw;
            
            // 处理第一次运行
            if (firstRun) {
                lastYaw = currentYaw;
                firstRun = false;
            }
            
            // 检测并处理±180°跳变
            if (lastYaw > 150.0f && currentYaw < -150.0f) {
                // 从正到负的跳变，如180°到-180°
                lastYaw = currentYaw;
            } else if (lastYaw < -150.0f && currentYaw > 150.0f) {
                // 从负到正的跳变，如-180°到180°
                lastYaw = currentYaw;
            } else {
                lastYaw = currentYaw;
            }
            
            // 转换为弧度制
            yawInRad = currentYaw * DEG_TO_RAD;
            
            // 使用正确的坐标变换公式
            globalX += (local_dx * 0.0009769 * cos(yawInRad) - local_dy * 0.0009769 * sin(yawInRad));
            globalY += (local_dx * 0.0009769 * sin(yawInRad) + local_dy * 0.0009769 * cos(yawInRad));
            
            // 已处理过的局部位移清零
            local_dx = 0;
            local_dy = 0;
        }
        else {
            // 如果陀螺仪不可用，退回到简单计算方式
            globalX += dx * 0.0009769;
            globalY += dy * 0.0009769;
        }
        
        xSemaphoreGive(mutex);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void mainSequence(void *ptr)
{
    // 等待启动按钮按下
    while (startup == 0)
        vTaskDelay(50 / portTICK_PERIOD_MS);
        
    /*--------------------主流程开始--------------------*/
    
    /* 二维码扫描 */
    uniMove(YP, 450, 0.5, GLOBAL_MIN_ACC_TIME, true, false, false); // 移动至二维码前
    servoAction(3);                                                 // 机械臂运动到扫描二维码状态
    uniMove(YP, 150, 0.15, 5, false, false, false);                 // 缓慢通过二维码

    while (rpi.mode != 1 || rpi.data1 == 0 || rpi.data2 == 0)
    {
        writeRPI(1);                         // 向树莓派发出扫码指令
        vTaskDelay(50 / portTICK_PERIOD_MS); // 不断等待直至扫描成功
    }
    
    // 保存和解析任务码
    taskCode1Temp = rpi.data1; 
    taskCode2Temp = rpi.data2;
    snprintf(taskCode1Full, sizeof(taskCode1Full), "%d", taskCode1Temp); // 保存任务码字符串
    snprintf(taskCode2Full, sizeof(taskCode2Full), "%d", taskCode2Temp);
    taskCodeNeedUpdate = 1;             // 更新LVGL显示的任务码
    
    // 解析任务码1（整数）
    taskCode1[0] = taskCode1Temp / 100; 
    taskCode1Temp %= 100;              
    taskCode1[1] = taskCode1Temp / 10;  
    taskCode1Temp %= 10;                
    taskCode1[2] = taskCode1Temp;       
    
    // 解析任务码2（整数）
    taskCode2[0] = taskCode2Temp / 100; 
    taskCode2Temp %= 100;               
    taskCode2[1] = taskCode2Temp / 10;  
    taskCode2Temp %= 10;                
    taskCode2[2] = taskCode2Temp;       
    
    writeRPI(0);                        // 向树莓派发出复位指令
    servoAction(2);                     // 机械臂复位至normal

    /* 第一次在原料区抓取物料 */
    uniMove(YP, 740, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    servoAction(2);
    LED(2);
    servoAction(4); // 机械臂运动至转盘检测状态
    writeRPI(2);    // 向树莓派发出识别实心圆及颜色指令

    while (rpi.mode != 2)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }

    caliPos(targetPosX, targetPosY, true); // 校准车身位置

    while (rpi.mode != 2 || rpi.color != taskCode1[0])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第一个物料
    }
    servoAction(4 + taskCode1[0]); // 从转盘抓取第一个物料至车上
    servoAction(4); // 回到转盘检测状态
    
    while (rpi.mode != 2 || rpi.color != taskCode1[1])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第二个物料
    }
    servoAction(4 + taskCode1[1]); // 从转盘抓取第二个物料至车上
    servoAction(4); // 回到转盘检测状态
    
    while (rpi.mode != 2 || rpi.color != taskCode1[2])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第三个物料
    }
    servoAction(4 + taskCode1[2]); // 从转盘抓取第三个物料至车上
    
    writeRPI(0); // 树莓派待命
    LED(false);
    servoAction(2); // 机械臂复位至normal

    // 移动至粗加工区
    uniMove(YP, 100, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YP, 90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YP, 1360, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YP, 90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YP, 650, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    uniMove(XP, 100, 0.1, 10, true, false, true);

    while (uniOnMove)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    /* 第一次在粗加工区放置并抓取物料 */
    servoAction(8); // 机械臂运动至地面标志检测状态
    writeRPI(3); // 向树莓派发出识别色环指令
    
    while (rpi.mode != 3)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }

    LED(1);
    caliPos(targetPosX, targetPosY, true); // 校准车身位置
    LED(0);
    writeRPI(0);                   // 校准完回到待命状态
    
    // 放置物料
    servoAction(8 + taskCode1[0]); // 把第一个物料从车上放到地上对应位置
    servoAction(8 + taskCode1[1]); // 把第二个物料从车上放到地上对应位置
    servoAction(8 + taskCode1[2]); // 把第三个物料从车上放到地上对应位置

    // 抓取物料
    servoAction(11 + taskCode1[0]); // 把第一个粗加工完的物料从地上放回车上
    servoAction(11 + taskCode1[1]); // 把第二个粗加工完的物料从地上放回车上
    servoAction(11 + taskCode1[2]); // 把第三个粗加工完的物料从地上放回车上
    servoAction(2);                 // 回到normal

    // 移动至暂存区
    uniMove(YN, 660, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YN, -90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YN, 730, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    uniMove(XP, 50, 0.1, 10, true, false, true);
    
    while (uniOnMove)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    /* 第一次在暂存区放置物料 */
    servoAction(8); // 机械臂运动至地面标志检测状态
    writeRPI(3); // 向树莓派发出识别色环指令
    
    while (rpi.mode != 3)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }

    LED(1);
    caliPos(targetPosX, targetPosY, true); // 校准车身位置
    LED(0);

    writeRPI(0);                   // 校准完回到待命状态
    
    // 放置物料
    servoAction(8 + taskCode1[0]); // 把第一个物料从车上放到地上对应位置
    servoAction(8 + taskCode1[1]); // 把第二个物料从车上放到地上对应位置
    servoAction(8 + taskCode1[2]); // 把第三个物料从车上放到地上对应位置
    servoAction(2); // 回到normal

    // 移动至原料区
    uniMove(YN, 715, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YN, -90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YN, 330, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    
    while (uniOnMove)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    /*--------------------第二轮开始--------------------*/
    /* 第二次在原料区抓取物料 */
    LED(2);
    servoAction(4); // 转盘处检测状态
    writeRPI(2); // 向树莓派发出识别实心圆及颜色指令
    
    while (rpi.mode != 2)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }

    caliPos(targetPosX, targetPosY, true); // 校准车身位置
    
    while (rpi.mode != 2 || rpi.color != taskCode2[0])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第一个物料
    }
    servoAction(4 + taskCode2[0]); // 从转盘抓取第一个物料至车上
    servoAction(4); // 回到转盘检测状态
    
    while (rpi.mode != 2 || rpi.color != taskCode2[1])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第二个物料
    }
    servoAction(4 + taskCode2[1]); // 从转盘抓取第二个物料至车上
    servoAction(4); // 回到转盘检测状态
    
    while (rpi.mode != 2 || rpi.color != taskCode2[2])
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待抓取第三个物料
    }
    servoAction(4 + taskCode2[2]); // 从转盘抓取第三个物料至车上
    
    writeRPI(0); // 树莓派待命
    LED(0);
    servoAction(2); // 机械臂复位至normal

    // 移动至粗加工区
    uniMove(YP, 100, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YP, 90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YP, 1360, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YP, 90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YP, 650, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    uniMove(XP, 100, 0.1, 10, false, false, true);
    
    while (uniOnMove)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    /* 第二次在粗加工区放置并抓取物料 */
    servoAction(8); // 机械臂运动至地面标志检测状态
    writeRPI(3); // 向树莓派发出识别色环指令
    
    while (rpi.mode != 3)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }

    LED(1);
    caliPos(targetPosX, targetPosY, true); // 校准车身位置
    LED(0);

    writeRPI(0);                   // 校准完回到待命状态
    
    // 放置物料
    servoAction(8 + taskCode2[0]); // 把第一个物料从车上放到地上对应位置
    servoAction(8 + taskCode2[1]); // 把第二个物料从车上放到地上对应位置
    servoAction(8 + taskCode2[2]); // 把第三个物料从车上放到地上对应位置

    // 抓取物料
    servoAction(11 + taskCode2[0]); // 把第一个粗加工完的物料从地上放回车上
    servoAction(11 + taskCode2[1]); // 把第二个粗加工完的物料从地上放回车上
    servoAction(11 + taskCode2[2]); // 把第三个粗加工完的物料从地上放回车上
    servoAction(2); // 回到normal

    // 移动至暂存区
    uniMove(YN, 660, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YN, -90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YN, 730, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    uniMove(XP, 50, 0.1, 10, true, false, true);
    
    while (uniOnMove)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    /* 第二次在暂存区放置物料（码垛） */
    servoAction(8); // 机械臂运动至地面标志检测状态
    writeRPI(3); // 向树莓派发出识别圆形指令
    
    while (rpi.mode != 3)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); // 等待进行车身位置校准
    }
    
    LED(1);
    caliPos(targetPosX, targetPosY, true); // 校准车身位置
    yawAdjustOn = true;
    LED(0);

    writeRPI(0);                    // 校准完回到待命状态
    
    // 放置物料(码垛)
    servoAction(14 + taskCode2[0]); // （码垛）把第一个物料从车上放到地上对应颜色物料上
    servoAction(14 + taskCode2[1]); // （码垛）把第二个物料从车上放到地上对应颜色物料上
    servoAction(14 + taskCode2[2]); // （码垛）把第三个物料从车上放到地上对应颜色物料上
    servoAction(2); // 回到normal

    /* 回到启停区 */
    servoAction(0);
    uniMove(YN, 700, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, false, false);
    uniSwerve(YN, -90.0, GLOBAL_SWERVE_SPEED, GLOBAL_SWERVE_OMEGA, false);
    uniMove(YN, 1780, GLOBAL_MAX_SPEED, GLOBAL_MIN_ACC_TIME, true, true, true);
    uniMove(XP, 125, 0.1, 10, true, false, true);

    /*--------------------主流程结束--------------------*/
    vTaskDelete(NULL);
}

TaskHandle_t *mainSequenceHandle;

void initTaskManager(void)
{
    xTaskCreate(
        lvglTask,    // 任务函数
        "LVGL Task", // 任务名称
        8192,        // 堆栈大小（1 字 = 4 字节）
        NULL,        // 任务参数
        1,           // 任务优先级
        NULL         // 任务句柄
    );
    xTaskCreate(checkTask, "Check Task", 2048, NULL, 2, NULL);
    xTaskCreate(pidTask, "PID Task", 2048, NULL, 2, NULL); // 已并入uniMotionCtrl
    xTaskCreate(rpiTask, "RPI Read Task", 2048, NULL, 2, NULL);
    xTaskCreate(mainSequence, "Main Sequence Task", 4096, NULL, 2, mainSequenceHandle);

    uniMotionCtrlInit();

    xTaskCreatePinnedToCore(paw3395Task, "paw3395 task", 2048, NULL, 2, NULL, 0);
}
