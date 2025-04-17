#include "servoEx.h"
#include "servo.h"
#include "myTaskManager.h"

bool onRun = false; // 舵机运行中标志位，防止动作顺序错乱

void actionGroup_fold(void *ptr)
{ // 0
    // onRun = true;
    int finalPos[5] = {2048, 2530, 548, 1000, CLOSE};
    // servoAction(1);
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    servoAction(2);
    for (int i = 0; i < 4; i++)
    {
        writeServo(i, finalPos[i]);
    }
    setClaw(finalPos[4]);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_middle(void *ptr)
{ // 1
    onRun = true;
    writeServo(2, 2048);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    writeServo(1, 2048);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    writeServo(3, 2048);
    writeServo(0, 2048);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_normal(void *ptr)
{ // 2
    onRun = true;
    regWriteServo(1, 2037);
    regWriteServo(2, 1033);
    regWriteServo(3, 1030);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeServo(0, 2048);
    setClaw(OPEN);
    onRun = false;
}
void actionGroup_scanQRcode(void *ptr)
{ // 3 上一状态应为normal状态
    onRun = true;
    writeServo(0, 2048);
    setClaw(2);
    regWriteServo(1, 3333);
    regWriteServo(2, 537);
    regWriteServo(3, 988);
    regWriteServoAction();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_turntableDetect(void *ptr)
{ // 4 上一状态应为normal状态
    onRun = true;
    regWriteServo(1, 1777);
    regWriteServo(2, 1340);
    regWriteServo(3, 1003);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeServo(0, 2048);
    setClaw(OPEN);
    onRun = false;
}
void actionGroup_turntableCatchRedToPlate(void *ptr)
{ // 5 上一状态应为turntableDetect状态
    onRun = true;
    // 下降准备抓取
    regWriteServo(1, 1710);
    regWriteServo(2, 1090);
    regWriteServo(3, 1300);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抓取
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抬起
    regWriteServo(1, 2174);
    regWriteServo(2, 806);
    regWriteServo(3, 1106);
    regWriteServoAction();
    vTaskDelay(250 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, RED_PLATE_ANGLE);
    vTaskDelay(350 / portTICK_PERIOD_MS);
    // 放下机械臂
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2037);
    regWriteServo(2, 1033);
    regWriteServo(3, 1030);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_turntableCatchGreenToPlate(void *ptr)
{ // 6 上一状态应为turntableDetect状态
    onRun = true;
    // 下降准备抓取
    regWriteServo(1, 1710);
    regWriteServo(2, 1090);
    regWriteServo(3, 1300);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抓取
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抬起
    regWriteServo(1, 2174);
    regWriteServo(2, 806);
    regWriteServo(3, 1106);
    regWriteServoAction();
    vTaskDelay(350 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, GREEN_PLATE_ANGLE);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    // 放下机械臂
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2037);
    regWriteServo(2, 1033);
    regWriteServo(3, 1030);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_turntableCatchBlueToPlate(void *ptr)
{ // 7 上一状态应为turntableDetect状态
    onRun = true;
    // 下降准备抓取
    regWriteServo(1, 1710);
    regWriteServo(2, 1090);
    regWriteServo(3, 1300);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抓取
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 抬起
    regWriteServo(1, 2174);
    regWriteServo(2, 806);
    regWriteServo(3, 1487);
    regWriteServoAction();
    vTaskDelay(250 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, BLUE_PLATE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    // 放下机械臂
    writeServo(3, 1106);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2037);
    regWriteServo(2, 1033);
    regWriteServo(3, 1030);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_groundDetect(void *ptr)
{ // 8
    onRun = true;
    regWriteServo(0, 2048);
    regWriteServo(1, 1688);
    regWriteServo(2, 1381);
    regWriteServo(3, 1033);
    regWriteServoAction();
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchRedToGround(void *ptr)
{ // 9
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, RED_PLATE_ANGLE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, RED_CIRCLE_ANGLE);
    regWriteServo(1, 1325);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(800 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1182, 2500, 30);
    regWriteServo(2, 1635, 2500, 30);
    regWriteServo(3, 1296, 3400, 50);
    regWriteServoAction();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchGreenToGround(void *ptr)
{ // 10
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, GREEN_PLATE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, GREEN_CIRCLE_ANGLE);
    regWriteServo(1, 1617);
    regWriteServo(2, 1125);
    regWriteServo(3, 1362);
    regWriteServoAction();
    vTaskDelay(700 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1420, 2000, 30);
    regWriteServo(2, 1137, 2000, 30);
    regWriteServo(3, 1530, 2000, 50);
    regWriteServoAction();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchBlueToGround(void *ptr)
{ // 11
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, BLUE_PLATE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, BLUE_CIRCLE_ANGLE);
    regWriteServo(1, 1325);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(800 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1225, 2500, 30);
    regWriteServo(2, 1559, 2500, 30);
    regWriteServo(3, 1315, 3400, 50);
    regWriteServoAction();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_groundCatchRedToPlate(void *ptr)
{ // 12
    onRun = true;
    /*机械臂运动至物料上方*/
    regWriteServo(1, 1325);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeServo(0, RED_CIRCLE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    /*抓取物料*/
    regWriteServo(1, 1182, 2500, 30);
    regWriteServo(2, 1635, 2500, 30);
    regWriteServo(3, 1296, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/ 
    regWriteServo(1, 2176, 3400, 50);
    regWriteServo(2, 950, 3400, 50);
    regWriteServo(3, 976, 2000, 30);
    regWriteServoAction();
    vTaskDelay(600 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, RED_PLATE_ANGLE);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    regWriteServo(1, 2176, 3400, 50);
    regWriteServo(2, 950, 3400, 50);
    regWriteServo(3, 976, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下机械臂
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2176);
    regWriteServo(2, 950);
    regWriteServo(3, 976);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_groundCatchGreenToPlate(void *ptr)
{ // 13
    onRun = true;
    /*机械臂运动至物料上方*/
    regWriteServo(1, 1325);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeServo(0, GREEN_CIRCLE_ANGLE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抓取物料*/
    regWriteServo(1, 1420, 2000, 30);
    regWriteServo(2, 1137, 2000, 30);
    regWriteServo(3, 1530, 2000, 50);
    regWriteServoAction();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 3400, 50);
    regWriteServo(2, 950, 3400, 50);
    regWriteServo(3, 976, 3400, 50);
    regWriteServoAction();
    vTaskDelay(600 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, GREEN_PLATE_ANGLE);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // 放下机械臂
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2176);
    regWriteServo(2, 950);
    regWriteServo(3, 976);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_groundCatchBlueToPlate(void *ptr)
{ // 14
    onRun = true;
    /*机械臂运动至物料上方*/
    regWriteServo(1, 1325);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    writeServo(0, BLUE_CIRCLE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    /*抓取物料*/
    regWriteServo(1, 1225, 2500, 30);
    regWriteServo(2, 1559, 2500, 30);
    regWriteServo(3, 1315, 3400, 50);
    regWriteServoAction();
    vTaskDelay(750 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 1869, 3400, 50);
    regWriteServo(2, 949, 3400, 50);
    regWriteServo(3, 1362, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 云台旋转
    writeServo(0, BLUE_PLATE_ANGLE);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    // 放下机械臂
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 放下物料
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // 结束，抬起机械臂
    regWriteServo(1, 2176);
    regWriteServo(2, 950);
    regWriteServo(3, 976);
    regWriteServoAction();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchRedToRed(void *ptr)
{ // 15
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, RED_PLATE_ANGLE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, RED_CIRCLE_ANGLE, 3400, 50);
    regWriteServo(1, 1400);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(700 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1310, 2000, 30);
    regWriteServo(2, 1798, 2000, 30);
    regWriteServo(3, 988, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchGreenToGreen(void *ptr)
{ // 16
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, GREEN_PLATE_ANGLE);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, GREEN_CIRCLE_ANGLE, 3400, 50);
    regWriteServo(1, 1713);
    regWriteServo(2, 1278);
    regWriteServo(3, 1132);
    regWriteServoAction();
    vTaskDelay(700 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1670, 2000, 30);
    regWriteServo(2, 1121, 2000, 30);
    regWriteServo(3, 1330, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void actionGroup_plateCatchBlueToBlue(void *ptr)
{ // 17
    onRun = true;
    /*抬起机械臂*/
    regWriteServo(1, 2210);
    regWriteServo(2, 800);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    setClaw(OPEN);
    writeServo(0, BLUE_PLATE_ANGLE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    /*放下机械臂*/
    regWriteServo(1, 2117, 2000, 30);
    regWriteServo(2, 611, 2000, 30);
    regWriteServo(3, 1380, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(CLOSE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*抬起机械臂*/
    regWriteServo(1, 2176, 500, 30);
    regWriteServo(2, 950, 1000, 30);
    regWriteServo(3, 976, 2000, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    /*将物料夹至色环上方*/
    regWriteServo(0, BLUE_CIRCLE_ANGLE, 3400, 50);
    regWriteServo(1, 1400);
    regWriteServo(2, 1795);
    regWriteServo(3, 1024);
    regWriteServoAction();
    vTaskDelay(700 / portTICK_PERIOD_MS);
    /*放置物料*/
    regWriteServo(1, 1410, 2000, 30);
    regWriteServo(2, 1587, 2000, 30);
    regWriteServo(3, 1122, 3400, 50);
    regWriteServoAction();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setClaw(OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    onRun = false;
}
void (*actionGroups[50])(void *ptr) = { // 函数指针数组，用于调用不同的动作组
    actionGroup_fold,
    actionGroup_middle,
    actionGroup_normal,
    actionGroup_scanQRcode,
    actionGroup_turntableDetect,
    actionGroup_turntableCatchRedToPlate,
    actionGroup_turntableCatchGreenToPlate,
    actionGroup_turntableCatchBlueToPlate,
    actionGroup_groundDetect,
    actionGroup_plateCatchRedToGround,
    actionGroup_plateCatchGreenToGround,
    actionGroup_plateCatchBlueToGround,
    actionGroup_groundCatchRedToPlate,
    actionGroup_groundCatchGreenToPlate,
    actionGroup_groundCatchBlueToPlate,
    actionGroup_plateCatchRedToRed,
    actionGroup_plateCatchGreenToGreen,
    actionGroup_plateCatchBlueToBlue};

/*
0  折叠状态
1  所有舵机置于中位
2  normal状态
3  扫码状态
4  转盘处检测状态
5  从转盘抓取红色物料放置在车上对应位置
6  从转盘抓取绿色物料放置在车上对应位置
7  从转盘抓取蓝色物料放置在车上对应位置
8  地面标志检测状态
9  将车上的红色物料放在地面对应位置
10 将车上的绿色物料放在地面对应位置
11 将车上的蓝色物料放在地面对应位置
12 从地面上抓取红色物料放置在车上对应位置
13 从地面上抓取绿色物料放置在车上对应位置
14 从地面上抓取蓝色物料放置在车上对应位置
15 将车上的红色物料放在地面的红色物料之上（码垛）
16 将车上的绿色物料放在地面的绿色物料之上（码垛）
17 将车上的蓝色物料放在地面的蓝色物料之上（码垛）
*/
void servoAction(int num)
{
    while (onRun)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    actionGroups[num](nullptr);
}