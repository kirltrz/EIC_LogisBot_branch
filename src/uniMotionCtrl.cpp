#include "uniMotionCtrl.h"

#define R 0.0375 // 轮子半径
#define L 0.14   // 轮心到中心的距离
struct uniMotionCtrlData
{
    float vx = 0.0f;
    float vy = 0.0f;
    float rawOmega = 0.0f;
    float omega = 0.0f; // 叠加角度环的角速度数据
} UMCData;
struct
{
    int direction;
    int distance;
    float v;
    int accTimeMs;
    bool startAcc;
    bool stopAcc;
    bool autoStop;
} uniMoveData;
struct
{
    int direction;
    float angle;
    float v;
    float omega;
    bool autoStop;
} uniSwerveData;
bool uniOnMove = false;

void uniCtrl(float vx, float vy, float omega)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    UMCData.vx = vx;
    UMCData.vy = vy;
    UMCData.rawOmega = omega;
    xSemaphoreGive(mutex);
}
void uniComputeAndSend(void *ptr)
{
    float omega_0, omega_1, omega_2, omega_3; // 各个轮子的转速数据
    float rawYawAdjustOmega, yawAdjustOmega;
    bool yawNeedAdjust = 0;
    while (true)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (emergencyStop)
        {
            UMCData.vx = 0.0;
            UMCData.vy = 0.0;
            UMCData.omega = 0.0;
        }
        rawYawAdjustOmega = -(currentYaw - targetYaw) * 0.05;
        if (abs(rawYawAdjustOmega) > 0.1)
            yawAdjustOmega = rawYawAdjustOmega > 0 ? 0.1 : -0.1;
        else
            yawAdjustOmega = rawYawAdjustOmega;
        if (abs(currentYaw - targetYaw) > 1.0f)
            yawNeedAdjust = true;
        else
            yawNeedAdjust = false;
        UMCData.omega = UMCData.rawOmega + ((yawNeedAdjust&&uniYawAdjustOn) ? yawAdjustOmega : 0);
        omega_0 = (UMCData.vy - UMCData.omega * L) / R;
        omega_1 = (UMCData.vx - UMCData.omega * L) / R;
        omega_2 = (-UMCData.vy - UMCData.omega * L) / R;
        omega_3 = (-UMCData.vx - UMCData.omega * L) / R;
        xSemaphoreGive(mutex);
        if (!uniStop)
        {
            motorWrite(0, MOTOR_MODE_SPEED, omega_0 > 0 ? 1 : 0, 100, abs(omega_0));
            motorWrite(1, MOTOR_MODE_SPEED, omega_1 > 0 ? 1 : 0, 100, abs(omega_1));
            motorWrite(2, MOTOR_MODE_SPEED, omega_2 > 0 ? 1 : 0, 100, abs(omega_2));
            motorWrite(3, MOTOR_MODE_SPEED, omega_3 > 0 ? 1 : 0, 100, abs(omega_3));
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void uniMoveTask(void *ptr)
{
    float vx, vy;
    // 获取初始位置、计算目标位置
    int startPos, currentPos, targetPos;
    float startV; // 获取初始速度
    xSemaphoreTake(mutex, portMAX_DELAY);
    switch (uniMoveData.direction)
    {
    case 0:
        startV = UMCData.vy;
        startPos = globalY;
        targetPos = startPos + uniMoveData.distance;
        break;
    case 1:
        startV = UMCData.vx;
        startPos = globalX;
        targetPos = startPos + uniMoveData.distance;
        break;
    case 2:
        startV = UMCData.vy;
        startPos = globalY;
        targetPos = startPos - uniMoveData.distance;
        break;
    case 3:
        startV = UMCData.vx;
        startPos = globalX;
        targetPos = startPos - uniMoveData.distance;
        break;
    default:
        break;
    }
    xSemaphoreGive(mutex);
    // 加速阶段
    if (uniMoveData.startAcc)
    {
        for (float currentV = startV; currentV < uniMoveData.v - 0.03; currentV += 0.03)
        {
            switch (uniMoveData.direction)
            {
            case 0:
                uniCtrl(0.0, currentV, 0.0);
                break;
            case 1:
                uniCtrl(currentV, 0.0, 0.0);
                break;
            case 2:
                uniCtrl(0.0, -currentV, 0.0);
                break;
            case 3:
                uniCtrl(-currentV, 0.0, 0.0);
                break;
            default:
                break;
            }
            vTaskDelay(uniMoveData.accTimeMs / portTICK_PERIOD_MS);
        }
    }
    // 匀速阶段
    while (1)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        switch (uniMoveData.direction)
        {
        case 0:
            currentPos = globalY;
            UMCData.vx = 0.0;
            UMCData.vy = uniMoveData.v;
            break;
        case 1:
            currentPos = globalX;
            UMCData.vx = uniMoveData.v;
            UMCData.vy = 0.0;
            break;
        case 2:
            currentPos = globalY;
            UMCData.vx = 0.0;
            UMCData.vy = -uniMoveData.v;
            break;
        case 3:
            currentPos = globalX;
            UMCData.vx = -uniMoveData.v;
            UMCData.vy = 0.0;
            break;
        default:
            break;
        }
        xSemaphoreGive(mutex);
        // 判断进入减速阶段的条件，满足则退出循环进入减速
        if (((uniMoveData.direction <= 1 && currentPos >= targetPos - 100) ||
             (uniMoveData.direction >= 2 && currentPos <= targetPos + 100)) &&
            uniMoveData.stopAcc)
            break;
        // 满足条件不需要加减速直接停止并退出函数
        if (((uniMoveData.direction <= 1 && currentPos >= targetPos - 10) ||
             (uniMoveData.direction >= 2 && currentPos <= targetPos + 10)) &&
            (!uniMoveData.stopAcc))
        {
            if (uniMoveData.autoStop)
            {
                xSemaphoreTake(mutex, portMAX_DELAY);
                UMCData.vx = 0.0;
                UMCData.vy = 0.0;
                xSemaphoreGive(mutex);
            }
            uniOnMove = false;
            vTaskDelete(NULL);
            // return;
        }
    }
    // 减速阶段
    if (uniMoveData.stopAcc)
    {
        while (1)
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            switch (uniMoveData.direction)
            {
            case 0:
                currentPos = globalY;
                UMCData.vx = 0.0;
                UMCData.vy = (targetPos - currentPos) * uniMoveData.v * 0.01;
                break;
            case 1:
                currentPos = globalX;
                UMCData.vx = (targetPos - currentPos) * uniMoveData.v * 0.01;
                UMCData.vy = 0.0;
                break;
            case 2:
                currentPos = globalY;
                UMCData.vx = 0.0;
                UMCData.vy = (targetPos - currentPos) * uniMoveData.v * 0.01;
                break;
            case 3:
                currentPos = globalX;
                UMCData.vx = (targetPos - currentPos) * uniMoveData.v * 0.01;
                UMCData.vy = 0.0;
                break;
            default:
                break;
            }
            if (abs(currentPos - targetPos) < 3) // 到达目标位置退出
            {
                UMCData.vx = 0.0;
                UMCData.vy = 0.0;
                xSemaphoreGive(mutex);
                uniOnMove = false;
                vTaskDelete(NULL);
                // return;
            }
            xSemaphoreGive(mutex);
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}
void uniMove(int direction, int distance, float v, int accTimeMs, bool startAcc, bool stopAcc, bool autoStop)
{
    while (uniOnMove)
        vTaskDelay(50 / portTICK_PERIOD_MS);
    uniOnMove = true;
    uniMoveData.direction = direction;
    uniMoveData.distance = distance;
    uniMoveData.v = v;
    uniMoveData.accTimeMs = accTimeMs;
    uniMoveData.startAcc = startAcc;
    uniMoveData.stopAcc = stopAcc;
    uniMoveData.autoStop = autoStop;
    xTaskCreate(
        uniMoveTask,
        "uniMove Task",
        2048,
        NULL, //&uniMoveData,
        2,
        NULL);
}
void uniSwerveTask(void *ptr)
{
    float startYaw, local_currentYaw, local_targetYaw;
    xSemaphoreTake(mutex, portMAX_DELAY);
    startYaw = currentYaw;
    local_targetYaw = startYaw + uniSwerveData.angle;
    targetYaw = local_targetYaw;
    xSemaphoreGive(mutex);

    yawAdjustOn = false;
    while (1)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        local_currentYaw = currentYaw;
        xSemaphoreGive(mutex);

        if (abs(local_currentYaw - local_targetYaw) < 5.0)
        {
            xSemaphoreTake(mutex, portMAX_DELAY);
            UMCData.rawOmega = 0.0; // 转弯弯后角速度归零
            if (uniSwerveData.autoStop)
            {
                UMCData.vx = 0;
                UMCData.vy = 0;
            }
            xSemaphoreGive(mutex);

            yawAdjustOn = true;
            uniOnMove = false;
            vTaskDelete(NULL);
            // return;
        }
        switch (uniSwerveData.direction)
        {
        case 0:
            uniCtrl(0.0, uniSwerveData.v, uniSwerveData.omega);
            break;
        case 1:
            uniCtrl(uniSwerveData.v, 0.0, uniSwerveData.omega);
            break;
        case 2:
            uniCtrl(0.0, -uniSwerveData.v, -uniSwerveData.omega);
            break;
        case 3:
            uniCtrl(-uniSwerveData.v, 0.0, -uniSwerveData.omega);
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void uniSwerve(int direction, float angle, float v, float omega, bool autoStop)
{
    while (uniOnMove)
        vTaskDelay(50 / portTICK_PERIOD_MS);
    uniOnMove = true;
    uniSwerveData.direction = direction;
    uniSwerveData.angle = angle;
    uniSwerveData.v = v;
    uniSwerveData.omega = omega;
    uniSwerveData.autoStop = autoStop;
    xTaskCreate(
        uniSwerveTask,
        "uniSwerve Task",
        2048,
        NULL, //&uniSwerveData,
        2,
        NULL);
}
void uniMotionCtrlInit(void)
{
    xTaskCreate(
        uniComputeAndSend,
        "uniMotionCtrl Task",
        4096,
        NULL,
        2,
        NULL);
}