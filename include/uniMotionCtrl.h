/*该库用于统筹小车运动，与motorEx独立*/
#include <Arduino.h>
#include "motor.h"
#include "sensor.h"
#include "myTaskManager.h"
#include "globalValue.h"

#define XP 1
#define XN 3
#define YP 0
#define YN 2

extern bool uniOnMove;

void uniMotionCtrlInit(void);
void uniCtrl(float vx, float vy, float omega);
/**
 * @brief 控制机器人沿指定方向直线移动指定距离（支持加减速控制）
 *
 * @param direction 移动方向：
 *                  - YP: 正Y轴方向（前进）
 *                  - XP: 正X轴方向（右移）
 *                  - YN: 负Y轴方向（后退）
 *                  - XN: 负X轴方向（左移）
 * @param distance 目标移动距离（单位：mm）
 * @param v 巡航速度（单位：米/秒）
 * @param accTimeMs 加速阶段每个阶梯加速阶段时间（单位：毫秒，仅当 startAcc=true 时生效）
 * @param startAcc 是否启用加速阶段（平滑启动）
 * @param stopAcc 是否启用减速阶段（平滑停止）
 * @param autoStop 到达目标距离后是否自动停止（若为 false，需外部调用停止，仅当stopAcc=false时生效）
 *
 * @note
 * - 非阻塞函数：通过 FreeRTOS 任务实现，调用后立即返回。
 * - 重复调用需等待前次任务完成（通过 uniOnMove 标志位判断）。
 * - 使用全局位置反馈（globalX/globalY）进行闭环控制。
 * - 加减速阶段通过线性速度插值实现。
 */
void uniMove(int direction, int distance, float v, int accTimeMs, bool startAcc, bool stopAcc, bool autoStop);
/**
 * @brief 控制机器人平移同时绕中心旋转（合成运动，如弧形轨迹）
 *
 * @param direction 平移方向：
 *                  - YP: 正Y轴方向（前进）
 *                  - XP: 正X轴方向（右移）
 *                  - YN: 负Y轴方向（后退）
 *                  - XN: 负X轴方向（左移）
 * @param angle 目标旋转角度（单位：角度，正值逆时针，负值顺时针，由于传感器在±180°跳变，支持旋转范围为(-135°,225°)开区间）
 * @param v 平移线速度（单位：米/秒）
 * @param omega 绕中心旋转的角速度（单位：弧度/秒）
 * @param autoStop 到达目标角度后是否自动停止（若为 false，则转完弯之后继续向前移动，需外部调用停止）
 *
 * @note
 * - 非阻塞函数：通过 FreeRTOS 任务实现，调用后立即返回。
 * - 旋转角度基于当前航向（currentYaw）的增量（相对运动）。
 * - 通过叠加平移速度（vx/vy）和角速度（omega）实现合成运动。
 * - 航向控制通过 PID 调节（yawAdjustOmega）保持稳定。
 */
void uniSwerve(int direction, float angle, float v, float omega, bool autoStop);