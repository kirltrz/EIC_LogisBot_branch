/*云台角度*/
// 盘子角度
#define RED_PLATE_ANGLE 2568
#define GREEN_PLATE_ANGLE 1540
#define BLUE_PLATE_ANGLE 496
// 地面色环角度
#define RED_CIRCLE_ANGLE 2476
#define GREEN_CIRCLE_ANGLE 2052
#define BLUE_CIRCLE_ANGLE 1619

extern void (*actionGroups[50])(void *ptr);
void servoAction(int num);