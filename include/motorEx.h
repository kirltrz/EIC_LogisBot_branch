extern int caliDeadZone;

/// @brief 控制小车移动
/// @param direction 0-7
/// @param distance 移动距离，米
/// @param speed 速度，度/s
/// @param acc 加速度，度/s^2
void move(int direction, float distance, float speed, int acc);

/// @brief 控制小车旋转
/// @param direction 0为逆时针，1为顺时针
/// @param angle 旋转角度，度
/// @param speed 旋转速度，度/s
/// @param acc 旋转加速度，度/s^2
void rotate(bool direction, float angle, float speed, int acc);

void caliPos(int targetPosX, int targetPosY, bool flag);