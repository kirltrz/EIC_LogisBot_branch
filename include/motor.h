#define LENGTH_OF_FRAME 11
// 第一个字节：帧头：为固定值0x7B
#define MOTOR_HEAD 0x7B
// 第二个字节：控制ID，默认为0x01。
#define MOTOR_ID_0 0x00
#define MOTOR_ID_1 0x01
#define MOTOR_ID_2 0x02
#define MOTOR_ID_3 0x03
// 第三个字节：控制模式，0x01为速度控制模式，0x02为位置控制模式(相对角度），0x03为力矩控制模式、0x04为单圈绝对角度控制模式。
#define MOTOR_MODE_SPEED 0x01
#define MOTOR_MODE_POSITION 0x02
#define MOTOR_MODE_FORCE 0x03
#define MOTOR_MODE_ANGLE 0x04
// 第四个字节：步进电机的转向，0为逆时针转动，1为顺时针转动。
#define MOTOR_DIRECTION_CCW 0x00 // 逆时针 (Counter-Clockwise)
#define MOTOR_DIRECTION_CW 0x01  // 顺时针 (Clockwise)
/*
第五个字节：步进电机细分值。例如：0x20=32，就是32细分，支持2、4、
8、16、32细分。（尽量不要在低下使用位置控制模式和绝对角度控制模式，会
影响精度）
*/
#define MOTOR_SUBDIVISION 0x20//32细分
#define MOTOR_END 0x7D

extern HardwareSerial* motorSerial;

void initMotor(void);
/**
 * @brief 控制电机运动
 * @param id 要控制的电机的ID,范围0-3
 * @param mode 电机运动模式，0x01为速度控制模式，0x02为位置控制模式(相对角度），0x03为力矩控制模式、0x04为单圈绝对角度控制模式
 * @param direc 步进电机的转向，0为顺时针转动，1为逆时针转动
 * @param pos 速度控制模式下为0；位置控制模式下的角度数据，单位为角度，放大十倍传输；力矩控制模式下的电流大小数据，单位为毫安；绝对角度控制模式下的目标角度，放大十倍发送
 * @param spd 运行速度，不超过44，超过30会概率触发卡死
 */
void motorWrite(int id, int mode, int direc, float angle, float spd);

uint32_t motorRead(int id);