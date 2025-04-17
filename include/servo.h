#include <SCServo.h>

#define GLOBAL_SERVO_SPEED 3400
#define GLOBAL_SERVO_ACC 50
#define CLOSE 1
#define OPEN 0
#define BIG_OPEN 2

extern SMS_STS servo;

void initServo(void);
bool checkServo(void);
void stopAllServo(void);
int writeServo(u8 ID, s16 Position, u16 Speed = GLOBAL_SERVO_SPEED, u8 ACC = GLOBAL_SERVO_ACC);
int regWriteServo(u8 ID, s16 Position, u16 Speed = GLOBAL_SERVO_SPEED, u8 ACC = GLOBAL_SERVO_ACC);
int regWriteServoAction(u8 ID = 0xfe);
void syncWriteServo(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);
int readServo(int ID);

// 控制机械爪，参数1闭0开，2大开
void setClaw(int state);