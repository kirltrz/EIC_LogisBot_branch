#include "RPI.h"

extern bool rpiReady;
extern bool hwt101Ready;
extern bool motorReady;
extern bool servoReady;
extern bool startup;

extern bool uniYawAdjustOn;
extern bool yawAdjustOn;
extern float deadband; // 死区角度
extern float targetYaw;
extern float currentYaw;
extern bool YawPIDmotor[4];
extern volatile int16_t dx;
extern volatile int16_t dy;
extern volatile double globalX;
extern volatile double globalY;

extern int taskCode1[3];
extern int taskCode2[3];
extern char taskCode1Full[5];
extern char taskCode2Full[5];

extern int temperOffset[5];
extern int Kt;
extern int temperRefer;

extern bool uniStop;
extern bool emergencyStop;