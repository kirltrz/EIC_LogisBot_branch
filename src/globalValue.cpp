#include "globalValue.h"
bool rpiReady = 0;
bool hwt101Ready = 0;
bool motorReady = 1;
bool servoReady = 0;
bool startup = 0;

bool uniYawAdjustOn=0;
bool yawAdjustOn = 0;
float deadband = 0.3f;
float targetYaw = 0.0f;
float currentYaw = 0.0f;
bool YawPIDmotor[4] = {1, 1, 1, 1};
volatile int16_t dx = 0;
volatile int16_t dy = 0;
volatile double globalX = 0;
volatile double globalY = 0;

int taskCode1[3] = {-1, -1, -1};
int taskCode2[3] = {-1, -1, -1};
char taskCode1Full[5];
char taskCode2Full[5];

int temperOffset[5] = {0, 0, 0, 0, 0};
int Kt = 1;
int temperRefer = 27;

bool uniStop=false;
bool emergencyStop = false;