#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>    // 信号量相关定义
void initTaskManager(void);

extern TaskHandle_t *mainSequenceHandle;
extern SemaphoreHandle_t mutex;