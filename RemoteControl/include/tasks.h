#ifndef TASKS_h
#define TASKS_h

#include <Arduino.h>

#ifdef USE_MULTI_THREAD
void tft_task(void *pvParameters);
#endif

#endif