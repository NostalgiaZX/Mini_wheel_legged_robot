
#ifndef MINI_WHEEL_LEGGED_ROBOT_ST3215_H
#define MINI_WHEEL_LEGGED_ROBOT_ST3215_H
#include "main.h"
#include  "usart.h"
#define datalen 26
extern uint8_t data[datalen];
#ifdef __cplusplus
extern "C"{
#endif
void addchecksum();
void applyintensity();
void transmitdata();

#ifdef __cplusplus
}
#endif
#endif