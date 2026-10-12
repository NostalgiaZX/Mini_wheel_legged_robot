
#ifndef MINI_WHEEL_LEGGED_ROBOT_ST3215_H
#define MINI_WHEEL_LEGGED_ROBOT_ST3215_H
#include "main.h"
#include  "usart.h"
#include  "servo.h"
#define datalen_write 26
#define datalen_read 14 // sync read request frame length (FF FF FE LEN 82 ADDR DATALEN id1..id6 CHECKSUM)

#define SERVO_NUM 6
#define READ_PARAM_LEN 15 // bytes read per servo starting at addr 0x38: position,speed,load,voltage,temp,asyncflag,status,moving,targetpos,current
#define READ_FRAME_LEN (READ_PARAM_LEN + 6) // FF FF ID LEN STATUS <params> CHECKSUM
#define SERVO_READ_TIMEOUT_MS 20 // 一轮读取的最长等待时间，超时则放弃仍在READING的舵机

extern uint8_t data_write[datalen_write];
extern uint8_t data_read[datalen_read];

extern uint8_t ptrread;

#ifdef __cplusplus
extern "C"{
#endif
void addchecksum();
void applyintensity();
void servos_writing();
void servos_reading();
uint8_t servos_all_idle();
void servos_read_timeout();
void st3215_on_byte_received(uint8_t byte);
#ifdef __cplusplus
}
#endif
#endif