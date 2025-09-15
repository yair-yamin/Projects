#ifndef INVERTERS_H
#define INVERTERS_H
#include "callbacks.h"

/* =============================== Inverters Defines =============================== */
#define bInverterOn 0x01
#define bDcOn 0x01 << 1
#define bEnable 0x01 << 2
#define bErrorReset 0x01 << 3

#define BE1_PIN GPIO_PIN_8
#define BE1_GROUP GPIOB
#define INV12_CAN Can1
#define INV34_CAN Can1
#define MAX_VELOCITY 1000


/* ========================== Function Declarations =============================== */

void InvertersInitFC(void);
void inv_CyclicTransmission(void);
void inv_SetInvParameters_FC(int16_t posTorqueLimit, int16_t negTorqueLimit);
void inv_SetZeroTorque(int16_t posTorqueLimit, int16_t negTorqueLimit);
void inv_DrivingRoutine();
uint8_t inv_CheckInit();
uint8_t inv_CheckHV();
void inv_TurnOnBE1();
void inv_set_ErrorReset();
void inv_CheckInvertersError();
#endif