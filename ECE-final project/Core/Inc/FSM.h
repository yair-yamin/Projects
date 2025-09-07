#ifndef FSM_H
#define FSM_H

#include "inverters.h"
#include "operators.h"

#define R2D_TIMEOUT 10
/* **************************Functions Declarations *****************************  */
void FSM_Init(void);
void FSM();
void FSM_Stage_1(void);
void FSM_Stage_2(void);
void FSM_Stage_2half(void);
void FSM_Stage_3(void);
void FSM_InAnyStage(void);
void FSM_Error_Handler(void);
#endif