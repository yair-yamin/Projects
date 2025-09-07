#ifndef OPERATORS_H
#define OPERATORS_H
#include "callbacks.h"

/******Stages LED's Defines *******/
#define STAGE1_LED_GROUP   GPIOC
#define STAGE1_LED_PIN     GPIO_PIN_8

#define STAGE2_LED_GROUP   GPIOC
#define STAGE2_LED_PIN     GPIO_PIN_6

#define STAGE3_LED_GROUP   GPIOB
#define STAGE3_LED_PIN     GPIO_PIN_9

#define BRAKE_LIGHT_GROUP   GPIOB
#define BRAKE_LIGHT_PIN     GPIO_PIN_10

/******BUZZER Defines *******/
#define BUZZER_TIMER Tim2
#define BUZZER_TIMER_CH TIM_CHANNEL_2
#define BUZZER_FRQ 1500 // Frequency in Hz
#define BUZZER_DC 55.0f // Duty cycle in percentage (0-100)
#define BUZZER_TIMEOUT 150 // Timeout in milliseconds
#define BRAKE_LIGHT_TRASHOLD 5
#define LastMSG_TIMEOUT 220 
#define BUZZER_1_FREQ 2000
#define BUZZER_2_FREQ 5300
#define BUZZER_STOP_VAL 200


/* **************************Functions Declarations *****************************  */
void opr_Stage_Leds(Stage_t stage);
void opr_Init(void);
void opr_SCSCheck(void);
uint8_t opr_SensorsCheck();
uint8_t opr_CommunicationCheck();
void opr_KeepAliveCheck();
void opr_ClearKLList() ;
void opr_BrakeLight() ;
void opr_Buzzer();
#endif