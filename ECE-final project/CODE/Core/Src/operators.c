#include "operators.h"

// Operators: High-level vehicle logic, sensor checks, and actuator control.

/* =============================== Global Variables =============================== */
static database_t* pMainDB = NULL;
static uint16_t* pSystemError = NULL;

/* ========================== Function Definitions ============================ */

/**
  * @brief  Initializes the operators module.
  * @note   This function initializes the operators module by getting pointers to the main database and system error variable.
*/
void opr_Init()
{
    pMainDB = db_GetDBPointer();
    pSystemError = &pMainDB->vcu_node->error_group.system_error;
}


/**
  * @brief  Controls the stage indicator LEDs.
  * @param  stage The current FSM stage
  * @note   This function turns on the LED corresponding to the current stage and turns off the others.
*/
void opr_Stage_Leds(Stage_t stage)
{
    // Turn off all stage LEDs first
    HAL_GPIO_WritePin(STAGE1_LED_GROUP, STAGE1_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAGE2_LED_GROUP, STAGE2_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAGE3_LED_GROUP, STAGE3_LED_PIN, GPIO_PIN_RESET);

    // Turn on the selected stage LED
    switch(stage) {
        case Stage1:
            HAL_GPIO_WritePin(STAGE1_LED_GROUP, STAGE1_LED_PIN, GPIO_PIN_SET);
            break;
        case Stage2:
            HAL_GPIO_WritePin(STAGE2_LED_GROUP, STAGE2_LED_PIN, GPIO_PIN_SET);
            break;
        case Stage2half:
            HAL_GPIO_TogglePin(STAGE2_LED_GROUP, STAGE2_LED_PIN); // Toggle Stage 2 LED for 2half stage
            break;
        case Stage3:
            HAL_GPIO_WritePin(STAGE3_LED_GROUP, STAGE3_LED_PIN, GPIO_PIN_SET);
            break;
        default:
            // Do nothing if invalid stage
            break;
    }
}




/**
  * @brief  Manages the buzzer.
  * @note   This function activates the buzzer for a predefined duration.
*/
void opr_Buzzer()
{
    pMainDB = db_GetDBPointer();
    uint8_t* BuzzerCounter =&pMainDB->vcu_node->counters.buzzer_counter;
    if(*BuzzerCounter == 0)
    {
        plt_StartPWM(BUZZER_TIMER, BUZZER_TIMER_CH, BUZZER_2_FREQ, BUZZER_DC);
        (*BuzzerCounter) ++ ;
        return;
    }
    if(*BuzzerCounter >= BUZZER_TIMEOUT)
    {
        plt_StopPWM(BUZZER_TIMER, BUZZER_TIMER_CH);
        (*BuzzerCounter) = BUZZER_STOP_VAL;        
        return;
    }
    if(*BuzzerCounter < BUZZER_STOP_VAL)
    {
         (*BuzzerCounter) ++ ;
    }
}

/**
  * @brief  Checks if critical sensors are uncalibrated.
  * @retval 1 if all sensors are calibrated, 0 otherwise.
  * @note   This function checks the gas pedal, brake pedal, steering wheel, and BIOPS for uncalibrated values.
*/
uint8_t opr_SensorsCheck()
{
    uint16_t Gas_Value = pMainDB->pedal_node->gas_value;
    uint16_t Brake_Value = pMainDB->pedal_node->brake_value;
    uint16_t SWValue = pMainDB->pedal_node->steering_wheel_angle;
    uint16_t BIOPSValues = pMainDB->pedal_node->BIOPS ;
    if(Gas_Value == UC_GAS_VALUE  || Brake_Value == UC_BRAKE_VALUE || SWValue == UC_SW_VALUE || BIOPSValues == UC_BIOPS_VALUE)
    {
        (*pSystemError) = SENSORS_NOT_CALIBRATED_ERROR;
        return 0 ;
    }
    return 1;
}

/**
  * @brief  Clears the keep-alive list.
  * @note   This function resets the keep-alive status for all monitored nodes.
*/
void opr_ClearKLList()
{
     uint8_t* KL = pMainDB->vcu_node->keep_alive;
    for (keep_alive_t NODE=PEDALNODE;NODE<=INV4;NODE ++)
    {
        KL[NODE] = 0 ;
    }
}
/**
  * @brief  Checks for communication from all nodes.
  * @retval Returns the ID of the first node that has not communicated, or 100 if all are active.
  * @note   This function iterates through the keep-alive list to find silent nodes.
*/
uint8_t opr_CommunicationCheck()
{
    uint8_t* KL = pMainDB->vcu_node->keep_alive;
    for (keep_alive_t NODE=PEDALNODE;NODE<=INV4;NODE ++)
    {
        if(KL[NODE] == 0)
        {
            return NODE;
        }
    }
    return 100 ;    
}

/**
  * @brief  Periodically checks for communication timeouts.
  * @note   This function increments a counter and, upon timeout, checks for silent nodes,
  *         sets a system error if any are found, and resets the keep-alive list.
*/
void opr_KeepAliveCheck()
{
    uint8_t NODE ;
    uint8_t* counter = &pMainDB->vcu_node->counters.communication_counter;
    (*counter) ++ ;
    if((*counter) >= LastMSG_TIMEOUT)
    {
        NODE = opr_CommunicationCheck();
        if(NODE != 100)
        {
            switch (NODE)
            {
                case PEDALNODE:
                    (*pSystemError) = PEDAL_COMMUNICTION_ERROR;
                    break;
                case DBNODE:
                    (*pSystemError) = DB_COMMUNICTION_ERROR;
                    break;
                case INV1:
                    (*pSystemError) = INV_COMMUNICTION_ERROR;
                    break;
                case INV2:
                    (*pSystemError) = INV_COMMUNICTION_ERROR;
                    break;
                case INV3:
                    (*pSystemError) = INV_COMMUNICTION_ERROR;
                    break;
                case INV4:
                    (*pSystemError) = INV_COMMUNICTION_ERROR;
                    break;
                default:
                    break;
            }
        }
        opr_ClearKLList();
        (*counter) = 0 ;
    }
}

/**
  * @brief  Checks for short-circuit conditions in the pedal sensors.
  * @note   This function checks for short-to-ground or short-to-VCC faults in the gas and brake pedals.
*/
void opr_SCSCheck(void)
{
    
    uint16_t Gas_Value = pMainDB->pedal_node->gas_value;
    uint16_t Brake_Value = pMainDB->pedal_node->brake_value;
    if(Gas_Value == SHORT_TO_GND_VALUE || Brake_Value == SHORT_TO_GND_VALUE)
    {
        (*pSystemError) = SCS_SHORT_TO_GND_ERROR;
    }
    if(Gas_Value == SHORT_TO_VCC_VALUE || Brake_Value == SHORT_TO_VCC_VALUE)
    {
        (*pSystemError) = SCS_SHORT_TO_VCC_ERROR;
    }
}

/**
  * @brief  Controls the brake light.
  * @note   This function activates the brake light if the brake pressure exceeds a certain threshold.
*/
void opr_BrakeLight()
{
    if(pMainDB->pedal_node->BIOPS > BRAKE_LIGHT_TRASHOLD)
    {
        HAL_GPIO_WritePin(BRAKE_LIGHT_GROUP,BRAKE_LIGHT_PIN,SET);
    }
    else
    {
        HAL_GPIO_WritePin(BRAKE_LIGHT_GROUP,BRAKE_LIGHT_PIN,RESET);
    }
}