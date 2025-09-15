#include "inverters.h"

// Inverters: Handles communication and control of the motor inverters.


/* =============================== Global Variables =============================== */
static database_t *pMainDB = NULL;
static CanChanel_t channel;
static uint8_t BPPC = 0;
static uint8_t *R2D_Pressed = 0; // Variable to check if R2D is pressed

static can_message_t INV_Setpoints_msgs[] = {
    {INV1_Setpoints_ID, {0}},
    {INV2_Setpoints_ID, {0}},
    {INV3_Setpoints_ID, {0}},
    {INV4_Setpoints_ID, {0}}};

/* ========================== Function Definitions ============================ */

/**
 * @brief  Initializes the inverters and sets the initial parameters.
 * @note   This function initializes the inverters and sets the initial parameters.

 */
void InvertersInitFC(void)
{


    pMainDB = db_GetDBPointer();
    R2D_Pressed = &pMainDB->dashboard_node->R2D;

    for (uint8_t i = 0; i < 4; i++)
    {
        channel = (i < 2) ? INV12_CAN : INV34_CAN;// Select the CAN channel based on the inverter index
        AMK_Status_t *Inv_status = &pMainDB->vcu_node->inverters[i].AMK_Status;

        INV_Setpoints_msgs[i].data[2] = 0;
        INV_Setpoints_msgs[i].data[3] = 0;
        if(Inv_status->AMK_bQuitInverterOn)
        {
            inv_SetInvParameters_FC(1000,-1000);
        }
        INV_Setpoints_msgs[i].data[1] |= bDcOn;
        INV_Setpoints_msgs[i].data[1] |= bInverterOn;
        INV_Setpoints_msgs[i].data[1] |= bEnable; // Set the Enable bit to 1
        
    }
    
}

/**
 * @brief  Sets the velocity and torque limits for all inverters.
 * @param  posTorqueLimit The positive torque limit to set.
 * @param  negTorqueLimit The negative torque limit to set.
 * @retval None
 * @note   This function calculates the target velocity based on the gas pedal input
 *         and updates the setpoint messages for all inverters with the new
 *         velocity and torque limits.
 */
void inv_SetInvParameters_FC(int16_t posTorqueLimit, int16_t negTorqueLimit)
{
  
    int16_t velocity = MAX_VELOCITY * ((float)pMainDB->pedal_node->gas_value / 100);

    for(int i=0 ;i<4;i++)
    {
        INV_Setpoints_msgs[i].data[2] = (uint8_t)(velocity & 0xFF);
        INV_Setpoints_msgs[i].data[3] = (uint8_t)(velocity >> 8);
        INV_Setpoints_msgs[i].data[4] = (uint8_t)(posTorqueLimit & 0xFF);
        INV_Setpoints_msgs[i].data[5] = (uint8_t)(posTorqueLimit >> 8);
        INV_Setpoints_msgs[i].data[6] = (uint8_t)(negTorqueLimit & 0xFF);
        INV_Setpoints_msgs[i].data[7] = (uint8_t)(negTorqueLimit >> 8);
    }
}

/**
 * @brief  Commands zero velocity (torque) to all inverters.
 * @param  posTorqueLimit The positive torque limit to maintain.
 * @param  negTorqueLimit The negative torque limit to maintain.
 * @retval None
 * @note   This function sets the target velocity to zero in the setpoint messages
 *         for all inverters, effectively commanding zero torque, while keeping the
 *         specified torque limits.
 */
void inv_SetZeroTorque(int16_t posTorqueLimit, int16_t negTorqueLimit)
{
    for(int i=0; i<4; i++){
        INV_Setpoints_msgs[i].data[2] = (uint8_t)(0 & 0xFF);
        INV_Setpoints_msgs[i].data[3] = (uint8_t)(0 >> 8);
        INV_Setpoints_msgs[i].data[4] = (uint8_t)(posTorqueLimit & 0XFF);
        INV_Setpoints_msgs[i].data[5] = (uint8_t)(posTorqueLimit >> 8);
        INV_Setpoints_msgs[i].data[6] = (uint8_t)(negTorqueLimit & 0xFF);
        INV_Setpoints_msgs[i].data[7] = (uint8_t)(negTorqueLimit >> 8);
    }
}


/**
 * @brief Send Setpoints values to the inverters every timer elpsed.
 * @note This function sends the Setpoints values to the inverters every timer elpsed.
 */
void inv_CyclicTransmission(void)
{
        for (uint8_t i = 0; i < 4; i++)
        {
            channel = (i < 2) ? INV12_CAN : INV34_CAN;          
            INV_Setpoints_msgs[i].data[1] |= bDcOn;
            plt_CanSendMsg(channel, &INV_Setpoints_msgs[i]);
            HAL_Delay(1);
        }
        printf("Sending Setpoints to the Inverters \r\n");
}

/**
 * @brief  Checks if the high voltage system is active on all inverters.
 * @param  None
 * @retval 1 if HV is active on all inverters, 0 otherwise.
 * @note   This function checks the status flags of all four inverters to confirm
 *         that the DC bus is energized and acknowledged.
 */
uint8_t inv_CheckHV()
{
    inverter_t* pInverters = pMainDB->vcu_node->inverters;
    for (int i = 0; i < 4; ++i) {

        if (pInverters[i].AMK_Status.AMK_bQuitDCon != 1 ||
            pInverters[i].AMK_Status.AMK_bDcOn != 1) {
            return 0;
        }
    }
    printf("High Voltage Detected \r\n");
    return 1;
}


/**
 * @brief  Turns on the BE1 pin as part of the inverter initialization sequence.
 * @note   This function verifies that all inverters are at the right position before enabling the BE1 pin.
 */
void inv_TurnOnBE1()
{
    inverter_t* pInverters = pMainDB->vcu_node->inverters;
    for (int i = 0; i < 4; ++i) {
        if (pInverters[i].AMK_Status.AMK_bInverterOn != 1) {
            return ;
        }
    }
    HAL_GPIO_WritePin(BE1_GROUP,BE1_PIN,SET);
    printf("BE1 Turned ON \r\n");
}

/**
 * @brief  Checks if all inverters have successfully initialized.
 * @param  None
 * @retval 1 if all inverters are initialized, 0 otherwise.
 * @note   This function verifies that all inverters have acknowledged the
 *         "InverterOn" command and are ready for operation.
 */
uint8_t inv_CheckInit()
{
    inverter_t* pInverters = pMainDB->vcu_node->inverters;
    for (int i = 0; i < 4; ++i) {
        if (pInverters[i].AMK_Status.AMK_bQuitInverterOn != 1 ||
            pInverters[i].AMK_Status.AMK_bInverterOn != 1) {
            return 0;
        }
    }
    printf("Inverters Init  Succeeded\r\n");
    return 1;
}

/**
 * @brief  Manages the driving logic, including the Brake Pedal Plausibility Check (BPPC).
 * @param  None
 * @retval None
 * @note   This function implements a state machine for hard braking (BPPC).
 *         It commands zero torque if the gas and brake pedals are pressed
 *         simultaneously (Hard Brake state). Otherwise, it sends normal torque
 *         commands based on the gas pedal position.
 */
void inv_DrivingRoutine()
{   
    uint16_t Gas_Value = pMainDB->pedal_node->gas_value;
    uint16_t Brake_Value = pMainDB->pedal_node->brake_value;
    uint8_t* counter = &pMainDB->vcu_node->counters.hard_brake;

    
    // --- Hard Brake (BPPC) State Machine ---

    if (BPPC) // Currently in Hard Brake state
    {
        // Condition to exit Hard Brake state: Gas pedal is released
        if (Gas_Value <= HB_GAS_LOW_VAL)
        {
            (*counter)++;
            if ((*counter) >= HB_EXIT_TIMEOUT)
            {
                printf("Hard Brake Released \r\n");
                BPPC = 0;
                (*counter) = 0;
            }
        }
        else // Gas pedal still pressed, reset exit timer
        {
            (*counter) = 0;
        }
        // While in Hard Brake state or during exit timeout, command zero torque
        inv_SetZeroTorque(1000, -1000);
    }
    else // Not in Hard Brake state
    {
        // Condition to enter Hard Brake state: Gas and Brake pedals pressed simultaneously
        if (Gas_Value >= HB_GAS_HIGH_VAL && Brake_Value >= HB_BRAKE_HIGH_VAL)
        {
            (*counter)++;
            if ((*counter) >= HB_ENTRY_TIMEOUT)
            {
                printf("Hard Brake Detected \r\n");
                BPPC = 1;
                (*counter) = 0;
                inv_SetZeroTorque(1000, -1000); // Immediately cut torque
            }
        }
        else // Normal driving condition
        {
            (*counter) = 0;
            InvertersInitFC(); // Send torque based on gas value
        }
    }
    
}

/**
 * @brief  Sends an error reset command to all inverters.
 * @param  None
 * @retval None
 * @note   This function sets the "bErrorReset" bit in the setpoint messages for
 *         all inverters and commands zero torque to attempt to clear any existing faults.
 */
void inv_set_ErrorReset()
{
    for (int i=0; i<4; i++){
        inv_SetZeroTorque(0,0);
        INV_Setpoints_msgs[i].data[1] |= bErrorReset;
    }
}

/**
 * @brief  Checks for error flags from the inverters.
 * @param  None
 * @retval None
 * @note   This function checks the error status bit of the inverters. If an error
 *         is detected, it sets the system error code to indicate an inverter
 *         communication/fault error.
 */
void inv_CheckInvertersError()
{
    uint16_t* pSystemError = &pMainDB->vcu_node->error_group.system_error;
    uint8_t reset_indicator = 0;
    for (int i=0; i<2; i++){
    reset_indicator |= pMainDB->vcu_node->inverters[i].AMK_Status.AMK_bError;
    }
    if(reset_indicator){
        (*pSystemError) = INV_COMMUNICTION_ERROR;
    }
}
