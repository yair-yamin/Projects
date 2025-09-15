#include "FSM.h"

// FSM: Manages the vehicle's state machine and main operational stages.

/* =============================== Global Variables =============================== */

static database_t* pMainDB = NULL;
static Stage_t* FSM_Stage = NULL; // Initial stage set to Stage1
uint8_t Sensors_Ok = 0; // Variable to check if sensors are OK
uint8_t Communication_Ok = 0; // Variable to check if communication is OK
uint8_t* BuzzerCounter =NULL;
static uint8_t *R2D_Pressed = 0; // Variable to check if R2D is pressed
static uint8_t *R2D_counter = 0;
uint8_t Brake_Pedal_Pressed = 0; // Variable to check if brake pedal is pressed
uint8_t HV_DETECTED = 0; // Variable to check if High Voltage is detected
uint8_t Inverters_OK = 0 ; // Variable to check if Inverters are well initialized

/* ========================== Function Definitions ============================ */

/**
 * @brief  Initializes the Finite State Machine (FSM).
 * @note   This function initializes the FSM by getting a pointer to the main
 *         database, setting the initial stage to Stage1, and initializing
 *         the operators module and buzzer counter.
 */
void FSM_Init(void)
{
    pMainDB = db_GetDBPointer();
    FSM_Stage = &pMainDB->vcu_node->fsm_stage;
    opr_Init();
    (*FSM_Stage) = Stage1; // Set initial stage to Stage1
    BuzzerCounter =  &pMainDB->vcu_node->counters.buzzer_counter;
}


/**
 * @brief  Executes the current stage of the Finite State Machine.
 * @note   This function acts as a router, calling the appropriate function
 *         based on the current value of the FSM_Stage variable.
 */
void FSM()
{
    switch(*FSM_Stage) {
        case Stage1:
            FSM_Stage_1();
            break;
        case Stage2:
            FSM_Stage_2();
            break;
        case Stage2half:
            FSM_Stage_2half();
            break;
        case Stage3:
            FSM_Stage_3();
            break;
        default:
            break;
    }
}

/**
 * @brief  Handles the logic for Stage 1: Initial Checks.
 * @note   In this stage, the system checks for sensor calibration and communication
 *         status. If both are nominal, it transitions to Stage 2. It also performs
 *         checks common to all stages.
 */
void FSM_Stage_1()
{
    FSM_InAnyStage();
    opr_Stage_Leds(Stage1); // Set the LED for Stage 1
    printf("Stage 1: Initializing\r\n");
    Sensors_Ok = opr_SensorsCheck(); // Check if sensors are calibrated  
    Communication_Ok = (opr_CommunicationCheck()>=100) ? 1:0; // Check if communication is OK 
    

    if(Sensors_Ok && Communication_Ok) {
        (*FSM_Stage) = Stage2; // Move to Stage 2 if sensors and communication are OK
        printf("Stage 2: Sensors and Communication OK\r\n");
    }
}

/**
 * @brief  Handles the logic for Stage 2: Ready to Drive Pre-check.
 * @note   This stage waits for the driver to press the "Ready to Drive" (R2D)
 *         button while the brake pedal is pressed and high voltage is present.
 *         Upon meeting these conditions, it transitions to Stage 2.5.
 */
void FSM_Stage_2()
{
    FSM_InAnyStage();
    opr_Stage_Leds(Stage2);
    InvertersInitFC();
    inv_CyclicTransmission();
    R2D_Pressed = &pMainDB->dashboard_node->R2D;
    Brake_Pedal_Pressed = (pMainDB->pedal_node->BIOPS > BRAKE_PEDAL_THRESHOLD) ? 1 : 0; // Check if brake pedal is pressed
    HV_DETECTED = inv_CheckHV();
    
    if((*R2D_Pressed) && (*R2D_counter) < R2D_TIMEOUT){
        if(Brake_Pedal_Pressed && HV_DETECTED) {
            *FSM_Stage = Stage2half;
            *R2D_Pressed = 0;
            *R2D_counter = 0;
            return;
        } else {
            (*R2D_counter)++;
        }
    }

    if(*R2D_counter >= R2D_TIMEOUT){
        *R2D_counter = 0;
        *R2D_Pressed = 0;
    }
}

/**
 * @brief  Handles the logic for Stage 2.5: Inverter Activation.
 * @note   This intermediate stage is responsible for turning on the inverters
 *         and verifying that they have initialized correctly. Once confirmed,
 *         it transitions to Stage 3.
 */
void FSM_Stage_2half()
{
    FSM_InAnyStage();
    opr_Stage_Leds(Stage2half); // Set the LED for Stage 2.5
    InvertersInitFC();
    inv_CyclicTransmission();
    inv_TurnOnBE1(); // Turn on BE1
    Inverters_OK = inv_CheckInit();
    
    if(Inverters_OK)
    {
        (*FSM_Stage) = Stage3; // Move to Stage 3 if Inverters are well initialized
        printf("Stage 3: Inverters Initialized, Ready to Drive\r\n");
        Inverters_OK = 0;

    }

    (*R2D_Pressed) = 0;
}

/**
 * @brief  Handles the logic for Stage 3: Driving.
 * @note   This is the main driving stage. It executes the driving routine,
 *         continuously checks for high voltage presence, sends cyclic CAN messages
 *         to the inverters, and activates the buzzer.
 */
void FSM_Stage_3()
{
   FSM_InAnyStage();
   opr_Stage_Leds(Stage3);
   inv_DrivingRoutine();
   if(!(inv_CheckHV()))
    {
        pMainDB->vcu_node->error_group.system_error = HV_ERROR;
    }
   inv_CyclicTransmission();
   opr_Buzzer();
}


/**
 * @brief  Executes operations common to all FSM stages.
 * @note   This function is called from every stage to perform essential,
 *         state-independent tasks like processing incoming CAN messages,
 *         checking for short circuits, monitoring node keep-alives, and
 *         controlling the brake light.
 */
void FSM_InAnyStage()
{
    plt_CanProcessRxMsgs();
    opr_SCSCheck();
    opr_KeepAliveCheck();
    inv_CheckInvertersError();
    opr_BrakeLight();
    FSM_Error_Handler();
}

/**
 * @brief  Handles system errors based on the current error code.
 * @note   This function is called to manage system faults. It takes specific
 *         actions depending on the error code, such as resetting the FSM to
 *         Stage 1, resetting inverters, or printing error messages.
 */
void FSM_Error_Handler(void)
{
    error_group_t* pErrorGroup = &pMainDB->vcu_node->error_group;

    switch (pErrorGroup->system_error)
    {
    case NO_ERROR:
        return;
        break;
    case PEDAL_COMMUNICTION_ERROR:
        printf("Pedal Communication Error Detected\r\n");
        break;
    case INV_COMMUNICTION_ERROR:
        (*FSM_Stage) = Stage1;
        inv_set_ErrorReset();
        HAL_GPIO_WritePin(BE1_GROUP,BE1_PIN,RESET);
        (*BuzzerCounter) = 0 ;
        break;

    case HV_ERROR:
        (*FSM_Stage) = Stage1;
        inv_set_ErrorReset();
        printf("HV fall Detected\r\n");
        break;
    default:
        break;
    }
}

