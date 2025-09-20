
#ifndef DATABASE_H
#define DATABASE_H

/* =============================== Includes ======================================= */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* =============================== Structs ======================================= */

/**
 * @brief Inverter Status struct.
 * @note This struct contain the status of the inverter that upsated by incoming CAN messages.
 */
typedef struct {
uint8_t AMK_bReserve;
uint8_t AMK_bSystemReady;
uint8_t AMK_bError;
uint8_t AMK_bWarn;
uint8_t AMK_bQuitDCon;
uint8_t AMK_bDcOn;
uint8_t AMK_bQuitInverterOn;
uint8_t AMK_bInverterOn;
uint8_t AMK_bDerating;

} AMK_Status_t;

/**
 * @brief Inverter struct.
 * @note This struct is used to store the inverter paramets for the database layer
 */

 typedef struct{
    AMK_Status_t AMK_Status;
    int16_t torque; //0.1% Mn change to meaningful value

    int16_t dc_bus_voltage; //not sure about the unit
    int16_t dc_bus_voltage_monitoring; //not sure about the unit
    int32_t actual_power; //not sure about the unit
    int16_t actual_magnetizing_current; //not sure about the unit

    int16_t actual_speed;   //rpm
    int16_t torque_current; //Raw data to calculate 'actual torque current'
    int16_t magnetizing_current; //Raw data to calculate 'actual magnetizing current'
    /// Viarables from Actual values  2

    int16_t motor_temperature; //0.1 degree C change to meaningful value
    int16_t plate_temperature; //0.1 degree C change to meaningful value
    int16_t igbt_temperature; //0.1 degree C change to meaningful value
  
    
    struct {
        uint16_t control_word;
        int16_t target_velocity; //rpm
        int16_t positive_torque_limit; //0.1% Mn change to meaningful value
        int16_t negative_torque_limit; //0.1% Mn change to meaningful value

    }setpoints;
} inverter_t;


typedef enum{
	Stage1 = 1,
	Stage2 = 2,
    Stage2half = 25, 
    Stage3 = 3
}Stage_t;

typedef struct{
    uint16_t system_error;
    uint16_t inv1_error;
    uint16_t inv2_error;
    uint16_t inv3_error;
    uint16_t inv4_error;
    uint16_t canbus_error;
}error_group_t;

typedef enum{
     PEDALNODE  = 0,
     DBNODE =  1,
     INV1  = 2,
     INV2 = 3,
     INV3 = 4,
     INV4 = 5,
}keep_alive_t;

typedef struct{
    uint8_t buzzer_counter;
    uint8_t communication_counter;
    uint8_t hard_brake;
}counters_t;


/**
 * @brief Dashboard node struct.
 * @note This struct is used to store the Dashboard node paramets for the database layer
 */
typedef struct{
     uint8_t R2D;
}dashboard_node_t;


/**
 * @brief Pedal node struct.
 * @note This struct is used to store the Pedal node paramets for the database layer
 */

typedef struct{

    uint16_t gas_value;
    uint16_t brake_value;
    int16_t steering_wheel_angle;
    uint16_t BIOPS;

}pedal_node_t;



/**
 * @brief VCU node struct.
 * @note This struct is used to store the VCU node paramets for the database layer
 */

typedef struct{
    inverter_t inverters[4];
    error_group_t error_group;
    uint8_t keep_alive[6];
    counters_t counters;
    Stage_t fsm_stage ;
    uint8_t error_reset_flag;
}vcu_node_t;





/**
 * @brief DB struct.
 * @note This struct is used to store the pointers to all nodes in the DB
 */

typedef struct {

    pedal_node_t* pedal_node;
    vcu_node_t*	vcu_node;
    dashboard_node_t* dashboard_node;

} database_t;


/* ========================== Messages ID's =============================== */

#define INV1_AV1_ID 0x283
#define INV1_AV2_ID 0x285
#define INV1_Setpoints_ID 0x184

#define INV2_AV1_ID 0x284
#define INV2_AV2_ID 0x286
#define INV2_Setpoints_ID 0x185

#define INV3_AV1_ID 0x287
#define INV3_AV2_ID 0x289
#define INV3_Setpoints_ID 0x188

#define INV4_AV1_ID 0x288
#define INV4_AV2_ID 0x290
#define INV4_Setpoints_ID 0x189


#define BMS_ID 0x191
#define RES_ID 0x192
#define PEDAL_ID 0x193
#define DB_ID 0x194



/* ========================== Error codes =============================== */

#define NO_ERROR 0
#define PEDAL_COMMUNICTION_ERROR 1
#define DB_COMMUNICTION_ERROR 2
#define INV_COMMUNICTION_ERROR 3
#define SCS_SHORT_TO_GND_ERROR 4
#define SCS_SHORT_TO_VCC_ERROR 5
#define SENSORS_NOT_CALIBRATED_ERROR 6
#define HV_ERROR 7


/* =============================== Global Defines =============================== */

#define BRAKE_PEDAL_THRESHOLD 5 // Threshold for brake pedal pressed state
#define UC_GAS_VALUE 0xFF15
#define UC_BRAKE_VALUE 0xFF15
#define UC_SW_VALUE 0xFF15
#define UC_BIOPS_VALUE 0xFF15
#define SHORT_TO_GND_VALUE 0xFF10
#define SHORT_TO_VCC_VALUE 0xFF11

/**** Hard Brake (BPPC) Defines ******/
#define HB_ENTRY_TIMEOUT 18 // for 360 ms 
#define HB_EXIT_TIMEOUT 5  // for 100 ms
#define MIN_RANGE 0 //for Gas pedal
#define MAX_RANGE 1000 //for Gas pedal
#define HB_GAS_HIGH_VAL 250
#define HB_GAS_LOW_VAL 50
#define HB_BRAKE_HIGH_VAL 300


/* ========================== Function Declarations =============================== */

database_t* db_AllocateMemory();
void db_FreeMemory(database_t* db_ptr);
database_t* db_Init();
database_t* db_GetDBPointer();
#endif // DATABASE_H




