/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Norhan Khairy
 ******************************************************************************/
#ifndef PORT_H
#define PORT_H

/* Id for the company in Autosar
 * I will stick to Mohamed Tarek's ID
 */
#define PORT_VENDOR_ID		(1000U)

/* Port Module ID */
#define PORT_MODULE_ID		(124U)

/* Port Instance ID */
#define PORT_INSTANCE_ID	(0U)

/* Module Version 1.0.0 */
#define PORT_SW_MAJOR_VERSION			(1U)
#define PORT_SW_MINOR_VERSION			(0U)
#define	PORT_SW_PATCH_VERSION			(1U)

/* AUTOSAR Version 4.0.3 */
#define PORT_AR_RELEASE_MAJOR_VERSION	(4U)
#define PORT_AR_RELEASE_MINOR_VERSION	(0U)
#define PORT_AR_RELEASE_PATCH_VERSION	(3U)

/* 
 * Port Status Macros 
 */
#define PORT_INITIALIZED			(1U)
#define PORT_UNINITIALIZED			(0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR Version checking between Std Types and Port Module */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header */
#include "Port_cfg.h"

/* AUTOSAR Version checking between Port_cfg.h and  Port Module files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_cfg.h and Port Module files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_cfg.h does not match the expected version"
#endif

/* Non AUTOSAR Header files */
#include "Common_Macros.h"


/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port_init */
#define PORT_INIT_SID                     (uint8)0x00

/* Service ID for Port_SetPinDirection */
#define PORT_SET_PIN_DIRECTION_SID        (uint8)0x01

/* Service ID for Port_RefreshPortDirection */
#define PORT_REFRESH_PORT_DIRECTION_SID   (uint8)0x02

/* Service ID for Port getVersionInfo */
#define PORT_GET_VERSION_INFO_SID         (uint8)0x03

/* Service ID for Port_SetPinMode */
#define PORT_SET_PIN_MODE_SID             (uint8)0x04

 /*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET error for invalid port pin id */
#define PORT_E_PARAM_PIN                  (uint8)0x0A

/* DET error for port pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE	    (uint8)0x0B

/* DET error for Port_init API service called with NULL Pointer parameter */
#define PORT_E_PARAM_CONFIG               (uint8)0x0C

/* DET error for Port_SetPinMode API service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE         (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE          (uint8)0x0E

/* DET error for any API service called without module initialization */
#define PORT_E_UNINIT                     (uint8)0x0F

/* DET error for any API service called with a NULL pointer */
#define PORT_E_PARAM_POINTER              (uint8)0x10

/* DET error for accessing JTAG pins */
#define PORT_JTAG_ACCESS                  (uint8)0x00

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT,OUTPUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: a variable to cover all available port pins number (0-42) */
typedef uint8 Port_Pin;

/* Description: a variable for all possible modes(alternative functions) that could be configured on a specific pin */
typedef uint8 Port_PinMode;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *	5. the initial value of the pin
 *	6. the pin mode/alternative function
 *	7. the changeability of the pin's direction during runtime (STD_ON, or STD_OFF)
 *	8. the changeability of the pin's mode during runtime (STD_ON, or STD_OFF)
 */
typedef struct 
{
    uint8 port_num; 
    Port_Pin pin_num; 
    Port_PinDirection direction;
    Port_InternalResistor resistor;
    uint8 initial_value;
    Port_PinMode pin_mode;
    boolean pin_direction_changeable;
    boolean pin_mode_changeable;

}Port_Config_Channels;

/* A data structure containing the initialization data for this module */
typedef struct {
	Port_Config_Channels Port_channels[PORT_MAX_PINS];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/* Function for initializing all port pins for Port API */
void Port_Init(const Port_ConfigType* ConfigPtr);

/* Function that sets the port pin direction during runtime */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_Pin Pin, Port_PinDirection Direction);
#endif

/* Function to refresh the configured direction of all configured ports */
void Port_RefreshPortDirection(void);

/* Function to get Port Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/* Function to set the port pin mode for the referenced pin during runtime */
void Port_SetPinMode(Port_Pin Pin, Port_PinMode Mode);

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
extern const Port_ConfigType Port_ConfigurationArray;


/*******************************************************************************
 *                       Pins & Ports Macros                                   *
 *******************************************************************************/
/* Ports IDs */
#define PORT_A 		(uint8)0x00
#define PORT_B 		(uint8)0x01
#define PORT_C 		(uint8)0x02
#define PORT_D 		(uint8)0x03
#define PORT_E 		(uint8)0x04
#define PORT_F 		(uint8)0x05

/* Pins IDs */
#define PA0		(Port_Pin)0
#define PA1		(Port_Pin)1
#define PA2		(Port_Pin)2
#define PA3		(Port_Pin)3
#define PA4		(Port_Pin)4
#define PA5		(Port_Pin)5
#define PA6		(Port_Pin)6
#define PA7		(Port_Pin)7
#define PB0		(Port_Pin)8
#define PB1		(Port_Pin)9
#define PB2		(Port_Pin)10
#define PB3		(Port_Pin)11
#define PB4		(Port_Pin)12
#define PB5		(Port_Pin)13
#define PB6		(Port_Pin)14
#define PB7		(Port_Pin)15
#define PC0		(Port_Pin)16
#define PC1		(Port_Pin)17
#define PC2		(Port_Pin)18
#define PC3		(Port_Pin)19
#define PC4		(Port_Pin)20
#define PC5		(Port_Pin)21
#define PC6		(Port_Pin)22
#define PC7		(Port_Pin)23
#define PD0		(Port_Pin)24
#define PD1		(Port_Pin)25
#define PD2		(Port_Pin)26
#define PD3		(Port_Pin)27
#define PD4		(Port_Pin)28
#define PD5		(Port_Pin)29
#define PD6		(Port_Pin)30
#define PD7		(Port_Pin)31
#define PE0		(Port_Pin)32
#define PE1		(Port_Pin)33
#define PE2		(Port_Pin)34
#define PE3		(Port_Pin)35
#define PE4		(Port_Pin)36
#define PE5		(Port_Pin)37
#define PF0		(Port_Pin)38
#define PF1		(Port_Pin)39
#define PF2		(Port_Pin)40
#define PF3		(Port_Pin)41
#define PF4		(Port_Pin)42

#define PortA_START (0U)
#define PortB_START	(8U)
#define PortC_START (16U)
#define PortD_START (24U)
#define PortE_START	(32U)
#define PortF_START (38U)

/*******************************************************************************
 *                       Other Definitions                                     *
 *******************************************************************************/
/* Conversion to 8-bit pointer */
#define UINT8_PTR	volatile uint8 *

/* Conversion to 32-bit pointer */
#define UINT32_PTR	volatile uint32 *

#endif /* PORT_H */