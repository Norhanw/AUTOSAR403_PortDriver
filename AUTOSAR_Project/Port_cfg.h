 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Norhan Khairy
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/* Module Version 1.0.0 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (1U)

/* AUTOSAR Version 4.0.3 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT			(STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API           (STD_OFF)

/* Pre-compile option for Port_Set_PinDirection API */
#define PORT_SET_PIN_DIRECTION_API		(STD_ON)

/* Maximum number of pins */
#define PORT_MAX_PINS			(uint8)43

/* Maximum Number of Modes */
#define PORT_MAX_PIN_MODE		(uint8)16

/* Analog ADC mode */
#define ADC_MODE		(uint8)0

/* SSI mode for module 3 (SSI3Clk, SSI3Fss, SSI3Rx, SSI3Tx)*/
#define SSI3_MODE		(uint8)0x1

/* UART mode for modules 0~7 and for UART request-to-send or UART clear to send modes
 * U#Rx, U#Tx (# = 0~7), U1RTS, U1CTS
 */
#define UART_MODE 		(uint8)0x1

/* SSI mode for modules 0,1,2
 * SSI#Clk, SSI#Fss, SSI#Rx, SSI#Tx (# = 0~2)
 */
#define SSI_MODE 		(uint8)0x2

/* UART mode for module 1 in PC4 and PC5 */
#define UART1_MODE		(uint8)0x2

/* CAN mode (CAN0Rx, CAN0Tx) for PF0 and PF3 */
#define CAN0_MODE		(uint8)0x3

/* I2C mode */
#define I2C_MODE		(uint8)0x3

/* PWM mode for module 0
 * M0PWM# (# = 0~7), M0FAULT0
 */
#define PWM0_MODE		(uint8)0x4

/* PWM mode for module 1
 * M1PWM# (# = 0~7), M1FAULT0
 */
#define PWM1_MODE		(uint8)0x5

/* QEI mode
 * IDX#, PHA#, PHB# (# = 0~1)
 */
#define QEI_MODE		(uint8)0x6

/* GPT mode */
#define GPT_MODE		(uint8)0x7

/* CAN mode (CAN0Rx, CAN0Tx, CAN1Rx, CAN1Tx) for other pins */
#define CAN1_MODE		(uint8)0x8

/* UART mode for module 1 Clear To Send, and Request to Send for PC4 and PC5 */
#define UART_CTL_MODE	(uint8)0x8

/* USB mode */
#define USB_MODE 		(uint8)0x8

/* NMI mode */
#define NMI_MODE		(uint8)0x8

/* Analog comparator output mode 
 * C0o, C1o
 */
#define ANALOG_CMP_MODE	(uint8)0x9

/* Core trace mode (TRD1, TRD0, TRCLK) */
#define TRACE_MODE		(uint8)0xE

/* Digital IO mode, this mode number is not used on the microcontroller */
#define DIO_MODE		(uint8)0xA

/* Default pin direction */
#define PORT_DEFAULT_PIN_DIRECTION		(INPUT)

/* Default pin initial value */
#define PORT_DEFAULT_PIN_INITIAL_VALUE	(STD_LOW)

/* Default pin mode */
#define PORT_DEFAULT_PIN_MODE 			(DIO_MODE)

/* Default pin internal resistor value */
#define PORT_DEFAULT_PIN_RESISTANCE		(Port_InternalResistor)OFF


#endif /* PORT_CFG_H */