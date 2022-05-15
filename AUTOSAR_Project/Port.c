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
#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"


#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Module */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

/************************************************************************************
 *                             Static Variables                                     *
 ************************************************************************************/
STATIC uint8 Port_Status = PORT_UNINITIALIZED;
STATIC const Port_Config_Channels* Port_Configuration = NULL_PTR;
/************************************************************************************
 *                             Static Functions                                     *
 ************************************************************************************/
STATIC uint8 get_pin(Port_Pin p);
STATIC UINT32_PTR get_port_base_addr(uint8 port);


/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to initialize the Port module
************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr)
{
	volatile uint32 delay = 0;
	UINT32_PTR Port_base_addr = NULL_PTR;
	uint8 pin_bit = 0;
	uint8 i;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if Configuration pointer is a NULL_PTR */
	if(NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_INIT_SID, PORT_E_PARAM_CONFIG);
	}
	else{}
#endif
	{		
		/* Address of the 1st structure of Port_channels[0] */
		Port_Configuration = ConfigPtr->Port_channels; 

		/* Enable clock for all Ports and allow time for clock to start*/
		SYSCTL_REGCGC2_REG |= RCGC2_PORTS_MASK;
		delay = SYSCTL_REGCGC2_REG;

   		/* Initializing Pins in Port_CofigurationArray */
		for (i = 0; i < PORT_MAX_PINS; i++)
		{
			/* Point to the required port base address registers based on the port number */
 			Port_base_addr = get_port_base_addr(Port_Configuration[i].port_num);

			/* Get the corresponding physical bit for the pin in the current Port_Configuration[i] */
			pin_bit = get_pin(Port_Configuration[i].pin_num);

			/* Unlocking & commiting are needed for Pins PC3-0, PD7, PF0 */
		 	if ((Port_Configuration[i].pin_num == PD7) || (Port_Configuration[i].pin_num == PF0))
		 	{
		 		*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_LOCK_REG_OFFSET) = PORT_UNLOCK_MAGIC_VALUE;
		 		SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_COMMIT_REG_OFFSET), pin_bit);
		 	}
		 	/* PC3-0(JTAG pins) are skipped */
		 	else if ((Port_Configuration[i].port_num == PORT_C) && (Port_Configuration[i].pin_num <= 3))
		 	{
		 		continue;

		 	}
			else {}

			/* Setting direction of the pin */
		 	if (Port_Configuration[i].direction == OUTPUT)
			{	/* if direction is ouptut, set the initial value */

				// Set #bit(pin_bit) in GPIODIR
				SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
				// Assign initial value in GPIODATA
				if(Port_Configuration[i].initial_value == STD_LOW)
				{
					CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DATA_REG_OFFSET), pin_bit);
				}
				else
				{
					SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DATA_REG_OFFSET), pin_bit);

				}
			}
			else if (Port_Configuration[i].direction == INPUT)
			{	/* if direction is input, set the internal resistance */

				// Clear #bit(pin_bit) in GPIODIR
				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
				if(Port_Configuration[i].resistor == PULL_UP)
				{
					SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_PULL_UP_REG_OFFSET), pin_bit);
				}
				else if (Port_Configuration[i].resistor == PULL_DOWN)
				{
					SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_PULL_DOWN_REG_OFFSET), pin_bit);

				}
				else
				{
					// Disable both pull-up and pull-down
					CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_PULL_UP_REG_OFFSET), pin_bit);
					CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_PULL_DOWN_REG_OFFSET), pin_bit);
				}
			}
			else {}

			/* Set mode of the pin whether digital or analog */
			if(Port_Configuration[i].pin_mode == ADC_MODE)
			{	/* In the case of ADC mode, enable analog function, disable alternative function, clear PMCX bits, and disable digital IO */

				SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ANALOG_MODE_SEL_REG_OFFSET), pin_bit);
				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIGITAL_ENABLE_REG_OFFSET), pin_bit);
				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ALT_FUNC_REG_OFFSET), pin_bit);
				*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (pin_bit * PMCx_bits));
			}
			else
			{	/* In the case of other digital functionality: disable analog function, enable digital function */

				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ANALOG_MODE_SEL_REG_OFFSET), pin_bit);
				SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIGITAL_ENABLE_REG_OFFSET), pin_bit);

				if (Port_Configuration[i].pin_mode == DIO_MODE)
				{ 	/* For digital IO, disable alternative function and clear PMCX bits*/
					CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ALT_FUNC_REG_OFFSET), pin_bit);
					*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (pin_bit * PMCx_bits));

				}
				else 
				{	/* For other alternative functions, enable alternative function and assign pin_mode to GPIOCTL */
					SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ALT_FUNC_REG_OFFSET), pin_bit);
					*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_CTL_REG_OFFSET) |= (Port_Configuration[i].pin_mode << (pin_bit * PMCx_bits));
				}
			}

		} // End of for loop

		/* Set the module status to be initialized */
		Port_Status = PORT_INITIALIZED;	
	}

}


/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in):	Pin - Port Pin ID number
					Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to set a port pin direction during runtime
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_Pin Pin, Port_PinDirection Direction)
{	
	boolean error = FALSE;
	UINT32_PTR Port_base_addr = NULL_PTR;
	uint8 pin_bit;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if Port module is not initialized */
	if (PORT_UNINITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID,	PORT_INSTANCE_ID,
						PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else {}
	/* Check if incorrect Port Pin ID is passed */
	if(PORT_MAX_PINS < Pin)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else {}
	/* Check if Port pin's direction is not configured as changeable */	
	if(Port_Configuration[Pin].pin_direction_changeable == STD_OFF)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		error = TRUE;
	}
	else {}
	/* Don't access JTAG pins PC0-PC3 */
	if ((Port_Configuration[Pin].port_num == PORT_C) && (Port_Configuration[Pin].pin_num <= 3))
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_JTAG_ACCESS);
		error = TRUE;
	}
	else {}
#endif /* No errors found */
	if(FALSE == error)
	{
		Port_base_addr = get_port_base_addr(Port_Configuration[Pin].port_num);
		pin_bit = get_pin(Port_Configuration[Pin].pin_num);
		if (Direction == OUTPUT)
		{
			// Set #bit(pin_bit) in GPIODIR
			SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
		}
		else if (Direction == INPUT)
		{
			// Clear #bit(pin_bit) in GPIODIR
			CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
		}
		else {}
	}
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in):	None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to refresh the direction of all configured ports
************************************************************************************/
void Port_RefreshPortDirection(void)
{
	UINT32_PTR Port_base_addr = NULL_PTR;
	uint8 pin_bit;
	uint8 i;
	boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if Port module is not initialized */
	if (PORT_UNINITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID,	PORT_INSTANCE_ID,
						PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else {}

#endif
	{
		if (FALSE == error)
		{ 
			for (i = 0; i < PORT_MAX_PINS; i++)
			{
				/* Get port base address and needed #bit of the pin to that port */
				Port_base_addr = get_port_base_addr(Port_Configuration[i].port_num);
				/* Get the corresponding physical bit for the pin in the current Port_Configuration[i] */
				pin_bit = get_pin(Port_Configuration[i].pin_num);

				/* Don't access JTAG pins PC0-PC3 */
				if ((Port_Configuration[i].port_num == PORT_C) && (Port_Configuration[i].pin_num <= 3))
				{
					/* skip */
					continue;
				}
				/* Check if Pin direction of the pin is unchangeable, 
				 * based on AUTOSAR_SWS_PortDriver (PORT061), this API excludes the pins whose pin directions are changeable during runtime */
				if (Port_Configuration[i].pin_direction_changeable == STD_OFF)
				{
					if (Port_Configuration[i].direction == OUTPUT)
					{
						// Set #bit(pin_num) in GPIODIR
						SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
					}
					else if (Port_Configuration[i].direction == INPUT)
					{
						// Clear #bit(pin_num) in GPIODIR
						CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIR_REG_OFFSET), pin_bit);
					}
					else {}
				}
				else {}/* Pin direction is changeable, so skip */
				
			} // end of for loop
		} // end of if (false == error)
		else {}
	}// end of #endif
}


/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): VersionInfo - Pointer to where to store the version information of this module
* Return value: None
* Description: Function returns the version information of this module
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else {}
#endif /* (PORT_DEV_ERROR_DETECT == STD_OFF) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): 	Pin - Port Pin ID number
					Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out):  None
* Return value: None
* Description: Function to set the port pin mode during runtime
************************************************************************************/
void Port_SetPinMode(Port_Pin Pin, Port_PinMode Mode)
{
	boolean error = FALSE;
	UINT32_PTR Port_base_addr = NULL_PTR;
	uint8 pin_bit;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if Port module is not initialized */
	if (PORT_UNINITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID,	PORT_INSTANCE_ID,
						PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else {}
	/* Check if incorrect Port Pin ID is passed */
	if (PORT_MAX_PINS < Pin)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else {}
	/* Check Port Pin mode passed is not valid */
	if (PORT_MAX_PIN_MODE <= Mode)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
		error = TRUE;
	}
	else {}
	/* Check Port Pin mode is not configured as changeable */
	if (Port_Configuration[Pin].pin_mode_changeable == STD_OFF)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
		error = TRUE;
	}
	else {}
	/* Don't access JTAG pins PC0-PC3 */
	if ((Port_Configuration[Pin].port_num == PORT_C) && (Port_Configuration[Pin].pin_num <= 3))
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
						PORT_SET_PIN_MODE_SID, PORT_JTAG_ACCESS);
		error = TRUE;
	}
	else {}

#endif /* No errors found */
	{
		if(FALSE == error)
		{
			/* Get the corresponding port base address register based on port_num */
			Port_base_addr = get_port_base_addr(Port_Configuration[Pin].port_num);
			/* Get the physical bit*/
			pin_bit = get_pin(Port_Configuration[Pin].pin_num);
			
			/* Set mode of the pin whether digital or analog */
			if(Port_Configuration[Pin].pin_mode == ADC_MODE)
			{	/* In the case of ADC mode, enable analog function, and disable digital IO */

				SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ANALOG_MODE_SEL_REG_OFFSET), pin_bit);
				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIGITAL_ENABLE_REG_OFFSET), pin_bit);
			}
			else
			{	/* In the case of other digital functionality: disable analog function, enable digital function */

				CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ANALOG_MODE_SEL_REG_OFFSET), pin_bit);
				SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_DIGITAL_ENABLE_REG_OFFSET), pin_bit);

				if (Port_Configuration[Pin].pin_mode == DIO_MODE)
				{ 	/* For digital IO, disable alternative function and clear PMCX bits*/
					CLEAR_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ALT_FUNC_REG_OFFSET), pin_bit);
					*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (pin_bit * PMCx_bits));

				}
				else 
				{	/* For other alternative functions, enable alternative function and assign pin_mode to GPIOCTL */
					SET_BIT(*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_ALT_FUNC_REG_OFFSET), pin_bit);
					*(UINT32_PTR)((UINT8_PTR)Port_base_addr + PORT_CTL_REG_OFFSET) |= (Port_Configuration[Pin].pin_mode << (pin_bit * PMCx_bits));
				}
			}
			
		
		}
		else {}
	}
}

/************************************************************************************
* Service Name: get_pin
* Parameters (in): p : Pin number 0-43
* Parameters (inout): None
* Parameters (out): None
* Return value: uint8 - Pin's physical bit field considering the total number of pins to each port
* Description: Function to transform pin number (0~43) to a physical bit field in Port registers 
* Example: get_pin(PB7): Pin #7 in Port B is ordered to be 15, the function should return 7
************************************************************************************/
STATIC uint8 get_pin(Port_Pin p)
{
	uint8 pin_bit = 0;
	/* Identify which bit field corresponding to Pin is */
	switch(p)
	{
		// PORTA
		case PA0: 
		case PA1: 
		case PA2: 
		case PA3: 
		case PA4: 
		case PA5: 
		case PA6: 
		case PA7:
			pin_bit = p - PortA_START;
			break;			
		// PORTB		 
		case PB0: 
		case PB1: 
		case PB2: 
		case PB3: 
		case PB4: 
		case PB5: 
		case PB6: 
		case PB7: 
			pin_bit = p - PortB_START;
			break;
		// PORTC
		case PC0: 
		case PC1: 
		case PC2: 
		case PC3:
		case PC4: 
		case PC5: 
		case PC6: 
		case PC7: 
			pin_bit = p - PortC_START;
			break;
		// PORTD
		case PD0: 
		case PD1: 
		case PD2: 
		case PD3: 
		case PD4: 
		case PD5: 
		case PD6: 
		case PD7: 
			pin_bit = p - PortD_START;
			break;
		// PORTE
		case PE0: 
		case PE1: 
		case PE2: 
		case PE3: 
		case PE4: 
		case PE5:
			pin_bit = p - PortE_START;
			break; 
		// PORTF
		case PF0: 
		case PF1: 
		case PF2: 
		case PF3: 
		case PF4: 
			pin_bit = p - PortF_START;
			break;
	}
	return pin_bit;
}

/************************************************************************************
* Service Name: get_port_base_addr
* Parameters (in): port - port number
* Parameters (inout): None
* Parameters (out): None
* Return value: base address register of the needed port 
* Description: Function returns base address register of a specific port
************************************************************************************/
STATIC UINT32_PTR get_port_base_addr(uint8 port)
{
	UINT32_PTR Port_base_addr = NULL_PTR;
	switch(port)
	{
		case PORT_A: Port_base_addr = (UINT32_PTR)GPIO_PORTA_BASE_ADDRESS;
		break;
		case PORT_B: Port_base_addr = (UINT32_PTR)GPIO_PORTB_BASE_ADDRESS;
		break;
		case PORT_C: Port_base_addr = (UINT32_PTR)GPIO_PORTC_BASE_ADDRESS;
		break;
		case PORT_D: Port_base_addr = (UINT32_PTR)GPIO_PORTD_BASE_ADDRESS; 
		break;
		case PORT_E: Port_base_addr = (UINT32_PTR)GPIO_PORTE_BASE_ADDRESS;
		break;
		case PORT_F: Port_base_addr = (UINT32_PTR)GPIO_PORTF_BASE_ADDRESS; 
		break;

	}
	return Port_base_addr;
}