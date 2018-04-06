/**
  ******************************************************************************
  * File Name          : ControllerType.c
  * Description        : This file provides code for the detection and usage
  *                    : controller.  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "adc.h"
#include "ErrorHandler.h"
#include "ControllerType.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Structure to store information about connected controller
TEvent Event;
// Constant pointer to structure above
TEvent *const ev = &Event;

// Flag indication to initialize scan to find controller attached
bool DeviceFound = false;
// Flag for control controller detection time
__IO uint16_t ControllerTimeDetection = 0;

/* Variables for ADC conversion data */
extern __IO uint16_t uhADCxConvertedData;
extern __IO uint8_t ubAdcGrpRegularUnitaryConvStatus;
extern __IO uint16_t uhADCxConvertedData_Voltage_mVolt;
extern __IO uint16_t uhADCxConversionCompleted;
extern uint32_t ControllerType;

void ControllerTypeDetection(void){
  /* Reset status variable of ADC group regular unitary conversion before     */
  /* performing a new ADC group regular conversion start.                     */
  /* Note: Optionally, for this example purpose, check ADC unitary            */
  /*       conversion status before starting another ADC conversion.          */
	while(!uhADCxConversionCompleted && !ControllerTimeDetection){
		if (ubAdcGrpRegularUnitaryConvStatus != 0)
		{
			ubAdcGrpRegularUnitaryConvStatus = 0;
  
			/* Init variable containing ADC conversion data */
			uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;
  
  /* Start ADC group regular conversion */
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features are not             */
  /*       conditioned to ADC state.                                          */
  /*       However, in order to be compliant with other STM32 series          */
  /*       and to show the best practice usages, ADC state is checked.        */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
			if (LL_ADC_IsEnabled(ADC1) == 1)
			{
				LL_ADC_REG_StartConversionSWStart(ADC1);
			}
			else
			{
			/* Error: ADC conversion start could not be performed */
#ifdef APDEBUG
				_Error_Handler(__FILE__, __LINE__);
#endif
				break;
			}
		}
	}

	if(uhADCxConvertedData_Voltage_mVolt > 3000 && uhADCxConvertedData_Voltage_mVolt <= 3300){
		ControllerType = Eliwell;
	} else if(uhADCxConvertedData_Voltage_mVolt > 1400 && uhADCxConvertedData_Voltage_mVolt < 1500){ 
		ControllerType = CarelMPX;
	} else if (uhADCxConvertedData_Voltage_mVolt > 800 && uhADCxConvertedData_Voltage_mVolt < 900){
		ControllerType = CarelEasy;
	} else if (uhADCxConvertedData_Voltage_mVolt > 0 && uhADCxConvertedData_Voltage_mVolt < 400){
		ControllerType = Dixel;
	} else {
		ControllerType = Unknown;
	}
	uhADCxConversionCompleted = 0;
}


/************************ (C) COPYRIGHT *****END OF FILE****/
