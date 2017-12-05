/**
  ******************************************************************************
  * File Name          : ControllerType.h
  * Description        : This file provides data definition for
  *                      Controller port usage.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROLLERTYPE_H
#define __CONTROLLERTYPE_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f1xx_hal.h"

#define Dixel

/* USER CODE BEGIN Includes */
/** @addtogroup Controller_Included
  * @{
  */

#if defined(Eliwel)
  #include "Televis.h"
#elif defined(Carel)
  #include "Carel.h"
#elif defined(Modbus)
  #include "Modbus.h"
#elif defined(Dixel)
  #include "Dixel.h"
#elif defined(Evco)
  #include "Evco.h"
#else
 #error "Please select first the target controller device used in your application (in ControllerPort.h file)"
#endif

/**
  * @}
  */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */





#endif /*__CONTROLLERTYPE_H */

/**
  * @}
  */

/**
  * @}
  */
