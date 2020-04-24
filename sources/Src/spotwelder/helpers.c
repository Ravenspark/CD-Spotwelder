/*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
* Copyright 2020 Ravenspark (S.Gerber)                              *  *
*                                                                   *     *
* Licensed under the Apache License, Version 2.0 (the "License");   * * * * *
* you may not use this file except in compliance with the License.          *
* You may obtain a copy of the License at                                   *
*                                                                           *
*     http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                           *
* Unless required by applicable law or agreed to in writing, software       *
* distributed under the License is distributed on an "AS IS" BASIS,         *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
* See the License for the specific language governing permissions and       *
* limitations under the License.                                            *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stdint.h"
#include "main.h"
#include <stdbool.h>
//#include "stm32f1xx_hal.h"

#include "helpers.h"
//#include "UI/UILogic.h"


inline void setLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
	TIM1->CCR3 = r;
	TIM1->CCR2 = g;
	TIM1->CCR1 = b;
}

/****** AMPLIFIER **************************************************************************************/

inline void ampPowerOn()
{
	ampPowerSet(true);
}
inline void ampPowerOff()
{
	ampPowerSet(false);
}

inline void ampPowerSet(bool on)
{
	HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port, AMP_SHDN_Pin, on);
}

/****** CHARGER **************************************************************************************/
inline void chargerPowerOn()
{
	HAL_GPIO_WritePin(CHARGER_EN_GPIO_Port, CHARGER_EN_Pin, RESET);
}
inline void chargerPowerOff()
{
	HAL_GPIO_WritePin(CHARGER_EN_GPIO_Port, CHARGER_EN_Pin, SET);
}


/****** CAP DISCHARGE *********************************************************************************/
inline void capDischargerOn()
{
	HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, SET);
}

inline void capDischargerOff()
{
	HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, RESET);
}

/****** POWER LED **************************************************************************************/
inline void powerLEDOn()
{
	HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, SET);
}
inline void powerLEDOff()
{
	HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, RESET);
}

inline void powerLEDToggle()
{
	HAL_GPIO_TogglePin(PWR_LED_GPIO_Port, PWR_LED_Pin);
}



/****** POWER LED **************************************************************************************/

inline void fanPowerOn()
{
	HAL_GPIO_WritePin(FAN_EN_GPIO_Port,FAN_EN_Pin,SET);
}
inline void fanPowerOff()
{
	HAL_GPIO_WritePin(FAN_EN_GPIO_Port,FAN_EN_Pin,RESET);
}

#define MIN_FAN_TEMP 25.0
#define MAX_FAN_PWM 255
#define MIN_FAN_PWM 50

uint16_t fanSetValue(float temp)
{
	static uint16_t lastPwmVal=1000;
	uint16_t pwmVal =0;
	if(temp>=MIN_FAN_TEMP)
	{
		fanPowerOn();
		pwmVal = FAN_PWM_INC * (temp-MIN_FAN_TEMP) + MIN_FAN_PWM;
		if(pwmVal>MAX_FAN_PWM) pwmVal = MAX_FAN_PWM;
	}
	else
	{
		fanPowerOff();

	}
	//if((pwmVal>0) && (lastPwmVal==0))
		//drawFan(1);
	//else if(pwmVal==0 && lastPwmVal>0)
		//drawFan(0);
	lastPwmVal = pwmVal;

	htim3.Instance->CCR3 = pwmVal;

	return pwmVal;
}


