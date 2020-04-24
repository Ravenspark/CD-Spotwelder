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

#ifndef HELPERS_H_
#define HELPERS_H_

#include <stdbool.h>

void setLEDColor(uint8_t r, uint8_t g, uint8_t b);

/****** AMPLIFIER **************************************************************************************/

void ampPowerOn();
void ampPowerOff();

void ampPowerSet(bool on);

/****** CHARGER **************************************************************************************/
void chargerPowerOn();
void chargerPowerOff();


/****** CAP DISCHARGE *********************************************************************************/
void capDischargerOn();
void capDischargerOff();

/****** POWER LED **************************************************************************************/
void powerLEDOn();
void powerLEDOff();

void powerLEDToggle();



/****** POWER LED **************************************************************************************/
#define FAN_PWM_MIN 0
#define FAN_PWM_MAX 100
#define FAN_TEMP_MIN 18.0f
#define FAN_TEMP_MAX 30.0f

#define FAN_PWM_INC ((FAN_PWM_MAX-FAN_PWM_MIN)/(FAN_TEMP_MAX-FAN_TEMP_MIN))

void fanPowerOn();
void fanPowerOff();

uint16_t fanSetValue(float temp);

extern TIM_HandleTypeDef htim3;



#endif /* HELPERS_H_ */
