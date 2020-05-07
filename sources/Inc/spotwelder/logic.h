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

#ifndef LOGIC_H_
#define LOGIC_H_

#include "monoImages.h"
#include "stdbool.h"


//time in 10ms steps
#define SHRT_PRESS_TIME	200
#define LNG_PRESS_TIME	1000

//typedef union {int16_t iValue; float fValue;}uVal_t;
typedef enum {vINT, vFLOAT, vSTRING} type_t;
//typedef struct value_t{uVal_t min; uVal_t val; uVal_t max; uVal_t inc; const char *desc} value_t;
typedef struct value_t{int16_t min; int16_t val; int16_t max; int16_t inc; /*const char *fmt;*/ const char *desc; type_t type} value_t;


enum buttonEvent_t
{
	NONE_PRESSED,
	SHRT_PRESSED,
	LONG_PRESSED,
	DOUBLE_PRESSED
	};

typedef enum buttonEvent_t buttonEvent_t;

enum rotaryEvent_t
{
	ROT_NONE,
	ROT_INC,
	ROT_DEC
	};

typedef enum rotaryEvent_t rotaryEvent_t;

typedef enum {
	ENTER_IDLE,
	IDLE,
	ENTER_CHARGING,
	CHARGING,
	ENTER_CHARGED,
	CHARGED,
	ENTER_DISCHARGING,
	DISCHARGING,
	ENTER_WELDING,
	WELDING,
	ENTER_SETTINGS,
	SETTINGS,
    ENTER_WELDQUALITY,
    WELDQUALITY
} state_t;

struct dataCollection_t
{
	uint8_t var1;
	uint8_t var2;
	float vCap;
	float vBar;
	float vSupply;
	float temp1;
	float temp2;
	};

typedef struct dataCollection_t dataCollection_t;


typedef struct menuIcon_t
{
    const tImage *image;
    uint16_t xPos;
    uint16_t yPos;
    void (*pSubMenuFun)(buttonEvent_t,rotaryEvent_t);
} menuIcon_t;

typedef struct dataItem_t
{
    union{
        float fValue;
        uint32_t uValue;
        int32_t sValue;
    };
    const char *desc;
    uint16_t xPos;
    uint16_t yPos;
} dataItem_t;

typedef struct menuSetting_t
{
    uint16_t *value;
    uint16_t min;
    uint16_t max;
    uint16_t inc;
} menuSetting_t;


struct settingsCollection_t
{
	uint16_t tPulse1;
	uint16_t tPause;
	uint16_t tPulse2;
	bool autoweld;
	float vCap;
	};

typedef struct settingsCollection_t settingsCollection_t;

struct data_t
{
    int16_t val;
    const char *desc;
    type_t type;
};
typedef struct data_t data_t;

//extern volatile uint8_t adcReady;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc;

dataCollection_t* initLogic();
//void updateState(volatile buttonEvent_t *menuBtnEvent, volatile rotaryEvent_t *rotEvent, volatile buttonEvent_t *footSwEvent);
void updateState();

void checkButton(volatile buttonEvent_t *event);

void weldingTimerISR();

#endif /* LOGIC_H_ */
