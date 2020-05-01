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

#ifndef AUDIOPLAYER_H_
#define AUDIOPLAYER_H_

#include "stdint.h"
#include "stm32f0xx_hal.h"

//#define RD_BUFFER_SIZE 1024
#define TMP_BUFFER_SIZE 256
#define PLY_BUFFER_SIZE 2*TMP_BUFFER_SIZE


typedef struct soundStream_t{
	uint32_t length;
	uint32_t samplerate;
	uint32_t address;
}soundStream_t;



void playerInit(TIM_HandleTypeDef *hTime, DAC_HandleTypeDef *hdac, DMA_HandleTypeDef *hdma_dac);
void playerStart();
void playerStop();
void playerInitFinishCallback(void(*fun)());


#endif /* AUDIOPLAYER_H_ */
