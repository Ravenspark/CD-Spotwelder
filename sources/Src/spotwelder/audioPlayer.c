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


#include "audioPlayer.h"
//#include "spi_flash.h"
#include "stm32_dsp.h"

#include "helpers.h"
#include "main.h"

uint8_t  sine_wave[256] = {
  0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
  0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
  0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
  0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
  0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
  0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
  0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
  0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
  0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
  0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
  0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
  0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
  0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
  0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
  0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
  0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
  0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
  0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
  0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
  0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
  0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
  0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
  0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
  0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
  0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
  0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
  0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
  0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
  0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
  0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};

static uint8_t playing=0;
static uint32_t playIndex=0;
static uint8_t first=0, hasHeader=0, playCnt=0;

static soundStream_t *strm;
static TIM_HandleTypeDef *hDACTime;
static DAC_HandleTypeDef *hDAC;
static DMA_HandleTypeDef *hDMA_DAC;

//static uint8_t rdBuffer[RD_BUFFER_SIZE];
static uint8_t tmpBuffer[TMP_BUFFER_SIZE],playBuffer[PLY_BUFFER_SIZE];

static const int8_t filterCoeffs[]={36, -59, 229, 36, -114, 26, -40, 206, 26, -91, 14, -15, 186, 14, -72, 5, 4, 173, 5, -59};


static void (*onFinishCallback)();

void fillBufferWithSine(uint8_t *buffer, uint16_t buffSz)
{
    for(uint16_t i=0; i<buffSz;i++)
    buffer[i]=sine_wave[(i<<2)%sizeof(sine_wave)];
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	fillBufferWithSine(&playBuffer[0],PLY_BUFFER_SIZE/2);

	//clearing buffer and upsample
	//memset(tmpBuffer,0,sizeof(tmpBuffer));
	//for(int i=0; i<TMP_BUFFER_SIZE; i++)	tmpBuffer[6*i]=sine_wave[i+playCnt*TMP_BUFFER_SIZE];
	//iir_biquad(&playBuffer[0], tmpBuffer, filterCoeffs, PLY_BUFFER_SIZE/2);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	++playCnt;

	fillBufferWithSine(&playBuffer[PLY_BUFFER_SIZE/2],PLY_BUFFER_SIZE/2);

	//clearing buffer and upsample
	//memset(tmpBuffer,0,sizeof(tmpBuffer));
	//for(int i=0; i<TMP_BUFFER_SIZE; i++)	tmpBuffer[6*i]=sine_wave[i+playCnt*TMP_BUFFER_SIZE];
	//iir_biquad(&playBuffer[PLY_BUFFER_SIZE/2], tmpBuffer, filterCoeffs, PLY_BUFFER_SIZE/2);
}

void playerInitFinishCallback(void(*fun)())
{
	//TODO: add auto finish!
	onFinishCallback = fun;
}


void playerInit(TIM_HandleTypeDef *hTime, DAC_HandleTypeDef *hdac, DMA_HandleTypeDef *hdma_dac)
{
	hDACTime = hTime;
	hDAC = hdac;
	hDMA_DAC = hdma_dac;
	fillBufferWithSine(&playBuffer[0],PLY_BUFFER_SIZE);
    //HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	//  HAL_DAC_Start_DMA(hDAC, DAC_CHANNEL_1, (uint32_t*)playBuffer, PLY_BUFFER_SIZE, DAC_ALIGN_8B_R);
}
/*void playerPlayFile(soundStream_t *stream, TIM_HandleTypeDef *hTime, DAC_HandleTypeDef *hdac, DMA_HandleTypeDef *hdma_dac)
{
	strm = stream;
	hDACTime = hTime;
	hDAC = hdac;
	hDMA_DAC = hdma_dac;
	playIndex=stream->address;
//	memcpy(rdBuffer,&strm.buffer[strm.index],BUFFER_SIZE);
	SPI_FLASH_BufferRead(&rdBuffer[0],playIndex,RD_BUFFER_SIZE);
	playIndex+=RD_BUFFER_SIZE;

//TODO:verify.. and only play sound data
	if(strcmp((char*)rdBuffer,"RIFF"))
	{
		hasHeader=1;
		strm->samplerate = *((uint32_t*) rdBuffer + 0x18);

		/*hDACTime->Init.Period = 24000000UL/strm->samplerate -1;
		if (HAL_TIM_Base_Init(&hDACTime) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}*/
	/*}

// first time play buffersize - header bytes
//then restart dma with corect buffersize
	//--> take care: interrupt to refill buffer in middle of data...


//	  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	  HAL_DAC_Start_DMA(hDAC, DAC_CHANNEL_1, (uint32_t*)rdBuffer, PLY_BUFFER_SIZE, DAC_ALIGN_8B_R);
//	  HAL_TIM_Base_Start(hDACTime);
//	   HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port,AMP_SHDN_Pin,GPIO_PIN_SET);
}*/



void playerStop()
{
    ampPowerOff();
//ampPowerOn();
    HAL_DAC_Stop_DMA(hDAC,DAC_CHANNEL_1);
	HAL_TIM_Base_Stop(hDACTime);
}

void playerStart()
{


ampPowerOn();
  //  ampPowerOff();

     HAL_DAC_Start_DMA(hDAC, DAC_CHANNEL_1, (uint32_t*)playBuffer, PLY_BUFFER_SIZE, DAC_ALIGN_8B_R);
	HAL_TIM_Base_Start(hDACTime);
}
