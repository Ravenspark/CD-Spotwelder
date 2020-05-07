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

#include "main.h"
#include "logic.h"
#include "helpers.h"

#include "audioPlayer.h"
#include "monoImages.h"
#include "st7735.h"



dataCollection_t *pdata;


volatile uint16_t tacho=0;

//dataCollection_t *pData;
volatile buttonEvent_t buttonEvent = NONE_PRESSED;
volatile buttonEvent_t footSwEvent= NONE_PRESSED;
volatile rotaryEvent_t rotEvent = ROT_NONE;



#define ADC_BUFF_SIZE 7

volatile uint8_t adcReady=0;
uint16_t adcBuffer[ADC_BUFF_SIZE];

typedef enum weldingstate_t {WAITING, WELDING1, WELDING2, PAUSING} weldingstate_t;


uint8_t xTest[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14}, yTest[]={5,7,8,9,8,7,5,3,2,1,2,3,5};

volatile weldingstate_t weldingState=WAITING;
value_t p1Values ={1,20,20,1}, p2Values={10,500,500,10}, pauseValues={5,100,100,5};


//value_t settingValues[] = {{1,10,20,1, "pulse1: %d",vINT}, {10,150,500,10, "pulse2: %d",vINT}, {5,20,100,5, "pause: %d",vINT}, {10,50,100,5, "voltage: %02d.%1d",vFLOAT}, {0,0,0,0, "exit?",vSTRING}}; //pulse1 duration, pulse2 duration, pause duration, voltage x10, exit
value_t settingValues[] = {{1,10,20,1, "pulse1:",vSTRING}, {10,150,500,10, "pulse2:",vSTRING}, {5,20,100,5, "pause:",vSTRING}, {10,50,100,5, "voltage:",vSTRING}, {0,0,0,0, "exit?",vSTRING}}; //pulse1 duration, pulse2 duration, pause duration, voltage x10, exit

//typedef menuIcon_t menuIcon_t;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcReady=1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t lastState=0;
	static uint16_t cnt=0;

  if(htim->Instance == htim2.Instance)
    {
    	weldingTimerISR();

    }
  else if(htim->Instance == htim15.Instance)
  {
	  checkButton(&buttonEvent);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  if(GPIO_Pin == ROT_A_Pin)//Rotary encoder turned
	  {
		  if(HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin)==GPIO_PIN_SET)
			  //turned clockwise
		  	  rotEvent = ROT_INC;
		  else
			  //turned counterclockwise
			  rotEvent = ROT_DEC;
	  }
      else if(GPIO_Pin == FOOT_SW_Pin)
      {
          footSwEvent = SHRT_PRESSED;
      }
}






void demoFunc(buttonEvent_t btnEvent, rotaryEvent_t rotEvent)
{
}

menuIcon_t menuIcons[] ={
			{&fan_32x32, 128, 0, &demoFunc},
			{&resart_32x32, 128, 32,&demoFunc},
			{&pwr_32x32, 128, 64,&demoFunc},
			{&settings_32x32, 128, 96,&demoFunc}
			};


menuIcon_t menuIcons2[] ={
            {&fan_32x32, 0, 0, &demoFunc},
            {&resart_32x32, 32,0, &demoFunc},
            {&pwr_32x32, 64, 0, &demoFunc},
            {&settings_32x32, 96, 0, &demoFunc}
            };

static dataCollection_t data;

static data_t testData[] = {{0,"Vin: %2d.%1d",vFLOAT},{0,"Vcap:%2d.%1d",vFLOAT},{0,"Temp:%2d.%1d",vFLOAT}};

static bool autoweld=false, fanOn=false;


static state_t state = ENTER_IDLE;

static settingsCollection_t settings  = {100, 100, 100, false, 5.0f};

static char strBuffer[32];

uint8_t counter=0;

dataCollection_t* initLogic()
{
    ST7735_Init();
	ST7735_FillScreen(ST7735_GREEN);
    HAL_ADC_Start_DMA(&hadc,(uint32_t *)adcBuffer,ADC_BUFF_SIZE);
	return &data;
}




static float calcVoltage(uint16_t val)
{
    //return (5.0f*3.3f*val)/(1<<12);
    return (5.2f*3.3f*val)/(1<<12);
}


static float calcTemp(uint16_t val)
{
    //R = R25 * e ^ (B * (1/T25 - 1/T))

    //T = 1/(ln(R/R25)-1/T)
    float fVal = (float) val;
    float rNTC = 82*(((1<<12) - fVal)/fVal); // 3.3/(3.3*val/(1<<12)/82)

    float t0 = 273.15;
    float t25 = 298.15;
    float beta = 3950;

    return t25/(t25/beta*logf(rNTC/100)+1)-t0;
}


static void drawBar(uint16_t x, uint16_t y, value_t value, uint16_t color)
{
	const uint8_t barHeight = 15, barMid =7, barSegWidth=5, NSeg=10;
	uint8_t barlen = barSegWidth*NSeg;

	uint8_t len = ((value.val-value.min)*barlen)/(value.max-value.min);


	ST7735_FillRectangle(x,y,2,barHeight,ST7735_WHITE);                     //  ||         |         |
	                                                                         //  ||----|----|----|----|
	for(int i =1; i<NSeg; i++)                                              //  ||         |         |
	{
		if(i%2)
			ST7735_FillRectangle(x+2+i*barSegWidth,y+barMid-3,1,barMid,ST7735_WHITE);
		else
			ST7735_FillRectangle(x+2+i*barSegWidth,y+1,1,barHeight-2,ST7735_WHITE);
	}
		ST7735_FillRectangle(x+2+NSeg*barSegWidth,y,2,barHeight,ST7735_WHITE);

		ST7735_FillRectangle(x+2+len,y+barMid-2,barlen-len,5,ST7735_BLACK);
        //ST7735_FillRectangle(x+2,y+barMid,barlen,1,ST7735_WHITE);
		ST7735_FillRectangle(x+2,y+barMid-2,len,5,color);

		sprintf(strBuffer,"%dms",value.val);
		ST7735_WriteString(x+5+barlen, y+3, strBuffer, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

void demoMenu(buttonEvent_t menuBtnEvent, rotaryEvent_t *rotEvent)
{
	static int8_t index=-1;

	if(*rotEvent == ROT_INC)
	{
		if(++index > 8) index=0;
	}
	else if(*rotEvent == ROT_DEC)
	{
		if(--index < 0) index = 8;
	}
	*rotEvent = ROT_NONE;


 	ST7735_DrawImageMono(0, 0, Hot_32.data, Hot_32.width, Hot_32.height, index==0? ST7735_GREEN:ST7735_RED, ST7735_GREY);
	ST7735_DrawImageMono(0, 32, resart_32x32.data, resart_32x32.width, resart_32x32.height, index==1? ST7735_GREEN:ST7735_RED, ST7735_BLACK);
	ST7735_DrawImageMono(0, 64, ok_32x32.data, ok_32x32.width, ok_32x32.height, index==2? ST7735_GREEN:ST7735_RED, ST7735_BLACK);
	ST7735_DrawImageMono(0, 96, settings_32x32.data, settings_32x32.width, settings_32x32.height, index==3? ST7735_GREEN:ST7735_RED, ST7735_BLACK);

	ST7735_DrawImageMono(32, 0, temp_32x32.data, temp_32x32.width, temp_32x32.height, index==4? ST7735_GREEN:ST7735_RED, ST7735_BLACK);
	ST7735_DrawImageMono(32, 32, image_data_attention_32x32, attention_32x32.width, attention_32x32.height, index==5? ST7735_GREEN:ST7735_RED, ST7735_BLACK);
	ST7735_DrawImageMono(32, 96, image_data_fan_32x32, fan_32x32.width, fan_32x32.height, index==7? ST7735_GREEN:ST7735_RED, ST7735_BLACK);

	ST7735_DrawImageMono(64, 0, image_data_battery_32x32, battery_32x32.width, battery_32x32.height, index==8? ST7735_GREEN:ST7735_RED, ST7735_BLACK);

}

void drawIcon(menuIcon_t *menuIcon, bool isSelected)
{
    ST7735_DrawImageMono(menuIcon->xPos, menuIcon->yPos, menuIcon->image->data, menuIcon->image->width, menuIcon->image->height, isSelected? ST7735_GREEN:ST7735_GREY, ST7735_BLACK);
}

void menu1()
{
	static uint8_t sel=0;


	for(int i=0; i<4; i++)
	{
		drawIcon(&menuIcons[i],i==2);
	}

/*	for(int i=0; i<4; i++)
	{
		ST7735_DrawImageMono(menuIcons2[i].xPos, menuIcons2[i].yPos, menuIcons2[i].image->data, menuIcons2[i].image->width, menuIcons2[i].image->height, ST7735_GREEN, ST7735_BLACK);
	}*/
}

void drawMenuBar()
{
    drawIcon(&menuIcons[0],fanOn);
    //drawIcon(&menuIcons[3],state==SETTINGS);
}

void plotGraph(uint8_t *xValues, uint8_t *yValues, uint8_t xRange, uint8_t yRange, uint8_t N)
{
#define X_REF 22
#define Y_REF 96
	int x0=0, y0=0;
	uint16_t x=0, y=0;

	ST7735_FillRectangle(X_REF,32,1,65,ST7735_WHITE);
	ST7735_FillRectangle(X_REF,96,128,1,ST7735_WHITE);
	for(int i=1; i<128/16; i++)
	{
		ST7735_FillRectangle(X_REF+16*i,96-3,1,7,ST7735_WHITE);
		sprintf(strBuffer,"%02d",(i*xRange)/8);
		ST7735_WriteString(22+16*i-7,96+4,strBuffer,Font_7x10,ST7735_WHITE,ST7735_BLACK);
	}
	for(int i=1; i<64/16; i++)
	{
		ST7735_FillRectangle(X_REF-3,96-i*16,7,1,ST7735_WHITE);
		sprintf(strBuffer,"%02d",(i*yRange)/4);
		ST7735_WriteString(0,96-i*16-4,strBuffer,Font_7x10,ST7735_WHITE,ST7735_BLACK);
	}

	for(int i=0; i<N; i++)
	{
		 x= xValues[i];
		 y= yValues[i];

		 x = (x*128)/xRange;
		 y = (y*64)/yRange;

		ST7735_DrawPixel(23+x,96-y,ST7735_CYAN);
	}

}


void makeStr(char *strbuf, value_t *value)
{
    switch(value->type)
    {
        default:
        case vINT: sprintf(strbuf,value->desc,value->val); break;
        case vFLOAT: sprintf(strbuf,value->desc,value->val/10,value->val%10); break;
        case vSTRING: sprintf(strbuf,value->desc); break;
    }
}


void makeStr2(char *strbuf, data_t *data)
{
    switch(data->type)
    {
        default:
        case vINT: sprintf(strbuf,data->desc,data->val); break;
        case vFLOAT: sprintf(strbuf,data->desc,data->val/10,data->val%10); break;
    }
}

bool drawSettingsMenu(buttonEvent_t menuBtnEvent, rotaryEvent_t rotEvent, bool force)
{
    bool exit=false;
    static bool selected=false;
	static uint8_t idx=0;

	if(menuBtnEvent==SHRT_PRESSED) 
    {
        selected = !selected;
    }
	if(!selected)
	{
		if(rotEvent==ROT_INC) idx++;
		if(rotEvent==ROT_DEC) idx--;
        if(idx==255) idx=4;
        if(idx>4) idx=0;
	}
	else
	{
		if(rotEvent==ROT_INC)
		    {
		        settingValues[idx].val += settingValues[idx].inc;
                if(settingValues[idx].val>settingValues[idx].max) settingValues[idx].val=settingValues[idx].max;
		    }
		if(rotEvent==ROT_DEC)
		    {
                settingValues[idx].val -= settingValues[idx].inc;
                if(settingValues[idx].val<settingValues[idx].min) settingValues[idx].val=settingValues[idx].min;
		    }
	}

//TODO: only draw when something has changed?
   for(int i=0; i<5; i++)
   {	
        if(rotEvent!=ROT_NONE || menuBtnEvent==SHRT_PRESSED || force)
        {
            makeStr(strBuffer,&settingValues[i]);
            ST7735_WriteString(0,i*15,strBuffer,Font_7x10,(idx==i)? (selected? ST7735_BLUE:ST7735_RED):ST7735_WHITE, ST7735_BLACK);
            if((i==idx || force)&& i<4)
            { 
                drawBar(64,i*15,settingValues[i],ST7735_BLUE);
            }
        }
   }

	if((idx==4)&&selected)
	{
	    idx=0;
	    selected = false;
	    exit=true;
	}

	return exit;
}


void drawTiming()
{
    sprintf(strBuffer,"%2d",settingValues[0].val);
    ST7735_WriteString(0,110,strBuffer,Font_11x18,ST7735_GREEN,ST7735_GREY);
    sprintf(strBuffer,"%3d",settingValues[2].val);
    ST7735_WriteString(30,110,strBuffer,Font_11x18,ST7735_GREEN,ST7735_GREY);
    sprintf(strBuffer,"%3d",settingValues[1].val);
    ST7735_WriteString(82,110,strBuffer,Font_11x18,ST7735_GREEN,ST7735_GREY);
}


void drawVolume(uint8_t x, uint8_t y, uint8_t vol)
{
	const uint8_t height=16, segWidth=6, segSpace=2;
	ST7735_FillRectangle(x,y+height-height/4,segWidth,height/4,ST7735_BLUE);
	ST7735_FillRectangle(x+segWidth+segSpace,y+height-height/2,segWidth,height/2, vol>=20? ST7735_BLUE:ST7735_BLACK);
	ST7735_FillRectangle(x+2*(segWidth+segSpace),y+height-(3*height)/4,segWidth,(3*height)/4, vol>=50? ST7735_BLUE:ST7735_BLACK);
	ST7735_FillRectangle(x+3*(segWidth+segSpace),y,segWidth,height, vol>=75? ST7735_BLUE:ST7735_BLACK);
}


void startPulse(uint16_t length, uint16_t delay)
{
    TIM2->ARR=(length+delay);
    TIM2->CCR4=length;

    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;

    TIM2->CCER |= TIM_CCER_CC4E;
    TIM2->BDTR|=TIM_BDTR_MOE;
    TIM2->CR1|=TIM_CR1_CEN;

}

inline void weldingTimerISR()
{
    switch(weldingState)
            {
            case WELDING1:
                weldingState=WELDING2;
              startPulse(settingValues[1].val-1, settingValues[2].val);
              break;

            case WELDING2:
                weldingState=WAITING;

          break;
            default:
            break;
            }
}


//TODO: check this register
// hadc.Instance->CHSELR &= ~(ADC_CHSELR_CHANNEL(ADC_CHANNEL_0)|ADC_CHSELR_CHANNEL(ADC_CHANNEL_1)|ADC_CHSELR_CHANNEL(ADC_CHANNEL_2));
//re-enable all adc channels
//hadc.Instance->CHSELR |= (ADC_CHSELR_CHANNEL(ADC_CHANNEL_0)|ADC_CHSELR_CHANNEL(ADC_CHANNEL_1)|ADC_CHSELR_CHANNEL(ADC_CHANNEL_2));


//void updateState(volatile buttonEvent_t *menuBtnEvent, volatile rotaryEvent_t *rotEvent, volatile buttonEvent_t *footSwEvent)
void updateState()
{
    static state_t lastState=999;
	static uint8_t mode=0;
    static int cnt=0;

    //save values to local variables to check at the end wether they have been changed by an interrupt
    buttonEvent_t tmpMenuBtnEvent = buttonEvent, tmpFootSwEvent = footSwEvent;
    rotaryEvent_t tmpRotEvent = rotEvent; 

	static int tmp=0;


	  if(adcReady)
	  {
		  data.vBar = calcVoltage(adcBuffer[4]);
		  data.vCap = calcVoltage(adcBuffer[5]);
		  data.vSupply = calcVoltage(adcBuffer[6]);

		  data.temp1 = calcTemp(adcBuffer[1]);
		  data.temp2 = calcTemp(adcBuffer[2]);

		  fanSetValue(data.temp1);

          testData[0].val = (int16_t)(10*data.vSupply);
          testData[1].val = (int16_t)(10*data.vCap);
          testData[2].val = (int16_t)(10*data.temp1);

		  adcReady=0;
		  HAL_ADC_Start_DMA(&hadc,(uint32_t *)adcBuffer,ADC_BUFF_SIZE);
	  }

	  //if(state!=lastState)
      //{
	  //    drawMenuBar();
      //  }
	  //lastState=state;

      if(++cnt>100)
      {
          cnt=0;
          if(state!=SETTINGS)
          {
            for(int i=0; i<3; i++)
            {
                makeStr2(strBuffer,&testData[i]);
                ST7735_WriteString(0,i*19+15,strBuffer,Font_11x18,ST7735_BLUE,ST7735_BLACK);
            }
            drawTiming();
	      drawMenuBar();

          }
      }

	switch(state)
	{
		default:
		case ENTER_IDLE:
			capDischargerOff();
            //ST7735_DrawImageMono(0,0,boot_160x128.data,boot_160x128.width,boot_160x128.height,ST7735_WHITE,ST7735_BLACK);
            //HAL_Delay(2000);
			ST7735_FillScreen(ST7735_BLACK);
            drawIcon(&menuIcons[2],false);
            drawIcon(&menuIcons[1],false);
            drawIcon(&menuIcons[3],false);
			setLEDColor(0,0,100);
			state = IDLE;
			break;
		case IDLE:
		    if(tmpMenuBtnEvent == DOUBLE_PRESSED) state=ENTER_CHARGING;
		    else if(tmpMenuBtnEvent == LONG_PRESSED) state=ENTER_SETTINGS;
            if(tmpFootSwEvent == SHRT_PRESSED) state=ENTER_WELDING;
		break;

		case ENTER_CHARGING:
			setLEDColor(100,0,0);
		    drawIcon(&menuIcons[2],true);
		    drawIcon(&menuIcons[1],autoweld);
            drawVolume(50,50,100);
            //playerStart();
			chargerPowerOn();
			state = CHARGING;
			break;
		case CHARGING:
			if(++counter>100){counter=0; state=ENTER_CHARGED;}
			//if(data.vCap >= settings.vCap) state=ENTER_CHARGED;
		break;

		case ENTER_CHARGED:
            //playerStop();
			setLEDColor(0,100,0);
			chargerPowerOff();
			state = CHARGED;
			break;

		case CHARGED:
            if(tmpMenuBtnEvent==DOUBLE_PRESSED){state=ENTER_DISCHARGING;}
            if(tmpMenuBtnEvent==LONG_PRESSED)
            {
                autoweld = !autoweld;
                drawIcon(&menuIcons[1],autoweld);
            }
			if(tmpFootSwEvent==SHRT_PRESSED) state=ENTER_WELDING;
		break;

		case ENTER_DISCHARGING:
			setLEDColor(100,0,0);
            //drawIcon(&menuIcons[2],false);
            //drawIcon(&menuIcons[1],false);
			capDischargerOn();
			state = DISCHARGING;
			break;

		case DISCHARGING:
			if(++counter>510){counter=0; state=ENTER_IDLE;}
			if(data.vCap <= 0.5f) state=ENTER_IDLE;
				break;

		case ENTER_WELDING:
			setLEDColor(100,100,100);
            weldingState=WELDING1;
            startPulse(settingValues[0].val-1,settingValues[2].val-1);
			state=WELDING;
			break;

		case WELDING:
		    //if(weldingState==WAITING) state = ENTER_WELDQUALITY;
		    if(weldingState==WAITING) state = ENTER_IDLE;
		    break;

		case ENTER_SETTINGS: 
            state = SETTINGS; 
			ST7735_FillScreen(ST7735_BLACK);
            drawIcon(&menuIcons[3],true);
            drawSettingsMenu(tmpMenuBtnEvent,tmpRotEvent,true);
            break;
		case SETTINGS:
		    if(drawSettingsMenu(tmpMenuBtnEvent,tmpRotEvent,false)) state=ENTER_IDLE;
		    break;

		case ENTER_WELDQUALITY:
		    if(data.vCap>0.5*settings.vCap)
		    {
		        ST7735_DrawImageMono(10,50,ok_32x32.data,ok_32x32.width,ok_32x32.height,ST7735_GREEN,ST7735_BLACK);
		    }
		    else
		    {
                ST7735_DrawImageMono(10,50,attention_32x32.data,attention_32x32.width,attention_32x32.height,ST7735_RED,ST7735_BLACK);
		    }
            setLEDColor(0,0,100);
		    state=WELDQUALITY;
		    break;
		case WELDQUALITY:
            if(++counter>20)
            {
                counter=0;
                ST7735_FillRectangle(10,50,32,32,ST7735_BLACK);
                state=ENTER_CHARGING;
            }
		    break;
	}

    //if(*menuBtnEvent == tmpMenuBtnEvent) *menuBtnEvent=NONE_PRESSED; //Only reset if no new event is waiting
    //if(*footSwEvent == tmpFootSwEvent) *footSwEvent=NONE_PRESSED; //Only reset if no new event is waiting
    //if(*rotEvent == tmpRotEvent) *rotEvent = ROT_NONE; //Only reset if no new event is waiting
    if(buttonEvent == tmpMenuBtnEvent) buttonEvent=NONE_PRESSED; //Only reset if no new event is waiting
    if(footSwEvent == tmpFootSwEvent) footSwEvent=NONE_PRESSED; //Only reset if no new event is waiting
    if(rotEvent == tmpRotEvent) rotEvent = ROT_NONE; //Only reset if no new event is waiting

}

static enum btnStates {BTN_PREPARE, BTN_NONE, BTN_IS_PRESSED, BTN_IS_RELEASED, BTN_WAIT, BTN_RECOVER};

 void checkButton(volatile buttonEvent_t *event)
{
	static uint16_t counter=0;
	static uint8_t btnState=0;

	//buttonEvent_t event = NONE_PRESSED;

	switch(btnState)
	{
	default:
	case BTN_PREPARE:
		counter = 0;
		btnState=BTN_NONE;
		break;

	case BTN_NONE: //not pressed
		if(HAL_GPIO_ReadPin(ROT_SW_GPIO_Port,ROT_SW_Pin)==GPIO_PIN_SET) btnState = BTN_IS_PRESSED;
		break;

	case BTN_IS_PRESSED: //is pressed
		if(++counter >=LNG_PRESS_TIME)
		{
			*event = LONG_PRESSED;
			btnState = BTN_RECOVER;
			counter=0;
		}
		else if(HAL_GPIO_ReadPin(ROT_SW_GPIO_Port,ROT_SW_Pin)==GPIO_PIN_RESET)
		{
			counter=0;
			btnState=BTN_WAIT;
		}


		break;

	case BTN_WAIT:
		if(++counter>=SHRT_PRESS_TIME)
		{
			*event = SHRT_PRESSED;
			btnState = BTN_PREPARE;
		}
		else if(HAL_GPIO_ReadPin(ROT_SW_GPIO_Port,ROT_SW_Pin)==GPIO_PIN_SET)
		{
			*event = DOUBLE_PRESSED;
			btnState = BTN_RECOVER;
		}
		break;

	case BTN_IS_RELEASED: //is released
		break;

	case BTN_RECOVER: //recover, wait for release
		if(HAL_GPIO_ReadPin(ROT_SW_GPIO_Port,ROT_SW_Pin)==GPIO_PIN_RESET) btnState=BTN_PREPARE;
		break;

	}
	//return event;

}





