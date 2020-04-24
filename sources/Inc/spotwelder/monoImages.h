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

#ifndef UI_MONOIMAGES_H_
#define UI_MONOIMAGES_H_

#include "main.h"
#include "stdint.h"

 typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     uint8_t dataSize;
     } tImage;


     const uint8_t image_data_Hot_32[128];
     const tImage Hot_32;

     const uint8_t image_data_resart_32x32[128];
     const tImage resart_32x32;

     const uint8_t image_data_ok_32x32[128];
     const tImage ok_32x32;

     const uint8_t image_data_settings_32x32[128];
     const tImage settings_32x32;

     const uint8_t image_data_temp_32x32[128];
     const tImage temp_32x32;

     const uint8_t image_data_attention_32x32[128];
     const tImage attention_32x32;

     const uint8_t image_data_battery1_32x32[128];
     const tImage battery1_32x32;

     const uint8_t image_data_fan_32x32[128];
     const tImage fan_32x32;

     const tImage boot_160x128;

     const tImage pwr_32x32;




#endif /* UI_MONOIMAGES_H_ */
