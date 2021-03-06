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


#include "monoImages.h"


const uint8_t image_data_Hot_32[128] = {
    // ∙**********∙*******************∙
    // ***********∙∙∙******************
    // ***********∙∙∙∙*****************
    // ************∙∙∙∙∙***************
    // ************∙∙∙∙∙∙**************
    // ************∙∙∙∙∙∙∙*************
    // ************∙∙∙∙∙∙∙∙************
    // ************∙∙∙∙∙∙∙∙∙***********
    // ***********∙∙∙∙∙∙∙∙∙∙∙**********
    // ********∙***∙∙∙∙∙∙∙∙∙∙****∙*****
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙*∙*******
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙*∙*******
    // ******∙***∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙******
    // *****∙∙**∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*****
    // *****∙∙∙*∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙∙∙*****
    // *****∙∙∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙*****
    // ****∙∙∙∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙∙∙****
    // ****∙∙∙∙∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙∙****
    // ***∙∙∙∙∙∙∙∙∙∙∙∙∙*****∙∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙∙∙*∙∙∙******∙∙∙∙∙∙****
    // **∙∙∙∙∙∙∙∙∙∙**∙*******∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙∙***********∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙************∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙*************∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙************∙∙∙∙∙∙****
    // ***∙∙∙∙∙∙∙*************∙∙∙∙∙∙***
    // ***∙∙∙∙∙∙**************∙∙∙∙∙****
    // ***∙∙∙∙∙∙*************∙∙∙∙∙*****
    // ****∙∙∙∙∙∙*************∙∙∙∙*****
    // *****∙∙∙∙*************∙∙∙∙******
    // *******∙∙∙*********∙**∙∙∙*******
    // ********∙∙∙***********∙*********
    0x7f, 0xef, 0xff, 0xfe,
    0xff, 0xe3, 0xff, 0xff,
    0xff, 0xe1, 0xff, 0xff,
    0xff, 0xf0, 0x7f, 0xff,
    0xff, 0xf0, 0x3f, 0xff,
    0xff, 0xf0, 0x1f, 0xff,
    0xff, 0xf0, 0x0f, 0xff,
    0xff, 0xf0, 0x07, 0xff,
    0xff, 0xe0, 0x03, 0xff,
    0xff, 0x70, 0x03, 0xdf,
    0xff, 0xe0, 0x01, 0x7f,
    0xff, 0xe0, 0x01, 0x7f,
    0xfd, 0xc0, 0x00, 0x3f,
    0xf9, 0x80, 0x00, 0x1f,
    0xf8, 0x80, 0x40, 0x1f,
    0xf8, 0x00, 0x60, 0x1f,
    0xf0, 0x00, 0xe0, 0x0f,
    0xf0, 0x00, 0x70, 0x0f,
    0xe0, 0x00, 0xf8, 0x07,
    0xc0, 0x08, 0xfc, 0x0f,
    0xc0, 0x0d, 0xfc, 0x07,
    0xc0, 0x1f, 0xfc, 0x07,
    0xc0, 0x3f, 0xfc, 0x07,
    0xc0, 0x3f, 0xfe, 0x07,
    0xc0, 0x3f, 0xfc, 0x0f,
    0xe0, 0x3f, 0xfe, 0x07,
    0xe0, 0x7f, 0xfe, 0x0f,
    0xe0, 0x7f, 0xfc, 0x1f,
    0xf0, 0x3f, 0xfe, 0x1f,
    0xf8, 0x7f, 0xfc, 0x3f,
    0xfe, 0x3f, 0xec, 0x7f,
    0xff, 0x1f, 0xfd, 0xff
};
const tImage Hot_32 = { image_data_Hot_32, 32, 32,
    8 };


const uint8_t image_data_resart_32x32[128] = {
    // ***********∙∙∙∙∙∙∙∙∙∙***********
    // *********∙∙∙∙∙∙∙∙∙∙∙∙∙∙*********
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // ******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙******
    // *****∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*****
    // ****∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙****
    // ***∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**
    // **∙∙∙∙∙∙∙∙∙∙∙*******∙∙∙∙∙∙∙∙∙∙**
    // *∙∙∙∙∙∙∙*∙∙***********∙∙∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙*******∙∙∙*****∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙*****∙∙∙∙∙∙∙****∙∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙****∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙*****∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙******∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙***∙∙∙∙∙∙∙****∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙∙∙****∙∙∙∙*****∙∙∙∙∙∙∙∙*
    // **∙∙∙∙∙∙∙∙∙***********∙∙∙∙∙∙∙∙**
    // **∙∙∙∙∙∙∙∙∙∙∙*******∙∙∙∙∙∙∙∙∙∙**
    // ***∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙***
    // ****∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙****
    // *****∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*****
    // ******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *********∙∙∙∙∙∙∙∙∙∙∙∙∙∙*********
    // ***********∙∙∙∙∙∙∙∙∙∙***********
    0xff, 0xe0, 0x07, 0xff,
    0xff, 0x80, 0x01, 0xff,
    0xfe, 0x00, 0x00, 0x7f,
    0xfc, 0x00, 0x00, 0x3f,
    0xf8, 0x00, 0x00, 0x1f,
    0xf0, 0x00, 0x00, 0x0f,
    0xe0, 0x00, 0x00, 0x07,
    0xc0, 0x00, 0x00, 0x03,
    0xc0, 0x07, 0xf0, 0x03,
    0x80, 0x9f, 0xfc, 0x01,
    0x80, 0xfe, 0x3e, 0x01,
    0x00, 0xf8, 0x0f, 0x00,
    0x00, 0xf0, 0x07, 0x00,
    0x00, 0xf8, 0x03, 0x80,
    0x00, 0xfc, 0x03, 0x80,
    0x00, 0x00, 0x01, 0x80,
    0x00, 0x00, 0x01, 0x80,
    0x00, 0x00, 0x01, 0x80,
    0x00, 0x00, 0x03, 0x80,
    0x00, 0x00, 0x03, 0x80,
    0x00, 0x00, 0x07, 0x00,
    0x80, 0x38, 0x0f, 0x01,
    0x80, 0x3c, 0x3e, 0x01,
    0xc0, 0x1f, 0xfc, 0x03,
    0xc0, 0x07, 0xf0, 0x03,
    0xe0, 0x00, 0x00, 0x07,
    0xf0, 0x00, 0x00, 0x0f,
    0xf8, 0x00, 0x00, 0x1f,
    0xfc, 0x00, 0x00, 0x3f,
    0xfe, 0x00, 0x00, 0x7f,
    0xff, 0x80, 0x01, 0xff,
    0xff, 0xe0, 0x07, 0xff
};
const tImage resart_32x32 = { image_data_resart_32x32, 32, 32,
    8 };


const uint8_t image_data_ok_32x32[128] = {
    // *************∙*∙∙*∙*************
    // **********∙∙∙∙∙∙*∙∙∙∙∙**********
    // ********∙∙∙∙*∙∙∙∙∙∙*∙∙∙∙********
    // ******∙∙∙*∙∙∙∙*∙*∙∙∙∙*∙∙∙*******
    // *****∙*∙∙∙∙*∙∙∙∙∙∙*∙∙∙∙*∙∙∙*****
    // ****∙∙∙*∙∙∙∙∙*∙∙*∙∙∙*∙∙∙∙*∙*****
    // ****∙*∙∙∙*∙*∙∙∙∙∙∙∙∙∙∙*∙∙∙∙∙∙***
    // ***∙∙∙∙∙∙∙∙∙∙∙*∙*∙*∙*∙∙∙*∙∙*∙***
    // **∙∙∙*∙*∙*∙∙*∙∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙**
    // **∙∙∙∙∙∙∙∙∙∙∙∙*∙∙*∙∙***∙*∙∙*∙∙**
    // *∙∙*∙∙*∙∙*∙*∙∙∙∙∙∙∙∙****∙∙∙∙∙*∙*
    // *∙∙∙*∙∙∙∙∙∙∙∙*∙∙*∙*∙****∙*∙*∙∙∙*
    // *∙*∙∙∙∙*∙∙*∙∙∙∙∙∙∙∙****∙∙∙∙∙∙∙**
    // *∙∙∙∙*∙∙∙∙∙∙*∙*∙∙******∙*∙∙*∙∙∙∙
    // ∙∙∙*∙∙∙∙*∙*∙∙∙∙∙∙∙****∙∙∙∙∙∙∙*∙*
    // *∙∙∙∙∙*∙∙∙∙∙∙∙*∙*****∙∙*∙*∙*∙∙∙∙
    // ∙∙*∙∙∙∙∙∙***∙∙∙∙*****∙∙∙∙∙∙∙∙∙*∙
    // *∙∙∙*∙*∙*****∙∙*****∙∙*∙∙∙*∙∙∙∙*
    // ∙∙*∙∙∙∙∙∙*****∙****∙∙∙∙∙*∙∙∙*∙∙*
    // *∙∙∙∙*∙∙*∙*********∙∙*∙∙∙∙∙∙∙∙∙*
    // *∙∙*∙∙∙∙∙*∙*******∙∙∙∙∙*∙∙*∙*∙∙*
    // **∙∙∙∙*∙∙∙∙******∙∙∙*∙∙∙∙∙∙∙∙∙**
    // **∙∙*∙∙∙*∙∙∙****∙∙*∙∙∙*∙*∙*∙∙*∙*
    // **∙∙∙∙∙∙∙∙*∙∙***∙∙∙∙∙∙∙∙∙∙∙∙∙∙**
    // ***∙∙*∙*∙∙∙∙∙∙∙∙*∙∙*∙*∙∙*∙∙*∙***
    // ****∙∙∙∙∙*∙*∙∙*∙∙∙∙∙∙∙∙∙∙∙∙∙****
    // ****∙*∙∙∙∙∙∙∙∙∙∙∙*∙*∙∙*∙*∙*∙****
    // *****∙∙*∙*∙∙*∙∙*∙∙∙∙∙∙∙∙∙∙∙*****
    // *******∙∙∙∙∙∙∙∙∙∙∙*∙*∙*∙∙*******
    // ********∙∙*∙*∙*∙*∙∙∙∙∙∙∙********
    // **********∙∙∙∙∙∙∙∙∙∙*∙**********
    // **************∙*∙***************
    0xff, 0xfa, 0x5f, 0xff,
    0xff, 0xc0, 0x83, 0xff,
    0xff, 0x08, 0x10, 0xff,
    0xfc, 0x42, 0x84, 0x7f,
    0xfa, 0x10, 0x21, 0x1f,
    0xf1, 0x04, 0x88, 0x5f,
    0xf4, 0x50, 0x02, 0x07,
    0xe0, 0x02, 0xa8, 0x97,
    0xc5, 0x48, 0x02, 0x03,
    0xc0, 0x02, 0x4e, 0x93,
    0x92, 0x50, 0x0f, 0x05,
    0x88, 0x04, 0xaf, 0x51,
    0xa1, 0x20, 0x1e, 0x03,
    0x84, 0x0a, 0x7e, 0x90,
    0x10, 0xa0, 0x3c, 0x05,
    0x82, 0x02, 0xf9, 0x50,
    0x20, 0x70, 0xf8, 0x02,
    0x8a, 0xf9, 0xf2, 0x21,
    0x20, 0x7d, 0xe0, 0x89,
    0x84, 0xbf, 0xe4, 0x01,
    0x90, 0x5f, 0xc1, 0x29,
    0xc2, 0x1f, 0x88, 0x03,
    0xc8, 0x8f, 0x22, 0xa5,
    0xc0, 0x27, 0x00, 0x03,
    0xe5, 0x00, 0x94, 0x97,
    0xf0, 0x52, 0x00, 0x0f,
    0xf4, 0x00, 0x52, 0xaf,
    0xf9, 0x49, 0x00, 0x1f,
    0xfe, 0x00, 0x2a, 0x7f,
    0xff, 0x2a, 0x80, 0xff,
    0xff, 0xc0, 0x0b, 0xff,
    0xff, 0xfd, 0x7f, 0xff
};
const tImage ok_32x32 = { image_data_ok_32x32, 32, 32,
    8 };


const uint8_t image_data_settings_32x32[128] = {
    // ***********∙∙******∙∙***********
    // ********∙∙∙∙∙∙****∙∙∙∙∙∙********
    // *******∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // **∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**
    // *∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙∙∙∙∙∙*****∙∙∙∙∙∙∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙∙∙∙*********∙∙∙∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙∙∙∙**********∙∙∙∙∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙************∙∙∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙**************∙∙∙∙∙∙∙∙*
    // **∙∙∙∙∙∙∙**************∙∙∙∙∙∙∙**
    // ***∙∙∙∙∙∙**************∙∙∙∙∙∙***
    // ***∙∙∙∙∙∙**************∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙**************∙∙∙∙∙∙∙**
    // *∙∙∙∙∙∙∙∙∙************∙∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙∙∙************∙∙∙∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙∙**********∙∙∙∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙∙∙********∙∙∙∙∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙∙∙∙∙∙*****∙∙∙∙∙∙∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*
    // **∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *******∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙*******
    // *********∙∙∙∙∙****∙∙∙∙∙∙********
    // ***********∙∙******∙∙***********
    0xff, 0xe7, 0xe7, 0xff,
    0xff, 0x03, 0xc0, 0xff,
    0xfe, 0x01, 0x80, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xc0, 0x00, 0x00, 0x03,
    0x80, 0x00, 0x00, 0x01,
    0x80, 0x07, 0xc0, 0x01,
    0x80, 0x1f, 0xf0, 0x01,
    0x00, 0x1f, 0xf8, 0x00,
    0x00, 0x3f, 0xfc, 0x00,
    0x80, 0x7f, 0xfe, 0x01,
    0xc0, 0x7f, 0xfe, 0x03,
    0xe0, 0x7f, 0xfe, 0x07,
    0xe0, 0x7f, 0xfe, 0x07,
    0xc0, 0x7f, 0xfe, 0x03,
    0x80, 0x3f, 0xfc, 0x01,
    0x00, 0x3f, 0xfc, 0x00,
    0x00, 0x1f, 0xf8, 0x00,
    0x80, 0x0f, 0xf0, 0x01,
    0x80, 0x07, 0xc0, 0x01,
    0x80, 0x00, 0x00, 0x01,
    0xc0, 0x00, 0x00, 0x03,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x00, 0x00, 0x7f,
    0xfe, 0x01, 0x80, 0x7f,
    0xff, 0x83, 0xc0, 0xff,
    0xff, 0xe7, 0xe7, 0xff
};
const tImage settings_32x32 = { image_data_settings_32x32, 32, 32,
    8 };


const uint8_t image_data_temp_32x32[128] = {
    // ************∙∙∙∙****************
    // ***********∙∙∙*∙∙∙**************
    // ***********∙****∙∙**************
    // **********∙∙*****∙**************
    // **********∙∙*****∙∙**∙*∙********
    // **********∙******∙**∙∙∙∙********
    // **********∙∙*****∙∙*************
    // **********∙******∙∙*************
    // **********∙∙*****∙***∙**********
    // **********∙***∙**∙∙*∙∙∙*********
    // **********∙∙*∙∙*∙∙**************
    // **********∙**∙∙**∙∙*************
    // **********∙∙*∙∙*∙∙***∙∙*∙*******
    // **********∙**∙∙**∙**∙∙∙∙∙*******
    // **********∙∙*∙∙**∙∙*************
    // **********∙**∙∙∙*∙**************
    // **********∙∙*∙∙**∙∙*************
    // **********∙**∙∙**∙∙*************
    // **********∙∙*∙∙*∙∙**************
    // **********∙**∙∙**∙∙*************
    // *********∙∙**∙∙**∙∙∙************
    // ********∙∙**∙∙∙∙**∙∙************
    // ********∙**∙∙∙∙∙∙**∙∙***********
    // *******∙∙*∙∙∙∙∙∙∙∙*∙∙***********
    // ********∙*∙∙∙∙∙∙∙∙**∙***********
    // *******∙∙**∙∙∙∙∙∙∙*∙∙***********
    // ********∙*∙∙∙∙∙∙∙∙*∙∙***********
    // ********∙**∙∙∙∙∙∙**∙∙***********
    // ********∙∙***∙∙∙**∙∙************
    // *********∙∙******∙∙∙************
    // **********∙∙∙∙*∙∙∙**************
    // ***********∙∙∙∙∙∙***************
    0xff, 0xf0, 0xff, 0xff,
    0xff, 0xe2, 0x3f, 0xff,
    0xff, 0xef, 0x3f, 0xff,
    0xff, 0xcf, 0xbf, 0xff,
    0xff, 0xcf, 0x9a, 0xff,
    0xff, 0xdf, 0xb0, 0xff,
    0xff, 0xcf, 0x9f, 0xff,
    0xff, 0xdf, 0x9f, 0xff,
    0xff, 0xcf, 0xbb, 0xff,
    0xff, 0xdd, 0x91, 0xff,
    0xff, 0xc9, 0x3f, 0xff,
    0xff, 0xd9, 0x9f, 0xff,
    0xff, 0xc9, 0x39, 0x7f,
    0xff, 0xd9, 0xb0, 0x7f,
    0xff, 0xc9, 0x9f, 0xff,
    0xff, 0xd8, 0xbf, 0xff,
    0xff, 0xc9, 0x9f, 0xff,
    0xff, 0xd9, 0x9f, 0xff,
    0xff, 0xc9, 0x3f, 0xff,
    0xff, 0xd9, 0x9f, 0xff,
    0xff, 0x99, 0x8f, 0xff,
    0xff, 0x30, 0xcf, 0xff,
    0xff, 0x60, 0x67, 0xff,
    0xfe, 0x40, 0x27, 0xff,
    0xff, 0x40, 0x37, 0xff,
    0xfe, 0x60, 0x27, 0xff,
    0xff, 0x40, 0x27, 0xff,
    0xff, 0x60, 0x67, 0xff,
    0xff, 0x38, 0xcf, 0xff,
    0xff, 0x9f, 0x8f, 0xff,
    0xff, 0xc2, 0x3f, 0xff,
    0xff, 0xe0, 0x7f, 0xff
};
const tImage temp_32x32 = { image_data_temp_32x32, 32, 32,
    8 };



const uint8_t image_data_attention_32x32[128] = {
    // ********************************
    // *************∙∙∙∙∙∙*************
    // *************∙∙∙∙∙∙*************
    // ************∙∙∙**∙∙∙************
    // ***********∙∙∙***∙∙∙∙***********
    // ***********∙∙∙****∙∙∙***********
    // **********∙∙∙******∙∙∙**********
    // **********∙∙∙******∙∙∙**********
    // *********∙∙∙********∙∙∙*********
    // *********∙∙∙**∙∙∙∙**∙∙∙*********
    // ********∙∙∙***∙∙∙∙***∙∙∙********
    // ********∙∙∙***∙∙∙∙***∙∙∙********
    // *******∙∙∙****∙∙∙∙****∙∙∙*******
    // *******∙∙∙****∙∙∙∙****∙∙∙∙******
    // ******∙∙∙*****∙∙∙∙*****∙∙∙******
    // *****∙∙∙∙*****∙∙∙∙*****∙∙∙******
    // *****∙∙∙******∙∙∙*******∙∙∙*****
    // *****∙∙∙*******∙∙∙******∙∙∙∙****
    // ****∙∙∙*******∙∙∙∙*******∙∙∙****
    // ***∙∙∙∙********∙∙*********∙∙****
    // ***∙∙∙*********∙∙*********∙∙∙***
    // **∙∙∙**********∙∙*********∙∙∙∙**
    // **∙∙∙**********************∙∙∙**
    // *∙∙∙**********∙∙∙∙**********∙∙∙*
    // *∙∙∙**********∙∙∙∙**********∙∙∙*
    // *∙∙***********∙∙∙∙***********∙∙*
    // ∙∙∙*************∙************∙∙∙
    // ∙∙∙**************************∙∙∙
    // ∙∙∙***∙*∙*∙*********∙**∙*∙**∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*
    // **∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙**
    // ********************************
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0xf1, 0x8f, 0xff,
    0xff, 0xe3, 0x87, 0xff,
    0xff, 0xe3, 0xc7, 0xff,
    0xff, 0xc7, 0xe3, 0xff,
    0xff, 0xc7, 0xe3, 0xff,
    0xff, 0x8f, 0xf1, 0xff,
    0xff, 0x8c, 0x31, 0xff,
    0xff, 0x1c, 0x38, 0xff,
    0xff, 0x1c, 0x38, 0xff,
    0xfe, 0x3c, 0x3c, 0x7f,
    0xfe, 0x3c, 0x3c, 0x3f,
    0xfc, 0x7c, 0x3e, 0x3f,
    0xf8, 0x7c, 0x3e, 0x3f,
    0xf8, 0xfc, 0x7f, 0x1f,
    0xf8, 0xfe, 0x3f, 0x0f,
    0xf1, 0xfc, 0x3f, 0x8f,
    0xe1, 0xfe, 0x7f, 0xcf,
    0xe3, 0xfe, 0x7f, 0xc7,
    0xc7, 0xfe, 0x7f, 0xc3,
    0xc7, 0xff, 0xff, 0xe3,
    0x8f, 0xfc, 0x3f, 0xf1,
    0x8f, 0xfc, 0x3f, 0xf1,
    0x9f, 0xfc, 0x3f, 0xf9,
    0x1f, 0xff, 0x7f, 0xf8,
    0x1f, 0xff, 0xff, 0xf8,
    0x1d, 0x5f, 0xf6, 0xb0,
    0x80, 0x00, 0x00, 0x01,
    0xc0, 0x00, 0x00, 0x03,
    0xff, 0xff, 0xff, 0xff
};
const tImage attention_32x32 = { image_data_attention_32x32, 32, 32,
    8 };


const uint8_t image_data_battery_32x32[128] = {
    // *************∙∙∙∙∙**************
    // *************∙∙∙∙∙∙*************
    // *********∙∙∙∙∙∙∙∙∙∙∙∙∙**********
    // ********∙*************∙∙********
    // *******∙∙**************∙********
    // *******∙***************∙∙*******
    // *******∙***************∙********
    // *******∙***************∙∙*******
    // *******∙∙***************∙*******
    // *******∙***************∙********
    // *******∙***************∙∙*******
    // *******∙***************∙********
    // *******∙∙**************∙********
    // *******∙***************∙∙*******
    // *******∙***************∙********
    // *******∙***************∙∙*******
    // *******∙∙*********∙****∙********
    // *******∙***************∙********
    // *******∙***************∙∙*******
    // *******∙****************∙*******
    // *******∙∙*∙∙∙∙∙∙∙∙∙∙∙∙*∙********
    // *******∙**∙∙∙∙∙∙∙∙∙∙∙∙*∙********
    // *******∙*∙∙∙∙∙∙∙∙∙∙∙∙∙*∙∙*******
    // *******∙**∙∙∙∙∙∙∙∙∙∙∙∙**∙*******
    // *******∙∙*∙*∙**∙**∙****∙********
    // *******∙***∙*∙*∙*∙*∙∙**∙∙*******
    // *******∙*∙∙∙∙∙∙∙∙∙∙∙∙∙*∙********
    // *******∙**∙∙∙∙∙∙∙∙∙∙∙∙*∙********
    // *******∙*∙∙∙∙∙∙∙∙∙∙∙∙∙*∙∙*******
    // *******∙**∙∙*∙∙*∙∙*∙∙**∙********
    // ********∙*************∙∙********
    // ********∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*********
    0xff, 0xf8, 0x3f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0x80, 0x03, 0xff,
    0xff, 0x7f, 0xfc, 0xff,
    0xfe, 0x7f, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0xff, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0x7f, 0xff, 0x7f,
    0xfe, 0xff, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0xff, 0xfe, 0xff,
    0xfe, 0x7f, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0xff, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0x7f, 0xde, 0xff,
    0xfe, 0xff, 0xfe, 0xff,
    0xfe, 0xff, 0xfe, 0x7f,
    0xfe, 0xff, 0xff, 0x7f,
    0xfe, 0x40, 0x02, 0xff,
    0xfe, 0xc0, 0x02, 0xff,
    0xfe, 0x80, 0x02, 0x7f,
    0xfe, 0xc0, 0x03, 0x7f,
    0xfe, 0x56, 0xde, 0xff,
    0xfe, 0xea, 0xa6, 0x7f,
    0xfe, 0x80, 0x02, 0xff,
    0xfe, 0xc0, 0x02, 0xff,
    0xfe, 0x80, 0x02, 0x7f,
    0xfe, 0xc9, 0x26, 0xff,
    0xff, 0x7f, 0xfc, 0xff,
    0xff, 0x00, 0x01, 0xff
};
const tImage battery_32x32 = { image_data_battery_32x32, 32, 32,
    8 };


const uint8_t image_data_fan_32x32[128] = {
    // **************∙∙∙∙∙∙∙***********
    // ************∙∙∙∙∙∙∙∙∙∙∙*********
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙∙********
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙∙********
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙∙********
    // ***********∙∙∙∙∙∙∙∙∙∙∙∙*********
    // ***********∙∙∙∙∙∙∙∙∙∙***********
    // ***********∙∙∙∙∙∙∙∙∙************
    // **∙∙∙******∙∙∙∙∙∙∙∙∙************
    // *∙∙∙∙∙******∙∙∙∙∙∙∙*************
    // ∙∙∙∙∙∙*******∙∙∙*∙**************
    // ∙∙∙∙∙∙∙∙***************∙∙∙∙∙∙∙**
    // ∙∙∙∙∙∙∙∙******∙∙∙∙***∙∙∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙**∙∙∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙∙∙∙*
    // ∙∙∙∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙∙∙∙∙
    // ∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙∙*∙∙∙∙∙∙∙**∙∙∙∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙∙**∙∙∙∙∙∙**∙∙∙∙∙∙∙∙∙∙∙
    // *∙∙∙∙∙∙∙∙∙∙***∙∙∙*******∙∙∙∙∙∙∙∙
    // **∙∙∙∙∙∙**********∙*****∙∙∙∙∙∙∙∙
    // **************∙*∙∙∙*******∙∙∙∙∙*
    // *************∙∙∙∙∙∙∙******∙∙∙∙∙*
    // ************∙∙∙∙∙∙∙∙∙******∙∙∙**
    // ************∙∙∙∙∙∙∙∙∙***********
    // **********∙∙∙∙∙∙∙∙∙∙∙***********
    // *********∙∙∙∙∙∙∙∙∙∙∙∙***********
    // ********∙∙∙∙∙∙∙∙∙∙∙∙∙***********
    // ********∙∙∙∙∙∙∙∙∙∙∙∙∙***********
    // ********∙∙∙∙∙∙∙∙∙∙∙∙∙***********
    // *********∙∙∙∙∙∙∙∙∙∙∙************
    // ***********∙∙∙∙∙∙***************
    0xff, 0xfc, 0x07, 0xff,
    0xff, 0xf0, 0x01, 0xff,
    0xff, 0xe0, 0x00, 0xff,
    0xff, 0xe0, 0x00, 0xff,
    0xff, 0xe0, 0x00, 0xff,
    0xff, 0xe0, 0x01, 0xff,
    0xff, 0xe0, 0x07, 0xff,
    0xff, 0xe0, 0x0f, 0xff,
    0xc7, 0xe0, 0x0f, 0xff,
    0x83, 0xf0, 0x1f, 0xff,
    0x03, 0xf8, 0xbf, 0xff,
    0x00, 0xff, 0xfe, 0x03,
    0x00, 0xfc, 0x38, 0x01,
    0x00, 0x18, 0x18, 0x01,
    0x00, 0x10, 0x08, 0x01,
    0x00, 0x10, 0x08, 0x00,
    0x00, 0x30, 0x0c, 0x00,
    0x80, 0x10, 0x18, 0x00,
    0x80, 0x18, 0x18, 0x00,
    0x80, 0x1c, 0x7f, 0x00,
    0xc0, 0xff, 0xdf, 0x00,
    0xff, 0xfd, 0x1f, 0xc1,
    0xff, 0xf8, 0x0f, 0xc1,
    0xff, 0xf0, 0x07, 0xe3,
    0xff, 0xf0, 0x07, 0xff,
    0xff, 0xc0, 0x07, 0xff,
    0xff, 0x80, 0x07, 0xff,
    0xff, 0x00, 0x07, 0xff,
    0xff, 0x00, 0x07, 0xff,
    0xff, 0x00, 0x07, 0xff,
    0xff, 0x80, 0x0f, 0xff,
    0xff, 0xe0, 0x7f, 0xff
};
const tImage fan_32x32 = { image_data_fan_32x32, 32, 32,
    8 };


static const uint8_t image_data_pwr_32x32[128] = {
    // ***************∙∙***************
    // **************∙∙∙∙**************
    // *************∙∙∙∙∙∙*************
    // *************∙∙∙∙∙∙*************
    // *************∙∙∙∙∙∙*************
    // *************∙∙∙∙∙∙*************
    // *****∙∙∙∙****∙∙∙∙∙∙****∙∙∙∙*****
    // ****∙∙∙∙∙∙***∙∙∙∙∙∙***∙∙∙∙∙∙****
    // ***∙∙∙∙∙∙∙***∙∙∙∙∙∙***∙∙∙∙∙∙∙***
    // ***∙∙∙∙∙∙∙***∙∙∙∙∙∙***∙∙∙∙∙∙∙***
    // **∙∙∙∙∙∙∙****∙∙∙∙∙∙****∙∙∙∙∙∙∙**
    // *∙∙∙∙∙∙∙*****∙∙∙∙∙∙*****∙∙∙∙∙∙∙*
    // *∙∙∙∙∙∙******∙∙∙∙∙∙******∙∙∙∙∙∙*
    // *∙∙∙∙∙∙******∙∙∙∙∙∙******∙∙∙∙∙∙*
    // ∙∙∙∙∙∙*******∙∙∙∙∙∙*******∙∙∙∙∙∙
    // ∙∙∙∙∙∙*******∙∙∙∙∙∙*******∙∙∙∙∙∙
    // ∙∙∙∙∙∙********∙∙∙∙********∙∙∙∙∙∙
    // ∙∙∙∙∙∙********************∙∙∙∙∙∙
    // ∙∙∙∙∙∙********************∙∙∙∙∙∙
    // ∙∙∙∙∙∙********************∙∙∙∙∙∙
    // ∙∙∙∙∙∙********************∙∙∙∙∙∙
    // *∙∙∙∙∙∙******************∙∙∙∙∙∙*
    // *∙∙∙∙∙∙******************∙∙∙∙∙∙*
    // **∙∙∙∙∙∙****************∙∙∙∙∙∙**
    // **∙∙∙∙∙∙∙**************∙∙∙∙∙∙∙**
    // ***∙∙∙∙∙∙∙∙**********∙∙∙∙∙∙∙∙***
    // ****∙∙∙∙∙∙∙∙∙∙****∙∙∙∙∙∙∙∙∙∙****
    // *****∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*****
    // ******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙******
    // *******∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙∙*******
    // *********∙∙∙∙∙∙∙∙∙∙∙∙∙∙*********
    // ************∙∙∙∙∙∙∙∙************
    0xff, 0xfe, 0x7f, 0xff,
    0xff, 0xfc, 0x3f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xff, 0xf8, 0x1f, 0xff,
    0xf8, 0x78, 0x1e, 0x1f,
    0xf0, 0x38, 0x1c, 0x0f,
    0xe0, 0x38, 0x1c, 0x07,
    0xe0, 0x38, 0x1c, 0x07,
    0xc0, 0x78, 0x1e, 0x03,
    0x80, 0xf8, 0x1f, 0x01,
    0x81, 0xf8, 0x1f, 0x81,
    0x81, 0xf8, 0x1f, 0x81,
    0x03, 0xf8, 0x1f, 0xc0,
    0x03, 0xf8, 0x1f, 0xc0,
    0x03, 0xfc, 0x3f, 0xc0,
    0x03, 0xff, 0xff, 0xc0,
    0x03, 0xff, 0xff, 0xc0,
    0x03, 0xff, 0xff, 0xc0,
    0x03, 0xff, 0xff, 0xc0,
    0x81, 0xff, 0xff, 0x81,
    0x81, 0xff, 0xff, 0x81,
    0xc0, 0xff, 0xff, 0x03,
    0xc0, 0x7f, 0xfe, 0x03,
    0xe0, 0x1f, 0xf8, 0x07,
    0xf0, 0x03, 0xc0, 0x0f,
    0xf8, 0x00, 0x00, 0x1f,
    0xfc, 0x00, 0x00, 0x3f,
    0xfe, 0x00, 0x00, 0x7f,
    0xff, 0x80, 0x01, 0xff,
    0xff, 0xf0, 0x0f, 0xff
};
const tImage pwr_32x32 = { image_data_pwr_32x32, 32, 32,
    8 };


