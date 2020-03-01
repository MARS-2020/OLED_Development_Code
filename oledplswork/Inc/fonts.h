/*
 * fonts.h
 *
 *  Created on: Feb 29, 2020
 *      Author: Morgan
 */

#ifndef INC_FONTS_H_
#define INC_FONTS_H_


uint8_t fonts[]= {0x7E, 0x09, 0x09, 0x09, 0x7E,0x00, //A 0
				0x7F, 0x49, 0x49, 0x49, 0x36,0x00, //B
				0x3E, 0x41, 0x41, 0x41, 0x22,0x00, //C
				0x7F, 0x41, 0x41, 0x22, 0x1C,0x00, //D
				0x7F, 0x49, 0x49, 0x49, 0x41,0x00, //E
				0x7F, 0x09, 0x09, 0x09, 0x01,0x00, //F
				0x3E, 0x41, 0x41, 0x49, 0x38,0x00, //G
				0x7F, 0x08, 0x08, 0x08, 0x7F,0x00, //H
				0x41, 0x41, 0x7F, 0x41, 0x41,0x00, //I
				0x31, 0x41, 0x3F, 0x01, 0x01,0x00, //J
				0x7F, 0x08, 0x14, 0x22, 0x41,0x00, //K
				0x7F, 0x40, 0x40, 0x40, 0x40,0x00, // L
				0x7F, 0x02, 0x14, 0x02, 0x7F,0x00, //M
				0x7F, 0x04, 0x08, 0x10, 0x7F,0x00, //N
				0x3E, 0x41, 0x41, 0x41, 0x3E,0x00, //O
				0x7F, 0x09, 0x09, 0x09, 0x06,0x00, //P
				0x3E, 0x41, 0x51, 0x21, 0x5E,0x00, //Q
				0x7F, 0x09, 0x19, 0x29, 0x46,0x00, //R
				0x26, 0x49, 0x49, 0x49, 0x32,0x00, //S
				0x01, 0x01, 0x7F, 0x01, 0x01,0x00, //T
				0x3F, 0x40, 0x40, 0x40, 0x3F,0x00, //U
				0x1F, 0x20, 0x40, 0x20, 0x1F,0x00, //V
				0x3F, 0x40, 0x30, 0x40, 0x3F,0x00, //W
				0x63, 0x14, 0x08, 0x14, 0x63,0x00, //X
				0x03, 0x04, 0x78, 0x04, 0x03,0x00, //Y
				0x61, 0x51, 0x49, 0x45, 0x43,0x00, //Z
				0x3E, 0x51, 0x49, 0x45, 0x3E,0x00, //0 -26*6 =
				0x44, 0x42, 0x7F, 0x40, 0x40,0x00, //1
				0x42, 0x61, 0x51, 0x49, 0x46,0x00, //2
				0x22, 0x49, 0x49, 0x49, 0x36,0x00, //3
				0x18, 0x14, 0x12, 0x7F, 0x10,0x00, //4
				0x2F, 0x49, 0x49, 0x49, 0x31,0x00, //5
				0x3E, 0x49, 0x49, 0x49, 0x32,0x00, //6
				0x03, 0x01, 0x71, 0x09, 0x07,0x00, //7
				0x1B, 0x49, 0x49, 0x49, 0x1B,0x00, //8
				0x26, 0x49, 0x49, 0x49, 0x3E,0x00, //9
				0x22, 0x10, 0x08, 0X04, 0x22,0x00,// % -36
				0x06, 0x19, 0x62, 0x19, 0x06,0x00, //heart? -37
				0x00, 0x00, 0x00, 0x00, 0x00,0x00, //space 38
				0X22, 0x00, //: -39
				//numbers




};

//uint8_t A[] = {0x7E, 0x09, 0x09, 0x09, 0x7E,0x00};
//uint8_t B[] = {0x7F, 0x49, 0x49, 0x49, 0x36,0x00};
//uint8_t C[] = {0x3E, 0x41, 0x41, 0x41, 0x22,0x00};
//uint8_t D[] = {0x7F, 0x41, 0x41, 0x22, 0x1C,0x00};
//uint8_t E[] = {0x7F, 0x49, 0x49, 0x49, 0x41,0x00};
//uint8_t F[] = {0x7F, 0x09, 0x09, 0x09, 0x01,0x00};
//uint8_t G[] = {0x3E, 0x41, 0x41, 0x49, 0x38,0x00};
//uint8_t H[] = {0x7F, 0x08, 0x08, 0x08, 0x7F,0x00};
//uint8_t I[] = {0x41, 0x41, 0x7F, 0x41, 0x41,0x00};
//uint8_t J[] = {0x31, 0x41, 0x3F, 0x01, 0x01,0x00};
//uint8_t K[] = {0x7F, 0x08, 0x14, 0x22, 0x41,0x00};
//uint8_t L[] = {0x7F, 0x40, 0x40, 0x40, 0x40,0x00};
//uint8_t M[] = {0x7F, 0x02, 0x14, 0x02, 0x7F,0x00};
//uint8_t N[] = {0x7F, 0x04, 0x08, 0x10, 0x7F,0x00};
//uint8_t O[] = {0x3E, 0x41, 0x41, 0x41, 0x3E,0x00};
//uint8_t P[] = {0x7F, 0x09, 0x09, 0x09, 0x06,0x00};
//uint8_t Q[] = {0x3E, 0x41, 0x51, 0x21, 0x5E,0x00};
//uint8_t R[] = {0x7F, 0x09, 0x19, 0x29, 0x46,0x00};
//uint8_t S[] = {0x26, 0x49, 0x49, 0x49, 0x32,0x00};
//uint8_t T[] = {0x01, 0x01, 0x7F, 0x01, 0x01,0x00};
//uint8_t U[] = {0x3F, 0x40, 0x40, 0x40, 0x3F,0x00};
//uint8_t V[] = {0x1F, 0x20, 0x40, 0x20, 0x1F,0x00};
//uint8_t W[] = {0x3F, 0x40, 0x30, 0x40, 0x3F,0x00};
//uint8_t X[] = {0x63, 0x14, 0x08, 0x14, 0x63,0x00};
//uint8_t Y[] = {0x03, 0x04, 0x78, 0x04, 0x03,0x00};
//uint8_t Z[] = {0x61, 0x51, 0x49, 0x45, 0x43,0x00};


/*
#define uint8_t 0[] = {0x7F, 0x49, 0x49, 0x49, 0x41,0x00};
#define uint8_t 1[] = {0x7F, 0x40, 0x40, 0x40, 0x40,0x00};
#define uint8_t 2[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 3[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 4[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 5[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 6[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 7[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 8[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t 9[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
#define uint8_t %[] = {0x7F, 0x41, 0x41, 0x41, 0x7F,0x00};
*/
uint8_t space[] = {0x00};



#endif /* INC_FONTS_H_ */
