/*
 * oled.h
 *
 *  Created on: 19.12.2017
 *      Author: daniel
 */

#ifndef OLED_H_
#define OLED_H_

#include "chip.h"
#include <stdint.h>
#include <stdlib.h>

extern const unsigned char Image1 [];
extern const unsigned char Image2 [];

#define SSD_1306
//#define  SH_1106


#define __SET_COL_START_ADDR() 	{OLED_Write_Byte(0x02, OLED_CMD); OLED_Write_Byte(0x10, OLED_CMD);}

#define OLED_CMD    		 0
#define OLED_DAT    		 1

#define LPC_SSP           	 LPC_SSP1
//#define OLED_CLK_PIN       20
//#define OLED_DIN_PIN       22
#define OLED_CS_PIN          23
#define OLED_DC_PIN          24
#define OLED_RES_PIN         25

#define OLED_CS_GPIO         1
#define OLED_RES_GPIO        1
#define OLED_DC_GPIO         1

#define OLED_WIDTH    	     128
#define OLED_HEIGHT          64

#define __OLED_CS_SET()      Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_CS_GPIO, OLED_CS_PIN, true)
#define __OLED_CS_CLR()      Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_CS_GPIO, OLED_CS_PIN, false)

#define __OLED_RES_SET()     Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_RES_GPIO, OLED_RES_PIN, true)
#define __OLED_RES_CLR()     Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_RES_GPIO, OLED_RES_PIN, false)

#define __OLED_DC_SET()      Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_DC_GPIO, OLED_DC_PIN, true)
#define __OLED_DC_CLR()      Chip_GPIO_WritePortBit(LPC_GPIO_PORT, OLED_DC_GPIO, OLED_DC_PIN, false)


void OLED_Write_Byte(uint8_t chData, uint8_t chCmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Clear_Screen(uint8_t chFill);
void OLED_Draw_Point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint);
void OLED_Draw_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OLED_Draw_Bitmap(const uint8_t *pchBmp);
void OLED_Puts(uint8_t x, uint8_t y, char *text);
void OLED_Init(void);

#endif /* OLED_H_ */
