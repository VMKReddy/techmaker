/**
 *	TechMaker
 *	https://techmaker.ua
 *
 *	STM32 LCD OLED Library for 0.96" / 1.3" displays using I2C bus
 *	based on library by Tilen Majerle, Adafruit GFX & Adafruit TFT LCD libraries
 *	28 Sep 2017 by Alexander Olenyev <sasha@techmaker.ua>
 *
 *	Changelog:
 *		- v1.1 added Courier New font family with Cyrillic (CP1251), created using TheDotFactory font generator
 *		- v1.0 added support for SSD1306 and SH1106 chips
 */

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "registers.h"
#include "Fonts/fonts.h"
#include "stm32f1xx_hal.h"

// Please uncomment one of the lines to select your OLED chip
//#define SSD1306
//#define SH1106

#if defined (SSD1306)
#define USE_HORZMODE
//#define USE_PAGEMODE
#elif defined (SH1106)
#define USE_PAGEMODE
#else
#error Please select your OLED chip in oled.h, lines 26-27
#endif

#define OLED_I2C_ADDR		0x3C	// 0x3D

#define OLEDWIDTH			128
#define OLEDHEIGHT			64

typedef enum {
	Black = 0x00,	/*!< Black color, no pixel */
	White = 0x01	/*!< Pixel is set. Color depends on LCD */
} OLED_Color_t;

#define swap(a, b)			do {\
								int16_t t = a;\
								a = b;\
								b = t;\
							} while(0)

void OLED_Init(I2C_HandleTypeDef * hi2c);
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_Color_t color);
void OLED_UpdateScreen(void);
void OLED_FillScreen(OLED_Color_t Color);
void OLED_ToggleInvert(void);

void OLED_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, OLED_Color_t color);
void OLED_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_Color_t color);
void OLED_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, OLED_Color_t color);

void OLED_DrawCircle(int16_t x0, int16_t y0, int16_t r, OLED_Color_t color);
void OLED_DrawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, OLED_Color_t color);
void OLED_FillCircle(int16_t x0, int16_t y0, int16_t r, OLED_Color_t color);
void OLED_FillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, int16_t delta, OLED_Color_t color);
void OLED_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, OLED_Color_t color);
void OLED_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, OLED_Color_t color);
void OLED_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, OLED_Color_t color);
void OLED_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, OLED_Color_t color);

void OLED_DrawChar(int16_t x, int16_t y, uint8_t c, OLED_Color_t color, uint8_t fontIndex);
void OLED_Printf(const char *fmt, ...);

void OLED_SetCursor(uint16_t x, uint16_t y);
void OLED_SetTextSize(uint8_t s);
void OLED_SetTextColor(OLED_Color_t c);
void OLED_SetTextWrap(uint8_t w);
int16_t OLED_GetCursorX(void);
int16_t OLED_GetCursorY(void);

void OLED_DisplayOn(void);
void OLED_DisplayOff(void);

#if defined(SSD1306)
void OLED_ScrollStartHorzRight(uint8_t startrow, uint8_t stoprow, uint8_t step);
void OLED_ScrollStartHorzLeft(uint8_t startrow, uint8_t stoprow, uint8_t step);
void OLED_ScrollStartDiagRight(uint8_t startrow, uint8_t stoprow, uint8_t step, uint8_t vertoffset);
void OLED_ScrollStartDiagLeft(uint8_t startrow, uint8_t stoprow, uint8_t step, uint8_t vertoffset);
void OLED_ScrollStop(void);
void OLED_FadeStart(uint8_t mode, uint8_t step);
void OLED_FadeStop(void);
#if OLEDHEIGHT > 32
void OLED_SetZoomIn(uint8_t enable);
#endif
#endif

#endif /* __OLED_H */
