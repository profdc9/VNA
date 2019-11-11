#ifndef _TOUCHSCREEN_H
#define _TOUCHSCREEN_H

/*
 * Copyright (c) 2018 Daniel Marks

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GFX_AS.h"
#include "Adafruit_ILI9341_STM.h"
#include "XPT2046_Touchscreen.h"

void touchscreen_task(void);
void touchscreen_setup(void);
void touchscreen_test(void);

typedef int  (*touchscreen_commandfunc)(int code, void *v);

typedef struct _touchscreen_button_panel_entry
{
   int16_t   code;               /* button code */
   int16_t   x,y;                /* position of button */
   int16_t   r;                  /* corner radius */
   int16_t   w,h;                /* width an height of button */
   uint16_t  outline_color;      /* outline button color */
   uint16_t  fill_color;         /* fill color */
   uint16_t  text_color;         /* text color */
   const char *label;            /* text label */
   uint8_t   textsize;           /* text size */
   touchscreen_commandfunc tc;   /* function to call */
} touchscreen_button_panel_entry;

#define TOUCHSCREEN_FIXEDPOINT_OFFSET 65536
#define TOUCHSCREEN_SELECT_COLOR 0xF03F

typedef struct _touchscreen_calibration
{
  int32_t a,b,d,e,k;
  int64_t c,f;
  bool iscal;
} touchscreen_calibration;

bool touchscreen_solid_press(int16_t &x, int16_t &y, int16_t &z, bool applycal=true, int16_t ms=150);
bool touchscreen_solid_release(int16_t ms=150);
void touchscreen_draw_button_panel(Adafruit_GFX *gfx, int n_entries, const touchscreen_button_panel_entry *tbpe);
int16_t touchscreen_get_button_press(Adafruit_GFX *gfx, int n_entries, const touchscreen_button_panel_entry *tbpe, void *v=NULL);
void touchscreen_draw_button(Adafruit_GFX *gfx, const touchscreen_button_panel_entry *t, int disptype=0);
void touchscreen_calibrate(void);

void touchscreen_display_block(int16_t x, int16_t y, const char *c, int16_t textsize, bool center);
void touchscreen_display_message(const char *c);
bool touchscreen_abort(void);

extern touchscreen_calibration tcal;

#endif  /* _TOUCHSCREEN_H */
