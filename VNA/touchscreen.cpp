/*
   Copyright (c) 2018 Daniel Marks

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

#include "Arduino.h"
#include <stdarg.h>
#include "VNA.h"
#include "debugmsg.h"
#include "mini-printf.h"

#include "touchscreen.h"

// For the Adafruit shield, these are the default.
#define TFT_DC PB10
#define TFT_CS PB8

#define XPT2046_CS PB9

XPT2046_Touchscreen ts(XPT2046_CS);
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC);

touchscreen_calibration tcal;

/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

void touchscreen_spi_reset(void)
{
  SPI.beginTransaction(SPISettings(180000000ul, MSBFIRST, SPI_MODE0, DATA_SIZE_16BIT));
}

void touchscreen_draw_button(Adafruit_GFX *gfx, const touchscreen_button_panel_entry *t, bool inverted)
{
  int16_t len = strlen(t->label);
  if (t->w == 0)
  {
    int16_t w = 6 * (len + 1) * t->textsize;
    int16_t h = 12 * t->textsize;
    gfx->fillRoundRect(t->x, t->y, w, h, t->r, inverted ? t->outline_color : t->fill_color);
    gfx->drawRoundRect(t->x, t->y, w, h, t->r, inverted ? t->fill_color : t->outline_color);
    gfx->setCursor(t->x + 3 * t->textsize, t->y + 2 * t->textsize);
  } else
  {
    gfx->fillRoundRect(t->x, t->y, t->w, t->h, t->r, inverted ? t->outline_color : t->fill_color);
    gfx->drawRoundRect(t->x, t->y, t->w, t->h, t->r, inverted ? t->fill_color : t->outline_color);
    gfx->setCursor(t->x + (t->w / 2) - (len * 3 * t->textsize), t->y + (t->h / 2) - (4 * t->textsize));
  }
  gfx->setTextSize(t->textsize);
  gfx->setTextColor(inverted ? t->fill_color : t->outline_color);
  gfx->print(t->label);
}

void touchscreen_draw_button_panel(Adafruit_GFX *gfx, int n_entries, const touchscreen_button_panel_entry *tbpe)
{
  int n;
  for (n = 0; n < n_entries; n++)
  {
    const touchscreen_button_panel_entry *t = &tbpe[n];
    touchscreen_draw_button(gfx, t);
  }
}

bool touchscreen_solid_release(int16_t ms)
{
  int n;
  ms = ms / 25;
  ms = (ms < 1) ? 1 : ms;
  for (n = 0; n < ms; n++)
  {
    if (!ts.touched())
    {
      touchscreen_spi_reset();
      return true;
    }
    delay(25);
  }
  touchscreen_spi_reset();
  return false;
}

bool touchscreen_solid_press(int16_t &x, int16_t &y, int16_t &z, bool applycal, int16_t ms )
{
  int px = 0, py = 0, pz = 0;
  ms = ms / 25;
  ms = (ms < 1) ? 1 : ms;
  for (int n = 0; n < ms; n++)
  {
    if (!ts.touched())
    {
      touchscreen_spi_reset();
      return false;
    }
    TS_Point p = ts.getPoint();
    px += p.x;
    py += p.y;
    pz += p.z;
    delay(25);
  }
  touchscreen_spi_reset();
  x = px / ms;
  y = py / ms;
  z = pz / ms;
  if (applycal)
  {
    int64_t xd = ((int64_t)x * (int64_t)tcal.a) + ((int64_t)y * (int64_t)tcal.b) + (int64_t)tcal.c;
    int64_t yd = ((int64_t)x * (int64_t)tcal.d) + ((int64_t)y * (int64_t)tcal.e) + (int64_t)tcal.f;
    x = xd / TOUCHSCREEN_FIXEDPOINT_OFFSET;
    y = yd / TOUCHSCREEN_FIXEDPOINT_OFFSET;
  }
  return true;
}

bool touchscreen_abort(void)
{
  int16_t xp,yp,zp;
  bool aborted = touchscreen_solid_press(xp, yp, zp, false, 300);
  touchscreen_solid_release(300);
  return aborted;
}

int16_t touchscreen_get_button_press(Adafruit_GFX *gfx, int n_entries, const touchscreen_button_panel_entry *tbpe, void *v)
{
  int16_t xp, yp, zp;
  int n;
  const touchscreen_button_panel_entry *t;
  if (!touchscreen_solid_press(xp, yp, zp))
    return -1;
  for (n = 0; n < n_entries; n++)
  {
    t = &tbpe[n];
    int16_t len = strlen(t->label);
    if (t->w == 0)
    {
      int16_t w = 6 * (len + 1) * t->textsize;
      int16_t h = 12 * t->textsize;
      if ((xp >= t->x) && (xp < (t->x + w)) && (yp >= t->y) && (yp < (t->y + h)))
        break;
    } else
    {
      if ((xp >= t->x) && (xp < (t->x + t->w)) && (yp >= t->y) && (yp < (t->y + t->h)))
        break;
    }
  }
  if (n == n_entries)
    return -1;
  touchscreen_draw_button(gfx, t, true);
  touchscreen_solid_release();
  touchscreen_draw_button(gfx, t, false);
  if (t->tc != NULL) t->tc(t->code, v);
  return n;
}

void touchscreen_wait(void)
{
  int16_t xp, yp, zp;
  while (!touchscreen_solid_press(xp, yp, zp, false));
  touchscreen_solid_release(300);
}

void touchscreen_display_block(int16_t x, int16_t y, const char *c, int16_t textsize, bool center)
{
  tft.setTextSize(textsize);
  tft.setTextColor(ILI9341_WHITE);
  if (x < 0) x = tft.width() / 2;
  if (y < 0) y = tft.height() / 2;
  int lineno = 0, len;
  while (*c)
  {
    len = 0;
    while ((c[len] != 0) && (c[len] != '\n')) len++;
    tft.setCursor(center ? x - (len * 3 * textsize) : x, y + (10 * lineno * textsize));
    while ((*c != 0) && (*c != '\n')) tft.write(*c++);
    if (*c == '\n') c++;
    lineno++;
  }
}

void touchscreen_display_message(const char *c)
{
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_display_block(-1, -1, c, 2, true);
}

int touchscreen_short(int code, void *v)
{
  touchscreen_display_message("Acquiring Short Cal");
  switch (vna_shortcalib())
  {
    case 0: touchscreen_display_message("Aborted");
            break;
    case 1: touchscreen_display_message("Short calibration\nsuccessful");
            break;
    case 2: touchscreen_display_message("Port calibration\nsuccessful");
            break;
  }
  touchscreen_wait();
  return 0;
}

int touchscreen_load(int code, void *v)
{
  touchscreen_display_message("Acquiring Load Cal");
  switch (vna_loadcalib())
  {
    case 0: touchscreen_display_message("Aborted");
            break;
    case 1: touchscreen_display_message("Load calibration\nsuccessful");
            break;
    case 2: touchscreen_display_message("Port calibration\nsuccessful");
            break;
  }
  touchscreen_wait();
  return 0;
}

int touchscreen_open(int code, void *v)
{
  touchscreen_display_message("Acquiring Open Cal");
  switch (vna_opencalib())
  {
    case 0: touchscreen_display_message("Aborted");
            break;
    case 1: touchscreen_display_message("Open calibration\nsuccessful");
            break;
    case 2: touchscreen_display_message("Port calibration\nsuccessful");
            break;
  }
  touchscreen_wait();
  return 0;
}

int touchscreen_thru(int code, void *v)
{
  touchscreen_display_message("Acquiring Thru Cal");
  switch (vna_thrucal())
  {
    case 0: touchscreen_display_message("Two Port\nMust Have Valid\nOne Port Cal");
            break;
    case 1: touchscreen_display_message("Calibration aborted");
            break;
    case 2: touchscreen_display_message("Two port calibration\nsuccessful");
            break;
  }
  touchscreen_wait();
}

const touchscreen_button_panel_entry calpanel[] =
{
  { 0, 255, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Esc", 2, NULL },
  { 1, 0, 0,  4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 1 ", 2, NULL },
  { 2, 0, 30, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 2 ", 2, NULL },
  { 3, 0, 60, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 3 ", 2, NULL },
  { 4, 0, 90, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 4 ", 2, NULL },
  { 5, 0, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 5 ", 2, NULL },
  { 6, 0, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 6 ", 2, NULL },
  { 7, 0, 180, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 7 ", 2, NULL },
  { 8, 0, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 8 ", 2, NULL },
};

void touchscreen_displaycal(void)
{
   int n;
   tft.fillScreen(ILI9341_BLACK);
   touchscreen_draw_button_panel(&tft, sizeof(calpanel) / sizeof(touchscreen_button_panel_entry), calpanel);
   for (n=1;n<=8;n++)
   {
     char s[120];
     vna_calentry(n,s,-(sizeof(s)-1));
     touchscreen_display_block(50,30*(n-1),s,1,0);
   }
}

int touchscreen_listcal(int code, void *v)
{
  touchscreen_displaycal();
  touchscreen_wait();
}

int touchscreen_readcal(int code, void *v)
{
  touchscreen_display_message("Read Calibration");
  delay(500);
  touchscreen_displaycal();
  int n;
  do
  {
    n = touchscreen_get_button_press(&tft, sizeof(calpanel) / sizeof(touchscreen_button_panel_entry), calpanel);
  } while (n < 0);
  if (n == 0) return 0;
  if (vna_readcal(n))
  {
    touchscreen_display_message("Successfully\nRead Calibration");
  } else
    touchscreen_display_message("Failed Reading\nCalibration");
  touchscreen_wait();
  return 0;
}

int touchscreen_writecal(int code, void *v)
{
  touchscreen_display_message("Write Calibration");
  delay(500);
  touchscreen_displaycal();
  int n;
  do
  {
    n = touchscreen_get_button_press(&tft, sizeof(calpanel) / sizeof(touchscreen_button_panel_entry), calpanel);
  } while (n < 0);
  if (n == 0) return 0;
  if (vna_writecal(n))
  {
    touchscreen_display_message("Successfully\nWrote Calibration");
  } else
    touchscreen_display_message("Failed Writing\nCalibration");
  touchscreen_wait();
  return 0;
}

const touchscreen_button_panel_entry numpanel[] =
{
  { 0, 0, 200, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 0 ", 2, NULL },
  { 1, 60, 200,  4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 1 ", 2, NULL },
  { 2, 110, 200, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 2 ", 2, NULL },
  { 3, 160, 200, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 3 ", 2, NULL },
  { 4, 60, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 4 ", 2, NULL },
  { 5, 110, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 5 ", 2, NULL },
  { 6, 160, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 6 ", 2, NULL },
  { 7, 60, 100, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 7 ", 2, NULL },
  { 8, 110, 100, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 8 ", 2, NULL },
  { 9, 160, 100, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 9 ", 2, NULL },
  { 10, 0, 100, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "ENT", 2, NULL },
  { 11, 0, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "BCK", 2, NULL },
  { 12, 255, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Esc", 2, NULL },
};

bool touchscreen_enter_number(const char *c1, const char *c2, unsigned int &number)
{
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_button_panel(&tft, sizeof(numpanel) / sizeof(touchscreen_button_panel_entry), numpanel);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  if (c1 != NULL)
  {
    tft.setCursor(0,0);
    tft.print(c1);
  } 
  if (c2 != NULL)
  {
    tft.setCursor(0,25);
    tft.print(c2);
  }
  number = 0;
  char s[20];
  while (1)
  {
    tft.fillRect(100, 50, 120, 30, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(100,50);
    mini_snprintf(s,sizeof(s)-1,"%u",number);
    tft.print(s);
    int n;
    do
    {
      n = touchscreen_get_button_press(&tft, sizeof(numpanel) / sizeof(touchscreen_button_panel_entry), numpanel);
    } while (n < 0);
    if (((n >= 0) && (n <= 9)) && (number < 400000000)) number = (number * 10 + n);
    if (n == 10) return true;
    if (n == 11) number = number / 10;
    if (n == 12) return false;
  }
}

int touchscreen_freqs(int code, void *v)
{
  char s[80];
  unsigned int startfreq, stopfreq, numfreq;
  bool entered;
  mini_snprintf(s,sizeof(s)-1,"%u to %u",VNA_MIN_FREQ,VNA_MAX_FREQ);
  entered = touchscreen_enter_number("Start frequency (Hz)",s,startfreq);
  if (!entered) return 0;
  if ((startfreq < VNA_MIN_FREQ) || (startfreq > VNA_MAX_FREQ))
  {
     touchscreen_display_message("Invalid Start\nFrequency");
     touchscreen_wait();  
     return 0;
  }
  mini_snprintf(s,sizeof(s)-1,"%u to %u",startfreq,VNA_MAX_FREQ);
  entered = touchscreen_enter_number("Stop frequency (Hz)",s,stopfreq);
  if (!entered) return 0;
  if ((stopfreq < startfreq) || (stopfreq > VNA_MAX_FREQ))
  {
     touchscreen_display_message("Invalid Stop\nFrequency");
     touchscreen_wait();  
     return 0;
  }
  mini_snprintf(s,sizeof(s)-1,"1 to %u",VNA_MAX_FREQS);
  entered = touchscreen_enter_number("Number of frequencies",s,numfreq);
  if (!entered) return 0;
  if ((numfreq < 1) || (numfreq > VNA_MAX_FREQS))
  {
     touchscreen_display_message("Invalid Number\nof Frequencies");
     touchscreen_wait();  
     return 0;
  }
  vna_set_frequencies(numfreq,startfreq,stopfreq);
  mini_snprintf(s,sizeof(s)-1,"Range %u Hz\nto %u Hz\n# Freq %u",startfreq,stopfreq,numfreq);
  touchscreen_display_message(s);
  touchscreen_wait();  
  return 0;
}

int touchscreen_recal(int code, void *v)
{
  touchscreen_calibrate();
  return 0;
}

#define TOUCHSCREEN_SMITH_RADIUS 0.45f

typedef struct _touchscreen_axes_parameters
{
  float minx, maxx, minrndx, maxrndx, stepspacex, slopex;
  float miny1, maxy1, minrndy1, maxrndy1, stepspacey1, slopey1;
  float miny2, maxy2, minrndy2, maxrndy2, stepspacey2, slopey2;
  char *axislabel1, *axislabel2;
  int16_t dec, dec1, dec2;
  int16_t w, h;
  int16_t lastx, lasty1, lasty2;
  unsigned char port;
} touchscreen_axes_parameters;

float pow10f(int p)
{
  float f = 1.0f;
  while (p>0)
  {
    p--;
    f = f * 10.0f;
  }
  while (p<0)
  {
    p++;
    f = f * 0.1f;
  }
  return f;
}

void touchscreen_round_spacing(int minticks, float minval, float maxval, float &stepspace, float &minrnd, float &maxrnd, int16_t &dec)
{
  float dif = maxval-minval;
  stepspace = pow10f((int)floorf(logf(dif)/logf(10.0f)));
  if ((dif / stepspace) < minticks) stepspace *= 0.5f;  
  if ((dif / stepspace) < minticks) stepspace *= 0.4f; 
  dec =  -((int)floorf(logf(stepspace)/logf(10.0f)));
  if (dec < 0) dec = 0;
  minrnd = ceil(minval/stepspace)*stepspace;
  maxrnd = floor(maxval/stepspace)*stepspace;
}

char *mini_ftoa(float f, int dec, char *s, int len)
{
  char *last = s+len;
  *(--last) = '\000';
  unsigned int temp;
  
  bool neg = false;
  if (f < 0)
  {
    f = -f;
    neg = true;
  }
  if (dec > 0)
  {
    temp = floorf(pow10f(dec)*(f-floorf(f)));
    while (dec > 0)
    {
       dec--;
       if (last > s)
          *(--last) = '0' + (temp % 10);
       temp /= 10;
    }
    if (last > s)
      *(--last) = '.';
  }
  temp = floorf(f);
  do
  {
     if (last > s)
         *(--last) = '0' + (temp % 10);
     temp /= 10;
  } while (temp > 0);
  if ((neg) && (last > s))
    *(--last) = '-';
  return last;  
}

int touchscreen_draw_smith_chart(Adafruit_GFX *gfx, touchscreen_axes_parameters *t)
{
  char s[40];
  char *c;
  int h2,w2;

  t->w = gfx->width();
  t->h = gfx->height();
  w2 = t->w/2;
  h2 = t->h/2;

  float radchart = (TOUCHSCREEN_SMITH_RADIUS*t->h);
  for (int k=3;k<12;k++)
  {
    int16_t r = radchart*3/k;
    gfx->drawCircle(w2+radchart-r,h2,r,ILI9341_DARKGREY);
  }
  gfx->writeLine(w2-radchart,h2,w2+radchart,t->h/2,ILI9341_DARKGREY);
  for (int k=1;k<5;k++)
  {
    int16_t r = radchart/k;
    gfx->drawCircle(w2+radchart,h2+r,r,ILI9341_DARKGREY);
    gfx->drawCircle(w2+radchart,h2-r,r,ILI9341_DARKGREY);
    r = radchart*k;
    gfx->drawCircle(w2+radchart,h2+r,r,ILI9341_DARKGREY);
    gfx->drawCircle(w2+radchart,h2-r,r,ILI9341_DARKGREY);
  }  
  w2 = w2*5/4;
  for (int k=radchart+2;k<w2;k++)
  {
    gfx->drawCircle(t->w/2,t->h/2,k,ILI9341_BLACK);
    gfx->drawCircle(t->w/2-1,t->h/2,k,ILI9341_BLACK);
  }
  gfx->setTextSize(1);
  gfx->setTextColor(ILI9341_BLUE);
  gfx->setCursor(0,0);
  gfx->print("Start Frequency");
  gfx->setCursor(0,10);
  gfx->print(mini_ftoa(t->minx,0,s,sizeof(s)-1));
  gfx->setTextColor(ILI9341_RED);
  gfx->setCursor(0,20);
  gfx
  ->print("Stop Frequency");
  gfx->setCursor(0,30);
  gfx->print(mini_ftoa(t->maxx,0,s,sizeof(s)-1));
}

int touchscreen_draw_axes(Adafruit_GFX *gfx, touchscreen_axes_parameters *t)
{
  float f;
  char s[40];
  t->w = gfx->width();
  t->h = gfx->height();
  t->slopex = ((float)t->w)/(t->maxx - t->minx);
  t->slopey1 = ((float)t->h)/(t->maxy1 - t->miny1);
  touchscreen_round_spacing(5, t->minx, t->maxx, t->stepspacex, t->minrndx, t->maxrndx, t->dec);
  touchscreen_round_spacing(4, t->miny1, t->maxy1, t->stepspacey1, t->minrndy1, t->maxrndy1, t->dec1);
  if (t->axislabel2 != NULL)
  {
    t->slopey2 = ((float)t->h)/(t->maxy2 - t->miny2);
    touchscreen_round_spacing(4, t->miny2, t->maxy2, t->stepspacey2, t->minrndy2, t->maxrndy2, t->dec2);
  }
  gfx->setTextSize(1);
  gfx->setTextColor(ILI9341_DARKGREY);
  for (f=t->minrndx;f<=t->maxrndx;f+=t->stepspacex)
  {
    char *c = mini_ftoa(f,t->dec,s,sizeof(s)-1);
    int l = strlen(c);
    int i;
    int16_t hval = (f-t->minx)*t->slopex;
    gfx->writeFastVLine(hval, 0, t->h, ILI9341_DARKGREY);
    if (hval > 10)
    {
      for (int i=0;i<l;i++)
      {
        gfx->setCursor(hval-10,t->h+8*(i-l));
        gfx->print(c[i]);
      }
    }
  }
  gfx->setTextColor(ILI9341_RED);
  for (f=t->minrndy1;f<=t->maxrndy1;f+=t->stepspacey1)
  {
    int16_t vval = (t->maxy1-f)*t->slopey1;
    gfx->writeFastHLine(0, vval, t->w, ILI9341_RED);
    if (vval >= 12)
    {
      gfx->setCursor(0,vval-12);
      gfx->print(mini_ftoa(f,t->dec1,s,sizeof(s)-1));
    }
  }
  gfx->setCursor(2,2);
  gfx->print(t->axislabel1);
  if (t->axislabel2 != NULL)
  {
     gfx->setTextColor(ILI9341_BLUE);
     for (f=t->minrndy2;f<=t->maxrndy2;f+=t->stepspacey2)
     {
       char *c = mini_ftoa(f,t->dec2,s,sizeof(s)-1);
       int16_t vval = (t->maxy2-f)*t->slopey2;
       gfx->writeFastHLine(0, vval, t->w, ILI9341_BLUE);
       if (vval >= 12)
       {
         gfx->setCursor(t->w- 6*strlen(c),vval-12);
         gfx->print(c);
       }
     }
    gfx->setCursor(t->w - 6*strlen(t->axislabel2),2);
    gfx->print(t->axislabel2);
  }
}

bool touchscreen_smith_chart = false;
float touchscreen_axes_impedance_scale = 500.0f;
float touchscreen_axes_db_scale = 50.0f;
touchscreen_axes_parameters taps;

int touchscreen_smith(int code, void *v)
{
  touchscreen_smith_chart = !touchscreen_smith_chart;
  if (touchscreen_smith_chart)
      touchscreen_display_message("Smith chart\nactivated");
  else
      touchscreen_display_message("Graph mode\nactivated");
  touchscreen_wait();  
}

int touchscreen_rtr_swr_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  float z0 = (float)vna_state.char_impedance;
  Complex ref = (imp-z0)/(imp+z0);
  float swr = ref.absv();
  swr = (1.0f+swr)/(1.0f-swr);
  if (swr < 1.0f) swr = 1.0f;
  if (swr > 10.0f) swr = 10.0f;
  int16_t ycoor1 = (t->maxy1-swr)*t->slopey1;
  int16_t xcoor = (freq-t->minx)*t->slopex;
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w-1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h-1;
  if (t->lasty1 >= 0)
      tft.writeLine(t->lastx,t->lasty1,xcoor,ycoor1,ILI9341_LIGHTGREY);  
  t->lastx = xcoor;
  t->lasty1 = ycoor1;
}

int touchscreen_swr(int code, void *v)
{
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = 0.5f;
  taps.maxy1 = 9.5f;
  taps.axislabel1 = "SWR";
  taps.axislabel2 = NULL;
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_axes(&tft,&taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_impedance(touchscreen_rtr_swr_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
            break;
    case 1: touchscreen_display_message("Acquisition aborted");
            break;
  } 
  touchscreen_wait();  
  return 0;
}

int touchscreen_rtr_zacq_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  int16_t xcoor = (freq-t->minx)*t->slopex;
  int16_t ycoor1, ycoor2;
  if (t->port == 1)
  {
    ycoor1 = (t->maxy1-imp.real)*t->slopey1;
    ycoor2 = (t->maxy2-imp.imag)*t->slopey2;
  } else
  {
    ycoor1 = (t->maxy1-zthru.real)*t->slopey1;
    ycoor2 = (t->maxy2-zthru.imag)*t->slopey2;    
  }
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w-1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h-1;
  if (ycoor2 < 0) ycoor2 = 0;
  if (ycoor2 >= t->h) ycoor2 = t->h-1;
  if (t->lasty1 >= 0)
      tft.writeLine(t->lastx,t->lasty1,xcoor,ycoor1,ILI9341_RED);  
  if (t->lasty2 >= 0)
      tft.writeLine(t->lastx,t->lasty2,xcoor,ycoor2,ILI9341_BLUE);  
  t->lastx = xcoor;
  t->lasty1 = ycoor1;
  t->lasty2 = ycoor2;
}

int touchscreen_zacq(int code, void *v)
{
  if ((code == 201) && (!(vna_state.calib_state & VNA_VALID_CALIB_2PT)))
  {
    touchscreen_display_message("2 port calibration\nrequired");
    touchscreen_wait();  
    return 0;
  }
  taps.port = (code == 201) ? 2 : 1;
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = -touchscreen_axes_impedance_scale*0.04f;
  taps.maxy1 = touchscreen_axes_impedance_scale;
  taps.axislabel1 = "Real";
  taps.miny2 = -touchscreen_axes_impedance_scale;
  taps.maxy2 = touchscreen_axes_impedance_scale;
  taps.axislabel2 = "Imag";
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_axes(&tft,&taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_impedance(touchscreen_rtr_zacq_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
            break;
    case 1: touchscreen_display_message("Acquisition aborted");
            break;
    case 2: touchscreen_draw_axes(&tft,&taps);
            break;
  } 
  touchscreen_wait();  
  return 0;
}

int touchscreen_rtr_sparm_display(int n, int total, unsigned int freq, bool ch2, Complex s11, Complex s21)
{
  touchscreen_axes_parameters *t = &taps;
  int16_t xcoor, ycoor1, ycoor2;
  uint16_t line1_color;
  if (touchscreen_smith_chart)
  {      
    if (taps.port == 2) s11 = s21;
    xcoor = t->w/2 + (s11.real*((float)t->h)*TOUCHSCREEN_SMITH_RADIUS);
    ycoor1 = t->h/2 + (s11.imag*((float)t->h)*TOUCHSCREEN_SMITH_RADIUS);
    line1_color = (n*31)/total;
    line1_color = (line1_color << 11) | (31-line1_color);
  } else
  { 
     float s11db = (10.0f / logf(10.0f)) * logf(s11.absq());
     float s11deg = RAD2DEG(s11.arg());
     float s21db, s21deg;
     if (ch2)
     {
        s21db = (10.0f / logf(10.0f)) * logf(s21.absq());
        s21deg = RAD2DEG(s21.arg());    
     }
     xcoor = (freq-t->minx)*t->slopex;
     if (t->port == 1)
     {
       ycoor1 = (t->maxy1-s11db)*t->slopey1;
       ycoor2 = (t->maxy2-s11deg)*t->slopey2;    
     } else
     {
       ycoor1 = (t->maxy1-s21db)*t->slopey1;
       ycoor2 = (t->maxy2-s21deg)*t->slopey2;    
     }
     line1_color = ILI9341_RED;
  }
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w-1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h-1;
  if (ycoor2 < 0) ycoor2 = 0;
  if (ycoor2 >= t->h) ycoor2 = t->h-1;
  if (t->lasty1 >= 0)
      tft.writeLine(t->lastx,t->lasty1,xcoor,ycoor1,line1_color);  
  if ((!touchscreen_smith_chart) && (t->lasty2 >= 0))
      tft.writeLine(t->lastx,t->lasty2,xcoor,ycoor2,ILI9341_BLUE);  
  t->lastx = xcoor;
  t->lasty1 = ycoor1;
  t->lasty2 = ycoor2;
}

int touchscreen_sparm(int code, void *v)
{
  if ((code == 301) && (!(vna_state.calib_state & VNA_VALID_CALIB_2PT)))
  {
    touchscreen_display_message("2 port calibration\nrequired");
    touchscreen_wait();  
    return 0;
  }
  taps.port = (code == 301) ? 2 : 1;
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = -touchscreen_axes_db_scale;
  taps.maxy1 = touchscreen_axes_db_scale*0.1f;
  taps.axislabel1 = "Mag (dB)";
  taps.miny2 = -180.0f;
  taps.maxy2 = 180.0f;
  taps.axislabel2 = "Phase";
  tft.fillScreen(ILI9341_BLACK);
  if (touchscreen_smith_chart)
      touchscreen_draw_smith_chart(&tft,&taps);
  else 
      touchscreen_draw_axes(&tft,&taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_sparm(touchscreen_rtr_sparm_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
            break;
    case 1: touchscreen_display_message("Acquisition aborted");
            break;
    case 2: if (touchscreen_smith_chart)
                touchscreen_draw_smith_chart(&tft,&taps);
            else
                touchscreen_draw_axes(&tft,&taps);
            break;
  }
  touchscreen_wait();  
  return 0;
}

int touchscreen_char_impedance(int code, void *v)
{
  unsigned int char_imped;
  bool entered;
  entered = touchscreen_enter_number("Characteristic Imped","1 to 100000 Ohms",char_imped);
  if (!entered) return 0;
  if ((char_imped < 1) || (char_imped > 100000))
  {
     touchscreen_display_message("Invalid Impedance\nScale");
     touchscreen_wait();  
     return 0;
  }
  vna_set_characteristic_impedance(char_imped);
}

  int vna_set_averages(unsigned short averages, unsigned short timeout);

int touchscreen_averages(int code, void *v)
{
  unsigned int averages;
  bool entered;
  entered = touchscreen_enter_number("Averages","1 to 5000",averages);
  if (!entered) return 0;
  if ((averages < 1) || (averages > 5000))
  {
     touchscreen_display_message("Invalid number\nof averages");
     touchscreen_wait();  
     return 0;
  }
  vna_set_averages(averages,averages);
}

int touchscreen_impedance_scale(int code, void *v)
{
  unsigned int imped_scale;
  bool entered;
  entered = touchscreen_enter_number("Impedance Scale","1 to 100000 Ohms",imped_scale);
  if (!entered) return 0;
  if ((imped_scale < 1) || (imped_scale > 100000))
  {
     touchscreen_display_message("Invalid Impedance\nScale");
     touchscreen_wait();  
     return 0;
  }
  touchscreen_axes_impedance_scale = imped_scale;
}

int touchscreen_db_scale(int code, void *v)
{
  unsigned int db_scale;
  bool entered;
  entered = touchscreen_enter_number("dB Scale","1 to 70 dB",db_scale);
  if (!entered) return 0;
  if ((db_scale < 1) || (db_scale > 70))
  {
     touchscreen_display_message("Invalid dB\nScale");
     touchscreen_wait();  
     return 0;
  }
  touchscreen_axes_db_scale = db_scale;
}

const touchscreen_button_panel_entry settingspanel[] =
{
  { 0, 10, 200, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Return to Main Menu", 2, NULL },
  { 200, 10, 40,   4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Char Imped", 2, touchscreen_char_impedance },
  { 300, 160, 40,   4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Averages", 2, touchscreen_averages },
  { 400, 10, 80, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Imped Scale", 2, touchscreen_impedance_scale },
  { 500, 160, 80, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "dB Scale", 2, touchscreen_db_scale },
  { 600, 10, 120,  4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Smith", 2, touchscreen_smith }
};

int touchscreen_settings(int code, void *v)
{
  bool panelback = false;
  for (;;)
  {
    if (!panelback)
    {
      panelback = true;
      tft.fillScreen(ILI9341_BLACK);
      touchscreen_draw_button_panel(&tft, sizeof(settingspanel) / sizeof(touchscreen_button_panel_entry), settingspanel);
    }
    int16_t cd = touchscreen_get_button_press(&tft, sizeof(settingspanel) / sizeof(touchscreen_button_panel_entry), settingspanel);
    if (cd == 0) break;
    if (cd >= 0) panelback = false;
  }
}

const touchscreen_button_panel_entry mainpanel[] =
{
  { 100, 10, 40,   4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Freqs", 2, touchscreen_freqs },
  { 1200, 90, 40, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Settings", 2, touchscreen_settings },
  { 1300, 240, 40, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "SWR", 2, touchscreen_swr },
  { 200, 10, 80,   4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "ZRef", 2, touchscreen_zacq },
  { 201, 80, 80,   4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "ZThru", 2, touchscreen_zacq },
  { 300, 170, 80, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "S11", 2, touchscreen_sparm },
  { 301, 230, 80, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "S21", 2, touchscreen_sparm },
  { 400, 10, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Open", 2, touchscreen_open },
  { 500, 80, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Short", 2, touchscreen_short },
  { 600, 170, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Load", 2, touchscreen_load },
  { 700, 240, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Thru", 2, touchscreen_thru },
  { 800, 10, 160, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "ListCal", 2, touchscreen_listcal },
  { 900, 110, 160, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "ReadCal", 2, touchscreen_readcal },
  { 1000, 210, 160, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "WriteCal", 2, touchscreen_writecal },
  { 1100, 10, 200, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, "Touch Scrn Recalibration", 2, touchscreen_recal },
};

void touchscreen_task(void)
{
  static bool panelback = false;
  if (!tcal.iscal)
  {
    touchscreen_calibrate();
    panelback = false;
    return;
  }
  if (!panelback)
  {
    panelback = true;
    tft.fillScreen(ILI9341_BLACK);
    touchscreen_draw_button_panel(&tft, sizeof(mainpanel) / sizeof(touchscreen_button_panel_entry), mainpanel);
  }
  int16_t cd = touchscreen_get_button_press(&tft, sizeof(mainpanel) / sizeof(touchscreen_button_panel_entry), mainpanel);
  if (cd >= 0) panelback = false;
}

#define TOUCHSCREEN_CAL_TEXTSIZE 2
#define TOUCHSCREEN_CAL_BOXSIZE 10

void touchscreen_calibrate(void)
{
  Adafruit_GFX *gfx = &tft;
  int64_t xd[3], yd[3], x[3], y[3];
  int n;

  int16_t w = gfx->width();
  int16_t h = gfx->height();

  xd[0] = TOUCHSCREEN_CAL_BOXSIZE;
  yd[0] = TOUCHSCREEN_CAL_BOXSIZE;

  xd[1] = w - 2 * TOUCHSCREEN_CAL_BOXSIZE;
  yd[1] = TOUCHSCREEN_CAL_BOXSIZE;

  xd[2] = w / 2 - TOUCHSCREEN_CAL_BOXSIZE;
  yd[2] = h - 2 * TOUCHSCREEN_CAL_BOXSIZE;

  tft.fillScreen(ILI9341_BLACK);
  gfx->setTextSize(TOUCHSCREEN_CAL_TEXTSIZE);
  gfx->setCursor(w / 2 - (3 * TOUCHSCREEN_CAL_TEXTSIZE * 11), h / 2 - (4 * TOUCHSCREEN_CAL_TEXTSIZE));
  gfx->print("Touch Boxes");

  for (n = 0; n < 3; n++)
  {
    int16_t xp, yp, zp;
    delay(500);
    gfx->fillRect(xd[n] - TOUCHSCREEN_CAL_BOXSIZE / 2, yd[n] - TOUCHSCREEN_CAL_BOXSIZE / 2, TOUCHSCREEN_CAL_BOXSIZE , TOUCHSCREEN_CAL_BOXSIZE , 0xFFFF);
    delay(500);
    while (!touchscreen_solid_press(xp, yp, zp, false));
    touchscreen_solid_release(300);
    gfx->fillRect(xd[n] - TOUCHSCREEN_CAL_BOXSIZE / 2, yd[n] - TOUCHSCREEN_CAL_BOXSIZE / 2, TOUCHSCREEN_CAL_BOXSIZE , TOUCHSCREEN_CAL_BOXSIZE , 0x0000);
    x[n] = xp; y[n] = yp;
  }

  tcal.k = (x[0]-x[2])*(y[1]-y[2]) - (x[1]-x[2])*(y[0]-y[2]); 
  tcal.a = (xd[0]-xd[2])*(y[1]-y[2]) - (xd[1]-xd[2])*(y[0]-y[2]); 
  tcal.a = (((int64_t)tcal.a) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.b = (x[0]-x[2])*(xd[1]-xd[2]) - (x[1]-x[2])*(xd[0]-xd[2]);
  tcal.b = (((int64_t)tcal.b) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.c = ((int64_t)xd[0])*(x[1]*y[2]-x[2]*y[1]) - ((int64_t)xd[1])*(x[0]*y[2]-x[2]*y[0]) + ((int64_t)xd[2])*(x[0]*y[1]-x[1]*y[0]);
  tcal.c = (((int64_t)tcal.c) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.d = (yd[0]-yd[2])*(y[1]-y[2]) - (yd[1]-yd[2])*(y[0]-y[2]); 
  tcal.d = (((int64_t)tcal.d) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.e = (x[0]-x[2])*(yd[1]-yd[2]) - (x[1]-x[2])*(yd[0]-yd[2]);
  tcal.e = (((int64_t)tcal.e) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.f = ((int64_t)yd[0])*(x[1]*y[2]-x[2]*y[1]) - ((int64_t)yd[1])*(x[0]*y[2]-x[2]*y[0]) + ((int64_t)yd[2])*(x[0]*y[1]-x[1]*y[0]);
  tcal.f = (((int64_t)tcal.f) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.iscal = true;
}

void touchscreen_setup() {
  tcal.iscal = false;
  SPI.begin(); //Initialize the SPI_1 port.
  touchscreen_spi_reset();
  tft.begin(SPI, 18000000ul);
  ts.begin();
  tft.setRotation(1);
}
