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

//#define TOUCHSCREEN_NOTOUCH
#define TOUCHSCREEN_JOGWHEEL

#ifdef TOUCHSCREEN_JOGWHEEL
#include "jogwheel.h"
JogWheel jogwheel;
#endif

#include "touchscreen.h"

// For the Adafruit shield, these are the default.
#define TFT_DC PB10
#define TFT_CS PB8
#define TFT_RESET PB12

#define XPT2046_CS PB9

XPT2046_Touchscreen ts(XPT2046_CS);
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RESET);

touchscreen_calibration tcal;

void touchscreen_spi_reset(void)
{
  SPI.beginTransaction(SPISettings(180000000ul, MSBFIRST, SPI_MODE0, DATA_SIZE_16BIT));
}

void touchscreen_draw_button(Adafruit_GFX *gfx, const touchscreen_button_panel_entry *t, int disptype)
{
  int16_t len = strlen(t->label);
  if (t->w == 0)
  {
    int16_t w = 6 * (len + 1) * t->textsize;
    int16_t h = (t->h == 0) ? 12 * t->textsize : t->h;
    gfx->fillRoundRect(t->x, t->y, w, h, t->r, disptype == 1 ? t->outline_color : t->fill_color);
    gfx->drawRoundRect(t->x, t->y, w, h, t->r, disptype == 1 ? t->fill_color : t->outline_color);
    gfx->setCursor(t->x + 3 * t->textsize, t->y + (h / 2) -  (4 * t->textsize));
  } else
  {
    gfx->fillRoundRect(t->x, t->y, t->w, t->h, t->r, disptype == 1 ? t->outline_color : t->fill_color);
    gfx->drawRoundRect(t->x, t->y, t->w, t->h, t->r, disptype == 1 ? t->fill_color : t->outline_color);
    gfx->setCursor(t->x + (t->w / 2) - (len * 3 * t->textsize), t->y + (t->h / 2) - (4 * t->textsize));
  }
  gfx->setTextSize(t->textsize);
  gfx->setTextColor(disptype == 2 ? TOUCHSCREEN_SELECT_COLOR : ((disptype == 1) ? t->fill_color : t->outline_color));
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
#ifndef TOUCHSCREEN_NOTOUCH
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
#else
  return false;
#endif
}

bool touchscreen_solid_press(int16_t &x, int16_t &y, int16_t &z, bool applycal, int16_t ms )
{
#ifndef TOUCHSCREEN_NOTOUCH
  int px = 0, py = 0, pz = 0;
  ms = (ms < 1) ? 1 : ms;
  for (int n = 0; n < ms; n++)
  {
    if (!ts.touched())
    {
      touchscreen_spi_reset();
      return false;
    }
    TS_Point p = ts.getPoint();
    if (ms > 50)
    {
      px += p.x;
      py += p.y;
      pz += p.z;
    }
    delay(1);
  }
  touchscreen_spi_reset();
  ms -= 50;
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
#else
  return false;
#endif
}

bool touchscreen_abort_enable = true;

bool touchscreen_abort(void)
{
  int16_t xp, yp, zp;
  if (!touchscreen_abort_enable) return false;
#ifdef TOUCHSCREEN_JOGWHEEL
  if (jogwheel.getSelect(true)) return true;
#endif
  bool aborted = touchscreen_solid_press(xp, yp, zp, false, 300);
  touchscreen_solid_release(300);
  return aborted;
}

const touchscreen_button_panel_entry *touchscreen_last_tbpe = NULL;
int16_t touchscreen_select_entry = 0;
bool touchscreen_redrawentry = false;

int16_t touchscreen_do_select(Adafruit_GFX *gfx, const touchscreen_button_panel_entry *tbpe, void *v, int n)
{
#ifdef TOUCHSCREEN_JOGWHEEL
  touchscreen_redrawentry = true;
#endif
  tbpe = &tbpe[n]; 
  touchscreen_draw_button(gfx, tbpe, 1);
  touchscreen_solid_release();
  touchscreen_draw_button(gfx, tbpe, 0);
  if (tbpe->tc != NULL) tbpe->tc(tbpe->code, v);
  return tbpe->code;
}

int16_t touchscreen_get_button_press(Adafruit_GFX *gfx, int n_entries, const touchscreen_button_panel_entry *tbpe, void *v)
{
  int16_t xp, yp, zp;
  int16_t n;
  const touchscreen_button_panel_entry *t;
  
#ifdef TOUCHSCREEN_JOGWHEEL
  if (touchscreen_last_tbpe != tbpe)
  {
    touchscreen_last_tbpe = tbpe;
    touchscreen_select_entry = 0;
    touchscreen_redrawentry = true;
  } 
  if (touchscreen_redrawentry)
  {
      touchscreen_draw_button(gfx, &tbpe[touchscreen_select_entry], 2);
      touchscreen_redrawentry = false;
  }
  int cts = jogwheel.readCounts();
  if (cts != 0)
  {
    touchscreen_draw_button(gfx, &tbpe[touchscreen_select_entry], 0);
    touchscreen_select_entry += cts;
    while (touchscreen_select_entry < 0) touchscreen_select_entry += n_entries;
    while (touchscreen_select_entry >= n_entries) touchscreen_select_entry -= n_entries;
    touchscreen_draw_button(gfx, &tbpe[touchscreen_select_entry], 2);
  }
  if (jogwheel.getSelect(true)) 
    return touchscreen_do_select(gfx,tbpe,v,touchscreen_select_entry);
#endif
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
  return touchscreen_do_select(gfx,tbpe,v,n);
}

void touchscreen_wait(void)
{
  int16_t xp, yp, zp;
  for (;;)
  {
    if (touchscreen_solid_press(xp, yp, zp, false))
    {
        touchscreen_solid_release(300);
        while (!touchscreen_solid_press(xp, yp, zp, false));
        break;
    }
#ifdef TOUCHSCREEN_JOGWHEEL
    if (jogwheel.getSelect(true)) break;
#endif
  }
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
  { 1, 0, 0,  4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 1 ", 2, NULL },
  { 2, 0, 30, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 2 ", 2, NULL },
  { 3, 0, 60, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 3 ", 2, NULL },
  { 4, 0, 90, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 4 ", 2, NULL },
  { 5, 0, 120, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 5 ", 2, NULL },
  { 6, 0, 150, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 6 ", 2, NULL },
  { 7, 0, 180, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 7 ", 2, NULL },
  { 8, 0, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " 8 ", 2, NULL },
  { 0, 240, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " Esc ", 2, NULL },
};

void touchscreen_displaycal(void)
{
  int n;
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_button_panel(&tft, sizeof(calpanel) / sizeof(touchscreen_button_panel_entry), calpanel);
  for (n = 1; n <= 8; n++)
  {
    char s[120];
    vna_calentry(n, s, -(sizeof(s) - 1));
    touchscreen_display_block(50, 30 * (n - 1), s, 1, 0);
  }
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
  { 10, 20, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "ENT", 2, NULL },
  { 7, 80, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 7 ", 2, NULL },
  { 8, 130, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 8 ", 2, NULL },
  { 9, 180, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 9 ", 2, NULL },
  { 11, 20, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "BCK", 2, NULL },
  { 4, 80, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 4 ", 2, NULL },
  { 5, 130, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 5 ", 2, NULL },
  { 6, 180, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 6 ", 2, NULL },
  { 0, 20, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 0 ", 2, NULL },
  { 1, 80, 200,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 1 ", 2, NULL },
  { 2, 130, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 2 ", 2, NULL },
  { 3, 180, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 3 ", 2, NULL },
  { 12, 240, 192, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " Esc " , 2, NULL },
};

void touchscreen_enter_start(const char *c1, const char *c2)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  if (c1 != NULL)
  {
    tft.setCursor(0, 0);
    tft.print(c1);
  }
  if (c2 != NULL)
  {
    tft.setCursor(0, 25);
    tft.print(c2);
  }
}

bool touchscreen_enter_number(const char *c1, const char *c2, unsigned int &number)
{
  touchscreen_enter_start(c1, c2);
  touchscreen_draw_button_panel(&tft, sizeof(numpanel) / sizeof(touchscreen_button_panel_entry), numpanel);
  number = 0;
  char s[20];
  while (1)
  {
    tft.fillRect(100, 50, 120, 30, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(100, 50);
    mini_snprintf(s, sizeof(s) - 1, "%u", number);
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

const touchscreen_button_panel_entry yesnopanel[] =
{
  { 0, 60, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " NO ", 2, NULL },
  { 1, 160, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " YES ", 2, NULL },
  { 2, 240, 190, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " Esc ", 2, NULL }
};

bool touchscreen_enter_yesno(const char *c1, const char *c2, bool &yesno)
{
  int n;
  touchscreen_enter_start(c1, c2);
  touchscreen_draw_button_panel(&tft, sizeof(yesnopanel) / sizeof(touchscreen_button_panel_entry), yesnopanel);
  do
  {
    n = touchscreen_get_button_press(&tft, sizeof(yesnopanel) / sizeof(touchscreen_button_panel_entry), yesnopanel);
  } while (n < 0);
  if (n == 2) return false;
  yesno = (n == 1);
  return true;
}

int touchscreen_calfreqs(int code, void *v)
{
  char s[80];
  unsigned int startfreq, stopfreq, numfreq;
  bool entered;
  mini_snprintf(s, sizeof(s) - 1, "%u to %u", VNA_MIN_FREQ / 1000u, VNA_MAX_FREQ / 1000u);
  entered = touchscreen_enter_number("Start frequency (kHz)", s, startfreq);
  if (!entered) return 0;
  if ((startfreq < (VNA_MIN_FREQ / 1000u)) || (startfreq > (VNA_MAX_FREQ / 1000u)))
  {
    touchscreen_display_message("Invalid Start\nFrequency");
    touchscreen_wait();
    return 0;
  }
  mini_snprintf(s, sizeof(s) - 1, "%u to %u", startfreq, VNA_MAX_FREQ / 1000u);
  entered = touchscreen_enter_number("Stop frequency (kHz)", s, stopfreq);
  if (!entered) return 0;
  if ((stopfreq < startfreq) || (stopfreq > (VNA_MAX_FREQ / 1000u)))
  {
    touchscreen_display_message("Invalid Stop\nFrequency");
    touchscreen_wait();
    return 0;
  }
  mini_snprintf(s, sizeof(s) - 1, "1 to %u", VNA_MAX_CAL_FREQS);
  entered = touchscreen_enter_number("Number of frequencies", s, numfreq);
  if (!entered) return 0;
  if ((numfreq < 1) || (numfreq > VNA_MAX_CAL_FREQS))
  {
    touchscreen_display_message("Invalid Number\nof Frequencies");
    touchscreen_wait();
    return 0;
  }
  vna_calset_frequencies(numfreq, startfreq * 1000u, stopfreq * 1000u);
  mini_snprintf(s, sizeof(s) - 1, "Cal Range %u kHz\nto %u kHz\n# Freq %u", startfreq, stopfreq, numfreq);
  touchscreen_display_message(s);
  touchscreen_wait();
  return 0;
}

int touchscreen_freqs(int code, void *v)
{
  char s[80];
  unsigned int cal_startfreq = vna_state.cal_startfreq / 1000u;
  unsigned int cal_endfreq = vna_state.cal_endfreq / 1000u;
  unsigned int startfreq, stopfreq, numfreq;
  bool entered;
  mini_snprintf(s, sizeof(s) - 1, "%u to %u", cal_startfreq, cal_endfreq);
  entered = touchscreen_enter_number("Start frequency (kHz)", s, startfreq);
  if (!entered) return 0;
  if ((startfreq < cal_startfreq) || (startfreq > cal_endfreq))
  {
    touchscreen_display_message("Invalid Start\nFrequency");
    touchscreen_wait();
    return 0;
  }
  mini_snprintf(s, sizeof(s) - 1, "%u to %u", startfreq, cal_endfreq);
  entered = touchscreen_enter_number("Stop frequency (kHz)", s, stopfreq);
  if (!entered) return 0;
  if ((stopfreq < startfreq) || (stopfreq > cal_endfreq))
  {
    touchscreen_display_message("Invalid Stop\nFrequency");
    touchscreen_wait();
    return 0;
  }
  mini_snprintf(s, sizeof(s) - 1, "1 to %u", VNA_MAX_ACQ_FREQS);
  entered = touchscreen_enter_number("Number of frequencies", s, numfreq);
  if (!entered) return 0;
  if ((numfreq < 1) || (numfreq > VNA_MAX_ACQ_FREQS))
  {
    touchscreen_display_message("Invalid Number\nof Frequencies");
    touchscreen_wait();
    return 0;
  }
  vna_set_frequencies(numfreq, startfreq * 1000u, stopfreq * 1000u);
  mini_snprintf(s, sizeof(s) - 1, "Range %u kHz\nto %u kHz\n# Freq %u", startfreq, stopfreq, numfreq);
  touchscreen_display_message(s);
  touchscreen_wait();
  return 0;
}

int touchscreen_recal(int code, void *v)
{
  touchscreen_calibrate();
  return 0;
}

#define TOUCHSCREEN_SMITH_RADIUS 0.48f
#define TOUCHSCREEN_BOTTOM_LINES 24

typedef struct _touchscreen_axes_parameters
{
  float minx, maxx, minrndx, maxrndx, stepspacex, slopex;
  float miny1, maxy1, minrndy1, maxrndy1, stepspacey1, slopey1;
  float miny2, maxy2, minrndy2, maxrndy2, stepspacey2, slopey2;
  const char *axislabel1, *axislabel2;
  int16_t dec, dec1, dec2;
  int16_t w, h;
  unsigned char port;
  bool polar;
  bool twotrace;
  bool sweeping;
  bool erase_marker;

  Complex *axis_data_acquire;
  Complex *axis_data_buffer;
  int16_t num_axis_data, current_selected;
  const char *format_string;
  Complex *sparms;
} touchscreen_axes_parameters;

bool touchscreen_smith_chart = false;
bool touchscreen_sweep_mode = false;
float touchscreen_axes_impedance_scale = 500.0f;
float touchscreen_axes_db_scale = 50.0f;
touchscreen_axes_parameters taps;

void touchscreen_round_spacing(int minticks, float minval, float maxval, float &stepspace, float &minrnd, float &maxrnd, int16_t &dec)
{
  float dif = maxval - minval;
  stepspace = ipow10f((int)floorf(logf(dif) / logf(10.0f)));
  if ((dif / stepspace) < minticks) stepspace *= 0.5f;
  if ((dif / stepspace) < minticks) stepspace *= 0.4f;
  dec =  -((int)floorf(logf(stepspace) / logf(10.0f)));
  if (dec < 0) dec = 0;
  minrnd = ceilf(minval / stepspace) * stepspace;
  maxrnd = floorf(maxval / stepspace) * stepspace;
}

void touchscreen_allocate_axis_data(const char *format_string, bool polar, bool twotrace)
{
  taps.num_axis_data = -1;
  if (taps.axis_data_acquire != NULL) free(taps.axis_data_acquire);
  if (taps.axis_data_buffer != NULL) free(taps.axis_data_buffer);
  taps.axis_data_acquire = NULL;
  taps.axis_data_buffer = NULL;
  taps.format_string = format_string;
  taps.polar = polar;
  taps.twotrace = twotrace;
  taps.current_selected = -1;
  taps.erase_marker = false;
  taps.sweeping = touchscreen_sweep_mode;
}

void touchscreen_free_axis_data(void)
{
  if (taps.axis_data_acquire != NULL) free(taps.axis_data_acquire);
  taps.axis_data_acquire = NULL;
  if (taps.axis_data_buffer != NULL) free(taps.axis_data_buffer);
  taps.axis_data_buffer = NULL;
}

void touchscreen_copy_to_buffer(void)
{
  if (taps.axis_data_buffer == NULL)
    taps.axis_data_buffer = (Complex *)malloc(sizeof(Complex) * taps.num_axis_data);
  if (taps.axis_data_buffer == NULL) return;
  for (int i=0;i<taps.num_axis_data;i++)
     taps.axis_data_buffer[i] = taps.axis_data_acquire[i];
}

void touchscreen_set_axis_data(int n, int total, const Complex &c)
{
  if (taps.num_axis_data == -1)
  {
    taps.num_axis_data = total;
    if (taps.axis_data_acquire != NULL) free(taps.axis_data_acquire);
    taps.axis_data_acquire = (Complex *)malloc(sizeof(Complex) * total);
  }
  if (taps.axis_data_acquire != NULL)
    taps.axis_data_acquire[n] = c;
}

void touchscreen_get_coor_axis_1(int16_t &x, int16_t &y, int n, int total, touchscreen_axes_parameters *t, Complex *axis_data);

#define MARKER_CROSS 7

void touchscreen_invert_pixel(int16_t x, int16_t y)
{
    uint16_t pixval = tft.readPixel(x, y);
    touchscreen_spi_reset();
    tft.drawPixel(x, y, pixval ^ (((uint16_t)0xFC) << 3));  
}

void touchscreen_correct_xy(int16_t &x, int16_t &y, touchscreen_axes_parameters *t)
{
  if (x < 0) x = 0;
  if (x >= t->w) x = t->w - 1; 
  if (y < 0) y = 0;
  if (y >= t->h) y = t->h - 1; 
}

void touchscreen_marker_line(void)
{
  int16_t w = tft.width();
  int16_t h = tft.height() - TOUCHSCREEN_BOTTOM_LINES;
  if (taps.polar)
  {
    int16_t x, y, mn, mx;
    touchscreen_get_coor_axis_1(x, y, taps.current_selected, taps.num_axis_data, &taps, taps.axis_data_acquire);
    mn = x - MARKER_CROSS;
    mn = mn < 0 ? 0 : mn;
    mx = x + MARKER_CROSS;
    mx = mx >= w ? w : mx;
    for (int16_t xx = mn; xx < mx; xx += 2)
      touchscreen_invert_pixel(xx, y);
    mn = y - MARKER_CROSS;
    mn = mn < 0 ? 0 : mn;
    mx = y + MARKER_CROSS;
    mx = mx >= h ? h : mx;
    for (int16_t yy = mn; yy < mx; yy += 2)
      touchscreen_invert_pixel(x, yy);      
    return;
  }
  int16_t x = taps.current_selected * ((int)w) / ((int)taps.num_axis_data);
  for (int16_t y = 0; y < h; y += 4)
    touchscreen_invert_pixel(x, y);
}

Complex touchscreen_rec2polar(Complex x)
{
  Complex y;
  y.real = (10.0f / logf(10.0f)) * logf(x.absq());
  y.imag = RAD2DEG(x.arg());
  return y;
}

bool touchscreen_get_touch_axis(bool axis_buffer, bool redobottom)
{
  int16_t xp, yp, zp;
  bool touchscreen_press = false;
#ifdef TOUCHSCREEN_JOGWHEEL
  int jogwheel_count = 0;
  bool jogwheel_stop = false;
#endif
  
  if (taps.num_axis_data < 0) return false;
  Complex *axis_data = axis_buffer ? taps.axis_data_buffer : taps.axis_data_acquire;
  if (axis_data == NULL) return true;
  if ((taps.current_selected >= 0) && (!redobottom))
  {
    touchscreen_press = touchscreen_solid_press(xp, yp, zp, true);
#ifdef TOUCHSCREEN_JOGWHEEL
    if (jogwheel.getSelect(true)) return true;
    jogwheel_count = jogwheel.readCounts();
    if ((!touchscreen_press) && (jogwheel_count == 0)) return false;
#else
    if (!touchscreen_press) return false;
#endif
  }
  
  int16_t h = tft.height() - TOUCHSCREEN_BOTTOM_LINES;
  int16_t w = tft.width();
  char s[80];

  if (!redobottom)
  {
    if (taps.current_selected >= 0)
    {
      if (taps.erase_marker) touchscreen_marker_line();
#ifdef TOUCHSCREEN_JOGWHEEL
      taps.current_selected += jogwheel_count;
#endif
      if (touchscreen_press)
      {
        if (yp >= h) return true;
        taps.current_selected += (8 * (xp - w / 2)) / w;
      }
      taps.current_selected = (taps.current_selected < 0) ? 0 : taps.current_selected;
      taps.current_selected = (taps.current_selected >= taps.num_axis_data) ? taps.num_axis_data - 1 : taps.current_selected;
    }
    else
      taps.current_selected = taps.num_axis_data / 2;
  }
  touchscreen_marker_line();
  taps.erase_marker = true;
  
  tft.fillRect(0, h, tft.width(), TOUCHSCREEN_BOTTOM_LINES, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  float xaxis = ((taps.maxx - taps.minx) * taps.current_selected) / ((float)taps.num_axis_data) + taps.minx;
  tft.setCursor(0, h + 8);
  Complex temp = axis_data[taps.current_selected];
  temp = taps.polar ? touchscreen_rec2polar(temp) : temp;
  mini_snprintf(s, sizeof(s) - 1, taps.format_string, float2int32(xaxis), float2int32(temp.real), float2int32(temp.imag));
  tft.print(s);
  tft.setCursor(250, h + 8);
  tft.print("Tap Escape");
  return false;
}

void touchscreen_touch_axis(bool axis_buffer)
{
  while (!touchscreen_get_touch_axis(axis_buffer, false));
}

int touchscreen_draw_smith_chart(Adafruit_GFX *gfx, touchscreen_axes_parameters *t)
{
  char s[40];
  char *c;
  int h2, w2;

  t->w = gfx->width();
  t->h = gfx->height() - TOUCHSCREEN_BOTTOM_LINES;
  w2 = t->w / 2;
  h2 = t->h / 2;

  float radchart = (TOUCHSCREEN_SMITH_RADIUS * t->h);
  for (int k = 3; k < 12; k++)
  {
    int16_t r = radchart * 3 / k;
    gfx->drawCircle(w2 + radchart - r, h2, r, ILI9341_DARKGREY);
  }
  gfx->writeLine(w2 - radchart, h2, w2 + radchart, t->h / 2, ILI9341_DARKGREY);
  for (int k = 1; k < 5; k++)
  {
    int16_t r = radchart / k;
    gfx->drawCircle(w2 + radchart, h2 + r, r, ILI9341_DARKGREY);
    gfx->drawCircle(w2 + radchart, h2 - r, r, ILI9341_DARKGREY);
    r = radchart * k;
    gfx->drawCircle(w2 + radchart, h2 + r, r, ILI9341_DARKGREY);
    gfx->drawCircle(w2 + radchart, h2 - r, r, ILI9341_DARKGREY);
  }
  w2 = w2 * 5 / 4;
  for (int k = radchart + 2; k < w2; k++)
  {
    gfx->drawCircle(t->w / 2, t->h / 2, k, ILI9341_BLACK);
    gfx->drawCircle(t->w / 2 - 1, t->h / 2, k, ILI9341_BLACK);
  }
#if 0
  gfx->setTextSize(1);
  gfx->setTextColor(ILI9341_BLUE);
  gfx->setCursor(0, 0);
  gfx->print("Start Frequency");
  gfx->setCursor(0, 10);
  mini_ftoa(t->minx, 0, s, sizeof(s) - 1);
  gfx->print(s);
  gfx->setTextColor(ILI9341_RED);
  gfx->setCursor(0, 20);
  gfx->print("Stop Frequency");
  gfx->setCursor(0, 30);
  mini_ftoa(t->maxx, 0, s, sizeof(s) - 1);
  gfx->print(s);
#endif
}

int touchscreen_draw_axes(Adafruit_GFX *gfx, touchscreen_axes_parameters *t)
{
  float f;
  char s[40];
  t->w = gfx->width();
  t->h = gfx->height() - TOUCHSCREEN_BOTTOM_LINES;
  t->slopex = ((float)t->w) / (t->maxx - t->minx);
  t->slopey1 = ((float)t->h) / (t->maxy1 - t->miny1);
  touchscreen_round_spacing(5, t->minx, t->maxx, t->stepspacex, t->minrndx, t->maxrndx, t->dec);
  touchscreen_round_spacing(4, t->miny1, t->maxy1, t->stepspacey1, t->minrndy1, t->maxrndy1, t->dec1);
  if (t->axislabel2 != NULL)
  {
    t->slopey2 = ((float)t->h) / (t->maxy2 - t->miny2);
    touchscreen_round_spacing(4, t->miny2, t->maxy2, t->stepspacey2, t->minrndy2, t->maxrndy2, t->dec2);
  }
  gfx->setTextSize(1);
  gfx->setTextColor(ILI9341_DARKGREY);
  for (f = t->minrndx; f <= t->maxrndx; f += t->stepspacex)
  {
    int len = mini_ftoa(f, t->dec, s, sizeof(s) - 1);
    int i;
    int16_t hval = (f - t->minx) * t->slopex;
    gfx->writeFastVLine(hval, 0, t->h, ILI9341_DARKGREY);
    if (hval > 10)
    {
      for (int i = 0; i < len; i++)
        gfx->drawChar(hval - 8, t->h - 3 - 6 * i, s[i], ILI9341_DARKGREY, ILI9341_BLACK, 1, true);
    }
  }
  gfx->setTextColor(ILI9341_RED);
  for (f = t->minrndy1; f <= t->maxrndy1; f += t->stepspacey1)
  {
    int16_t vval = (t->maxy1 - f) * t->slopey1;
    gfx->writeFastHLine(0, vval, t->w, ILI9341_RED);
    if (vval >= 12)
    {
      gfx->setCursor(0, vval - 12);
      mini_ftoa(f, t->dec1, s, sizeof(s) - 1);
      gfx->print(s);
    }
  }
  gfx->setCursor(2, 2);
  gfx->print(t->axislabel1);
  if (t->axislabel2 != NULL)
  {
    gfx->setTextColor(ILI9341_BLUE);
    for (f = t->minrndy2; f <= t->maxrndy2; f += t->stepspacey2)
    {
      int len = mini_ftoa(f, t->dec2, s, sizeof(s) - 1);
      int16_t vval = (t->maxy2 - f) * t->slopey2;
      gfx->writeFastHLine(0, vval, t->w, ILI9341_BLUE);
      if (vval >= 12)
      {
        gfx->setCursor(t->w - 6 * len, vval - 12);
        gfx->print(s);
      }
    }
    gfx->setCursor(t->w - 6 * strlen(t->axislabel2), 2);
    gfx->print(t->axislabel2);
  }
}

int touchscreen_smith(int code, void *v)
{
  touchscreen_smith_chart = !touchscreen_smith_chart;
  if (touchscreen_smith_chart)
    touchscreen_display_message("Smith chart\nactivated");
  else
    touchscreen_display_message("Graph mode\nactivated");
  touchscreen_wait();
}

void touchscreen_get_coor_axis_1(int16_t &x, int16_t &y, int n, int total, touchscreen_axes_parameters *t, Complex *axis_data)
{ 
  if (t->polar)
  {
    x = t->w / 2 + (axis_data[n].real * ((float)t->h) * TOUCHSCREEN_SMITH_RADIUS);
    y = t->h / 2 - (axis_data[n].imag * ((float)t->h) * TOUCHSCREEN_SMITH_RADIUS);
  } else
  {
    x = (n * (int)t->w) / total;
    y = (t->maxy1 - axis_data[n].real) * t->slopey1;
  }
  touchscreen_correct_xy(x, y, t);
}

void touchscreen_get_coor_axis_2(int16_t &x, int16_t &y, int n, int total, touchscreen_axes_parameters *t, Complex *axis_data)
{ 
  x = (n * (int)t->w) / total;
  y = (t->maxy2 - axis_data[n].imag) * t->slopey2;
  touchscreen_correct_xy(x, y, t);
}

void touchscreen_draw_axes_part(int total, int n1, int n2, bool axis_buffer)
{
  touchscreen_axes_parameters *t = &taps;
  
  Complex *axis_data = axis_buffer ? taps.axis_data_buffer : taps.axis_data_acquire;
  if (axis_data == NULL) return;
  
  if (n1 < 1) n1 = 1;
  for (int n=n1;n<n2;n++)
  {
    int16_t x1, y1, x2, y2;
    touchscreen_get_coor_axis_1(x1, y1, n-1, total, t, axis_data);
    touchscreen_get_coor_axis_1(x2, y2, n, total, t, axis_data);
#if 0
    if (t->polar)
    {
      uint16_t line1_color = (n * 31) / total;
      line1_color = (line1_color << 11) | (31 - line1_color);
      tft.writeLine(x1,y1,x2,y2,line1_color);
    } else
#endif
    {
      tft.writeLine(x1,y1,x2,y2,ILI9341_RED);
      if (t->twotrace)
      {
        touchscreen_get_coor_axis_2(x1, y1, n-1, total, t, axis_data);
        touchscreen_get_coor_axis_2(x2, y2, n, total, t, axis_data);
        tft.writeLine(x1,y1,x2,y2,ILI9341_BLUE);
      }
    }
  }
}

void touchscreen_draw_axes_all(int total, bool axis_buffer)
{
  touchscreen_draw_axes_part(total, 1, total, axis_buffer);
}

int touchscreen_rtr_swr_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  float z0 = (float)vna_state.char_impedance;
  Complex ref = (imp - z0) / (imp + z0);
  float swr = ref.absv();
  swr = (1.0f + swr) / (1.0f - swr);
  if ((swr > 0.5) && (swr < 1.0f)) swr = 1.0f;
  if ((swr <= 0.5) || (swr >= 999.0f)) swr = 999.0f;
  touchscreen_set_axis_data(n, total, Complex(swr));
  if (!t->sweeping) touchscreen_draw_axes_part(total, n, n+1, false);
}

bool touchscreen_check_calibration(void)
{
  if (vna_state.calib_state & VNA_VALID_CALIB_1PT) return true;
  touchscreen_display_message("Calibration required");
  touchscreen_wait();
  return false;
}

void touchscreen_display_acquiring(void)
{
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_display_message("Acquiring...");
  return;
}

bool touchscreen_check_two_port(void)
{
  if (vna_state.calib_state & VNA_VALID_CALIB_2PT) return true;
  touchscreen_display_message("2 port calibration\nrequired");
  touchscreen_wait();
  return false;
}

int touchscreen_acquire_cleanup(void)
{
      touchscreen_free_axis_data();
      touchscreen_abort_enable = true;
      return 0;
}

int touchscreen_swr(int code, void *v)
{
  if (!touchscreen_check_calibration()) return 0;
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = 0.5f;
  taps.maxy1 = 9.5f;
  taps.axislabel1 = "SWR";
  taps.axislabel2 = NULL;
  touchscreen_allocate_axis_data("%f: SWR %02f", false, false);

  touchscreen_abort_enable = true;
  if (taps.sweeping)
     touchscreen_display_acquiring();
  else
  {
    tft.fillScreen(ILI9341_BLACK);
    touchscreen_draw_axes(&tft, &taps);
  }
  for (;;)
  {
    vna_acquire_state vas;
    vna_setup_acquire_dataset(&vas, &vna_state, vna_display_acq_operation, (void *)touchscreen_rtr_swr_display);
    do
    {
      if ((vas.vacs == VNA_ACQUIRE_RECORDED_STATE) && (taps.sweeping) && (!touchscreen_abort_enable))
      {
        if (touchscreen_get_touch_axis(true,false))
          return touchscreen_acquire_cleanup();
      }  
      vna_operation_acquire_dataset(&vas);
    } while ((vas.vacs != VNA_ACQUIRE_TIMEOUT) && (vas.vacs != VNA_ACQUIRE_ABORT) && (vas.vacs != VNA_ACQUIRE_COMPLETE));
    if (vas.vacs != VNA_ACQUIRE_COMPLETE)
    {
      touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      return touchscreen_acquire_cleanup();
    }
    if (taps.sweeping)
    {
      touchscreen_copy_to_buffer();
      tft.fillScreen(ILI9341_BLACK);
      touchscreen_draw_axes(&tft, &taps);
      touchscreen_draw_axes_all(taps.num_axis_data,true);
      touchscreen_get_touch_axis(true,true);
    } else
    {
      touchscreen_touch_axis(false);
      return touchscreen_acquire_cleanup();
    }
    touchscreen_abort_enable = false;
  }
}

int touchscreen_rtr_zacq_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  touchscreen_set_axis_data(n, total, t->port == 1 ? imp : zthru);
  if (!t->sweeping) touchscreen_draw_axes_part(total, n, n+1, false);
}

int touchscreen_zacq(int code, void *v)
{
  if (!touchscreen_check_calibration()) return 0;
  if (code == 201)
    if (!touchscreen_check_two_port()) return 0;
  taps.port = (code == 201) ? 2 : 1;
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = -touchscreen_axes_impedance_scale * 0.04f;
  taps.maxy1 = touchscreen_axes_impedance_scale;
  taps.axislabel1 = "Real";
  taps.miny2 = -touchscreen_axes_impedance_scale;
  taps.maxy2 = touchscreen_axes_impedance_scale;
  taps.axislabel2 = "Imag";
  touchscreen_allocate_axis_data("%f: Z(%03f,%03f)", false, true);

  touchscreen_abort_enable = true;
  if (taps.sweeping)
     touchscreen_display_acquiring();
  else
  {  
     tft.fillScreen(ILI9341_BLACK);
     touchscreen_draw_axes(&tft, &taps);
  }
  for (;;)
  {
    vna_acquire_state vas;
    vna_setup_acquire_dataset(&vas, &vna_state, vna_display_acq_operation, (void *)touchscreen_rtr_zacq_display);
    do
    {
      if ((vas.vacs == VNA_ACQUIRE_RECORDED_STATE) && (taps.sweeping) && (!touchscreen_abort_enable))
      {
        if (touchscreen_get_touch_axis(true,false))
           return touchscreen_acquire_cleanup();
      }  
      vna_operation_acquire_dataset(&vas);
    } while ((vas.vacs != VNA_ACQUIRE_TIMEOUT) && (vas.vacs != VNA_ACQUIRE_ABORT) && (vas.vacs != VNA_ACQUIRE_COMPLETE));
    if (vas.vacs != VNA_ACQUIRE_COMPLETE)
    {
      touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      return touchscreen_acquire_cleanup();
    }
    if (taps.sweeping)
    {
      touchscreen_copy_to_buffer();
      tft.fillScreen(ILI9341_BLACK);
      touchscreen_draw_axes(&tft, &taps);
      touchscreen_draw_axes_all(taps.num_axis_data,true);
      touchscreen_get_touch_axis(true,true);
    } else
    {
      touchscreen_touch_axis(false);
      return touchscreen_acquire_cleanup();
    }
    touchscreen_abort_enable = false;
  }
}

int touchscreen_rtr_sparm_display(int n, int total, unsigned int freq, bool ch2, Complex s11, Complex s21)
{
  touchscreen_axes_parameters *t = &taps;
  Complex x = t->port == 1 ? s11 : s21;
  touchscreen_set_axis_data(n, total, touchscreen_smith_chart ? x :  touchscreen_rec2polar(x));
  if (!t->sweeping) touchscreen_draw_axes_part(total, n, n+1, false);
}

void touchscreen_sparm_background(touchscreen_axes_parameters *t)
{
  if (touchscreen_smith_chart)
    touchscreen_draw_smith_chart(&tft,t);
  else
    touchscreen_draw_axes(&tft,t);
}


int touchscreen_sparm(int code, void *v)
{
  if (!touchscreen_check_calibration()) return 0;
  if (code == 301)
    if (!touchscreen_check_two_port()) return 0;
  taps.port = (code == 301) ? 2 : 1;
  taps.minx = vna_state.startfreq;
  taps.maxx = vna_state.endfreq;
  taps.miny1 = -touchscreen_axes_db_scale;
  taps.maxy1 = touchscreen_axes_db_scale * 0.1f;
  taps.axislabel1 = "Mag (dB)";
  taps.miny2 = -180.0f;
  taps.maxy2 = 180.0f;
  taps.axislabel2 = "Phase";
  touchscreen_allocate_axis_data("%f: %03f dB %03f degs", touchscreen_smith_chart, !touchscreen_smith_chart);
  if (taps.polar) taps.sweeping = false;
  touchscreen_abort_enable = true;
  if (taps.sweeping)
     touchscreen_display_acquiring();
  else
  {
    tft.fillScreen(ILI9341_BLACK);
    touchscreen_sparm_background(&taps);
  }
  for (;;)
  {
    vna_acquire_state vas;
    vna_setup_acquire_dataset(&vas, &vna_state, vna_display_sparm_operation, (void *)touchscreen_rtr_sparm_display);
    do
    {
      if ((vas.vacs == VNA_ACQUIRE_RECORDED_STATE) && (taps.sweeping) && (!touchscreen_abort_enable))
      {
        if (touchscreen_get_touch_axis(true,false))
           return touchscreen_acquire_cleanup();
      }  
      vna_operation_acquire_dataset(&vas);
    } while ((vas.vacs != VNA_ACQUIRE_TIMEOUT) && (vas.vacs != VNA_ACQUIRE_ABORT) && (vas.vacs != VNA_ACQUIRE_COMPLETE));
    if (vas.vacs != VNA_ACQUIRE_COMPLETE)
    {
      touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      return touchscreen_acquire_cleanup();
    }
    if (taps.sweeping)
    {
      touchscreen_copy_to_buffer();
      tft.fillScreen(ILI9341_BLACK);
      touchscreen_sparm_background(&taps);
      touchscreen_draw_axes_all(taps.num_axis_data,true);
      touchscreen_get_touch_axis(true,true);
    } else
    {
      touchscreen_touch_axis(false);
      return touchscreen_acquire_cleanup();
    }
    touchscreen_abort_enable = false;
  }
}

const touchscreen_button_panel_entry spotpanel[] =
{
  { 0, 0, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "-10M", 1, NULL },
  { 1, 40, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "-1M", 1, NULL },
  { 2, 80, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "-100k ", 1, NULL },
  { 3, 120, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, " -10k", 1, NULL },
  { 4, 160, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "-1k", 1, NULL },
  { 5, 200, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "-100 ", 1, NULL },
  { 6, 240, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, " -10 ", 1, NULL },
  { 7, 280, 154, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, " -1 ", 1, NULL },
  { 8, 0, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+10M", 1, NULL },
  { 9, 40, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+1M", 1, NULL },
  { 10, 80, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+100k ", 1, NULL },
  { 11, 120, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, " +10k", 1, NULL },
  { 12, 160, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+1k", 1, NULL },
  { 13, 200, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+100 ", 1, NULL },
  { 14, 240, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+10 ", 1, NULL },
  { 15, 280, 180, 2, 36, 25, 0xFFFF, 0x0000, 0xFFFF, "+1 ", 1, NULL },
  { 16, 240, 210, 2, 0, 20, 0xFFFF, 0x0000, 0xFFFF, "Esc", 2, NULL },
};

int touchscreen_rtr_spot_display(int n, int total, unsigned int freq, bool ch2, Complex s11, Complex s21)
{
  taps.sparms[0] = s11;
  taps.sparms[1] = s21;
}

int touchscreen_spot(int code, void *v)
{
  Complex sparms[2];
  vna_acquisition_state vs;
  uint32_t curfreq;
  int16_t h = tft.height() - TOUCHSCREEN_BOTTOM_LINES;
  int16_t w = tft.width(), xp, yp, zp;
  bool endspot = false;
  char s[80];

  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT)) 
  {
    touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
    touchscreen_wait();
    return 0;
  }
  taps.sparms = sparms;
  curfreq = (vna_state.cal_startfreq + vna_state.cal_endfreq) / 2;

  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_button_panel(&tft, sizeof(spotpanel) / sizeof(touchscreen_button_panel_entry), spotpanel);
  while (!endspot)
  {
    if ((curfreq < vna_state.cal_startfreq) || (curfreq > 4284967295u))  /* This is 2^32-100000000 */
      curfreq = vna_state.cal_startfreq;
    if (curfreq >= vna_state.cal_endfreq)
      curfreq = vna_state.cal_endfreq - 1;
    vs = vna_state;
    vs.nfreqs = 1;
    vs.startfreq = curfreq;
    vs.endfreq = curfreq+1;
    
    int attempts = 0;
    while (attempts < 3)
    {
      if (vna_acquire_dataset(&vs, vna_display_sparm_operation, (void *)touchscreen_rtr_spot_display)) break;
      attempts++;
    } 
    if (attempts == 3) break;

    tft.fillRect(0,0,320,154,ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
  
    mini_snprintf(s,sizeof(s)-1,"Frequency %u Hz",curfreq);
    tft.setCursor(0,0);
    tft.print(s);
  
    float swr = sparms[0].absv();
    swr = (1.0f+swr)/(1.0f-swr);
    mini_snprintf(s,sizeof(s)-1,"SWR %04f\nS11 %04f+j%04f\nS11 pol %04f,%04f", float2int32(swr),float2int32(sparms[0].real), float2int32(sparms[0].imag),float2int32(sparms[0].absv()),float2int32(RAD2DEG(sparms[0].arg())));
    tft.setCursor(0,24);
    tft.print(s);
  
    Complex z = ((sparms[0]+1.0f)/(-sparms[0]+1.0f))*((float)vna_state.char_impedance);
    mini_snprintf(s,sizeof(s)-1,"Z %03f+j%03f\nZ pol %03f,%03f", float2int32(z.real), float2int32(z.imag), float2int32(z.absv()), float2int32(RAD2DEG(z.arg())));
    tft.setCursor(0,72);
    tft.print(s);
  
    tft.setTextSize(1);
    float rl = (10.0f / logf(10.0f)) * logf(sparms[0].absq());
    float Q = fabsf(z.imag)/fabsf(z.real);
    float Rp = z.absq()/z.real;
    float Xp = z.absq()/z.imag;
    mini_snprintf(s,sizeof(s)-1,"RL %04f dB, Q %04f\nRp %03f, Xp %03f", float2int32(rl), float2int32(Q), float2int32(Rp), float2int32(Xp));
    tft.setCursor(0,104);
    tft.print(s);
  
    float w = 2.0f*3.14159265359f*curfreq;
    float Ls = z.imag/w*1.0E6f;
    float Cs = -1.0e12f/(z.imag*w);
    float Lp = Xp/w*1.0E6f;
    float Cp = -1.0e12f/(Xp*w);
  
    mini_snprintf(s,sizeof(s)-1,"Ls %03f uH Cs %03f pF\nLp %03f uH Cp %03f pF",float2int32(Ls), float2int32(Cs), float2int32(Lp), float2int32(Cp));
    tft.setCursor(0,120);
    tft.print(s);
    attempts = 0;
    while (attempts < 500)
    {
      int n = touchscreen_get_button_press(&tft, sizeof(spotpanel) / sizeof(touchscreen_button_panel_entry), spotpanel);
      if (n >= 0)
      {
        switch (n)
        {
          case 0: curfreq -= 10000000u; break;
          case 1: curfreq -= 1000000u; break;
          case 2: curfreq -= 100000u; break;
          case 3: curfreq -= 10000u; break;
          case 4: curfreq -= 1000u; break;
          case 5: curfreq -= 100u; break;
          case 6: curfreq -= 10u; break;
          case 7: curfreq -= 1u; break;
          case 8: curfreq += 10000000u; break;
          case 9: curfreq += 1000000u; break;
          case 10: curfreq += 100000u; break;
          case 11: curfreq += 10000u; break;
          case 12: curfreq += 1000u; break;
          case 13: curfreq += 100u; break;
          case 14: curfreq += 10u; break;
          case 15: curfreq += 1u; break;
          case 16: endspot = true; break;
        }
        break;
      }
      attempts++;
      delay(1);
    }
  }
  return 0;
}

int touchscreen_char_impedance(int code, void *v)
{
  unsigned int char_imped;
  bool entered;
  entered = touchscreen_enter_number("Characteristic Imped", "1 to 60000 Ohms", char_imped);
  if (!entered) return 0;
  if ((char_imped < 1) || (char_imped > 60000))
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
  entered = touchscreen_enter_number("Averages", "1 to 50000", averages);
  if (!entered) return 0;
  if ((averages < 1) || (averages > 50000))
  {
    touchscreen_display_message("Invalid number\nof averages");
    touchscreen_wait();
    return 0;
  }
  vna_set_averages(averages, averages);
}

int touchscreen_impedance_scale(int code, void *v)
{
  unsigned int imped_scale;
  bool entered;
  entered = touchscreen_enter_number("Impedance Scale", "1 to 100000 Ohms", imped_scale);
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
  entered = touchscreen_enter_number("dB Scale", "1 to 70 dB", db_scale);
  if (!entered) return 0;
  if ((db_scale < 1) || (db_scale > 70))
  {
    touchscreen_display_message("Invalid dB\nScale");
    touchscreen_wait();
    return 0;
  }
  touchscreen_axes_db_scale = db_scale;
}

int touchscreen_atten(int code, void *v)
{
  bool attenchoice;
  if (!touchscreen_enter_yesno("Attenuator On?", NULL, attenchoice)) return 0;
  vna_state.atten = attenchoice ? 1 : 0;
  touchscreen_display_message(vna_state.atten == 1 ? "Attenuator On" : "Attenuator Off");
  touchscreen_wait();
}

int touchscreen_remote(int code, void *v)
{
  bool remotechoice;
  if (!touchscreen_enter_yesno("Serial Remote?", NULL, remotechoice)) return 0;
  vna_state.remote = remotechoice ? 1 : 0;
  vna_setup_remote_serial();
  touchscreen_display_message(vna_state.remote == 1 ? "Serial On" : "Serial Off");
  touchscreen_wait();
}

int touchscreen_series(int code, void *v)
{
  bool serieschoice;
  if (!touchscreen_enter_yesno("2-Port Impedance Series?", "NO=Series,YES=Shunt", serieschoice)) return 0;
  vna_state.series_shunt_two = serieschoice ? 1 : 0;
  touchscreen_display_message(vna_state.series_shunt_two == 1 ? "2-Port Shunt" : "2-Port Series");
  touchscreen_wait();
}

int touchscreen_sweep_set(int code, void *v)
{
  bool sweepchoice;
  touchscreen_sweep_mode = !touchscreen_sweep_mode;
  touchscreen_display_message(touchscreen_sweep_mode ? "Sweep On" : "Sweep Off");
  touchscreen_wait();
}

const touchscreen_button_panel_entry settingspanel[] =
{
  { 100, 0, 0,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Sweep", 2, touchscreen_sweep_set },
  { 200, 0, 40,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Char Imped", 2, touchscreen_char_impedance },
  { 300, 150, 40,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Averages", 2, touchscreen_averages },
  { 400, 0, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Imped Scale", 2, touchscreen_impedance_scale },
  { 500, 150, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "dB Scale", 2, touchscreen_db_scale },
  { 600, 0, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Smith", 2, touchscreen_smith },
  { 700, 80, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Atten", 2, touchscreen_atten },
  { 800, 160, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Series/Shunt", 2, touchscreen_series },
  { 900, 0, 160,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Serial Rem", 2, touchscreen_remote },
  { 1000, 160, 160,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Cal Freq", 2, touchscreen_calfreqs },
  { 0, 0, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Return to Main Menu", 2, NULL },
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
  { 100, 10, 40,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Freq", 2, touchscreen_freqs },
  { 1200, 78, 40, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Settings", 2, touchscreen_settings },
  { 1300, 195, 40, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "SWR", 2, touchscreen_swr },
  { 1400, 250, 40, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Spot", 2, touchscreen_spot },
  { 200, 10, 80,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "ZRef", 2, touchscreen_zacq },
  { 201, 80, 80,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "ZThru", 2, touchscreen_zacq },
  { 300, 160, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "S11", 2, touchscreen_sparm },
  { 301, 215, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "S21", 2, touchscreen_sparm },
  { 400, 10, 120, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Open", 2, touchscreen_open },
  { 500, 80, 120, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Short", 2, touchscreen_short },
  { 600, 160, 120, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Load", 2, touchscreen_load },
  { 700, 226, 120, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Thru", 2, touchscreen_thru },
  { 900, 10, 160, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "ReadCal", 2, touchscreen_readcal },
  { 1000, 110, 160, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "WriteCal", 2, touchscreen_writecal },
  { 1100, 10, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Touch Scrn Recalibration", 2, touchscreen_recal },
};


void touchscreen_title(void)
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(90, 0);
  tft.print("VNA-o-mizer by KW4TI");
  tft.setCursor(60, 10);
  tft.print("(c) 2019 D. Marks, CC-BY-SA 4.0");
}

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
    touchscreen_title();
    touchscreen_draw_button_panel(&tft, sizeof(mainpanel) / sizeof(touchscreen_button_panel_entry), mainpanel);
  }
  int16_t cd = touchscreen_get_button_press(&tft, sizeof(mainpanel) / sizeof(touchscreen_button_panel_entry), mainpanel);
  if (cd >= 0) panelback = false;
}

#define TOUCHSCREEN_CAL_TEXTSIZE 2
#define TOUCHSCREEN_CAL_BOXSIZE 10

void touchscreen_calibrate(void)
{
#ifndef TOUCHSCREEN_NOTOUCH
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

  tcal.k = (x[0] - x[2]) * (y[1] - y[2]) - (x[1] - x[2]) * (y[0] - y[2]);
  tcal.a = (xd[0] - xd[2]) * (y[1] - y[2]) - (xd[1] - xd[2]) * (y[0] - y[2]);
  tcal.a = (((int64_t)tcal.a) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.b = (x[0] - x[2]) * (xd[1] - xd[2]) - (x[1] - x[2]) * (xd[0] - xd[2]);
  tcal.b = (((int64_t)tcal.b) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.c = ((int64_t)xd[0]) * (x[1] * y[2] - x[2] * y[1]) - ((int64_t)xd[1]) * (x[0] * y[2] - x[2] * y[0]) + ((int64_t)xd[2]) * (x[0] * y[1] - x[1] * y[0]);
  tcal.c = (((int64_t)tcal.c) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.d = (yd[0] - yd[2]) * (y[1] - y[2]) - (yd[1] - yd[2]) * (y[0] - y[2]);
  tcal.d = (((int64_t)tcal.d) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.e = (x[0] - x[2]) * (yd[1] - yd[2]) - (x[1] - x[2]) * (yd[0] - yd[2]);
  tcal.e = (((int64_t)tcal.e) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
  tcal.f = ((int64_t)yd[0]) * (x[1] * y[2] - x[2] * y[1]) - ((int64_t)yd[1]) * (x[0] * y[2] - x[2] * y[0]) + ((int64_t)yd[2]) * (x[0] * y[1] - x[1] * y[0]);
  tcal.f = (((int64_t)tcal.f) * ((int64_t)TOUCHSCREEN_FIXEDPOINT_OFFSET)) / tcal.k;
#endif
  tcal.iscal = true;
}

void touchscreen_setup() {
#ifdef TOUCHSCREEN_JOGWHEEL
  jogwheel.setup();
#endif
  SPI.begin(); //Initialize the SPI_1 port.
#ifndef TOUCHSCREEN_NOTOUCH
  ts.begin();
  touchscreen_spi_reset();
#else
  pinMode(XPT2046_CS,OUTPUT);
  digitalWrite(XPT2046_CS,HIGH);
#endif
  tft.begin(SPI, 18000000ul);
  tft.setRotation(1);
}
