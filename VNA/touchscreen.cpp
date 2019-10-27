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
    int16_t h = (t->h == 0) ? 12 * t->textsize : t->h;
    gfx->fillRoundRect(t->x, t->y, w, h, t->r, inverted ? t->outline_color : t->fill_color);
    gfx->drawRoundRect(t->x, t->y, w, h, t->r, inverted ? t->fill_color : t->outline_color);
    gfx->setCursor(t->x + 3 * t->textsize, t->y + (h / 2) -  (4 * t->textsize));
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
}

bool touchscreen_abort(void)
{
  int16_t xp, yp, zp;
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
  { 0, 240, 210, 4, 0, 0, 0xFFFF, 0x0000, 0xFFFF, " Esc ", 2, NULL },
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
  { 0, 20, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 0 ", 2, NULL },
  { 1, 80, 200,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 1 ", 2, NULL },
  { 2, 130, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 2 ", 2, NULL },
  { 3, 180, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 3 ", 2, NULL },
  { 4, 80, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 4 ", 2, NULL },
  { 5, 130, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 5 ", 2, NULL },
  { 6, 180, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 6 ", 2, NULL },
  { 7, 80, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 7 ", 2, NULL },
  { 8, 130, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 8 ", 2, NULL },
  { 9, 180, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, " 9 ", 2, NULL },
  { 10, 20, 100, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "ENT", 2, NULL },
  { 11, 20, 150, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "BCK", 2, NULL },
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

#define TOUCHSCREEN_SMITH_RADIUS 0.45f
#define TOUCHSCREEN_BOTTOM_LINES 24

typedef struct _touchscreen_axes_parameters
{
  float minx, maxx, minrndx, maxrndx, stepspacex, slopex;
  float miny1, maxy1, minrndy1, maxrndy1, stepspacey1, slopey1;
  float miny2, maxy2, minrndy2, maxrndy2, stepspacey2, slopey2;
  const char *axislabel1, *axislabel2;
  int16_t dec, dec1, dec2;
  int16_t w, h;
  int16_t lastx, lasty1, lasty2;
  unsigned char port;

  Complex *axis_data;
  int16_t num_axis_data;
  const char *format_string;
  Complex *sparms;
} touchscreen_axes_parameters;

bool touchscreen_smith_chart = false;
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

void touchscreen_allocate_axis_data(const char *format_string)
{
  taps.num_axis_data = -1;
  if (taps.axis_data != NULL) free(taps.axis_data);
  taps.axis_data = NULL;
  taps.format_string = format_string;
}

void touchscreen_free_axis_data(void)
{
  if (taps.axis_data != NULL) free(taps.axis_data);
  taps.axis_data = NULL;
}

void touchscreen_set_axis_data(int n, int total, const Complex &c)
{
  if (taps.num_axis_data == -1)
  {
    taps.num_axis_data = total;
    taps.axis_data = (Complex *)malloc(sizeof(Complex) * total);
  }
  if (taps.axis_data != NULL)
    taps.axis_data[n] = c;
}

void touchscreen_marker_line(int16_t x)
{
  int16_t h = tft.height() - TOUCHSCREEN_BOTTOM_LINES;
  for (int16_t y = 0; y < h; y += 4)
  {
    uint16_t pixval = tft.readPixel(x, y);
    tft.drawPixel(x, y, pixval ^ (((uint16_t)0xFC) << 3));
  }
}

void touchscreen_touch_axis(void)
{
  char s[80];
  int16_t h = tft.height() - TOUCHSCREEN_BOTTOM_LINES;
  int16_t w = tft.width();
  int n = taps.num_axis_data / 2;
  for (;;)
  {
    int16_t xp, yp, zp;
    tft.fillRect(0, h, tft.width(), TOUCHSCREEN_BOTTOM_LINES, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    n = (n < 0) ? 0 : n;
    n = (n >= taps.num_axis_data) ? taps.num_axis_data - 1 : n;
    float xaxis = ((taps.maxx - taps.minx) * n) / ((float)taps.num_axis_data) + taps.minx;
    tft.setCursor(0, h + 8);
    mini_snprintf(s, sizeof(s) - 1, taps.format_string, float2int32(xaxis), float2int32(taps.axis_data[n].real), float2int32(taps.axis_data[n].imag));
    tft.print(s);
    tft.setCursor(250, h + 8);
    tft.print("Tap Escape");
    int col = n * ((int)w) / ((int)taps.num_axis_data);
    touchscreen_marker_line(col);
    //     tft.writeFastVLine(col, h, TOUCHSCREEN_BOTTOM_LINES, ILI9341_GREEN);
    while (!touchscreen_solid_press(xp, yp, zp, true));
    touchscreen_marker_line(col);
    if (yp >= h) break;
    n += (8 * (xp - w / 2)) / w;
  }
  touchscreen_wait();
}

int touchscreen_draw_smith_chart(Adafruit_GFX *gfx, touchscreen_axes_parameters *t)
{
  char s[40];
  char *c;
  int h2, w2;

  t->w = gfx->width();
  t->h = gfx->height();
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

int touchscreen_rtr_swr_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  float z0 = (float)vna_state.char_impedance;
  Complex ref = (imp - z0) / (imp + z0);
  float swr = ref.absv();
  swr = (1.0f + swr) / (1.0f - swr);
  touchscreen_set_axis_data(n, total, Complex(swr));
  if ((swr > 0.5) && (swr < 1.0f)) swr = 1.0f;
  if ((swr <= 0.5) || (swr > 9.5f)) swr = 9.5f;
  int16_t ycoor1 = (t->maxy1 - swr) * t->slopey1;
  int16_t xcoor = (freq - t->minx) * t->slopex;
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w - 1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h - 1;
  if (t->lasty1 >= 0)
    tft.writeLine(t->lastx, t->lasty1, xcoor, ycoor1, ILI9341_LIGHTGREY);
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
  touchscreen_allocate_axis_data("%f: SWR %02f");
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_draw_axes(&tft, &taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_impedance(touchscreen_rtr_swr_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
      touchscreen_wait();
      break;
    case 1: touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      break;
    case 2: touchscreen_touch_axis();
      break;
  }
  touchscreen_free_axis_data();
  return 0;
}

int touchscreen_rtr_zacq_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  touchscreen_axes_parameters *t = &taps;
  int16_t xcoor = (freq - t->minx) * t->slopex;
  int16_t ycoor1, ycoor2;
  if (t->port == 1)
  {
    ycoor1 = (t->maxy1 - imp.real) * t->slopey1;
    ycoor2 = (t->maxy2 - imp.imag) * t->slopey2;
    touchscreen_set_axis_data(n, total, imp);
  } else
  {
    ycoor1 = (t->maxy1 - zthru.real) * t->slopey1;
    ycoor2 = (t->maxy2 - zthru.imag) * t->slopey2;
    touchscreen_set_axis_data(n, total, zthru);
  }
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w - 1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h - 1;
  if (ycoor2 < 0) ycoor2 = 0;
  if (ycoor2 >= t->h) ycoor2 = t->h - 1;
  if (t->lasty1 >= 0)
    tft.writeLine(t->lastx, t->lasty1, xcoor, ycoor1, ILI9341_RED);
  if (t->lasty2 >= 0)
    tft.writeLine(t->lastx, t->lasty2, xcoor, ycoor2, ILI9341_BLUE);
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
  taps.miny1 = -touchscreen_axes_impedance_scale * 0.04f;
  taps.maxy1 = touchscreen_axes_impedance_scale;
  taps.axislabel1 = "Real";
  taps.miny2 = -touchscreen_axes_impedance_scale;
  taps.maxy2 = touchscreen_axes_impedance_scale;
  taps.axislabel2 = "Imag";
  tft.fillScreen(ILI9341_BLACK);
  touchscreen_allocate_axis_data("%f: Z(%03f,%03f)");
  touchscreen_draw_axes(&tft, &taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_impedance(touchscreen_rtr_zacq_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
      touchscreen_wait();
      break;
    case 1: touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      break;
    case 2: touchscreen_draw_axes(&tft, &taps);
      touchscreen_touch_axis();
      break;
  }
  touchscreen_free_axis_data();
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
    xcoor = t->w / 2 + (s11.real * ((float)t->h) * TOUCHSCREEN_SMITH_RADIUS);
    ycoor1 = t->h / 2 - (s11.imag * ((float)t->h) * TOUCHSCREEN_SMITH_RADIUS);
    line1_color = (n * 31) / total;
    line1_color = (line1_color << 11) | (31 - line1_color);
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
    xcoor = (freq - t->minx) * t->slopex;
    if (t->port == 1)
    {
      ycoor1 = (t->maxy1 - s11db) * t->slopey1;
      ycoor2 = (t->maxy2 - s11deg) * t->slopey2;
      touchscreen_set_axis_data(n, total, Complex(s11db, s11deg));
    } else
    {
      ycoor1 = (t->maxy1 - s21db) * t->slopey1;
      ycoor2 = (t->maxy2 - s21deg) * t->slopey2;
      touchscreen_set_axis_data(n, total, Complex(s21db, s21deg));
    }
    line1_color = ILI9341_RED;
  }
  if (xcoor < 0) xcoor = 0;
  if (xcoor >= t->w) xcoor = t->w - 1;
  if (ycoor1 < 0) ycoor1 = 0;
  if (ycoor1 >= t->h) ycoor1 = t->h - 1;
  if (ycoor2 < 0) ycoor2 = 0;
  if (ycoor2 >= t->h) ycoor2 = t->h - 1;
  if (t->lasty1 >= 0)
    tft.writeLine(t->lastx, t->lasty1, xcoor, ycoor1, line1_color);
  if ((!touchscreen_smith_chart) && (t->lasty2 >= 0))
    tft.writeLine(t->lastx, t->lasty2, xcoor, ycoor2, ILI9341_BLUE);
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
  taps.maxy1 = touchscreen_axes_db_scale * 0.1f;
  taps.axislabel1 = "Mag (dB)";
  taps.miny2 = -180.0f;
  taps.maxy2 = 180.0f;
  taps.axislabel2 = "Phase";
  touchscreen_allocate_axis_data("%f: %03f dB %03f degs");
  tft.fillScreen(ILI9341_BLACK);
  if (touchscreen_smith_chart)
    touchscreen_draw_smith_chart(&tft, &taps);
  else
    touchscreen_draw_axes(&tft, &taps);
  taps.lasty1 = -1;
  taps.lasty2 = -1;
  switch (vna_acquire_sparm(touchscreen_rtr_sparm_display))
  {
    case 0: touchscreen_display_message("Can only be\nperformed after 1\nor 2 port calibration");
      touchscreen_wait();
      break;
    case 1: touchscreen_display_message("Acquisition aborted");
      touchscreen_wait();
      break;
    case 2: if (touchscreen_smith_chart)
      {
        touchscreen_draw_smith_chart(&tft, &taps);
        touchscreen_wait();
      }
      else
      {
        touchscreen_draw_axes(&tft, &taps);
        touchscreen_touch_axis();
      }
      break;
  }
  touchscreen_free_axis_data();
  return 0;
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
  
    tft.setTextSize(1);
    mini_snprintf(s,sizeof(s)-1,"Ls %03f uH Cs %03f pF\nLp %03f uH Cp %03f pF",float2int32(Ls), float2int32(Cs), float2int32(Lp), float2int32(Cp));
    tft.setCursor(0,136);
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
  entered = touchscreen_enter_number("Averages", "1 to 5000", averages);
  if (!entered) return 0;
  if ((averages < 1) || (averages > 5000))
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

const touchscreen_button_panel_entry settingspanel[] =
{
  { 0, 0, 200, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Return to Main Menu", 2, NULL },
  { 200, 0, 40,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Char Imped", 2, touchscreen_char_impedance },
  { 300, 150, 40,   4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Averages", 2, touchscreen_averages },
  { 400, 0, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Imped Scale", 2, touchscreen_impedance_scale },
  { 500, 150, 80, 4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "dB Scale", 2, touchscreen_db_scale },
  { 600, 0, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Smith", 2, touchscreen_smith },
  { 700, 80, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Atten", 2, touchscreen_atten },
  { 800, 160, 120,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Series/Shunt", 2, touchscreen_series },
  { 900, 0, 160,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Serial Rem", 2, touchscreen_remote },
  { 1000, 160, 160,  4, 0, 36, 0xFFFF, 0x0000, 0xFFFF, "Cal Freq ", 2, touchscreen_calfreqs }
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
  tft.print("(c) 2018 D. Marks, CC-BY-SA 4.0");
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
  tcal.iscal = true;
}

void touchscreen_setup() {
  SPI.begin(); //Initialize the SPI_1 port.
  touchscreen_spi_reset();
  tft.begin(SPI, 18000000ul);
  ts.begin();
  tft.setRotation(1);
}
