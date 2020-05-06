/* Jog wheel class */

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

#include <Arduino.h>

#include "jogwheel.h"

static char constStateA = 0;
static char constStateB = 0;
static char constStatePUSH = 0;
volatile int curStateA = HIGH;
volatile int curStateB = HIGH;
volatile int curStatePUSH = HIGH;

#define JOGWHEEL_INPUT_A PA1
#define JOGWHEEL_INPUT_B PA15
#define JOGWHEEL_INPUT_PUSH PA8

#define JOGWHEEL_BUTTON_A PA1
#define JOGWHEEL_BUTTON_B PA15
#define JOGWHEEL_BUTTON_SELECT PA8

#define IFTIMER TIMER4

volatile int jogwheel_interrupts;
volatile int jogwheel_total_counts;
volatile int jogwheel_read_counts;
volatile bool jogwheel_read;

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

#define IFTIMER_POLLTIME (F_CPU/1000u)

JogWheel::JogWheel()
{
}

bool JogWheel::getSelect(bool waitselect)
{
  if (!curStatePUSH)
  {
    if (waitselect) while (!curStatePUSH);
    return true;
  }
  return false;
}

void JogWheel::poll()
{
}

void jogWheelInterrupt(void)
{
  int state;
  
  jogwheel_interrupts++;
  state = digitalRead(JOGWHEEL_INPUT_PUSH);
  if (curStatePUSH != state)
  {
    if ((++constStatePUSH) >= 5)
    {
        curStatePUSH = state;
        constStatePUSH = 0;
    }
  } else constStatePUSH = 0;
  state = digitalRead(JOGWHEEL_INPUT_A);
  if (curStateA != state)
  {
    if ((++constStateA) >= 5)
    {
        curStateA = state;
        constStateA = 0;
    }
  } else constStateA = 0;
  state = digitalRead(JOGWHEEL_INPUT_B);
  if (curStateB != state)
  {
    if ((++constStateB) >= 5)
    {
        curStateB = state;
        constStateB = 0;
        if (state == LOW)
        {
          if (jogwheel_read) jogwheel_read = false;
          int dir = (curStateA == LOW) ? 1 : -1;
          jogwheel_read_counts += dir;
          jogwheel_total_counts += dir;
        }
    }
  } else constStateB = 0;
}

int JogWheel::readCounts()
{ 
  jogwheel_read = true;
  int temp = jogwheel_read_counts;
  jogwheel_read_counts = 0;
  return temp;
}

int JogWheel::totalCounts()
{
  return jogwheel_total_counts;
}

int JogWheel::readInterrupts()
{
  return jogwheel_interrupts;
}

void JogWheel::setup()
{
  timer_init(IFTIMER);
  timer_set_prescaler(IFTIMER, 100);
  timer_set_mode(IFTIMER, 1, TIMER_OUTPUT_COMPARE);
  timer_set_count(IFTIMER, 0);
  timer_set_reload(IFTIMER, IFTIMER_POLLTIME/100);
  timer_set_compare(IFTIMER, 1, 0);
  timer_generate_update(IFTIMER);
  timer_attach_interrupt(IFTIMER, 1, jogWheelInterrupt);
  timer_resume(IFTIMER);
  
  pinMode(JOGWHEEL_INPUT_A, INPUT_FLOATING);
  pinMode(JOGWHEEL_INPUT_B, INPUT_FLOATING);
  pinMode(JOGWHEEL_INPUT_PUSH, INPUT_FLOATING);
  jogwheel_read_counts = 0;
  jogwheel_total_counts = 0;
  jogwheel_interrupts = 0;
  jogwheel_read = false;
}
