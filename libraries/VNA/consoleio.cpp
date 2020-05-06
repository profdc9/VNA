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
#include <stdarg.h>
#include "consoleio.h"
#include "mini-printf.h"

Stream *mainSerial = NULL;
Stream *externalSerial = NULL;

void console_setMainSerial(Stream *serialDevice)
{
  mainSerial = serialDevice;
}

void console_setExternalSerial(Stream *serialDevice)
{
  externalSerial = serialDevice;
}

void console_printchar(const char c)
{
  if (mainSerial != NULL) mainSerial->print(c);
  if (externalSerial != NULL) externalSerial->print(c);
}

int console_inchar(void)
{
  if ((mainSerial != NULL) && (mainSerial->available())) return mainSerial->read();
  if ((externalSerial != NULL) && (externalSerial->available())) return externalSerial->read();
  return -1;
}

void console_printcrlf(void)
{
  console_printchar('\r');
  console_printchar('\n');
}

void console_print(const char *c)
{
  while (*c) console_printchar(*c++);
}

void console_println(const char *c)
{
  console_print(c);
  console_printcrlf();
}

void console_print(int n)
{
  char s[20];
  mini_itoa(n, 10, 0, 0, s, 0);
	console_print(s);
}

void console_print(unsigned int n)
{
  char s[20];
  mini_itoa(n, 10, 0, 1, s, 0);
  console_print(s);
}
void console_println(int n)
{
  console_print(n);
  console_printcrlf();
}

void console_println(unsigned int n)
{
  console_print(n);
  console_printcrlf();
}
