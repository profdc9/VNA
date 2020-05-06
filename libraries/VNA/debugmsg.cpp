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

#include "Arduino.h"
#include <stdarg.h>
#include "debugmsg.h"
#include "consoleio.h"

#define USE_MINIPRINTF

#ifdef USE_MINIPRINTF
#include "mini-printf.h"
#endif

#ifdef DEBUGM

unsigned char debugmsg_state = 0;

void setDebugMsgMode(unsigned char state)
{
  debugmsg_state = state;
}

void debugmsgprintf(const char *format, ...)
{
  char msg[MAXMSG];
  va_list ap;

  if (!debugmsg_state) return;
	va_start(ap, format);
#ifdef USE_MINIPRINTF
  mini_vsnprintf(msg,sizeof(msg)-1,(const char *)format,ap);
#else
  vsnprintf(msg,sizeof(msg)-1,(const char *)format,ap);
#endif
	msg[MAXMSG-1] = '\000';
	console_println(msg);
}

#endif
