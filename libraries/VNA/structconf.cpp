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
#include <stddef.h>
#include <stdlib.h>
#include "structconf.h"
#include "mini-printf.h"
#include "debugmsg.h"
#include "consoleio.h"

void se_printchar(char c)
{
  console_printchar(c);
}

void se_printstring(const char *c)
{
  while (*c != 0) console_printchar(*c++);
}

void se_printnum(int num, int radix)
{
  char bf[24];
  int len = mini_itoa(num, radix, 1, 0, bf, 0);
  bf[len] = '\000';
  se_printstring(bf);
}

void se_print_structure(int nentries, const structure_entry se[], void *str)
{
  int f,n;
  for (f=0;f<nentries;f++)
  {
     const structure_entry *s = &se[f];
     void *dptr = ((char *)str) + s->offset;
     se_printstring(s->parmname);
     se_printchar('=');
     if (s->dtype == STRUCTCONF_STRING)
     {
        for (n=0;n<s->nentries;n++)
        {
          if (((char *)dptr)[n] == 0) break;
          se_printchar(((char *)dptr)[n]);
        }
     } else
     {
        for (n=0;n<s->nentries;n++)
        {
           if (n != 0) se_printchar(','); 
           switch (s->dtype)
           {
              case STRUCTCONF_INT32:  
              case STRUCTCONF_INT32_HEX:  {
                                            int temp = (((uint32_t *)dptr)[n]);              
                                            se_printnum(temp,s->dtype == STRUCTCONF_INT32_HEX ? 16 : 10);
                                          }
                                          break;
                                        
              case STRUCTCONF_INT16:  
              case STRUCTCONF_INT16_HEX:  {
                                            int temp = (((uint16_t *)dptr)[n]);
                                            se_printnum(temp,s->dtype == STRUCTCONF_INT16_HEX ? 16 : 10);
                                          }
                                          break;
              case STRUCTCONF_INT8:  
              case STRUCTCONF_INT8_HEX:   {
                                            int temp = (((uint8_t *)dptr))[n];
                                            se_printnum(temp,s->dtype == STRUCTCONF_INT8_HEX ? 16 : 10);
                                          }
                                          break;
              case STRUCTCONF_FLOAT:      {
                                            char st[20];
                                            mini_ftoa(((float *)dptr)[n], 2, st, sizeof(st)-1);
                                            se_printstring(st);
                                          }
                                          break;
           }
       }
     }
     if (s->description != NULL)
     {
       se_printchar(' ');
       se_printstring(s->description);
     }
     se_printstring("\r\n");
  }
}

char structconf_upcase(char c)
{
  if ((c >= 'a') && (c <= 'z')) return (c-32);
  return c; 
}

int structconf_hexdigit(char c)
{
  if ((c >= '0') && (c <= '9')) return (c-'0');
  c = structconf_upcase(c);
  if ((c >= 'A') && (c <= 'F')) return (c-'A'+10);
  return -1;
}

const char *structconf_skipline(const char *c)
{
  while ((*c != 0) && (*c != '\r') && (*c != ' ')) c++;
  while ((*c == '\r') || (*c == ' ')) c++;
  return c;
}

bool char2float(const char **c, float *f)
{
  int neg = 0;
  const char *ch = *c;
  float fl = 0.0f;
  if (*ch == '-')
  { 
    ch++; neg = 1;
  }
  if ((*ch < '0') || (*ch > '9')) 
    return false;
  while ((*ch >= '0') && (*ch <= '9'))
    fl = (fl * 10.0f) + (*ch++ - '0');
  if (*ch == '.')
  {
     float dp = 1.0f;
     ch++;
     while ((*ch >= '0') && (*ch <= '9'))
     {
       dp *= 0.1f;
       fl = fl + dp*(*ch++ - '0');
     }
  }
  if (neg) fl = -fl;
  *f = fl;
  *c = ch;
  return true;
}  

void se_set_structure_field(int nentries, const structure_entry se[], void *str, const char *c)
{
  int f, ct;
  const structure_entry *s;
  void *dptr; 
  while (*c)
  {
    for (f=0;f<nentries;f++)
    {
       s = &se[f];
       const char *parm = c;
       const char *parmname = s->parmname;
       while ((*parmname != 0) && (structconf_upcase(*parmname) == structconf_upcase(*parm)))
       {
          parm++; parmname++;
       }
       if ((*parmname == 0) && (*parm == '=')) 
       {
         c = parm+1;
         break;
       }
    }
    if (f == nentries)
    {
      c = structconf_skipline(c);
      continue;
    }
    dptr = ((char *)str) + s->offset;
    ct = 0;
    switch (s->dtype)
    {
       case STRUCTCONF_STRING:
          {
            while ((ct < s->nentries) && (*c != 0))
               ((char *)dptr)[ct++] = *c++;
            if (ct >= s->nentries) ct = s->nentries - 1; 
            ((char *)dptr)[ct] = '\000';
          }
          break;
       case STRUCTCONF_FLOAT:
          {
            while (ct < s->nentries)
            {
              float num;
              if (char2float(&c,&num))
              {
                if ((num >= s->minval) && (num <= s->maxval))
                  ((float *)dptr)[ct] = num;
              }
              else break;
              if (*c == ',')
              {
                c++; ct++;
              } else break;
            }
          }
          break;
       case STRUCTCONF_INT32:
       case STRUCTCONF_INT16:
       case STRUCTCONF_INT8:
       case STRUCTCONF_INT32_HEX:
       case STRUCTCONF_INT16_HEX:
       case STRUCTCONF_INT8_HEX:
          {
            while (ct < s->nentries)
            {
              int num = 0;
              int neg = 0;
              if (*c == '-')
              {
                c++;
                neg = 1;
              }
              if ((s->dtype == STRUCTCONF_INT32_HEX) || (s->dtype == STRUCTCONF_INT16_HEX) || (s->dtype == STRUCTCONF_INT8_HEX))
              {
                while (structconf_hexdigit(*c) >= 0)
                   num = (num << 4) + structconf_hexdigit(*c++);
              } else
              {              
                while ((*c >= '0') && (*c <= '9'))
                   num = num * 10 + (*c++ - '0');            
              }
              if (neg) num = -num;
              if ((num >= s->minval) && (num <= s->maxval))
              {
                if ((s->dtype == STRUCTCONF_INT32) || (s->dtype == STRUCTCONF_INT32_HEX)) ((uint32_t *)dptr)[ct] = num;
                if ((s->dtype == STRUCTCONF_INT16) || (s->dtype == STRUCTCONF_INT16_HEX)) ((uint16_t *)dptr)[ct] = num;
                if ((s->dtype == STRUCTCONF_INT8) || (s->dtype == STRUCTCONF_INT8_HEX)) ((uint8_t *)dptr)[ct] = num;
              }
              if (*c == ',')
              {
                 c++; ct++;
              } else break;
            }
          }
          break;
    }
    c = structconf_skipline(c);
  }
}
