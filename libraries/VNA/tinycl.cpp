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
#include "tinycl.h"
#include "consoleio.h"

char               tinycl_command_buffer[TINYCL_COMMAND_BUFFER+1];
int                tinycl_cur_char = 0;
bool               tinycl_do_echo = 1;
bool               tinycl_do_checksum = 0;

#ifdef TINYCL_ARDUINO_DEFAULT
int tinycl_arduino_getchar(void *v)
{
  return console_inchar();
}

void tinycl_arduino_putchar(char c,void *v)
{
  console_printchar(c);
}
tinycl_getchar tinycl_tg     = tinycl_arduino_getchar;
void           *tinycl_tg_ptr = NULL;
tinycl_putchar tinycl_tp     = tinycl_arduino_putchar;
void           *tinycl_tp_ptr = NULL;
#else
tinycl_getchar tinycl_tg     = NULL;
void           *tinycl_tg_ptr = NULL;
tinycl_putchar tinycl_tp     = NULL;
void           *tinycl_tp_ptr = NULL;
#endif

int tinycl_get_char(void)
{
  return tinycl_tg(tinycl_tg_ptr);
}

void tinycl_put_char(char c)
{
  tinycl_tp(c,tinycl_tp_ptr);
}

void tinycl_put_string(const char *c)
{
  while (*c != 0) tinycl_put_char(*c++);
}

void tinycl_set_getchar(tinycl_getchar tg, void *tgp)
{
  tinycl_tg = tg; tinycl_tg_ptr = tgp;
}

void tinycl_set_putchar(tinycl_putchar tp, void *tpp)
{
  tinycl_tp = tp; tinycl_tp_ptr = tpp;
}

char *tinycl_advance(char *c)
{
  while (*c == ' ') c++;
  return c;
}

char *tinycl_skip_end(char *c)
{
  while ((*c != ' ') && (*c != 0)) c++;
  return c;
}

char tinycl_upcase(char c)
{
  if ((c >= 'a') && (c <= 'z')) return (c-32);
  return c; 
}

int tinycl_hexdigit(char c)
{
  if ((c >= '0') && (c <= '9')) return (c-'0');
  c = tinycl_upcase(c);
  if ((c >= 'A') && (c <= 'F')) return (c-'A'+10);
  return -1;
}

void tinycl_put_stringcrlf(const char *c)
{
  tinycl_put_string(c);
  tinycl_put_string("\r\n");
}

void tinycl_print_commands(int num_cmd, const tinycl_command *tc)
{
  int i,j;
  for (i=0;i<num_cmd;i++)
  {
    const tinycl_command *tcmd = &tc[i];
    tinycl_put_string(tcmd->command_name);
    j = 0;
    while ((j < TINYCL_MAX_PARAMETERS) && (tcmd->tpt[j] != TINYCL_PARM_END))
    {
      switch (tcmd->tpt[j]) {
          case TINYCL_PARM_BOOL: tinycl_put_string(" Boolean"); break;
          case TINYCL_PARM_INT: tinycl_put_string(" Integer"); break;
          case TINYCL_PARM_STR: tinycl_put_string(" String"); break;
      }
      j++;
    }
    tinycl_put_string(": ");
    tinycl_put_stringcrlf(tcmd->description);
  }
}

int tinycl_get_command(void)
{
  for (;;)
  {
    int c = tinycl_get_char();
    if (c < 0) return 0;
    if ((c == '\r') || (c == '\n'))
    {
      tinycl_command_buffer[tinycl_cur_char]='\000';
      if (!tinycl_do_checksum)
      {
        if (tinycl_do_echo) tinycl_put_stringcrlf("");
        tinycl_cur_char = 0;
        return 1;
      }
      if (tinycl_cur_char >= 2)
      { 
        unsigned char checksum = 0x00;
        int i;
        for (i=2;i<tinycl_cur_char;i++) checksum += tinycl_command_buffer[i];
        i = tinycl_hexdigit(tinycl_command_buffer[1]);
        if (i >= 0)
        {
          checksum += i;
          i = tinycl_hexdigit(tinycl_command_buffer[0]);
          if (i >= 0)
          {
            checksum += (i << 4);
            if (checksum == 0)
            {
              tinycl_put_stringcrlf("\006ACK");
              tinycl_cur_char = 0;
              return 1;   
            }
          }
        }
      }
      tinycl_put_stringcrlf("\025NAK");
      tinycl_cur_char = 0;
    }
    if (c == 3)
    {
      tinycl_cur_char = 0;
      if (tinycl_do_echo) tinycl_put_stringcrlf("!");
    }
    if (((c == '\b') || (c == 127)) && (tinycl_cur_char > 0))
    {
       if (tinycl_do_echo) tinycl_put_string("\b \b");
       tinycl_cur_char--;
    }
    if (((c >= 32) && (c < 127)) && (tinycl_cur_char < TINYCL_COMMAND_BUFFER)) 
    {
       if (tinycl_do_echo) tinycl_put_char((char)c);
       tinycl_command_buffer[tinycl_cur_char]=c;
       tinycl_cur_char++;   
    }
  }
}

void tinycl_error_message(const char *c)
{
  if (!tinycl_do_echo) return;
  tinycl_put_string(">>> ");
  tinycl_put_string(c);
  tinycl_put_stringcrlf(" <<<");
}

int tinycl_task(int num_cmd, const tinycl_command *tc, void *v)
{
  int i;
  char *c;
  const tinycl_command *tcmd;
  tinycl_parameter tinycl_parameter_buffer[TINYCL_MAX_PARAMETERS];

  if (!tinycl_get_command()) return 0;
  c = tinycl_command_buffer + (tinycl_do_checksum ? 2 : 0);
  c = tinycl_advance(c);
  if (*c == 0) return 0;
  tcmd = NULL;
  for (i = 0; i < num_cmd; i++)
  {
    tcmd = &tc[i];
    const char *cmdname = tcmd->command_name;
    char *cmd = c;
    while ((*cmdname != 0) && (tinycl_upcase(*cmdname) == tinycl_upcase(*cmd)))
    {
      cmdname++;
      cmd++;
    }
    if ((*cmdname == 0) && ((*cmd == ' ') || (*cmd == 0)))
    {
       c = cmd;
       break;
    }
  }
  if (i == num_cmd)
  {
     tinycl_error_message("Command Not Found"); 
     return 0;
  }
  i = 0;
  while ((i < TINYCL_MAX_PARAMETERS) && (tcmd->tpt[i] != TINYCL_PARM_END))
  {
     tinycl_parameter *tcp = &tinycl_parameter_buffer[i];
     c = tinycl_advance(c);
     if (*c == '\000')
     {
       tinycl_error_message("Insufficient Number of Parameters"); 
       return 0; 
     }
     switch (tcmd->tpt[i])
     {
        case TINYCL_PARM_BOOL:
            {
              char b = *c;
              b = tinycl_upcase(b); 
              if ((b != 'Y') && (b != 'N') && (b != 'T') && (b != 'F'))
              {
                   tinycl_error_message("Boolean must be Y/N/T/F"); 
                   return 0;
              }
              tcp->tb.b = ((b == 'Y') || (b == 'T'));
              c = tinycl_skip_end(c);
            }
            break;
        case TINYCL_PARM_INT:
            {
               int intval = 0;
               int intsign = 0;
               if (*c == '-')
               {
                  intsign = 1;
                  c++;
               }
               if ((*c < '0') || (*c > '9'))
               {
                     tinycl_error_message("Badly formatted integer"); 
                     return 0;
               }
               if ((c[0] == '0') && (tinycl_upcase(c[1]) == 'X'))
               {
                 c += 2;
                 for (;;)
                 {
                    int ch = tinycl_hexdigit(*c);
                    if (ch < 0) break;
                    intval = (intval << 4) | ch;
                    c++;
                 }
               } else
               {
                 while ((*c >= '0') && (*c <= '9'))
                 {
                    intval = (intval*10) + (*c - '0');
                    c++;
                 }
               }
               if (intsign) intval = -intval;
               c = tinycl_skip_end(c);
               tcp->ti.i = intval;   
            }
            break;
        case TINYCL_PARM_STR:
            {
              char *beginstring;
              if (*c == '\"')
              {
                 beginstring = ++c;
                 while ((*c != '\"') && (*c != 0)) c++;
              } else
              {
                beginstring = c;
                c = tinycl_skip_end(c);
              }
              if (*c != 0)
              {
                *c = '\000';
                 c++;
              }
              tcp->ts.str = beginstring;                 
            }
            break;
        default:
            /* shouldn't get here ! */
            return 0;
     }
     i++;
  }
  return tcmd->tc(i,tinycl_parameter_buffer,v);
}
