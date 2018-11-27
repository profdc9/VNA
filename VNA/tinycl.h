#ifndef _TINYCL_H
#define _TINYCL_H

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

#define TINYCL_ARDUINO_DEFAULT
#define TINYCL_MAX_PARAMETERS 4
#define TINYCL_COMMAND_BUFFER 80

typedef enum { TINYCL_PARM_END=0, TINYCL_PARM_BOOL, TINYCL_PARM_INT, TINYCL_PARM_STR } tinycl_parmtype;

typedef struct _tinycl_bool
{
    bool b;
} tinycl_bool;

typedef struct _tinycl_integer
{ 
    int i;
} tinycl_integer;

typedef struct _tinycl_string
{
    char *str;
} tinycl_string;

typedef union _tinycl_parameter
{
   tinycl_integer ti;
   tinycl_string  ts;
   tinycl_bool    tb;
} tinycl_parameter;

typedef int  (*tinycl_getchar)(void *);
typedef void (*tinycl_putchar)(char c, void *v);
typedef int  (*tinycl_commandfunc)(int args, tinycl_parameter *tp, void *v);

typedef struct _tinycl_command
{
  const char *command_name;
  const char *description;
  tinycl_commandfunc tc;
  const tinycl_parmtype tpt[TINYCL_MAX_PARAMETERS]; 
} tinycl_command;

int tinycl_task(int num_cmd, const tinycl_command *tc, void *v);
void tinycl_print_commands(int num_cmd, const tinycl_command *tc);
void tinycl_set_getchar(tinycl_getchar tg, void *tgp);
void tinycl_set_putchar(tinycl_putchar tp, void *tpp);

#endif  /* _TINYCL_H */
