#ifndef _STRUCTCONF_H
#define _STRUCTCONF_H

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

typedef enum { STRUCTCONF_INT32=0, STRUCTCONF_INT16, STRUCTCONF_INT8, STRUCTCONF_INT32_HEX, STRUCTCONF_INT16_HEX, STRUCTCONF_INT8_HEX,
    STRUCTCONF_FLOAT, STRUCTCONF_STRING } structconf_datatype;

#define STRUCTCONF_INTMIN -2147483648
#define STRUCTCONF_INTMAX 2147483647

typedef struct _structure_entry
{ 
   const char          *parmname;
   structconf_datatype  dtype;
   uint16_t             offset;
   uint16_t             nentries;
   int                  minval;
   int                  maxval;
   const char          *description;
} structure_entry;

void se_print_structure(int nentries, const structure_entry se[], void *str);
void se_set_structure_field(int nentries, const structure_entry se[], void *str, const char *c);

#endif  /* _STRUCTCONF_H */
