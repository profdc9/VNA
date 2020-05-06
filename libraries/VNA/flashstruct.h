#ifndef _FLASHSTRUCT_H
#define _FLASHSTRUCT_H

#include <ff.h>
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

int readflashstruct(int flash_page_number, int num_blocks, void *blocks[], int blocklen[]);
int writeflashstruct(int flash_page_number, int num_blocks, void *blocks[], int blocklen[]);
void flash_mount_fs_card(void);

void usb_init_serial();
bool usb_init_mass_storage();
void usb_mass_storage_loop(void);

int opendatafile(FIL *fp, char *filename);

extern uint8_t flash_fs_mounted;

#endif  /* _FLASHSTRUCT_H */
