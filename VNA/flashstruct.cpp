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
#include "flashstruct.h"

#include "libmaple/util.h"
#include "libmaple/flash.h"

#define FLASH_KEY1      ((uint32_t)0x45670123)
#define FLASH_KEY2      ((uint32_t)0xCDEF89AB)

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

static int wait_flash_not_busy(void)
{
  unsigned int inittime = CPU_CYCLES;
  do
  {
     if (!(FLASH_BASE->SR & FLASH_SR_BSY))
        return 1;
  } while (((unsigned int)(CPU_CYCLES - inittime)) < (F_CPU / 1000u));
  return 0;
}

int writeflashstruct(void *flash_page, int num_blocks, void *blocks[], int blocklen[])
{
  int err = 1;
  __IO uint16_t *curloc = (uint16_t *) flash_page;
  int n;
  //FLASH_Unlock
  FLASH_BASE->KEYR = FLASH_KEY1;
  FLASH_BASE->KEYR = FLASH_KEY2;

  err = wait_flash_not_busy(); 
  for (n=0;(n<num_blocks) && (err != 0);n++)
  {
    uint16_t *bl = (uint16_t *) blocks[n];
    unsigned int shortlen = (blocklen[n]+1)/2;
    unsigned int c;
    for (c=0;(c<shortlen) && (err != 0);c++)
    {
      if ((((unsigned int)curloc) & 0x3FF) == 0)
      {
        FLASH_BASE->CR |= FLASH_CR_PER; 
        FLASH_BASE->AR = (unsigned int)curloc;
        FLASH_BASE->CR |= FLASH_CR_STRT;
        err = wait_flash_not_busy();
        FLASH_BASE->CR &= ~FLASH_CR_PER;
      }
      if (err)
      {
         FLASH_BASE->CR |= FLASH_CR_PG;
         if (err = wait_flash_not_busy())
            *curloc++ = bl[c];
         FLASH_BASE->CR &= ~FLASH_CR_PG;
      }
    }
  }
  FLASH_BASE->CR |= FLASH_CR_LOCK;
  return err;
}

int readflashstruct(void *flash_page, int num_blocks, void *blocks[], int blocklen[])
{
  int err = 1;
  uint16_t *curloc = (uint16_t *) flash_page;
  int n;

  for (n=0;(n<num_blocks) & (err != 0);n++)
  {
    uint16_t *bl = (uint16_t *) blocks[n];
    unsigned int shortlen = (blocklen[n]+1)/2;
    unsigned int c;
    if (bl == NULL)
       curloc += shortlen;
    else
    {
       for (c=0;(c<shortlen) & (err != 0);c++) bl[c] = *curloc++;
    }
  }
  return err;
}
