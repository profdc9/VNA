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
#include "consoleio.h"

#include <diskio.h>
#include <ff.h>
#include <USBComposite.h>
#include <usb_mass.h>

static FATFS fs;
uint8_t flash_fs_mounted = 0;
uint8_t usb_initialized = 0;

void flash_mount_fs_card(void)
{
  f_mount(NULL,"0:",0);
  flash_fs_mounted = (f_mount(&fs,"0:",1) == FR_OK);
}

const char config_filename[] = "CONFIGXX.BIN";

USBCompositeSerial CompositeSerial;
USBMassStorage MassStorage;

#define VENDOR_ID 0x1EAE
#define MASS_STORAGE_PRODUCT_ID 0x0450
#define SERIAL_CDC_PRODUCT_ID 0x0451

bool sd_write(const uint8_t *writebuff, uint32_t startSector, uint16_t numSectors) {
  return (disk_write(0,(BYTE *)writebuff, startSector, numSectors) == RES_OK);
}

bool sd_read(uint8_t *readbuff, uint32_t startSector, uint16_t numSectors) {
  return (disk_read(0,(BYTE *)readbuff, startSector, numSectors) == RES_OK);
}

void usb_mass_storage_loop(void)
{
  MassStorage.loop();  
}

void usb_reenumerate(void)
{
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, LOW);
  delay(100);
  digitalWrite(PA12, HIGH);
  pinMode(PA12, INPUT);  
}

void usb_init_serial()
{
  if (usb_initialized)
    USBComposite.end();
  usb_initialized = 1;
  usb_reenumerate();
  USBComposite.clear();
  USBComposite.setProductId(VENDOR_ID);
  USBComposite.setProductId(SERIAL_CDC_PRODUCT_ID);
  CompositeSerial.registerComponent();
  console_setMainSerial(&CompositeSerial);
  USBComposite.begin();
  //while (!USBComposite.isReady());
}

bool usb_init_mass_storage()
{
  DWORD sectors;
  flash_mount_fs_card();
  if (!flash_fs_mounted) return false;
  if (disk_ioctl(0, GET_SECTOR_COUNT, &sectors) != RES_OK) return false;
  
  if (usb_initialized)
    USBComposite.end();
  usb_initialized = 1;
  usb_reenumerate();
  USBComposite.clear();
  USBComposite.setProductId(VENDOR_ID);
  USBComposite.setProductId(MASS_STORAGE_PRODUCT_ID);
  MassStorage.setDriveData(0, sectors, sd_read, sd_write);
  MassStorage.registerComponent();
  console_setMainSerial(NULL);
  USBComposite.begin();
  while (!USBComposite.isReady());
  return true;
}

static void flash_config_filename(char *c, int num)
{
  memcpy(c, config_filename, 13);
  c[6] = (num / 10) + '0';
  c[7] = (num % 10) + '0';
}

int writeflashstruct(int flash_page_number, int num_blocks, void *blocks[], int blocklen[])
{
  int err = 1, n;
  char filename[15];
  FIL fp;

  flash_config_filename(filename, flash_page_number);
  if ((!flash_fs_mounted) || (f_open(&fp, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK))
    return 0;
    
  for (n=0;(n<num_blocks) && (err != 0);n++)
  {
    UINT bw;
    if (f_write(&fp, blocks[n], blocklen[n], &bw) != FR_OK) err = 0;
  }
  f_close(&fp);
  return err;
}

int readflashstruct(int flash_page_number, int num_blocks, void *blocks[], int blocklen[])
{
  int err = 1, n;
  char filename[15];
  FIL fp;

  flash_config_filename(filename, flash_page_number);
  if ((!flash_fs_mounted) || (f_open(&fp, filename, FA_READ) != FR_OK))
    err = 0;
  else
  {
    for (n=0;(n<num_blocks) && (err != 0);n++)
    {
      UINT br;
      if (f_read(&fp, blocks[n], blocklen[n], &br) != FR_OK) err = 0;
    }
    f_close(&fp);
  }
  if (err == 0)
  {
    for (n=0;n<num_blocks;n++)
      memset(blocks[n], '\000', blocklen[n]);
  }
  return err;
}

const char data_filename[] = "DATAXXX.CSV";

int opendatafile(FIL *fp, char *c)
{
  FRESULT fres;

  if (!flash_fs_mounted) return NULL;
  for (int num=0;num<1000;num++)
  {
    memcpy(c, data_filename, 12);
    c[4] = (num / 100) + '0';
    c[5] = ((num / 10) % 10) + '0';
    c[6] = (num % 10) + '0';
    fres = f_open(fp, c, FA_READ);
    f_close(fp);
    if (fres == FR_OK) continue;
    fres = f_open(fp, c, FA_WRITE | FA_CREATE_ALWAYS);
    return fres == FR_OK ? 1 : 0;
  }
  return 0;
}
